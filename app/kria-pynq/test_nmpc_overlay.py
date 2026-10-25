"""Host-side tests for NmpcOverlay's manifest PSO size and overlay-load guard, with a stubbed pynq.

Run on any PC: python3 -m pytest app/kria-pynq/test_nmpc_overlay.py -q
"""
from __future__ import annotations

import importlib.machinery
import json
import sys
import types

import numpy as np
import pytest


class FakeHardware:
    """Stands in for the PL: on ap_start it writes `iterations` fitness words to the output buffer."""

    def __init__(self) -> None:
        self.iterations = 100
        self.register_accesses = 0
        self.allocations = 0
        self.buffers: list[FakeBuffer] = []


HW = FakeHardware()


class FakeBuffer(np.ndarray):
    physical_address = 0

    def flush(self) -> None:
        pass

    def invalidate(self) -> None:
        pass

    def freebuffer(self) -> None:
        pass


def fake_allocate(shape, dtype):
    HW.allocations += 1
    buffer = np.zeros(shape, dtype).view(FakeBuffer)
    buffer.physical_address = 0x10000000 + 0x1000 * len(HW.buffers)
    HW.buffers.append(buffer)
    return buffer


class FakeMMIO:
    def __init__(self, base: int, length: int) -> None:
        self.base = base

    def write(self, offset: int, value: int) -> None:
        HW.register_accesses += 1
        if offset == 0x00 and value == 0x01:  # ap_start: the "hardware" writes its fitness history
            fitness = HW.buffers[-1]
            if HW.iterations > len(fitness):
                raise AssertionError(f"hardware wrote {HW.iterations} words into a {len(fitness)}-word buffer")
            fitness[: HW.iterations] = np.arange(HW.iterations, 0, -1) << 15

    def read(self, offset: int) -> int:
        HW.register_accesses += 1
        return {0x00: 0x06, 0x10: HW.iterations}.get(offset, 0)


@pytest.fixture()
def nmpc(monkeypatch, tmp_path):
    pynq = types.ModuleType("pynq")
    pynq.__spec__ = importlib.machinery.ModuleSpec("pynq", None)
    pynq.Overlay = lambda path, download=True: types.SimpleNamespace(ip_dict={})
    pynq.MMIO = FakeMMIO
    pynq.allocate = fake_allocate
    monkeypatch.setitem(sys.modules, "pynq", pynq)
    sys.modules.pop("nmpc_overlay", None)
    import nmpc_overlay

    boot_id = tmp_path / "boot_id"
    boot_id.write_text("boot-A\n")
    monkeypatch.setattr(nmpc_overlay, "LOADED_MARKER", tmp_path / "marker.json")
    monkeypatch.setattr(nmpc_overlay, "BOOT_ID", boot_id)
    global HW
    HW = FakeHardware()
    yield nmpc_overlay
    sys.modules.pop("nmpc_overlay", None)


def make_overlay(tmp_path, solver=None, name="overlay"):
    overlay = tmp_path / name
    overlay.mkdir()
    (overlay / "design_nmpc_solver.bit").write_bytes(b"bitstream " + name.encode())
    (overlay / "design_nmpc_solver.hwh").write_text("<hwh/>")
    manifest = {
        "design_revision": 4,
        "platform": {"pso_fsm_1": {
            "s_axi_control": {"base": 0xA0000000, "range": 0x10000},
            "s_axi_control_r": {"base": 0xA0010000, "range": 0x10000},
        }},
    }
    if solver is not None:
        manifest["solver"] = solver
    (overlay / "manifest.json").write_text(json.dumps(manifest))
    return overlay


def solve(overlay):
    return overlay.solve(np.array([7.0, 10.0] + [0.0] * 10), np.zeros((25, 12)))


def test_manifest_without_solver_is_fix6_size(nmpc, tmp_path):
    overlay = nmpc.NmpcOverlay(make_overlay(tmp_path), download=True)
    assert overlay.solver == {"particles": 10, "iterations": 100}
    HW.iterations = 100
    assert len(solve(overlay).best_fitness) == 100


def test_matching_manifest_returns_the_written_iterations(nmpc, tmp_path):
    overlay = nmpc.NmpcOverlay(make_overlay(tmp_path, {"particles": 3, "iterations": 24}), download=True)
    HW.iterations = 24
    result = solve(overlay)
    assert result.iterations == 24 and len(result.best_fitness) == 24
    assert result.best_fitness[-1] == 1.0  # last word written, not an unwritten zero


def test_understated_manifest_cannot_undersize_the_output_buffer(nmpc, tmp_path):
    # The manifest claims 24 iterations but the bitstream runs 100: the buffer must still hold all
    # 100 words (FakeMMIO fails on an overrun), and solve() must reject the mismatch.
    overlay = nmpc.NmpcOverlay(make_overlay(tmp_path, {"particles": 3, "iterations": 24}), download=True)
    HW.iterations = 100
    with pytest.raises(RuntimeError, match="ran 100 iterations but the overlay manifest says 24"):
        solve(overlay)


def test_overstated_manifest_is_rejected(nmpc, tmp_path):
    overlay = nmpc.NmpcOverlay(make_overlay(tmp_path, {"particles": 3, "iterations": 100}), download=True)
    HW.iterations = 24
    with pytest.raises(RuntimeError, match="ran 24 iterations but the overlay manifest says 100"):
        solve(overlay)


@pytest.mark.parametrize("solver", [
    {"particles": 3, "iterations": 0},
    {"particles": 3, "iterations": 129},
    {"particles": 1, "iterations": 24},
    {"particles": 21, "iterations": 24},
    {"particles": 3, "iterations": "24"},
    {"particles": 3.0, "iterations": 24},
])
def test_invalid_solver_metadata_is_rejected_before_any_hardware_access(nmpc, tmp_path, solver):
    with pytest.raises(RuntimeError, match="not an integer in"):
        nmpc.NmpcOverlay(make_overlay(tmp_path, solver), download=True)
    assert HW.allocations == 0 and HW.register_accesses == 0


def test_download_false_requires_a_marker_from_this_boot_and_bitstream(nmpc, tmp_path):
    overlay_dir = make_overlay(tmp_path, {"particles": 3, "iterations": 24})
    with pytest.raises(RuntimeError, match="not known to hold"):
        nmpc.NmpcOverlay(overlay_dir, download=False)
    assert HW.register_accesses == 0

    nmpc.NmpcOverlay(overlay_dir, download=True)  # programs and writes the marker
    nmpc.NmpcOverlay(overlay_dir, download=False)

    other = make_overlay(tmp_path, {"particles": 3, "iterations": 24}, name="other")
    accesses = HW.register_accesses
    with pytest.raises(RuntimeError, match="not known to hold"):
        nmpc.NmpcOverlay(other, download=False)  # a different bitstream is loaded
    nmpc.BOOT_ID.write_text("boot-B\n")
    with pytest.raises(RuntimeError, match="not known to hold"):
        nmpc.NmpcOverlay(overlay_dir, download=False)  # the marker is from another boot
    assert HW.register_accesses == accesses
