from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any
from pynq import Overlay

import numpy as np


DESIGN_REVISION = 4
NX = 12
NU = 4
HORIZON = 25
MAX_ITERATIONS = 100
FRACTIONAL_BITS = 15
FIXED_SCALE = 1 << FRACTIONAL_BITS

DDR_LOW_LIMIT = 0x80000000

AP_CTRL = 0x00
AP_RETURN = 0x10
AP_DEBUG_STATE = 0x18
FSM_INSTANCE = "pso_fsm_1"
FSM_PHASES = {
    0: "copy_inputs",
    1: "start_initialize",
    2: "wait_initialize",
    3: "start_evaluate",
    4: "wait_evaluate",
    5: "start_detect_minimum",
    6: "wait_detect_minimum",
    7: "start_update",
    8: "wait_update",
    9: "write_outputs",
    10: "complete",
}
ARGUMENT_REGISTERS = {
    "u_curr": 0x10,
    "x_curr": 0x18,
    "xref": 0x20,
    "last_best": 0x28,
    "new_best": 0x30,
    "bestfitness": 0x38,
}


def encode_fixed(values: Any) -> np.ndarray:
    scaled = np.ceil(np.asarray(values, dtype=np.float64) * FIXED_SCALE)
    clipped = np.clip(scaled, -0x7FFFFFFF, 0x7FFFFFFF)
    return clipped.astype(np.int32)


def decode_fixed(values: Any) -> np.ndarray:
    return np.asarray(values, dtype=np.int32).astype(np.float64) / FIXED_SCALE


@dataclass(frozen=True)
class NmpcResult:
    control_sequence: np.ndarray
    best_fitness: np.ndarray
    iterations: int
    elapsed_seconds: float


class NmpcOverlay:
    def __init__(self, overlay_dir: str | Path, download: bool = True) -> None:
        try:
            from pynq import MMIO, Overlay, allocate
        except ImportError as error:
            raise RuntimeError(
                "NmpcOverlay must run in the Kria PYNQ Python environment"
            ) from error

        overlay_path = Path(overlay_dir).expanduser().resolve()
        print("Overlay path: " + str(overlay_path))
        manifest = self._verify_manifest(overlay_path)
        bitstream = overlay_path / "design_nmpc_solver.bit"
        hwh = overlay_path / "design_nmpc_solver.hwh"

        print("Bitstream: " + str(bitstream))
        print("HWH: "+ str(hwh))
        if not bitstream.is_file() or not hwh.is_file():
            raise FileNotFoundError(
                "Expected design_nmpc_solver.bit and design_nmpc_solver.hwh in "
                f"{overlay_path}"
            )

        self.overlay = Overlay(str(bitstream), download=download)
        self.register_map = manifest["platform"][FSM_INSTANCE]
        control_map = self.register_map["s_axi_control"]
        argument_map = self.register_map["s_axi_control_r"]
        self.control = MMIO(control_map["base"], control_map["range"])
        self.arguments = MMIO(argument_map["base"], argument_map["range"])
        self._buffers = {
            "u_curr": allocate(shape=(NU,), dtype=np.int32),
            "x_curr": allocate(shape=(NX,), dtype=np.int32),
            "xref": allocate(shape=(HORIZON, NX), dtype=np.int32),
            "last_best": allocate(shape=(HORIZON, NU), dtype=np.int32),
            "new_best": allocate(shape=(HORIZON, NU), dtype=np.int32),
            "bestfitness": allocate(shape=(MAX_ITERATIONS,), dtype=np.int32),
        }
        self._closed = False
        self._configure_arguments()

    @staticmethod
    def _verify_manifest(overlay_path: Path) -> dict[str, Any]:
        manifest_path = overlay_path / "manifest.json"
        if not manifest_path.is_file():
            raise RuntimeError(
                "Overlay manifest is missing. Package the current XSA with "
                "package_overlay.py before loading the overlay."
            )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        revision = int(manifest.get("design_revision", 0))
        if revision != DESIGN_REVISION:
            raise RuntimeError(
                f"Overlay design revision {revision} does not match required "
                f"revision {DESIGN_REVISION}"
            )
        try:
            register_map = manifest["platform"][FSM_INSTANCE]
            for interface in ("s_axi_control", "s_axi_control_r"):
                aperture = register_map[interface]
                if int(aperture["base"]) < 0 or int(aperture["range"]) <= 0:
                    raise ValueError(interface)
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                "Overlay manifest does not contain a valid pso_fsm_1 register map. "
                "Repackage the XSA with package_overlay.py."
            ) from error
        return manifest

    def _configure_arguments(self) -> None:
        for name, register_offset in ARGUMENT_REGISTERS.items():
            physical_address = int(self._buffers[name].physical_address)
            if not 0 <= physical_address < DDR_LOW_LIMIT:
                self.close()
                raise RuntimeError(
                    f"Buffer {name} was allocated at 0x{physical_address:x}; "
                    "the 32-bit HP0 mapping only permits DDR below 0x80000000"
                )
            self.arguments.write(register_offset, physical_address)

    @staticmethod
    def _require_shape(name: str, values: Any, shape: tuple[int, ...]) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != shape:
            raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} contains non-finite values")
        return array

    def solve(
        self,
        curr_state: Any,
        xref: Any,
        u_curr: Any | None = None,
        last_best: Any | None = None,
        timeout_seconds: float = 120.0,
    ) -> NmpcResult:
        if self._closed:
            raise RuntimeError("Overlay driver is closed")
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")

        state = self._require_shape("curr_state", curr_state, (NX,))
        reference = self._require_shape("xref", xref, (HORIZON, NX))
        current_control = self._require_shape(
            "u_curr", np.zeros(NU) if u_curr is None else u_curr, (NU,)
        )
        warm_start = self._require_shape(
            "last_best",
            np.zeros((HORIZON, NU)) if last_best is None else last_best,
            (HORIZON, NU),
        )

        self._buffers["u_curr"][:] = encode_fixed(current_control)
        self._buffers["x_curr"][:] = encode_fixed(state)
        self._buffers["xref"][:] = encode_fixed(reference)
        self._buffers["last_best"][:] = encode_fixed(warm_start)
        self._buffers["new_best"].fill(0)
        self._buffers["bestfitness"].fill(0)
        for buffer in self._buffers.values():
            buffer.flush()

        self.control.read(AP_CTRL)
        started_at = time.monotonic()
        deadline = started_at + timeout_seconds
        self.control.write(AP_CTRL, 0x01)
        while True:
            status = self.control.read(AP_CTRL)
            if status & 0x02:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"NMPC solver did not complete within {timeout_seconds:.3f} s; "
                    f"last AP_CTRL=0x{status:08x}"
                )
            time.sleep(0.0001)

        elapsed = time.monotonic() - started_at
        self._buffers["new_best"].invalidate()
        self._buffers["bestfitness"].invalidate()
        return NmpcResult(
            control_sequence=decode_fixed(self._buffers["new_best"]).copy(),
            best_fitness=decode_fixed(self._buffers["bestfitness"]).copy(),
            iterations=int(self.control.read(AP_RETURN)),
            elapsed_seconds=elapsed,
        )

    def runtime_status(self) -> dict[str, Any]:
        if self._closed:
            raise RuntimeError("Overlay driver is closed")
        fsm_state = self.control.read(AP_DEBUG_STATE)
        status = {
            "ap_ctrl": self.control.read(AP_CTRL),
            "ap_return": self.control.read(AP_RETURN),
            "fsm_state": fsm_state,
            "fsm_phase": FSM_PHASES.get(fsm_state, "unknown"),
            "argument_addresses": {
                name: self.arguments.read(offset)
                for name, offset in ARGUMENT_REGISTERS.items()
            },
        }
        return status

    def close(self) -> None:
        if self._closed:
            return
        for buffer in getattr(self, "_buffers", {}).values():
            buffer.freebuffer()
        self._closed = True

    def __enter__(self) -> "NmpcOverlay":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
