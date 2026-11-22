"""Tests of the HLS bicycle emulation and the config generator (pure Python; no C build).
The C-level comparison itself is app/rc_car/hls_bicycle_check.py."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "checks"))
import gen_bicycle_config as gen  # noqa: E402
import hls_bicycle_emul as E  # noqa: E402
import nmpc_engine as ne  # noqa: E402


def test_fixed_point_rounding_and_saturation():
    assert E.fxi(0.05) == 1638                       # 0.0499878, the stored _Ts
    assert E.fxi(1.5 * E.LSB) == 2 and E.fxi(-1.5 * E.LSB) == -2   # ties away from zero
    assert E.fxi(1e9) == E.TOP_MAX_I and E.fxi(-1e9) == -E.TOP_MAX_I
    assert E.rnd(3 << 14, 15) == 2 and E.rnd(-(3 << 14), 15) == -2  # 1.5 -> 2, -1.5 -> -2
    assert E.rnd(np.array([1 << 47]), 15)[0] == E.TOP_MAX_I  # 2^32 LSB saturates


def test_glibc_rand_matches_known_sequence():
    g = E.GlibcRand(1)
    assert [g() for _ in range(3)] == [1804289383, 846930886, 1681692777]


def test_generated_blocks_are_in_tree_and_kappa_from_yaml():
    assert gen.main(["--check"]) == 0
    v = gen.values()
    assert math.isclose(v["kappa_max"], math.tan(v["delta_max"]) / v["L"])
    assert v["knots"] == (0, 6, 12, 18, 24)


def test_stop_command_predicts_standstill_and_straight_line():
    c = E.make_config()
    x0 = np.array([[E.fxi(0.1), E.fxi(-0.2), E.fxi(0.3)]])
    stop = np.array([[E.fxi(-100), E.fxi(80)]])
    assert np.array_equal(E.rk4(c, x0, stop), x0)
    straight = np.array([[E.fxi(0.0), 0]])  # v = 1 m/s, kappa = 0 from psi = 0
    x = E.rk4(c, np.zeros((1, 3), dtype=np.int64), straight)
    assert x[0, 1] == 0 and x[0, 2] == 0 and abs(x[0, 0] * E.LSB - 0.05) < 1e-3


def test_knot_interpolation_endpoints():
    c = E.make_config(("R2",))
    cg = np.array([[[E.fxi(10.0 * j), E.fxi(-5.0 * j)] for j in range(5)]], dtype=np.int64)
    assert np.array_equal(E.stage_u(c, cg, 0), cg[:, 0])
    assert np.array_equal(E.stage_u(c, cg, 6), cg[:, 1])
    assert np.array_equal(E.stage_u(c, cg, 24), cg[:, 4])
    assert abs(E.stage_u(c, cg, 3)[0, 0] * E.LSB - 5.0) < 2 * E.LSB


def test_aux_layout_lengths():
    assert E.make_config(E.SWITCHES).aux_len == 111
    assert E.make_config(("R3",)).aux_len == 151
    assert E.make_config(()).aux_len == 0


def test_path_frame_cost_is_rotation_invariant():
    """R3: a lateral offset costs the same whatever the reference heading."""
    c = E.make_config(("R3",))
    nh = c.nh
    aux = np.zeros(c.aux_len, dtype=np.int64)
    cs_off = 1 + 2 * c.nu + 2 * nh
    stop = np.tile([E.fxi(-100), 0], (1, c.nu, 1)).astype(np.int64)
    costs = []
    for th in (0.0, 0.7):
        aux[cs_off::2] = E.fxi(math.cos(th))
        aux[cs_off + 1::2] = E.fxi(math.sin(th))
        ref = np.array([[E.fxi(-0.01 * math.sin(th)), E.fxi(0.01 * math.cos(th)), E.fxi(th)]] * nh)
        costs.append(int(E.cost(c, np.array([0, 0, E.fxi(th)]), stop, ref, np.array([E.fxi(-100), 0]), aux)[0]))
    assert abs(costs[0] - costs[1]) <= 0.002 * costs[0]


def test_uniform_perturbation_option_keeps_default():
    assert ne.Problem.__dataclass_fields__["perturb_dist"].default == "normal"
