"""Offline test of the tuning selection rule."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tuning_select as ts  # noqa: E402


def row(t, m, rmse, ok=True):
    return {"grid": {"lookahead_time": t, "min_lookahead_dist": m}, "cross_track_rmse_m": rmse,
            "end_reason": "arm_reported_end" if ok else "timeout", "safety_event_total": 0, "reached_end": ok}


def test_best_point_next_to_an_instability_is_not_chosen():
    rows = [row(0.1, 0.2, 0.0010), row(0.1, 0.1, 0.3, ok=False),  # best, but its neighbour is unstable
            row(0.2, 0.2, 0.0012), row(0.2, 0.1, 0.0013), row(0.3, 0.2, 0.0020), row(0.3, 0.1, 0.0021)]
    res = ts.select(rows)
    assert res["best_unconstrained"] == {"lookahead_time": 0.1, "min_lookahead_dist": 0.2}
    assert res["chosen"]["lookahead_time"] == 0.2 and res["chosen"]["min_lookahead_dist"] == 0.2


def test_large_error_counts_as_unstable():
    rows = [row(0.1, 0.2, 0.001), row(0.2, 0.2, 0.0011), row(0.3, 0.2, 0.004), row(0.4, 0.2, 0.0012)]
    res = ts.select(rows)  # 0.3 is > 3x the best: 0.2 and 0.4 lose eligibility
    assert res["chosen"]["lookahead_time"] == 0.1


def test_nmpc_axes_and_edge_flag():
    rows = []
    for qp, qh, e in ((10, 1, 0.004), (10, 10, 0.003), (100, 1, 0.002), (100, 10, 0.0015), (1000, 1, 0.0012), (1000, 10, 0.001)):
        r = row(0, 0, e)
        r["grid"] = {"q_pos": qp, "q_psi": qh}
        rows.append(r)
    res = ts.select(rows)
    assert res["keys"] == ["q_pos", "q_psi"] and res["best_on_grid_edge"]
    assert res["chosen"]["q_pos"] == 1000 and res["chosen"]["q_psi"] == 10
