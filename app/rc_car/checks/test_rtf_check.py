"""Offline test of the RTF-invariance comparator."""

from __future__ import annotations

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rtf_check as rc  # noqa: E402


def write(d: Path, name: str, rmse: float, rtf: float, misses: int | None = None):
    (d / name).mkdir(parents=True)
    r = {"tracking": {"cross_track_rmse_m": rmse}, "rtf": rtf, "safety_event_total": 0, "end_reason": "arm_reported_end"}
    if misses is not None:
        r["nmpc"] = {"deadline_misses": misses, "stale_pose": 2, "fallbacks": 0, "overruns": 0, "python_busy": 0}
    tag = "baseline" if misses is None else "nmpc10x40_comp_half"
    (d / name / f"{tag}_figure_eight_v1_seed1.json").write_text(json.dumps(r))


def test_pass_and_fail(tmp_path):
    ok = tmp_path / "ok"
    write(ok, "rtfcheck_baseline_rtf1.0", 0.00200, 1.0)
    write(ok, "rtfcheck_baseline_rtf0.4", 0.00215, 0.4)
    write(ok, "rtfcheck_nmpc10x40_rtf0.5", 0.00210, 0.5, 0)
    write(ok, "rtfcheck_nmpc10x40_rtf0.4", 0.00200, 0.4, 0)
    assert rc.main([str(ok)]) == 0 and json.loads((ok / "rtf_decision.json").read_text())["pass"]
    bad = tmp_path / "bad"
    write(bad, "rtfcheck_baseline_rtf1.0", 0.00200, 1.0)
    write(bad, "rtfcheck_baseline_rtf0.4", 0.00230, 0.4)  # 0.3 mm apart
    write(bad, "rtfcheck_nmpc10x40_rtf0.5", 0.00210, 0.5, 0)
    write(bad, "rtfcheck_nmpc10x40_rtf0.4", 0.00210, 0.4, 1)  # an event count differs
    assert rc.main([str(bad)]) == 1
    d = json.loads((bad / "rtf_decision.json").read_text())
    assert not d["pass"] and not d["arms"]["baseline"]["pass"] and not d["arms"]["nmpc10x40"]["pass"]
    assert "0.35" in d["decision"]
