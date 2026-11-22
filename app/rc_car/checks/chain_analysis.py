"""Analysis of the phase 2 chain checks (numpy only; tested offline in test_chain_analysis.py)."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def motion_start(t: Sequence[float], value: Sequence[float], t_cmd: float, threshold: float,
                 baseline_window: float = 0.05) -> float:
    """Time from t_cmd until `value` first leaves its pre-command level (mean over
    `baseline_window` before t_cmd) by more than `threshold`; nan if it never does."""
    t, value = np.asarray(t, dtype=float), np.asarray(value, dtype=float)
    before = value[(t < t_cmd) & (t >= t_cmd - baseline_window)]
    if len(before) == 0:
        return math.nan
    base = float(before.mean())
    after = np.nonzero((t >= t_cmd) & (np.abs(value - base) > threshold))[0]
    return float(t[after[0]] - t_cmd) if len(after) else math.nan


def summary(samples: Sequence[float]) -> dict:
    """Distribution summary in ms (nan samples are counted, not used)."""
    s = np.asarray(samples, dtype=float)
    ok = s[np.isfinite(s)] * 1e3
    if len(ok) == 0:
        return {"n": 0, "n_nan": int(len(s))}
    q = np.percentile(ok, [5, 50, 95])
    return {"n": int(len(ok)), "n_nan": int(len(s) - len(ok)), "min_ms": float(ok.min()), "p5_ms": float(q[0]),
            "median_ms": float(q[1]), "mean_ms": float(ok.mean()), "p95_ms": float(q[2]), "max_ms": float(ok.max())}


def histogram(samples: Sequence[float], width_ms: float = 2.0) -> dict:
    """Counts per `width_ms` bin, keyed by the bin's lower edge in ms."""
    s = np.asarray(samples, dtype=float)
    s = s[np.isfinite(s)] * 1e3
    if len(s) == 0:
        return {}
    edges = np.arange(math.floor(s.min() / width_ms) * width_ms, s.max() + width_ms, width_ms)
    counts, edges = np.histogram(s, bins=edges)
    return {f"{e:.0f}": int(c) for e, c in zip(edges[:-1], counts) if c}


def first_transition(events: Sequence[tuple[float, str, str]], after: float, to: str) -> tuple[float, str] | None:
    """The first (time, reason) of a mode change to `to` at or after `after`, from (time, to, reason) rows."""
    for t, mode, reason in events:
        if t >= after and mode == to:
            return t, reason
    return None
