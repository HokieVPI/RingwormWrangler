"""
Sweep look-ahead distance using a simplified path of exactly 10 corner waypoints
(no intermediate points along straight segments).

We start from the same boustrophedon PATH_BASE used elsewhere, extract its
corner vertices (direction changes), then downsample those corners to exactly 10
points (including start and end).

Lookahead: 50..300 cm in 50 cm steps.

Run:
  python -u sweep_lookahead_10corners.py
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import simulate_trajectory_noise as sim
from generate_report_figures import compute_crosstrack


# ======================= Config =======================

OUT_DIR = os.path.join(os.path.dirname(__file__), "lookahead_sweep_10corners_figures")
N_RUNS = 10
MAX_STEPS = 20000
SEED_REP = 0

POSITION_NOISE_STD = 35.34  # cm, per-axis Gaussian (from logs)
HEADING_NOISE_STD = np.radians(10)

VELOCITY = 100.0
WHEEL_RADIUS = 15.24
TRACK_WIDTH = 43.18
WAYPOINT_RADIUS = 100.0
DT = 0.10

LOOKAHEAD_VALUES = list(range(50, 301, 50))  # 50,100,...,300


# Base path (boustrophedon passes)
PATH_BASE = np.array(
    [
        [200.0, 502.4],
        [200.0, 852.4],
        [200.0, 1202.4],
        [200.0, 1552.4],
        [200.0, 1902.4],
        [200.0, 2252.4],
        [442.5, 2252.4],
        [442.5, 1902.4],
        [442.5, 1552.4],
        [442.5, 1202.4],
        [442.5, 852.4],
        [442.5, 502.4],
        [442.5, 152.4],
        [792.5, 152.4],
        [792.5, 502.4],
        [792.5, 852.4],
        [792.5, 1202.4],
        [792.5, 1552.4],
        [792.5, 1902.4],
        [792.5, 2252.4],
        [1142.5, 2252.4],
        [1142.5, 1902.4],
        [1142.5, 1552.4],
        [1142.5, 1202.4],
        [1142.5, 852.4],
        [1142.5, 502.4],
        [1142.5, 152.4],
        [1492.5, 152.4],
        [1492.5, 502.4],
        [1492.5, 852.4],
        [1492.5, 1202.4],
        [1492.5, 1552.4],
        [1492.5, 1902.4],
        [1492.5, 2252.4],
    ],
    dtype=float,
)


def ensure_out_dir() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)


def patch_sim_constants(look_ahead_cm: float) -> None:
    sim.LOOK_AHEAD = float(look_ahead_cm)
    sim.VELOCITY = VELOCITY
    sim.WHEEL_RADIUS = WHEEL_RADIUS
    sim.TRACK_WIDTH = TRACK_WIDTH
    sim.WAYPOINT_RADIUS = WAYPOINT_RADIUS
    sim.DT = DT


def _unit_dir(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    if n <= 1e-9:
        return np.array([0.0, 0.0], dtype=float)
    return v / n


def extract_corners(path: np.ndarray) -> np.ndarray:
    """Keep start/end and any vertex where direction changes."""
    if len(path) < 2:
        return path.copy()
    keep = [0]
    prev_dir = _unit_dir(path[1] - path[0])
    for i in range(1, len(path) - 1):
        d1 = prev_dir
        d2 = _unit_dir(path[i + 1] - path[i])
        # if direction changes (not collinear), keep as corner
        if abs(float(d1[0] * d2[0] + d1[1] * d2[1]) - 1.0) > 1e-6:
            keep.append(i)
        prev_dir = d2
    keep.append(len(path) - 1)
    return path[np.array(keep, dtype=int)]


def downsample_polyline_vertices(path: np.ndarray, n_points: int) -> np.ndarray:
    """
    Downsample a polyline to n_points by arclength, but only selecting from
    existing vertices (so corners remain corners; no inserted midpoints).
    Always includes first and last vertex.
    """
    if n_points < 2:
        raise ValueError("n_points must be >= 2")
    if len(path) <= n_points:
        return path.copy()

    segs = path[1:] - path[:-1]
    seg_len = np.hypot(segs[:, 0], segs[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(cum[-1])
    if total <= 0.0:
        return np.repeat(path[:1], n_points, axis=0)

    targets = np.linspace(0.0, total, n_points)
    idxs = np.searchsorted(cum, targets, side="left")
    idxs[0] = 0
    idxs[-1] = len(path) - 1
    idxs = np.clip(idxs, 0, len(path) - 1)

    # Ensure uniqueness and exact length by adjusting neighbors if needed.
    # Start with unique increasing indices.
    idxs = np.unique(idxs)
    while len(idxs) < n_points:
        # add missing indices by splitting the largest gap
        gaps = np.diff(idxs)
        j = int(np.argmax(gaps))
        if gaps[j] <= 1:
            break
        new_idx = int((idxs[j] + idxs[j + 1]) // 2)
        idxs = np.sort(np.unique(np.append(idxs, new_idx)))
    # If still too many, thin by dropping interior points with smallest arclength change.
    while len(idxs) > n_points:
        # drop one interior index with smallest incremental segment length
        # compute distances between chosen vertices
        chosen = path[idxs]
        d = np.hypot(np.diff(chosen[:, 0]), np.diff(chosen[:, 1]))
        # dropping an interior point k removes two segments and adds one; approximate by smallest adjacent sum
        costs = d[:-1] + d[1:]
        k = int(np.argmin(costs)) + 1
        idxs = np.delete(idxs, k)

    return path[idxs]


PATH_CORNERS = extract_corners(PATH_BASE)
PATH_10 = downsample_polyline_vertices(PATH_CORNERS, 10)


@dataclass
class Stats:
    lookahead: float
    mean_ct: float
    p95_ct: float
    max_ct: float
    completion_pct: float
    mean_goal: float


def run_mc(path: np.ndarray, lookahead: float):
    xts: List[np.ndarray] = []
    yts: List[np.ndarray] = []
    xms: List[np.ndarray] = []
    yms: List[np.ndarray] = []
    cts: List[np.ndarray] = []
    comps: List[bool] = []
    goals: List[float] = []

    start_xy = tuple(path[0])
    goal_xy = path[-1]

    patch_sim_constants(lookahead)
    for seed in range(N_RUNS):
        xt, yt, xm, ym = sim.simulate(
            position_noise_std=POSITION_NOISE_STD,
            heading_noise_std=HEADING_NOISE_STD,
            max_steps=MAX_STEPS,
            seed=seed,
            path=path,
            start_xy=start_xy,
        )
        ct = compute_crosstrack(xt, yt, path)
        dist_goal = float(np.hypot(xt[-1] - goal_xy[0], yt[-1] - goal_xy[1]))
        completed = dist_goal < WAYPOINT_RADIUS

        xts.append(xt)
        yts.append(yt)
        xms.append(xm)
        yms.append(ym)
        cts.append(ct)
        comps.append(completed)
        goals.append(dist_goal)

    return xts, yts, xms, yms, cts, comps, goals


def summarize(lookahead: float, cts: List[np.ndarray], comps: List[bool], goals: List[float]) -> Stats:
    mean_ct = float(np.mean([ct.mean() for ct in cts]))
    p95_ct = float(np.mean([np.percentile(ct, 95) for ct in cts]))
    max_ct = float(np.mean([ct.max() for ct in cts]))
    completion_pct = float(100.0 * np.mean([1.0 if c else 0.0 for c in comps]))
    mean_goal = float(np.mean(goals))
    return Stats(lookahead=float(lookahead), mean_ct=mean_ct, p95_ct=p95_ct, max_ct=max_ct, completion_pct=completion_pct, mean_goal=mean_goal)


def plot_rep(path: np.ndarray, lookahead: float, xt: np.ndarray, yt: np.ndarray, xm: np.ndarray, ym: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(path[:, 0], path[:, 1], "g--", lw=1.5, alpha=0.8, label="10 corner waypoints")
    ax.plot(path[:, 0], path[:, 1], "go", ms=6)
    ax.plot(xm, ym, "r.", ms=1.0, alpha=0.15, label="Measured (noisy)")
    ax.plot(xt, yt, "b-", lw=1.2, alpha=0.85, label="Trajectory")
    ax.plot(xt[0], yt[0], "ms", ms=10, label="Start")
    ax.plot(xt[-1], yt[-1], "r^", ms=10, label="End")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(f"10 corners, lookahead={lookahead:.0f} cm (seed={SEED_REP})")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"trajectory_10corners_LA{int(lookahead):03d}_seed{SEED_REP}.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary(stats: List[Stats]) -> None:
    las = [s.lookahead for s in stats]
    mean_ct = [s.mean_ct for s in stats]
    p95_ct = [s.p95_ct for s in stats]
    max_ct = [s.max_ct for s in stats]
    completion = [s.completion_pct for s in stats]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(las, mean_ct, "o-", label="Mean CT")
    ax.plot(las, p95_ct, "o-", label="95th CT")
    ax.plot(las, max_ct, "o-", label="Mean Max CT")
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.6, label="Waypoint radius")
    ax.set_xlabel("Lookahead (cm)")
    ax.set_ylabel("Cross-track error (cm)")
    ax.set_title(f"10-corner path: error vs lookahead (N_RUNS={N_RUNS})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "error_vs_lookahead.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(las, completion, "o-", lw=1.8)
    ax.set_xlabel("Lookahead (cm)")
    ax.set_ylabel("Completion rate (%)")
    ax.set_title(f"10-corner path: completion vs lookahead (max_steps={MAX_STEPS})")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "completion_vs_lookahead.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ensure_out_dir()
    print(f"Base path vertices: {len(PATH_BASE)}")
    print(f"Extracted corners:  {len(PATH_CORNERS)}")
    print(f"Using waypoints:    {len(PATH_10)} (10 corners)")
    print(f"Sweeping lookahead: {LOOKAHEAD_VALUES}")

    all_stats: List[Stats] = []
    for la in LOOKAHEAD_VALUES:
        print(f"\nRunning lookahead={la} cm ({N_RUNS} runs)...")
        xts, yts, xms, yms, cts, comps, goals = run_mc(PATH_10, la)
        st = summarize(la, cts, comps, goals)
        all_stats.append(st)
        print(
            f"  LA={int(la):3d} | mean_ct={st.mean_ct:6.1f} | p95_ct={st.p95_ct:6.1f} | "
            f"max_ct={st.max_ct:6.1f} | complete={st.completion_pct:5.1f}% | mean_goal={st.mean_goal:7.1f}"
        )
        plot_rep(PATH_10, la, xts[SEED_REP], yts[SEED_REP], xms[SEED_REP], yms[SEED_REP])

    plot_summary(all_stats)
    print(f"\nSaved figures to: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

