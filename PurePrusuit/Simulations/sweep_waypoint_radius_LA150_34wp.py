"""
Sweep waypoint radius for fixed lookahead and path.

Fixed:
  - Path: 34-waypoint PATH_BASE (corner-preserving)
  - Lookahead: 150 cm

Sweep:
  - WAYPOINT_RADIUS: 25..200 cm in 25 cm steps

Run:
  python -u sweep_waypoint_radius_LA150_34wp.py
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import simulate_trajectory_noise as sim
from generate_report_figures import compute_crosstrack


# ======================= Config =======================

OUT_DIR = os.path.join(os.path.dirname(__file__), "waypoint_radius_sweep_figures")
N_RUNS = 10
MAX_STEPS = 20000
SEED_REP = 0

POSITION_NOISE_STD = 35.34  # cm
HEADING_NOISE_STD = np.radians(10)

LOOK_AHEAD = 150.0  # cm (fixed)
VELOCITY = 100.0
WHEEL_RADIUS = 15.24
TRACK_WIDTH = 43.18
DT = 0.10

RADII = list(range(25, 151, 25))  # 25,50,...,150


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


def patch_sim_constants(waypoint_radius: float) -> None:
    sim.LOOK_AHEAD = LOOK_AHEAD
    sim.VELOCITY = VELOCITY
    sim.WHEEL_RADIUS = WHEEL_RADIUS
    sim.TRACK_WIDTH = TRACK_WIDTH
    sim.WAYPOINT_RADIUS = float(waypoint_radius)
    sim.DT = DT


@dataclass
class Stats:
    radius: float
    mean_ct: float
    p95_ct: float
    max_ct: float
    completion_pct: float
    mean_goal: float


def classify_failure(completed: bool, n_steps: int) -> str:
    """
    Classify the most likely failure mode using only observable outputs.

    - completed: dist_to_goal < waypoint_radius
    - timeout: ran until MAX_STEPS (likely got stuck / didn't converge)
    - path_ended_not_within_radius: simulation loop ended early due to
      path segment index reaching the end, but we didn't get within radius
      (can happen under noisy localization + segment-advance logic)
    """
    if completed:
        return "completed"
    if n_steps >= MAX_STEPS:
        return "timeout"
    return "path_ended_not_within_radius"


def run_mc(path: np.ndarray, waypoint_radius: float):
    xts: List[np.ndarray] = []
    yts: List[np.ndarray] = []
    xms: List[np.ndarray] = []
    yms: List[np.ndarray] = []
    cts: List[np.ndarray] = []
    comps: List[bool] = []
    goals: List[float] = []
    failure_modes: List[str] = []

    start_xy = tuple(path[0])
    goal_xy = path[-1]

    patch_sim_constants(waypoint_radius)
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
        completed = dist_goal < waypoint_radius
        failure_modes.append(classify_failure(completed, len(xt)))

        xts.append(xt)
        yts.append(yt)
        xms.append(xm)
        yms.append(ym)
        cts.append(ct)
        comps.append(completed)
        goals.append(dist_goal)

    return xts, yts, xms, yms, cts, comps, goals, failure_modes


def summarize(radius: float, cts: List[np.ndarray], comps: List[bool], goals: List[float]) -> Stats:
    mean_ct = float(np.mean([ct.mean() for ct in cts]))
    p95_ct = float(np.mean([np.percentile(ct, 95) for ct in cts]))
    max_ct = float(np.mean([ct.max() for ct in cts]))
    completion_pct = float(100.0 * np.mean([1.0 if c else 0.0 for c in comps]))
    mean_goal = float(np.mean(goals))
    return Stats(radius=float(radius), mean_ct=mean_ct, p95_ct=p95_ct, max_ct=max_ct, completion_pct=completion_pct, mean_goal=mean_goal)


def plot_rep(path: np.ndarray, radius: float, xt: np.ndarray, yt: np.ndarray, xm: np.ndarray, ym: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(path[:, 0], path[:, 1], "g--", lw=1.5, alpha=0.7, label="Waypoints/polyline")
    ax.plot(path[:, 0], path[:, 1], "go", ms=5)
    ax.plot(xm, ym, "r.", ms=1.0, alpha=0.15, label="Measured (noisy)")
    ax.plot(xt, yt, "b-", lw=1.2, alpha=0.85, label="Trajectory")
    ax.plot(xt[0], yt[0], "ms", ms=10, label="Start")
    ax.plot(xt[-1], yt[-1], "r^", ms=10, label="End")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(f"34 waypoints, LA=150 cm, waypoint_radius={radius:.0f} cm (seed={SEED_REP})")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"trajectory_radius{int(radius):03d}_seed{SEED_REP}.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary(stats: List[Stats]) -> None:
    rs = [s.radius for s in stats]
    mean_ct = [s.mean_ct for s in stats]
    p95_ct = [s.p95_ct for s in stats]
    max_ct = [s.max_ct for s in stats]
    completion = [s.completion_pct for s in stats]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(rs, mean_ct, "o-", label="Mean CT")
    ax.plot(rs, p95_ct, "o-", label="95th CT")
    ax.plot(rs, max_ct, "o-", label="Mean Max CT")
    ax.set_xlabel("Waypoint radius (cm)")
    ax.set_ylabel("Cross-track error (cm)")
    ax.set_title(f"34-waypoint path, LA=150: error vs waypoint radius (N_RUNS={N_RUNS})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "error_vs_waypoint_radius.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(rs, completion, "o-", lw=1.8)
    ax.set_xlabel("Waypoint radius (cm)")
    ax.set_ylabel("Completion rate (%)")
    ax.set_title(f"34-waypoint path, LA=150: completion vs waypoint radius (max_steps={MAX_STEPS})")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "completion_vs_waypoint_radius.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_failure_modes(
    radii: List[int],
    mode_counts_by_r: Dict[int, Dict[str, int]],
) -> None:
    # Convert to percentages for stacked bars
    completed = []
    timeout = []
    path_end = []
    for r in radii:
        c = mode_counts_by_r[r]
        n = float(sum(c.values())) if sum(c.values()) else 1.0
        completed.append(100.0 * c.get("completed", 0) / n)
        timeout.append(100.0 * c.get("timeout", 0) / n)
        path_end.append(100.0 * c.get("path_ended_not_within_radius", 0) / n)

    # Find overall most common failure (exclude completed)
    total_fail = {"timeout": 0, "path_ended_not_within_radius": 0}
    for r in radii:
        total_fail["timeout"] += mode_counts_by_r[r].get("timeout", 0)
        total_fail["path_ended_not_within_radius"] += mode_counts_by_r[r].get("path_ended_not_within_radius", 0)
    most_common_failure = max(total_fail, key=lambda k: total_fail[k]) if sum(total_fail.values()) else "none"

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(radii))
    ax.bar(x, completed, label="Completed", color="#2ca02c")
    ax.bar(x, timeout, bottom=completed, label="Timeout (max_steps)", color="#d62728")
    ax.bar(x, path_end, bottom=(np.array(completed) + np.array(timeout)), label="Path ended, not within radius", color="#1f77b4")
    ax.set_xticks(x)
    ax.set_xticklabels([str(r) for r in radii])
    ax.set_xlabel("Waypoint radius (cm)")
    ax.set_ylabel("Percent of runs (%)")
    ax.set_title(f"Failure mode breakdown (most common failure: {most_common_failure})")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "failure_modes.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ensure_out_dir()
    print(f"Using path length: {len(PATH_BASE)} waypoints (fixed)")
    print(f"Fixed lookahead: {LOOK_AHEAD:.0f} cm")
    print(f"Sweeping waypoint_radius: {RADII}")

    all_stats: List[Stats] = []
    mode_counts_by_r: Dict[int, Dict[str, int]] = {}
    for r in RADII:
        print(f"\nRunning waypoint_radius={r} cm ({N_RUNS} runs)...")
        xts, yts, xms, yms, cts, comps, goals, modes = run_mc(PATH_BASE, r)
        st = summarize(r, cts, comps, goals)
        all_stats.append(st)

        # count failure modes
        counts = {"completed": 0, "timeout": 0, "path_ended_not_within_radius": 0}
        for m in modes:
            counts[m] = counts.get(m, 0) + 1
        mode_counts_by_r[int(r)] = counts

        print(
            f"  R={int(r):3d} | mean_ct={st.mean_ct:6.1f} | p95_ct={st.p95_ct:6.1f} | "
            f"max_ct={st.max_ct:6.1f} | complete={st.completion_pct:5.1f}% | mean_goal={st.mean_goal:7.1f}"
        )
        plot_rep(PATH_BASE, r, xts[SEED_REP], yts[SEED_REP], xms[SEED_REP], yms[SEED_REP])

    plot_summary(all_stats)
    plot_failure_modes([int(r) for r in RADII], mode_counts_by_r)
    print(f"\nSaved figures to: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

