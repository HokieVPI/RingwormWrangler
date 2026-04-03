"""
Sweep look-ahead distance for a fixed 48-waypoint path.

Path: corner-preserving densification of PATH_BASE to 48 waypoints.
Lookahead: 50..300 cm in 50 cm steps.

Run:
  python -u sweep_lookahead_fixed_48wp.py
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import simulate_trajectory_noise as sim
from generate_report_figures import compute_crosstrack


# ======================= Config =======================

OUT_DIR = os.path.join(os.path.dirname(__file__), "lookahead_sweep_figures")
N_RUNS = 10
MAX_STEPS = 20000
SEED_REP = 0

POSITION_NOISE_STD = 35.34  # cm, per-axis Gaussian (from logs)
HEADING_NOISE_STD = np.radians(10)

# Match RW_Tag_PF_v2.ino constants except lookahead (swept)
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


def densify_polyline_keep_corners(path: np.ndarray, n_points: int) -> np.ndarray:
    if n_points < len(path):
        raise ValueError("n_points must be >= number of base vertices")

    segs = path[1:] - path[:-1]
    seg_len = np.hypot(segs[:, 0], segs[:, 1])
    total = float(np.sum(seg_len))
    if total <= 0.0:
        return np.repeat(path[:1], n_points, axis=0)

    extra = n_points - len(path)
    alloc_f = extra * (seg_len / total)
    alloc = np.floor(alloc_f).astype(int)
    remainder = int(extra - int(np.sum(alloc)))
    if remainder > 0:
        frac = alloc_f - alloc
        order = np.argsort(-frac)
        for k in range(remainder):
            alloc[order[k % len(alloc)]] += 1

    out: List[np.ndarray] = [path[0].copy()]
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        m = int(alloc[i])
        if m > 0:
            for j in range(1, m + 1):
                t = j / (m + 1)
                out.append(a + t * (b - a))
        out.append(b.copy())

    out_arr = np.vstack(out)
    if out_arr.shape[0] != n_points:
        # should not happen, but keep it safe
        out_arr = out_arr[:n_points]
        if out_arr.shape[0] < n_points:
            out_arr = np.vstack([out_arr, np.repeat(out_arr[-1:], n_points - out_arr.shape[0], axis=0)])
    return out_arr


PATH_48 = densify_polyline_keep_corners(PATH_BASE, 48)


@dataclass
class Stats:
    lookahead: float
    mean_ct: float
    p95_ct: float
    max_ct: float
    completion_pct: float
    mean_goal: float


def run_mc(path: np.ndarray, lookahead: float) -> tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[bool], List[float]]:
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
    return Stats(
        lookahead=float(lookahead),
        mean_ct=mean_ct,
        p95_ct=p95_ct,
        max_ct=max_ct,
        completion_pct=completion_pct,
        mean_goal=mean_goal,
    )


def plot_rep(path: np.ndarray, lookahead: float, xt: np.ndarray, yt: np.ndarray, xm: np.ndarray, ym: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(path[:, 0], path[:, 1], "g--", lw=1.5, alpha=0.7, label="Waypoints/polyline")
    ax.plot(path[:, 0], path[:, 1], "go", ms=5)
    ax.plot(xm, ym, "r.", ms=1.0, alpha=0.15, label="Measured (noisy)")
    ax.plot(xt, yt, "b-", lw=1.2, alpha=0.8, label="Trajectory")
    ax.plot(xt[0], yt[0], "ms", ms=10, label="Start")
    ax.plot(xt[-1], yt[-1], "r^", ms=10, label="End")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(f"48 waypoints, lookahead={lookahead:.0f} cm (seed={SEED_REP})")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"trajectory_LA{int(lookahead):03d}_seed{SEED_REP}.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary(stats: List[Stats]) -> None:
    las = [s.lookahead for s in stats]
    mean_ct = [s.mean_ct for s in stats]
    p95_ct = [s.p95_ct for s in stats]
    max_ct = [s.max_ct for s in stats]
    completion = [s.completion_pct for s in stats]

    # error metrics vs lookahead
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(las, mean_ct, "o-", label="Mean CT")
    ax.plot(las, p95_ct, "o-", label="95th CT")
    ax.plot(las, max_ct, "o-", label="Mean Max CT")
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.6, label="Waypoint radius")
    ax.set_xlabel("Lookahead (cm)")
    ax.set_ylabel("Cross-track error (cm)")
    ax.set_title(f"48-waypoint path: error vs lookahead (N_RUNS={N_RUNS})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "error_vs_lookahead.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    # completion vs lookahead
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(las, completion, "o-", lw=1.8)
    ax.set_xlabel("Lookahead (cm)")
    ax.set_ylabel("Completion rate (%)")
    ax.set_title(f"48-waypoint path: completion vs lookahead (max_steps={MAX_STEPS})")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "completion_vs_lookahead.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ensure_out_dir()
    print(f"Path has {len(PATH_48)} waypoints (fixed).")
    print(f"Noise: pos_std={POSITION_NOISE_STD:.2f} cm, head_std={np.degrees(HEADING_NOISE_STD):.1f} deg")
    print(f"Sweeping lookahead: {LOOKAHEAD_VALUES}")

    all_stats: List[Stats] = []
    for la in LOOKAHEAD_VALUES:
        print(f"\nRunning lookahead={la} cm ({N_RUNS} runs)...")
        xts, yts, xms, yms, cts, comps, goals = run_mc(PATH_48, la)
        st = summarize(la, cts, comps, goals)
        all_stats.append(st)
        print(
            f"  LA={int(la):3d} | mean_ct={st.mean_ct:5.1f} | p95_ct={st.p95_ct:5.1f} | "
            f"max_ct={st.max_ct:5.1f} | complete={st.completion_pct:5.1f}% | mean_goal={st.mean_goal:6.1f}"
        )
        plot_rep(PATH_48, la, xts[SEED_REP], yts[SEED_REP], xms[SEED_REP], yms[SEED_REP])

    plot_summary(all_stats)
    print(f"\nSaved figures to: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

