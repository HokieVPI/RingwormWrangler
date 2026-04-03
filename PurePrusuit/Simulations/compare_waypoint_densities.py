"""
Compare pure-pursuit performance vs waypoint density using empirically-estimated noise.

Uses the 8-waypoint path in `PurePrusuit/RW_Tag_PF_v2/RW_Tag_PF_v2.ino` and
creates 16- and 32-waypoint versions by resampling along the same polyline.

Run:
  python compare_waypoint_densities.py
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

OUT_DIR = os.path.join(os.path.dirname(__file__), "waypoint_density_figures")
SEED_REP = 0
N_RUNS = 10
MAX_STEPS = 20000
# waypoint counts are chosen in main() (depends on PATH_BASE length)
WAYPOINT_COUNTS: List[int] = []

# From empirical log estimate (nights 11..18)
POSITION_NOISE_STD = 35.34  # cm, per-axis Gaussian
HEADING_NOISE_STD = np.radians(10)  # keep prior assumption (not estimated here)

# Match RW_Tag_PF_v2.ino (current values in repo)
LOOK_AHEAD = 50.0
VELOCITY = 100.0
WHEEL_RADIUS = 15.24
TRACK_WIDTH = 43.18
WAYPOINT_RADIUS = 100.0
DT = 0.10


# Base path modeled after your updated waypoint list (boustrophedon passes).
# We will compare 8/16/32 by resampling along this same polyline.
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


@dataclass
class RunResult:
    xt: np.ndarray
    yt: np.ndarray
    xm: np.ndarray
    ym: np.ndarray
    ct: np.ndarray
    completed: bool
    dist_to_goal: float


def ensure_out_dir() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)


def patch_sim_constants() -> None:
    sim.LOOK_AHEAD = LOOK_AHEAD
    sim.VELOCITY = VELOCITY
    sim.WHEEL_RADIUS = WHEEL_RADIUS
    sim.TRACK_WIDTH = TRACK_WIDTH
    sim.WAYPOINT_RADIUS = WAYPOINT_RADIUS
    sim.DT = DT


def densify_polyline_keep_corners(path: np.ndarray, n_points: int) -> np.ndarray:
    """
    Return a new polyline with exactly n_points such that:
      - every original vertex in `path` is preserved (corners stay exact)
      - extra waypoints are inserted only along segments (no corner rounding)
      - extra points are distributed proportional to segment length

    Requires n_points >= len(path).
    """
    if n_points < 2:
        raise ValueError("n_points must be >= 2")
    if len(path) < 2:
        raise ValueError("path must have >= 2 points")
    if n_points < len(path):
        raise ValueError("n_points must be >= number of base vertices (to preserve corners)")

    segs = path[1:] - path[:-1]
    seg_len = np.hypot(segs[:, 0], segs[:, 1])
    total = float(np.sum(seg_len))
    if total <= 0.0:
        # degenerate: all points same
        return np.repeat(path[:1], n_points, axis=0)

    extra = n_points - len(path)  # how many points to insert between vertices

    # Initial allocation of extra points per segment proportional to length
    alloc_f = extra * (seg_len / total)
    alloc = np.floor(alloc_f).astype(int)

    # Distribute remainder by largest fractional parts
    remainder = int(extra - int(np.sum(alloc)))
    if remainder > 0:
        frac = alloc_f - alloc
        order = np.argsort(-frac)  # descending
        for k in range(remainder):
            alloc[order[k % len(alloc)]] += 1

    # Build densified path. For each segment i->i+1, insert alloc[i] interior points.
    out: List[np.ndarray] = []
    out.append(path[0].copy())
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        m = int(alloc[i])
        if m > 0:
            # interior points at t = 1/(m+1), ..., m/(m+1)
            for j in range(1, m + 1):
                t = j / (m + 1)
                out.append(a + t * (b - a))
        out.append(b.copy())

    out_arr = np.vstack(out)
    # Safety: due to any numerical issues, trim/pad to exactly n_points
    if out_arr.shape[0] > n_points:
        out_arr = out_arr[:n_points]
    elif out_arr.shape[0] < n_points:
        out_arr = np.vstack([out_arr, np.repeat(out_arr[-1:], n_points - out_arr.shape[0], axis=0)])
    return out_arr


def run_mc(path: np.ndarray, *, n_runs: int) -> List[RunResult]:
    results: List[RunResult] = []
    start_xy = tuple(path[0])
    goal_xy = path[-1]

    for seed in range(n_runs):
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
        results.append(
            RunResult(
                xt=xt,
                yt=yt,
                xm=xm,
                ym=ym,
                ct=ct,
                completed=completed,
                dist_to_goal=dist_goal,
            )
        )
    return results


def summarize(results: List[RunResult]) -> Dict[str, float]:
    mean_ct = float(np.mean([r.ct.mean() for r in results]))
    p95_ct = float(np.mean([np.percentile(r.ct, 95) for r in results]))
    max_ct = float(np.mean([r.ct.max() for r in results]))
    completion = float(100.0 * np.mean([1.0 if r.completed else 0.0 for r in results]))
    mean_goal = float(np.mean([r.dist_to_goal for r in results]))
    return dict(
        mean_ct=mean_ct,
        p95_ct=p95_ct,
        max_ct=max_ct,
        completion_pct=completion,
        mean_dist_to_goal=mean_goal,
    )


def plot_rep_trajectory(path: np.ndarray, rep: RunResult, title: str, save_name: str) -> None:
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(path[:, 0], path[:, 1], "g--", lw=1.5, alpha=0.7, label="Waypoints/polyline")
    ax.plot(path[:, 0], path[:, 1], "go", ms=5)
    ax.plot(rep.xm, rep.ym, "r.", ms=1.0, alpha=0.15, label="Measured (noisy)")
    ax.plot(rep.xt, rep.yt, "b-", lw=1.2, alpha=0.8, label="Trajectory")
    ax.plot(rep.xt[0], rep.yt[0], "ms", ms=10, label="Start")
    ax.plot(rep.xt[-1], rep.yt[-1], "r^", ms=10, label="End")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, save_name), dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_crosstrack_box(all_res: Dict[str, List[RunResult]]) -> None:
    labels = sorted(all_res.keys(), key=lambda s: int(s))
    data = []
    for k in labels:
        pooled = np.concatenate([r.ct for r in all_res[k]])
        data.append(pooled)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.6, label="Waypoint radius")
    ax.set_xlabel("Waypoint count")
    ax.set_ylabel("Cross-track error (cm)")
    ax.set_title(f"Cross-track distribution (N_RUNS={N_RUNS}, noise={POSITION_NOISE_STD:.2f} cm)")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "crosstrack_boxplot.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ensure_out_dir()
    patch_sim_constants()

    waypoint_counts = [34, 48, 64, 95]
    paths = {str(n): densify_polyline_keep_corners(PATH_BASE, n) for n in waypoint_counts}

    all_res: Dict[str, List[RunResult]] = {}
    stats: Dict[str, Dict[str, float]] = {}

    for key, p in paths.items():
        print(f"\nRunning {key} waypoints ({N_RUNS} runs, max_steps={MAX_STEPS})...")
        res = run_mc(p, n_runs=N_RUNS)
        all_res[key] = res
        stats[key] = summarize(res)
        print(
            f"{key:>2s} waypoints | mean_ct={stats[key]['mean_ct']:.1f} cm | "
            f"p95_ct={stats[key]['p95_ct']:.1f} cm | max_ct={stats[key]['max_ct']:.1f} cm | "
            f"complete={stats[key]['completion_pct']:.1f}% | mean_goal={stats[key]['mean_dist_to_goal']:.1f} cm"
        )

        # representative trajectory plot (seed 0)
        plot_rep_trajectory(
            p,
            res[SEED_REP],
            title=f"{key} waypoints (seed={SEED_REP})",
            save_name=f"trajectory_{key}wps_seed{SEED_REP}.png",
        )

    plot_crosstrack_box(all_res)

    # simple summary bar chart
    fig, ax = plt.subplots(figsize=(9, 5))
    labels = sorted(stats.keys(), key=lambda s: int(s))
    mean_ct = [stats[k]["mean_ct"] for k in labels]
    p95_ct = [stats[k]["p95_ct"] for k in labels]
    max_ct = [stats[k]["max_ct"] for k in labels]
    x = np.arange(len(labels))
    w = 0.25
    ax.bar(x - w, mean_ct, w, label="Mean CT")
    ax.bar(x, p95_ct, w, label="95th CT")
    ax.bar(x + w, max_ct, w, label="Mean Max CT")
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.6, label="Waypoint radius")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel("Waypoint count")
    ax.set_ylabel("Cross-track error (cm)")
    ax.set_title(f"Waypoint density comparison (N_RUNS={N_RUNS}, noise={POSITION_NOISE_STD:.2f} cm)")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "summary_bars.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    # completion rate plot
    fig, ax = plt.subplots(figsize=(9, 4))
    completion = [stats[k]["completion_pct"] for k in labels]
    ax.plot(labels, completion, "o-", lw=1.8)
    ax.set_xlabel("Waypoint count")
    ax.set_ylabel("Completion rate (%)")
    ax.set_title(f"Completion vs waypoint count (N_RUNS={N_RUNS}, max_steps={MAX_STEPS})")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "completion_vs_waypoints.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"\nSaved figures to: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

