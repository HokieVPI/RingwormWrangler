"""
Senior Design Report Figure Generator

Generates all simulation figures for the written report in one run.
Saves PNGs to report_figures/ subfolder.

Run: python generate_report_figures.py
"""

import matplotlib
matplotlib.use("Agg")
import os
import numpy as np
import matplotlib.pyplot as plt

from simulate_trajectory_noise import (
    simulate, BORDER, LOOK_AHEAD, VELOCITY, WHEEL_RADIUS, TRACK_WIDTH,
    WAYPOINT_RADIUS, DT, generate_mowing_path, MOP_WIDTH, LANE_WIDTH,
)

OUT_DIR = "report_figures"
SEED = 42

SIMPLE_PATH = np.array([
    [200, 200], [200, 2441], [729, 2441],
    [729, 200], [1200, 200], [1200, 2441],
], dtype=float)

_raw_mowing = generate_mowing_path()
MOWING_PATH = np.delete(_raw_mowing, 17, axis=0)[:23]


# ======================= Helpers =======================

def ensure_output_dir():
    os.makedirs(OUT_DIR, exist_ok=True)


def draw_border(ax, label=True):
    ax.plot(BORDER[:, 0], BORDER[:, 1], "k-", linewidth=2.5,
            label="Room walls" if label else None)
    ax.fill(BORDER[:, 0], BORDER[:, 1], color="0.93", zorder=0)


def compute_crosstrack(x_true, y_true, path):
    n = len(x_true)
    ct = np.zeros(n)
    for i in range(n):
        min_d = np.inf
        for seg in range(len(path) - 1):
            a, b = path[seg], path[seg + 1]
            ab = b - a
            ap = np.array([x_true[i] - a[0], y_true[i] - a[1]])
            t = np.clip(np.dot(ap, ab) / (np.dot(ab, ab) + 1e-10), 0, 1)
            proj = a + t * ab
            d = np.hypot(x_true[i] - proj[0], y_true[i] - proj[1])
            min_d = min(min_d, d)
        ct[i] = min_d
    return ct


def savefig(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ======================= Figure Set 1: Noise Sweep =======================

NOISE_LEVELS = [
    (10.0,  np.radians(3),  "10 cm / 3°"),
    (30.0,  np.radians(10), "30 cm / 10°"),
    (50.0,  np.radians(17), "50 cm / 17°"),
    (75.0,  np.radians(25), "75 cm / 25°"),
]
NOISE_COLORS = ["#2ca02c", "#1f77b4", "#ff7f0e", "#d62728"]


def run_noise_sweep():
    print("Figure Set 1: Noise Robustness Sweep")
    results = []
    for pos_std, head_std, label in NOISE_LEVELS:
        xt, yt, xm, ym = simulate(
            position_noise_std=pos_std, heading_noise_std=head_std,
            seed=SEED, path=SIMPLE_PATH, start_xy=(200, 200),
        )
        ct = compute_crosstrack(xt, yt, SIMPLE_PATH)
        dist_goal = np.hypot(xt[-1] - SIMPLE_PATH[-1, 0],
                             yt[-1] - SIMPLE_PATH[-1, 1])
        results.append(dict(
            xt=xt, yt=yt, xm=xm, ym=ym, ct=ct,
            label=label, dist_goal=dist_goal,
        ))
        print(f"  {label}: steps={len(xt)}, mean_ct={ct.mean():.1f} cm, "
              f"max_ct={ct.max():.1f} cm, dist_goal={dist_goal:.1f} cm")

    # Fig 1a: 2x2 trajectory subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 16))
    for idx, (res, ax) in enumerate(zip(results, axes.flat)):
        draw_border(ax, label=(idx == 0))
        ax.plot(SIMPLE_PATH[:, 0], SIMPLE_PATH[:, 1], "g--", lw=1.5, alpha=0.7)
        ax.plot(SIMPLE_PATH[:, 0], SIMPLE_PATH[:, 1], "go", ms=7)
        ax.plot(res["xm"], res["ym"], "r.", alpha=0.15, ms=1)
        ax.plot(res["xt"], res["yt"], "b-", alpha=0.7, lw=1.2)
        ax.plot(res["xt"][0], res["yt"][0], "gs", ms=10, zorder=5)
        ax.plot(res["xt"][-1], res["yt"][-1], "r^", ms=10, zorder=5)
        ax.set_title(f"Noise: {res['label']}", fontsize=13)
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=9)
    fig.suptitle("Pure Pursuit Trajectory at Various Noise Levels", fontsize=15, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    savefig(fig, "noise_sweep_trajectories.png")

    # Fig 1b: overlaid cross-track error
    fig, ax = plt.subplots(figsize=(12, 5))
    for res, color in zip(results, NOISE_COLORS):
        t = np.arange(len(res["ct"])) * DT
        ax.plot(t, res["ct"], color=color, alpha=0.8, lw=1.2, label=res["label"])
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.5, label="Waypoint radius")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cross-Track Error (cm)")
    ax.set_title("Cross-Track Error vs Time — Noise Comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    savefig(fig, "noise_sweep_crosstrack.png")

    # Fig 1c: summary bar chart
    labels = [r["label"] for r in results]
    means = [r["ct"].mean() for r in results]
    maxes = [r["ct"].max() for r in results]
    goals = [r["dist_goal"] for r in results]

    x_pos = np.arange(len(labels))
    width = 0.25
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x_pos - width, means, width, label="Mean Cross-Track (cm)", color="#1f77b4")
    ax.bar(x_pos, maxes, width, label="Max Cross-Track (cm)", color="#ff7f0e")
    ax.bar(x_pos + width, goals, width, label="Final Dist to Goal (cm)", color="#2ca02c")
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Distance (cm)")
    ax.set_title("Controller Performance Summary — Noise Levels")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    plt.tight_layout()
    savefig(fig, "noise_sweep_summary.png")

    return results


# ======================= Figure Set 2: Starting Position =======================

START_POSITIONS = [
    ((200, 200),   "On-path (WP0)"),
    ((150, 150),   "Near-path (150,150)"),
    ((1000, 1300), "Off-path (1000,1300)"),
]
START_COLORS = ["#2ca02c", "#1f77b4", "#d62728"]


def run_start_position():
    print("\nFigure Set 2: Starting Position Comparison")
    results = []
    for (sx, sy), label in START_POSITIONS:
        xt, yt, xm, ym = simulate(
            position_noise_std=30.0, heading_noise_std=np.radians(10),
            seed=SEED, path=SIMPLE_PATH, start_xy=(sx, sy),
        )
        ct = compute_crosstrack(xt, yt, SIMPLE_PATH)
        dist_goal = np.hypot(xt[-1] - SIMPLE_PATH[-1, 0],
                             yt[-1] - SIMPLE_PATH[-1, 1])
        results.append(dict(
            xt=xt, yt=yt, xm=xm, ym=ym, ct=ct,
            label=label, start=(sx, sy), dist_goal=dist_goal,
        ))
        print(f"  {label}: steps={len(xt)}, mean_ct={ct.mean():.1f} cm, "
              f"dist_goal={dist_goal:.1f} cm")

    # Fig 2a: 1x3 trajectory subplots
    fig, axes = plt.subplots(1, 3, figsize=(20, 10))
    for idx, (res, ax) in enumerate(zip(results, axes)):
        draw_border(ax, label=(idx == 0))
        ax.plot(SIMPLE_PATH[:, 0], SIMPLE_PATH[:, 1], "g--", lw=1.5, alpha=0.7)
        ax.plot(SIMPLE_PATH[:, 0], SIMPLE_PATH[:, 1], "go", ms=7)
        for i, wp in enumerate(SIMPLE_PATH):
            ax.annotate(f"WP{i}", (wp[0], wp[1]), textcoords="offset points",
                        xytext=(8, 5), fontsize=8, color="green")
        ax.plot(res["xt"], res["yt"], "b-", alpha=0.7, lw=1.2)
        sx, sy = res["start"]
        ax.plot(sx, sy, "ms", ms=12, zorder=5, label=f"Start ({sx},{sy})")
        ax.plot(res["xt"][-1], res["yt"][-1], "r^", ms=12, zorder=5, label="End")
        ax.set_title(f"Start: {res['label']}", fontsize=12)
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
    fig.suptitle("Trajectory vs Starting Position (30 cm / 10° noise)", fontsize=14, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    savefig(fig, "start_position_trajectories.png")

    # Fig 2b: overlaid cross-track error
    fig, ax = plt.subplots(figsize=(12, 5))
    for res, color in zip(results, START_COLORS):
        t = np.arange(len(res["ct"])) * DT
        ax.plot(t, res["ct"], color=color, alpha=0.8, lw=1.2, label=res["label"])
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.5, label="Waypoint radius")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cross-Track Error (cm)")
    ax.set_title("Cross-Track Error vs Time — Starting Position Comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    savefig(fig, "start_position_crosstrack.png")

    return results


# ======================= Figure Set 3: Full Mowing Pattern =======================

def run_mowing():
    print("\nFigure Set 3: Full Boustrophedon Mowing Pattern")
    xt, yt, xm, ym = simulate(
        position_noise_std=30.0, heading_noise_std=np.radians(10),
        seed=SEED, path=MOWING_PATH, start_xy=(150, 150),
    )
    ct = compute_crosstrack(xt, yt, MOWING_PATH)
    dist_goal = np.hypot(xt[-1] - MOWING_PATH[-1, 0],
                         yt[-1] - MOWING_PATH[-1, 1])
    n_lanes = len(MOWING_PATH) // 2
    sim_min = len(xt) * DT / 60
    print(f"  Lanes: {n_lanes}, steps: {len(xt)}, time: {sim_min:.1f} min")
    print(f"  Mean CT: {ct.mean():.1f} cm, Max CT: {ct.max():.1f} cm, "
          f"Dist to goal: {dist_goal:.1f} cm")

    # Fig 3a: mowing trajectory
    fig, ax = plt.subplots(figsize=(12, 14))
    draw_border(ax)
    ax.plot(MOWING_PATH[:, 0], MOWING_PATH[:, 1], "g--", lw=1.5, alpha=0.7,
            label="Planned mowing path")
    ax.plot(MOWING_PATH[:, 0], MOWING_PATH[:, 1], "go", ms=6, zorder=4)
    for i in range(0, len(MOWING_PATH), 2):
        ax.annotate(f"{i}", (MOWING_PATH[i, 0], MOWING_PATH[i, 1]),
                    textcoords="offset points", xytext=(8, 4), fontsize=7,
                    color="green")
    ax.plot(xm, ym, "r.", alpha=0.15, ms=1.5, label="Measured positions (noisy)")
    ax.plot(xt, yt, "b-", alpha=0.7, lw=1.2, label="Actual trajectory")
    ax.plot(xt[0], yt[0], "ms", ms=14, label="Start (150,150)", zorder=5)
    ax.plot(xt[-1], yt[-1], "r^", ms=14, label="End", zorder=5)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(f"Boustrophedon Mowing — {n_lanes} lanes, "
                 f"{LANE_WIDTH} cm spacing, {MOP_WIDTH} cm mop")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    savefig(fig, "mowing_trajectory.png")

    # Fig 3b: mowing cross-track error
    fig, ax = plt.subplots(figsize=(12, 5))
    t = np.arange(len(ct)) * DT
    ax.plot(t, ct, "b-", lw=0.8, alpha=0.8, label="Cross-track error")
    ax.axhline(WAYPOINT_RADIUS, color="gray", ls="--", alpha=0.5,
               label="Waypoint radius")
    ax.axhline(ct.mean(), color="orange", ls="-.", alpha=0.7,
               label=f"Mean: {ct.mean():.1f} cm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cross-Track Error (cm)")
    ax.set_title("Cross-Track Error — Full Mowing Run")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    savefig(fig, "mowing_crosstrack.png")


# ======================= Main =======================

def main():
    print("=" * 60)
    print("Senior Design Report Figure Generator")
    print("=" * 60)
    ensure_output_dir()

    run_noise_sweep()
    run_start_position()
    run_mowing()

    n_files = len([f for f in os.listdir(OUT_DIR) if f.endswith(".png")])
    print(f"\nDone — {n_files} figures saved to {OUT_DIR}/")


if __name__ == "__main__":
    main()
