"""
Look-ahead Distance Sweep Simulation

Sweeps look-ahead distance from 25-150 cm (step 25) with 50 Monte Carlo
runs per value. Uses the current 6-waypoint path from RW_Tag_PF_v2.ino
with Gaussian noise (std = 7.5 cm position, 10 deg heading).

Outputs cross-track error statistics and three figures.

Run:  python sweep_lookahead.py
"""

import matplotlib
matplotlib.use("Agg")
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict

sys.path.insert(0, os.path.dirname(__file__))
import simulate_trajectory_noise as sim
from generate_report_figures import compute_crosstrack

# ======================= Configuration =======================

WAYPOINTS = np.array([
    [244, 122],
    [244, 420],
    [457, 420],
    [457, 122],
], dtype=float)

LOOKAHEAD_VALUES = list(range(25, 76, 1))  # 25 to 75 cm, step 1 cm (51 values)
N_RUNS = 50
POSITION_NOISE_STD = 7.5       # cm  (~2-sigma bound of 15 cm)
HEADING_NOISE_STD = np.radians(10)  # ~2-sigma bound of 20 deg
START_XY = (232.0, 96.0)

# Robot constants matching the current .ino
TRACK_WIDTH = 43.18
WHEEL_RADIUS = 15.0
VELOCITY = 25.0
WAYPOINT_RADIUS = 20.0
DT = 0.10


def patch_module_constants(look_ahead: float) -> None:
    """Override the module-level globals that simulate/pure_pursuit_step read."""
    sim.LOOK_AHEAD = look_ahead
    sim.TRACK_WIDTH = TRACK_WIDTH
    sim.WHEEL_RADIUS = WHEEL_RADIUS
    sim.VELOCITY = VELOCITY
    sim.WAYPOINT_RADIUS = WAYPOINT_RADIUS
    sim.DT = DT


# ======================= Sweep =======================

def run_sweep() -> List[Dict]:
    results = []

    for la in LOOKAHEAD_VALUES:
        patch_module_constants(la)
        run_stats = []

        # store one representative trajectory (seed=0)
        rep_xt, rep_yt = None, None

        for seed in range(N_RUNS):
            xt, yt, xm, ym = sim.simulate(
                position_noise_std=POSITION_NOISE_STD,
                heading_noise_std=HEADING_NOISE_STD,
                seed=seed,
                path=WAYPOINTS,
                start_xy=START_XY,
            )

            ct = compute_crosstrack(xt, yt, WAYPOINTS)
            dist_to_goal = np.hypot(xt[-1] - WAYPOINTS[-1, 0],
                                    yt[-1] - WAYPOINTS[-1, 1])
            completed = dist_to_goal < WAYPOINT_RADIUS

            run_stats.append(dict(
                mean_ct=ct.mean(),
                max_ct=ct.max(),
                p95_ct=np.percentile(ct, 95),
                completed=completed,
            ))

            if seed == 0:
                rep_xt, rep_yt = xt, yt

        means = [r["mean_ct"] for r in run_stats]
        maxes = [r["max_ct"] for r in run_stats]
        p95s = [r["p95_ct"] for r in run_stats]
        comp_rate = sum(r["completed"] for r in run_stats) / N_RUNS * 100

        entry = dict(
            look_ahead=la,
            mean_ct=np.mean(means),
            std_ct=np.std(means),
            mean_max_ct=np.mean(maxes),
            mean_p95_ct=np.mean(p95s),
            completion_pct=comp_rate,
            rep_xt=rep_xt,
            rep_yt=rep_yt,
        )
        results.append(entry)

        print(f"  LA={la:>4d} cm | Mean CT={entry['mean_ct']:6.1f} cm "
              f"| Max CT={entry['mean_max_ct']:6.1f} cm "
              f"| 95th CT={entry['mean_p95_ct']:6.1f} cm "
              f"| Complete={comp_rate:5.1f}%")

    return results


# ======================= Figures =======================

def plot_crosstrack_vs_lookahead(results: List[Dict], save_path: str) -> None:
    la_vals = [r["look_ahead"] for r in results]
    means = [r["mean_ct"] for r in results]
    stds = [r["std_ct"] for r in results]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.errorbar(la_vals, means, yerr=stds, fmt=".-", capsize=3,
                linewidth=1.5, markersize=5, label="Mean cross-track error")
    ax.set_xlabel("Look-ahead Distance (cm)", fontsize=12)
    ax.set_ylabel("Mean Cross-Track Error (cm)", fontsize=12)
    ax.set_title("Cross-Track Error vs Look-ahead Distance\n"
                 f"({N_RUNS} runs, noise: {POSITION_NOISE_STD} cm / "
                 f"{np.degrees(HEADING_NOISE_STD):.0f} deg)", fontsize=13)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


def plot_trajectories_grid(results: List[Dict], save_path: str) -> None:
    # pick 6 evenly spaced samples from the full results list
    indices = np.linspace(0, len(results) - 1, 6, dtype=int)
    subset = [results[i] for i in indices]

    cols = 3
    rows = 2
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows))

    for res, ax in zip(subset, axes.flat):
        ax.plot(WAYPOINTS[:, 0], WAYPOINTS[:, 1], "g--", lw=1.5, alpha=0.7)
        ax.plot(WAYPOINTS[:, 0], WAYPOINTS[:, 1], "go", ms=8)
        for i, wp in enumerate(WAYPOINTS):
            ax.annotate(f"WP{i}", (wp[0], wp[1]),
                        textcoords="offset points", xytext=(8, 5), fontsize=8)
        ax.plot(res["rep_xt"], res["rep_yt"], "b-", alpha=0.8, lw=1.2)
        ax.plot(res["rep_xt"][0], res["rep_yt"][0], "ms", ms=10, zorder=5)
        ax.plot(res["rep_xt"][-1], res["rep_yt"][-1], "r^", ms=10, zorder=5)
        ax.set_title(f"LA = {res['look_ahead']} cm  "
                     f"(CT={res['mean_ct']:.1f} cm)", fontsize=11)
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    fig.suptitle("Representative Trajectories per Look-ahead Value", fontsize=14, y=1.01)
    plt.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {save_path}")


def plot_summary(results: List[Dict], save_path: str) -> None:
    la_vals = np.array([r["look_ahead"] for r in results])
    mean_ct = np.array([r["mean_ct"] for r in results])
    max_ct = np.array([r["mean_max_ct"] for r in results])
    comp = np.array([r["completion_pct"] for r in results])

    fig, ax1 = plt.subplots(figsize=(12, 5))

    ax1.plot(la_vals, mean_ct, "b.-", lw=1.5, ms=4, label="Mean CT (cm)")
    ax1.plot(la_vals, max_ct, "r.-", lw=1.5, ms=4, label="Mean Max CT (cm)")
    ax1.set_xlabel("Look-ahead Distance (cm)", fontsize=12)
    ax1.set_ylabel("Cross-Track Error (cm)", fontsize=12)
    ax1.legend(loc="upper left", fontsize=10)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(la_vals, comp, "g.-", linewidth=1.5, markersize=4, label="Completion %")
    ax2.set_ylabel("Path Completion (%)", fontsize=12)
    ax2.set_ylim(0, 110)
    ax2.legend(loc="upper right", fontsize=10)

    ax1.set_title("Look-ahead Sweep Summary\n"
                  f"({N_RUNS} runs, noise: {POSITION_NOISE_STD} cm / "
                  f"{np.degrees(HEADING_NOISE_STD):.0f} deg)", fontsize=13)
    plt.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


# ======================= Main =======================

def main():
    print("=" * 60)
    print("Look-ahead Distance Sweep Simulation")
    print("=" * 60)
    print(f"Waypoints:        {len(WAYPOINTS)}")
    print(f"Look-ahead range: {LOOKAHEAD_VALUES[0]}-{LOOKAHEAD_VALUES[-1]} cm "
          f"(step {LOOKAHEAD_VALUES[1] - LOOKAHEAD_VALUES[0]})")
    print(f"Monte Carlo runs: {N_RUNS}")
    print(f"Position noise:   {POSITION_NOISE_STD} cm std")
    print(f"Heading noise:    {np.degrees(HEADING_NOISE_STD):.1f} deg std")
    print(f"Start position:   {START_XY}")
    print(f"Waypoint radius:  {WAYPOINT_RADIUS} cm")
    print(f"Track width:      {TRACK_WIDTH} cm")
    print()

    print("Running sweep...")
    results = run_sweep()

    best = min(results, key=lambda r: r["mean_ct"])
    print()
    print(f">>> Best look-ahead: {best['look_ahead']} cm "
          f"(mean CT = {best['mean_ct']:.1f} cm, "
          f"completion = {best['completion_pct']:.0f}%)")
    print()

    out_dir = os.path.dirname(__file__)
    print("Generating figures...")
    plot_crosstrack_vs_lookahead(
        results, os.path.join(out_dir, "sweep_lookahead_crosstrack.png"))
    plot_trajectories_grid(
        results, os.path.join(out_dir, "sweep_lookahead_trajectories.png"))
    plot_summary(
        results, os.path.join(out_dir, "sweep_lookahead_summary.png"))

    print("\nDone.")


if __name__ == "__main__":
    main()
