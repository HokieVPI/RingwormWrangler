"""
Pure Pursuit Monte Carlo Simulation — Adaptive vs Fixed
Matches the algorithm in RW_Tag_PF_v2.ino with injected sensor noise.
  - Position noise: ±15 cm (uniform)
  - Heading noise:  ±25 deg (uniform)
Compares fixed velocity/lookahead against adaptive speed + adaptive lookahead.
"""

import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- Robot / controller constants (from .ino) ----------
VELOCITY = 50.0           # cm/s
WHEEL_RADIUS = 15.0       # cm
TRACK_WIDTH = 43.18       # cm
LOOK_AHEAD = 200.0        # cm (max)
MIN_LOOKAHEAD = 80.0      # cm (min on sharp turns)
WAYPOINT_RADIUS = 50      # cm
MIN_SPEED_SCALE = 0.2     # floor at 20% velocity

# ---------- Waypoints ----------
PATH = np.array([
    [890, 308],
    [890, 1097],
    [1200, 1097],
    [1200, 308],
], dtype=float)

# ---------- Simulation parameters ----------
DT = 0.1                  # time step (s)
MAX_TIME = 300            # max sim seconds
NUM_RUNS = 30             # Monte Carlo runs per mode
POS_NOISE = 15.0          # cm uniform half-range
HDG_NOISE_DEG = 25.0      # deg uniform half-range
HDG_NOISE = math.radians(HDG_NOISE_DEG)

START_X, START_Y = 890, 200
START_THETA = math.pi / 2


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def find_lookahead_goal(cx, cy, seg_idx, path, L):
    Lsq = L * L
    for seg in range(seg_idx, len(path) - 1):
        dsx = path[seg + 1, 0] - path[seg, 0]
        dsy = path[seg + 1, 1] - path[seg, 1]
        fx = path[seg, 0] - cx
        fy = path[seg, 1] - cy

        qa = dsx * dsx + dsy * dsy
        qb = 2.0 * (fx * dsx + fy * dsy)
        qc = fx * fx + fy * fy - Lsq

        disc = qb * qb - 4.0 * qa * qc
        if disc < 0:
            continue

        sqrt_disc = math.sqrt(disc)
        t1 = (-qb - sqrt_disc) / (2.0 * qa)
        t2 = (-qb + sqrt_disc) / (2.0 * qa)

        best_t = -1.0
        if 0.0 <= t2 <= 1.0:
            best_t = t2
        elif 0.0 <= t1 <= 1.0:
            best_t = t1

        if best_t >= 0.0:
            gx = path[seg, 0] + best_t * dsx
            gy = path[seg, 1] + best_t * dsy
            return gx, gy

    nxt = min(seg_idx + 1, len(path) - 1)
    return path[nxt, 0], path[nxt, 1]


def run_sim(rng, pos_noise, hdg_noise, adaptive=False):
    x, y, theta = START_X, START_Y, START_THETA
    seg_idx = 0
    xs, ys = [x], [y]
    t = 0.0
    current_lookahead = LOOK_AHEAD

    while t < MAX_TIME:
        if seg_idx >= len(PATH) - 1:
            dx = PATH[-1, 0] - x
            dy = PATH[-1, 1] - y
            if dx * dx + dy * dy <= WAYPOINT_RADIUS ** 2:
                break

        meas_x = x + rng.uniform(-pos_noise, pos_noise)
        meas_y = y + rng.uniform(-pos_noise, pos_noise)
        meas_theta = theta + rng.uniform(-hdg_noise, hdg_noise)

        if seg_idx < len(PATH) - 1:
            nxt = seg_idx + 1
            ddx = PATH[nxt, 0] - meas_x
            ddy = PATH[nxt, 1] - meas_y
            if ddx * ddx + ddy * ddy <= WAYPOINT_RADIUS ** 2:
                seg_idx += 1
                if seg_idx >= len(PATH) - 1:
                    continue

        L = current_lookahead if adaptive else LOOK_AHEAD
        gx, gy = find_lookahead_goal(meas_x, meas_y, seg_idx, PATH, L)

        dx_g = gx - meas_x
        dy_g = gy - meas_y
        angle_to_goal = math.atan2(dy_g, dx_g)

        Ld = math.sqrt(dx_g * dx_g + dy_g * dy_g)
        if Ld < 1.0:
            v_cmd = 0.0
            omega = 0.0
        else:
            alpha = wrap_pi(angle_to_goal - meas_theta)

            if adaptive:
                abs_alpha = abs(alpha)
                speed_scale = 1.0
                if abs_alpha > math.pi / 4.0:
                    speed_scale = (math.pi - abs_alpha) / math.pi
                    if speed_scale < MIN_SPEED_SCALE:
                        speed_scale = MIN_SPEED_SCALE
                cmd_vel = VELOCITY * speed_scale
                current_lookahead = LOOK_AHEAD * speed_scale
                if current_lookahead < MIN_LOOKAHEAD:
                    current_lookahead = MIN_LOOKAHEAD
            else:
                cmd_vel = VELOCITY

            K = 2.0 * math.sin(alpha) / Ld
            omega = K * cmd_vel
            v_cmd = cmd_vel

        left_w = (v_cmd - omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        right_w = (v_cmd + omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        vl = left_w * WHEEL_RADIUS
        vr = right_w * WHEEL_RADIUS
        v_actual = (vr + vl) / 2.0
        omega_actual = (vr - vl) / TRACK_WIDTH

        theta += omega_actual * DT
        theta = wrap_pi(theta)
        x += v_actual * math.cos(theta) * DT
        y += v_actual * math.sin(theta) * DT

        xs.append(x)
        ys.append(y)
        t += DT

    return np.array(xs), np.array(ys)


def compute_cte(xs, ys, path):
    ctes = []
    for px, py in zip(xs, ys):
        min_dist = float("inf")
        for seg in range(len(path) - 1):
            ax_, ay_ = path[seg]
            bx_, by_ = path[seg + 1]
            abx = bx_ - ax_
            aby = by_ - ay_
            t_param = max(0, min(1, ((px - ax_) * abx + (py - ay_) * aby) / (abx * abx + aby * aby + 1e-12)))
            cx_ = ax_ + t_param * abx
            cy_ = ay_ + t_param * aby
            d = math.sqrt((px - cx_) ** 2 + (py - cy_) ** 2)
            min_dist = min(min_dist, d)
        ctes.append(min_dist)
    return ctes


# ---------- Run simulations ----------
rng_fixed = np.random.default_rng(42)
rng_adaptive = np.random.default_rng(42)  # same seed for fair comparison

fig, axes = plt.subplots(2, 2, figsize=(16, 14))

# ---- Top-left: Fixed runs ----
ax = axes[0, 0]
fixed_ctes = []
xs_ideal, ys_ideal = run_sim(np.random.default_rng(99), 0.0, 0.0, adaptive=False)
ax.plot(xs_ideal, ys_ideal, color="black", linewidth=2.5, label="Ideal (no noise)", zorder=5)

for i in range(NUM_RUNS):
    xs, ys = run_sim(rng_fixed, POS_NOISE, HDG_NOISE, adaptive=False)
    ax.plot(xs, ys, alpha=0.3, linewidth=0.8,
            label=f"±{int(POS_NOISE)}cm, ±{int(HDG_NOISE_DEG)}°" if i == 0 else None)
    fixed_ctes.extend(compute_cte(xs, ys, PATH))

ax.plot(PATH[:, 0], PATH[:, 1], 'r--o', linewidth=2, markersize=8, label="Waypoints", zorder=4)
for j, (wx, wy) in enumerate(PATH):
    circle = plt.Circle((wx, wy), WAYPOINT_RADIUS, fill=False, color='red', linestyle=':', alpha=0.5)
    ax.add_patch(circle)
    ax.annotate(f"WP{j}", (wx, wy), textcoords="offset points", xytext=(10, 10), fontsize=9)
ax.plot(START_X, START_Y, 'g^', markersize=12, label="Start", zorder=6)
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.set_title(f"FIXED speed/lookahead — {NUM_RUNS} runs")
ax.legend(loc="lower right", fontsize=8)
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)

# ---- Top-right: Adaptive runs ----
ax = axes[0, 1]
adaptive_ctes = []
xs_ideal_a, ys_ideal_a = run_sim(np.random.default_rng(99), 0.0, 0.0, adaptive=True)
ax.plot(xs_ideal_a, ys_ideal_a, color="black", linewidth=2.5, label="Ideal (no noise)", zorder=5)

for i in range(NUM_RUNS):
    xs, ys = run_sim(rng_adaptive, POS_NOISE, HDG_NOISE, adaptive=True)
    ax.plot(xs, ys, alpha=0.3, linewidth=0.8, color="green",
            label=f"±{int(POS_NOISE)}cm, ±{int(HDG_NOISE_DEG)}° (adaptive)" if i == 0 else None)
    adaptive_ctes.extend(compute_cte(xs, ys, PATH))

ax.plot(PATH[:, 0], PATH[:, 1], 'r--o', linewidth=2, markersize=8, label="Waypoints", zorder=4)
for j, (wx, wy) in enumerate(PATH):
    circle = plt.Circle((wx, wy), WAYPOINT_RADIUS, fill=False, color='red', linestyle=':', alpha=0.5)
    ax.add_patch(circle)
    ax.annotate(f"WP{j}", (wx, wy), textcoords="offset points", xytext=(10, 10), fontsize=9)
ax.plot(START_X, START_Y, 'g^', markersize=12, label="Start", zorder=6)
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.set_title(f"ADAPTIVE speed/lookahead — {NUM_RUNS} runs")
ax.legend(loc="lower right", fontsize=8)
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)

# Match axis limits between left and right
all_xlim = [axes[0, 0].get_xlim(), axes[0, 1].get_xlim()]
all_ylim = [axes[0, 0].get_ylim(), axes[0, 1].get_ylim()]
xl = (min(a[0] for a in all_xlim), max(a[1] for a in all_xlim))
yl = (min(a[0] for a in all_ylim), max(a[1] for a in all_ylim))
axes[0, 0].set_xlim(xl); axes[0, 0].set_ylim(yl)
axes[0, 1].set_xlim(xl); axes[0, 1].set_ylim(yl)

# ---- Bottom-left: CTE histogram comparison ----
ax = axes[1, 0]
fixed_arr = np.array(fixed_ctes)
adaptive_arr = np.array(adaptive_ctes)
max_bin = min(max(np.percentile(fixed_arr, 99), np.percentile(adaptive_arr, 99)), 1500)
bins = np.linspace(0, max_bin, 80)
ax.hist(fixed_arr, bins=bins, color="steelblue", edgecolor="white", alpha=0.7, label="Fixed")
ax.hist(adaptive_arr, bins=bins, color="green", edgecolor="white", alpha=0.5, label="Adaptive")
ax.set_xlabel("Cross-Track Error (cm)")
ax.set_ylabel("Count")
ax.set_title("CTE Distribution Comparison (clipped to 99th pctl)")
ax.legend()
ax.grid(True, alpha=0.3)

# ---- Bottom-right: Stats table ----
ax = axes[1, 1]
ax.axis("off")

def pct_under(arr, thresh):
    return 100.0 * np.sum(arr <= thresh) / len(arr)

rows = [
    ["Metric", "Fixed", "Adaptive"],
    ["Mean CTE (cm)", f"{np.mean(fixed_arr):.1f}", f"{np.mean(adaptive_arr):.1f}"],
    ["Median CTE (cm)", f"{np.median(fixed_arr):.1f}", f"{np.median(adaptive_arr):.1f}"],
    ["Std Dev (cm)", f"{np.std(fixed_arr):.1f}", f"{np.std(adaptive_arr):.1f}"],
    ["95th Percentile (cm)", f"{np.percentile(fixed_arr, 95):.1f}", f"{np.percentile(adaptive_arr, 95):.1f}"],
    ["Max CTE (cm)", f"{np.max(fixed_arr):.1f}", f"{np.max(adaptive_arr):.1f}"],
    ["% under 100cm", f"{pct_under(fixed_arr, 100):.1f}%", f"{pct_under(adaptive_arr, 100):.1f}%"],
    ["% under 200cm", f"{pct_under(fixed_arr, 200):.1f}%", f"{pct_under(adaptive_arr, 200):.1f}%"],
]

table = ax.table(cellText=rows[1:], colLabels=rows[0], loc='center', cellLoc='center')
table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1.0, 1.8)
for j in range(3):
    table[0, j].set_facecolor('#d4e6f1')
    table[0, j].set_text_props(weight='bold')
ax.set_title("Comparison Summary", fontsize=13, pad=20)

plt.suptitle(f"Pure Pursuit Simulation: ±{int(POS_NOISE)}cm position, ±{int(HDG_NOISE_DEG)}° heading noise\n{NUM_RUNS} runs per mode",
             fontsize=14, fontweight='bold', y=1.01)
plt.tight_layout()
out_path = "sim_results_comparison.png"
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"Plot saved to {out_path}")

print(f"\n{'='*55}")
print(f"  FIXED    — Mean: {np.mean(fixed_arr):7.1f}  Median: {np.median(fixed_arr):7.1f}  95th: {np.percentile(fixed_arr, 95):7.1f}  Max: {np.max(fixed_arr):7.1f}")
print(f"  ADAPTIVE — Mean: {np.mean(adaptive_arr):7.1f}  Median: {np.median(adaptive_arr):7.1f}  95th: {np.percentile(adaptive_arr, 95):7.1f}  Max: {np.max(adaptive_arr):7.1f}")
print(f"{'='*55}")
