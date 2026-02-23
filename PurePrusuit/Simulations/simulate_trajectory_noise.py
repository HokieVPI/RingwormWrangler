"""
Pure Pursuit Trajectory Simulation with Noise

Simulates the robot following the path from RW_Tag_PF_v2.ino with:
- Position and heading noise (mimics UWB/localization errors)
- Differential-drive kinematics
- Pure pursuit controller

Run: python simulate_trajectory_noise.py

Requires: numpy, matplotlib
  pip install numpy matplotlib
"""

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional

# ======================= Constants (from RW_Tag_PF_v2.ino) =======================

LOOK_AHEAD = 100.0      # cm
VELOCITY = 25.0         # cm/s (constant speed)
WHEEL_RADIUS = 15.0     # cm
TRACK_WIDTH = 86.0      # cm
WAYPOINT_RADIUS = 25    # cm

# Room border polygon (closed) for drawing walls
BORDER = np.array([
    [0,    0],
    [2134, 0],
    [2134, 823],
    [1570, 823],
    [1570, 2621],
    [0,    2621],
    [0,    0],
], dtype=float)

# Room geometry constants
ROOM_X_NARROW = 1570     # narrow section width (x=0 to x=1570)
ROOM_X_WIDE   = 2134     # wide section width (x=0 to x=2134, top only)
ROOM_Y_MAX    = 2621     # full room height
ROOM_Y_STEP   = 823      # y where step occurs (wide above, narrow below)
MOP_WIDTH     = 160       # cm - mop on the front of the robot
LANE_WIDTH    = 180       # cm - buffer/spacing between passes


def generate_mowing_path(lane_width=LANE_WIDTH, wall_offset=None):
    """
    Generate boustrophedon (back-and-forth) mowing path for the L-shaped room.
    180cm lane spacing for the 160cm mop gives 20cm overlap between passes.
    """
    if wall_offset is None:
        wall_offset = lane_width / 2  # 90cm from walls

    y_top = wall_offset                          # 90
    y_bottom = ROOM_Y_MAX - wall_offset          # 2531
    y_step_inner = ROOM_Y_STEP - wall_offset     # 733

    # --- Narrow section lanes (full height: y_top to y_bottom) ---
    narrow_lanes = []
    x = wall_offset
    x_narrow_max = ROOM_X_NARROW - wall_offset   # 1480
    while x <= x_narrow_max + 1:
        narrow_lanes.append(x)
        x += lane_width
    if narrow_lanes[-1] < x_narrow_max - 20:
        narrow_lanes.append(x_narrow_max)

    # --- Extension lanes (top section only: y_top to y_step_inner) ---
    ext_lanes = []
    x = ROOM_X_NARROW + wall_offset              # 1660
    x_wide_max = ROOM_X_WIDE - wall_offset       # 2044
    while x <= x_wide_max + 1:
        ext_lanes.append(x)
        x += lane_width
    if ext_lanes and ext_lanes[-1] < x_wide_max - 20:
        ext_lanes.append(x_wide_max)

    waypoints = []
    going_down = True  # first pass goes from y_top toward y_bottom

    for lx in narrow_lanes:
        if going_down:
            waypoints.append([lx, y_top])
            waypoints.append([lx, y_bottom])
        else:
            waypoints.append([lx, y_bottom])
            waypoints.append([lx, y_top])
        going_down = not going_down

    for lx in ext_lanes:
        if going_down:
            waypoints.append([lx, y_top])
            waypoints.append([lx, y_step_inner])
        else:
            waypoints.append([lx, y_step_inner])
            waypoints.append([lx, y_top])
        going_down = not going_down

    return np.array(waypoints, dtype=float)


_raw_path = generate_mowing_path()
# Remove WP17 (bottom of lane 9) so robot goes directly from WP16 to WP18,
# and end at WP23 (removing the last tiny lane at x=2044).
PATH = np.delete(_raw_path, 17, axis=0)[:23]

# Noise parameters (cm, radians)
POSITION_NOISE_STD = 30.0   # cm - std dev of Gaussian noise on x, y
HEADING_NOISE_STD = 0.175   # rad (~10 deg)
DT = 0.10                   # simulation timestep (s) - 10 Hz update rate


# ======================= Pure Pursuit (ported from .ino) =======================

def wrap_angle_pi(a: float) -> float:
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def find_lookahead_goal(
    cur_x: float, cur_y: float,
    path: np.ndarray, path_seg_idx: int,
    look_ahead: float
) -> Tuple[Optional[float], Optional[float], bool]:
    """Returns (gx, gy, found)."""
    path_len = len(path)
    Lsq = look_ahead ** 2

    for seg in range(path_seg_idx, path_len - 1):
        dsx = path[seg + 1, 0] - path[seg, 0]
        dsy = path[seg + 1, 1] - path[seg, 1]
        fx = path[seg, 0] - cur_x
        fy = path[seg, 1] - cur_y

        qa = dsx * dsx + dsy * dsy
        qb = 2.0 * (fx * dsx + fy * dsy)
        qc = (fx * fx + fy * fy) - Lsq
        discriminant = qb * qb - 4.0 * qa * qc

        if discriminant < 0:
            return (path[seg, 0], path[seg, 1], True)

        sqrt_disc = np.sqrt(discriminant)
        t1 = (-qb - sqrt_disc) / (2.0 * qa)
        t2 = (-qb + sqrt_disc) / (2.0 * qa)

        best_t = -1.0
        if 0 <= t2 <= 1:
            best_t = t2
        elif 0 <= t1 <= 1:
            best_t = t1

        if best_t >= 0:
            gx = path[seg, 0] + best_t * dsx
            gy = path[seg, 1] + best_t * dsy
            return (gx, gy, True)

    return (None, None, False)


def advance_path_segment(
    cur_x: float, cur_y: float,
    path: np.ndarray, path_seg_idx: int,
    waypoint_radius: float
) -> int:
    if path_seg_idx >= len(path) - 1:
        return path_seg_idx
    next_wp = path[path_seg_idx + 1]
    dx = next_wp[0] - cur_x
    dy = next_wp[1] - cur_y
    dist_sq = dx * dx + dy * dy
    if dist_sq <= waypoint_radius ** 2:
        return path_seg_idx + 1
    return path_seg_idx


def pure_pursuit_step(
    cur_x: float, cur_y: float, heading: float,
    path: np.ndarray, path_seg_idx: int
) -> Tuple[float, float, int, bool]:
    """
    Returns (left_motor_rad_s, right_motor_rad_s, new_path_seg_idx, valid).
    """
    path_seg_idx = advance_path_segment(
        cur_x, cur_y, path, path_seg_idx, WAYPOINT_RADIUS
    )
    gx, gy, found = find_lookahead_goal(
        cur_x, cur_y, path, path_seg_idx, LOOK_AHEAD
    )

    if not found or gx is None:
        return (0.0, 0.0, path_seg_idx, False)

    delta_x = gx - cur_x
    delta_y = gy - cur_y
    L_d = np.sqrt(delta_x * delta_x + delta_y * delta_y)

    if L_d < 1.0:
        return (0.0, 0.0, path_seg_idx, False)

    angle_to_goal = np.arctan2(delta_y, delta_x)
    alpha = wrap_angle_pi(angle_to_goal - heading)
    K = 2.0 * np.sin(alpha) / L_d
    omega = K * VELOCITY

    left_motor = (VELOCITY - omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    right_motor = (VELOCITY + omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    return (left_motor, right_motor, path_seg_idx, True)


# ======================= Differential Drive Kinematics =======================

def step_kinematics(
    x: float, y: float, theta: float,
    left_rad_s: float, right_rad_s: float,
    dt: float
) -> Tuple[float, float, float]:
    """Update state given wheel angular velocities. Returns (x, y, theta)."""
    v_left = left_rad_s * WHEEL_RADIUS
    v_right = right_rad_s * WHEEL_RADIUS
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / TRACK_WIDTH

    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    theta_new = wrap_angle_pi(theta_new)
    return (x_new, y_new, theta_new)


# ======================= Simulation =======================

def simulate(
    position_noise_std: float = POSITION_NOISE_STD,
    heading_noise_std: float = HEADING_NOISE_STD,
    max_steps: int = 80000,
    seed: Optional[int] = None,
    path: Optional[np.ndarray] = None,
    start_xy: Optional[Tuple[float, float]] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns (x_true, y_true, x_measured, y_measured) - all (N,) arrays.
    Controller uses noisy measurements; truth is used for physics.
    """
    if path is None:
        path = PATH
    if seed is not None:
        np.random.seed(seed)

    if start_xy is not None:
        x_true, y_true = float(start_xy[0]), float(start_xy[1])
    else:
        x_true = 150.0
        y_true = 150.0

    angle_to_wp0 = np.arctan2(path[0, 1] - y_true, path[0, 0] - x_true)
    theta_true = angle_to_wp0

    x_meas = x_true + np.random.normal(0, position_noise_std)
    y_meas = y_true + np.random.normal(0, position_noise_std)
    theta_meas = theta_true + np.random.normal(0, heading_noise_std)

    path_seg_idx = 0

    hist_x_true = [x_true]
    hist_y_true = [y_true]
    hist_x_meas = [x_meas]
    hist_y_meas = [y_meas]

    for _ in range(max_steps - 1):
        if path_seg_idx >= len(path) - 1:
            break

        left_m, right_m, path_seg_idx, valid = pure_pursuit_step(
            x_meas, y_meas, theta_meas, path, path_seg_idx
        )

        if not valid:
            left_m = right_m = 0.5 * VELOCITY / WHEEL_RADIUS

        x_true, y_true, theta_true = step_kinematics(
            x_true, y_true, theta_true, left_m, right_m, DT
        )

        x_meas = x_true + np.random.normal(0, position_noise_std)
        y_meas = y_true + np.random.normal(0, position_noise_std)
        theta_meas = theta_true + np.random.normal(0, heading_noise_std)

        hist_x_true.append(x_true)
        hist_y_true.append(y_true)
        hist_x_meas.append(x_meas)
        hist_y_meas.append(y_meas)

        dist_to_end = np.hypot(x_true - path[-1, 0], y_true - path[-1, 1])
        if dist_to_end < WAYPOINT_RADIUS:
            break

    return (
        np.array(hist_x_true),
        np.array(hist_y_true),
        np.array(hist_x_meas),
        np.array(hist_y_meas),
    )


# ======================= Plotting =======================

def _draw_border(ax):
    """Draw room walls as thick lines on an axis."""
    ax.plot(BORDER[:, 0], BORDER[:, 1], "k-", linewidth=3, label="Room walls")
    ax.fill(BORDER[:, 0], BORDER[:, 1], color="0.93", zorder=0)


def plot_trajectory_truth(
    x_true: np.ndarray, y_true: np.ndarray,
    save_path: str = "trajectory_truth.png"
) -> None:
    fig, ax = plt.subplots(figsize=(10, 12))
    _draw_border(ax)
    ax.plot(PATH[:, 0], PATH[:, 1], "k--", label="Reference path", linewidth=1.5, alpha=0.6)
    ax.plot(PATH[:, 0], PATH[:, 1], "ko", markersize=8)
    for i, wp in enumerate(PATH):
        ax.annotate(f"WP{i}", (wp[0], wp[1]), textcoords="offset points",
                    xytext=(10, 5), fontsize=9)
    ax.plot(x_true, y_true, "b-", alpha=0.8, linewidth=1.5, label="Actual trajectory (truth)")
    ax.plot(x_true[0], y_true[0], "gs", markersize=12, label="Start", zorder=5)
    ax.plot(x_true[-1], y_true[-1], "r^", markersize=12, label="End", zorder=5)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title("Trajectory with Noise (Truth) - Room Border")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"Saved: {save_path}")
    plt.close(fig)


def plot_measured_vs_actual(
    x_true: np.ndarray, y_true: np.ndarray,
    x_meas: np.ndarray, y_meas: np.ndarray,
    save_path: str = "measured_vs_actual.png"
) -> None:
    fig, ax = plt.subplots(figsize=(12, 14))
    _draw_border(ax)
    ax.plot(PATH[:, 0], PATH[:, 1], "g--", label="Planned mowing path", linewidth=1.5, alpha=0.7)
    ax.plot(PATH[:, 0], PATH[:, 1], "go", markersize=6, zorder=4)
    for i in range(0, len(PATH), 2):
        ax.annotate(f"{i}", (PATH[i, 0], PATH[i, 1]), textcoords="offset points",
                    xytext=(8, 4), fontsize=7, color="green")
    ax.plot(x_meas, y_meas, "r.", alpha=0.2, markersize=1.5, label="Measured positions (noisy)")
    ax.plot(x_true, y_true, "b-", alpha=0.7, linewidth=1.2, label="Actual trajectory")
    ax.plot(x_true[0], y_true[0], "ms", markersize=14, label="Start (150,150)", zorder=5)
    ax.plot(x_true[-1], y_true[-1], "r^", markersize=14, label="End", zorder=5)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    n_lanes = len(PATH) // 2
    ax.set_title(f"Boustrophedon Mowing - {n_lanes} lanes, {LANE_WIDTH}cm spacing, {MOP_WIDTH}cm mop")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"Saved: {save_path}")
    plt.close(fig)


def plot_tracking_error(
    x_true: np.ndarray, y_true: np.ndarray,
    x_meas: np.ndarray, y_meas: np.ndarray,
    save_path: str = "tracking_error.png"
) -> None:
    """Compute cross-track error (distance from path) over time."""
    n = len(x_true)
    cross_track = np.zeros(n)

    for i in range(n):
        # Find closest point on path (all segments)
        min_d = np.inf
        for seg in range(len(PATH) - 1):
            a, b = PATH[seg], PATH[seg + 1]
            # Project (x,y) onto segment a->b
            ab = b - a
            ap = np.array([x_true[i] - a[0], y_true[i] - a[1]])
            t = np.clip(np.dot(ap, ab) / (np.dot(ab, ab) + 1e-10), 0, 1)
            proj = a + t * ab
            d = np.hypot(x_true[i] - proj[0], y_true[i] - proj[1])
            min_d = min(min_d, d)
        cross_track[i] = min_d

    fig, ax = plt.subplots(figsize=(10, 4))
    t = np.arange(n) * DT
    ax.plot(t, cross_track, "b-", label="Cross-track error (cm)")
    ax.axhline(WAYPOINT_RADIUS, color="gray", linestyle="--", label="Waypoint radius")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance from path (cm)")
    ax.set_title("Cross-Track Error vs Time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"Saved: {save_path}")
    plt.close(fig)


# ======================= Main =======================

def main():
    print("Pure Pursuit Trajectory Simulation with Noise")
    print("=" * 50)
    print(f"Mop width: {MOP_WIDTH} cm")
    print(f"Lane spacing: {LANE_WIDTH} cm  (overlap: {MOP_WIDTH - LANE_WIDTH + LANE_WIDTH} cm)")
    print(f"Total waypoints: {len(PATH)}  ({len(PATH)//2} lanes)")
    print(f"Position noise std: {POSITION_NOISE_STD} cm")
    print(f"Heading noise std: {np.degrees(HEADING_NOISE_STD):.2f} deg")
    print(f"dt: {DT} s")
    print()

    print("Generated lane x-positions:")
    for i in range(0, len(PATH) - 1, 2):
        y_range = f"y: {PATH[i,1]:.0f} -> {PATH[i+1,1]:.0f}"
        print(f"  Lane {i//2+1}: x={PATH[i,0]:.0f}  {y_range}")
    if len(PATH) % 2 == 1:
        print(f"  Final WP{len(PATH)-1}: ({PATH[-1,0]:.0f}, {PATH[-1,1]:.0f})")
    print()

    x_true, y_true, x_meas, y_meas = simulate(seed=42)

    print(f"Simulation steps: {len(x_true)}")
    print(f"Sim time: {len(x_true)*DT:.1f} s ({len(x_true)*DT/60:.1f} min)")
    print(f"Final position (true): ({x_true[-1]:.1f}, {y_true[-1]:.1f})")
    print(f"Target end: ({PATH[-1, 0]}, {PATH[-1, 1]})")
    dist_to_goal = np.hypot(x_true[-1] - PATH[-1, 0], y_true[-1] - PATH[-1, 1])
    print(f"Distance to goal: {dist_to_goal:.1f} cm")
    print()

    plot_measured_vs_actual(x_true, y_true, x_meas, y_meas, "measured_vs_actual_v10.png")


if __name__ == "__main__":
    main()
