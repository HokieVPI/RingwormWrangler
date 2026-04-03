"""
Estimate UWB position measurement noise from PuTTY logs.

These logs are 6-line repeating blocks printed by `RW_Tag_PF_v2.ino`:
  1) goal.gx
  2) goal.gy
  3) DesiredHeading (deg)
  4) azimuth_deg (deg)
  5) currentX_global
  6) currentY_global

We use nights 11..18 and estimate a Gaussian position noise sigma (cm) to
feed into simulations (e.g. POSITION_NOISE_STD).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class NoiseStats:
    n: int
    sigma_x_hp: float
    sigma_y_hp: float
    sigma_r_hp: float
    sigma_x_stationary: float
    sigma_y_stationary: float
    sigma_r_stationary: float


def _robust_sigma(x: np.ndarray) -> float:
    """Robust sigma via MAD scaled for Gaussian."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 5:
        return float("nan")
    med = np.median(x)
    mad = np.median(np.abs(x - med))
    return 1.4826 * mad


def parse_putty_log_xy(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vals: list[float] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            try:
                vals.append(float(s))
            except ValueError:
                continue

    if len(vals) < 6:
        return np.array([]), np.array([])

    m = len(vals) // 6
    vals = vals[: m * 6]
    a = np.asarray(vals, dtype=float).reshape(m, 6)
    x = a[:, 4]
    y = a[:, 5]
    return x, y


def moving_average_reflect(x: np.ndarray, window: int) -> np.ndarray:
    """Centered moving average with reflect padding (odd window preferred)."""
    x = np.asarray(x, dtype=float)
    if window <= 1 or x.size == 0:
        return x.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    xp = np.pad(x, (pad, pad), mode="reflect")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(xp, kernel, mode="valid")


def estimate_noise_from_xy(
    x: np.ndarray,
    y: np.ndarray,
    *,
    smooth_window: int = 61,
    stationary_step_thresh_cm: float = 5.0,
) -> NoiseStats:
    if x.size != y.size:
        n = int(min(x.size, y.size))
        x = x[:n]
        y = y[:n]

    n = int(x.size)
    if n < 10:
        return NoiseStats(
            n=n,
            sigma_x_hp=float("nan"),
            sigma_y_hp=float("nan"),
            sigma_r_hp=float("nan"),
            sigma_x_stationary=float("nan"),
            sigma_y_stationary=float("nan"),
            sigma_r_stationary=float("nan"),
        )

    xs = moving_average_reflect(x, smooth_window)
    ys = moving_average_reflect(y, smooth_window)

    rx = x - xs
    ry = y - ys
    rr = np.hypot(rx, ry)

    sigma_x_hp = _robust_sigma(rx)
    sigma_y_hp = _robust_sigma(ry)
    sigma_r_hp = _robust_sigma(rr)

    # "Stationary-ish" points: based on step distance between successive *smoothed* samples.
    dxs = np.diff(xs)
    dys = np.diff(ys)
    step = np.hypot(dxs, dys)
    stationary_mask = step < stationary_step_thresh_cm

    if stationary_mask.sum() < 10:
        sigma_x_stationary = float("nan")
        sigma_y_stationary = float("nan")
        sigma_r_stationary = float("nan")
    else:
        # align residuals to step mask (mask is for transitions i->i+1, so use residual at i+1)
        rx_s = rx[1:][stationary_mask]
        ry_s = ry[1:][stationary_mask]
        rr_s = np.hypot(rx_s, ry_s)
        sigma_x_stationary = _robust_sigma(rx_s)
        sigma_y_stationary = _robust_sigma(ry_s)
        sigma_r_stationary = _robust_sigma(rr_s)

    return NoiseStats(
        n=n,
        sigma_x_hp=float(sigma_x_hp),
        sigma_y_hp=float(sigma_y_hp),
        sigma_r_hp=float(sigma_r_hp),
        sigma_x_stationary=float(sigma_x_stationary),
        sigma_y_stationary=float(sigma_y_stationary),
        sigma_r_stationary=float(sigma_r_stationary),
    )


def iter_night_paths(test_data_dir: Path, nights: Iterable[int]) -> list[Path]:
    paths: list[Path] = []
    for n in nights:
        p = test_data_dir / f"test_3_31_night{n}"
        if p.exists():
            paths.append(p)
    return paths


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    test_data_dir = repo_root / "PurePrusuit" / "Test data"
    nights = list(range(11, 19))
    paths = iter_night_paths(test_data_dir, nights)

    if not paths:
        print(f"No night logs found in: {test_data_dir}")
        return 2

    all_x: list[np.ndarray] = []
    all_y: list[np.ndarray] = []

    print("Per-night noise estimates (cm):")
    for p in paths:
        x, y = parse_putty_log_xy(p)
        st = estimate_noise_from_xy(x, y)
        all_x.append(x)
        all_y.append(y)
        print(
            f"  {p.name:>16s} | n={st.n:5d} | "
            f"hp sigma_x={st.sigma_x_hp:5.1f} sigma_y={st.sigma_y_hp:5.1f} sigma_r={st.sigma_r_hp:5.1f} | "
            f"stationary sigma_r={st.sigma_r_stationary:5.1f}"
        )

    X = np.concatenate(all_x) if all_x else np.array([])
    Y = np.concatenate(all_y) if all_y else np.array([])
    st_all = estimate_noise_from_xy(X, Y)

    # Recommended single scalar sigma for isotropic Gaussian simulation noise.
    # These logs are from a moving robot; the "stationary" heuristic can be
    # contaminated by slow motion, so default to the high-pass residual.
    sigma_r = st_all.sigma_r_hp

    # Convert radial robust sigma into per-axis sigma assuming isotropic Gaussian:
    # For x,y ~ N(0, σ^2), r = sqrt(x^2+y^2) is Rayleigh with std ~= 0.655 * σ.
    # So σ ≈ std(r) / 0.655. (Using robust sigma on r, still a useful approximation.)
    sigma_xy = float(sigma_r / 0.655) if np.isfinite(sigma_r) else float("nan")

    print("\nCombined (nights 11..18):")
    print(
        f"  n={st_all.n} | hp sigma_x={st_all.sigma_x_hp:.2f} sigma_y={st_all.sigma_y_hp:.2f} sigma_r={st_all.sigma_r_hp:.2f} | "
        f"stationary sigma_r={st_all.sigma_r_stationary:.2f}"
    )
    print("\nRecommended simulation noise (cm):")
    print(f"  POSITION_NOISE_STD ~ {sigma_xy:.2f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

