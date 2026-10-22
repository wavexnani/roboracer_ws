import math
from typing import Tuple
import numpy as np


def compute_centerline_geometry(
    waypoints: np.ndarray,
    s_arr: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    dx = np.gradient(waypoints[:, 0])
    dy = np.gradient(waypoints[:, 1])
    headings = np.arctan2(dy, dx)

    dpsi = (np.gradient(headings) + math.pi) % (2.0 * math.pi) - math.pi
    ds = np.gradient(s_arr)
    ds = np.where(ds < 1e-4, 1e-4, ds)
    kappa = dpsi / ds
    return headings, kappa


def generate_speed_profile(
    waypoints: np.ndarray,
    kappa: np.ndarray,
    lat_accel_max: float,
    min_corner_speed: float,
    max_straight_speed: float,
    a_brake_max: float,
    a_accel_max: float
) -> np.ndarray:
    """Computes curvature lookahead speed profile and pre-braking passes."""
    k_abs = np.abs(kappa)
    N_pts = len(waypoints)
    window_pts = min(20, max(5, N_pts // 20))
    k_lookahead = np.zeros(N_pts)
    for i in range(N_pts):
        k_lookahead[i] = np.max([k_abs[(i + w) % N_pts] for w in range(window_pts)])

    eff_lat_accel = np.clip(lat_accel_max - 1.0 * np.maximum(0.0, k_lookahead - 0.25), 1.6, lat_accel_max)
    v_corner = np.where(k_lookahead > 0.04, np.sqrt(eff_lat_accel / np.maximum(k_lookahead, 1e-4)), max_straight_speed)
    v_corner = np.clip(v_corner, min_corner_speed, max_straight_speed)
    target_speeds = np.where(k_lookahead < 0.04, max_straight_speed, v_corner)

    diffs = np.diff(waypoints, axis=0)
    dists = np.hypot(diffs[:, 0], diffs[:, 1])
    for _ in range(4):
        # Backward pass
        for i in range(N_pts - 1, -1, -1):
            i_next = (i + 1) % N_pts
            ds_step = dists[i] if i < len(dists) else dists[-1]
            v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
            if target_speeds[i] > v_max_brake:
                target_speeds[i] = v_max_brake

        # Forward pass
        for i in range(N_pts):
            i_prev = (i - 1) % N_pts
            ds_step = dists[i_prev] if i_prev < len(dists) else dists[0]
            v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
            if target_speeds[i] > v_max_accel:
                target_speeds[i] = v_max_accel

    return target_speeds
