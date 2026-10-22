import math
import numpy as np
from typing import Optional
from sensor_msgs.msg import LaserScan

try:
    from scipy.ndimage import gaussian_filter1d
    HAS_GAUSSIAN = True
except ImportError:
    HAS_GAUSSIAN = False

    def gaussian_filter1d(input, *args, **kwargs):
        return input


def profile_track_speeds(
    waypoints: np.ndarray,
    s_arr: np.ndarray,
    kappa: np.ndarray,
    dists: np.ndarray,
    lat_accel_max: float,
    min_corner_speed: float,
    max_straight_speed: float,
    a_brake_max: float,
    a_accel_max: float
) -> np.ndarray:
    """Compute corner-limited and acceleration-bounded speed profile across track waypoints."""
    k_abs = np.abs(kappa)
    n_pts = len(waypoints)
    window_pts = min(20, max(5, n_pts // 20))
    k_lookahead = np.zeros(n_pts)
    for i in range(n_pts):
        k_lookahead[i] = np.max([k_abs[(i + w) % n_pts] for w in range(window_pts)])

    eff_lat_accel = np.clip(lat_accel_max - 1.0 * np.maximum(0.0, k_lookahead - 0.25), 1.6, lat_accel_max)
    v_corner = np.where(k_lookahead > 0.04, np.sqrt(eff_lat_accel / np.maximum(k_lookahead, 1e-4)), max_straight_speed)
    v_corner = np.clip(v_corner, min_corner_speed, max_straight_speed)
    target_speeds = np.where(k_lookahead < 0.04, max_straight_speed, v_corner)

    closure_step = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0], waypoints[0, 1] - waypoints[-1, 1]))
    if closure_step < 1e-4:
        closure_step = dists[0] if len(dists) > 0 else 0.2

    for _ in range(6):
        for i in range(n_pts - 1, -1, -1):
            i_next = (i + 1) % n_pts
            ds_step = dists[i] if i < len(dists) else closure_step
            v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
            if target_speeds[i] > v_max_brake:
                target_speeds[i] = v_max_brake

        for i in range(n_pts):
            i_prev = (i - 1) % n_pts
            ds_step = dists[i_prev] if i_prev < len(dists) else closure_step
            v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
            if target_speeds[i] > v_max_accel:
                target_speeds[i] = v_max_accel

    if HAS_GAUSSIAN:
        target_speeds = gaussian_filter1d(target_speeds, sigma=3.0, mode='wrap')

    return target_speeds


def evaluate_lidar_corridor(
    scan: Optional[LaserScan],
    base_target_v: float,
    max_decel: float,
    safety_margin_dist: float,
    min_corner_speed: float
) -> float:
    """Adjust target velocity based on forward obstacle corridor in LiDAR scan."""
    if scan is None:
        return base_target_v

    ranges = np.array(scan.ranges)
    n_beams = len(ranges)
    angle_min = scan.angle_min
    angle_inc = scan.angle_increment if scan.angle_increment > 0 else (4.7 / max(n_beams, 1))
    angles = angle_min + np.arange(n_beams) * angle_inc

    valid = np.isfinite(ranges) & (ranges >= max(0.10, scan.range_min)) & (ranges <= min(25.0, scan.range_max))
    if not np.any(valid):
        return base_target_v

    r_val = ranges[valid]
    th_val = angles[valid]
    x_body = r_val * np.cos(th_val)
    y_body = r_val * np.sin(th_val)

    v_target = base_target_v
    corridor = (x_body > 0.35) & (x_body < 10.0) & (np.abs(y_body) <= 0.28)
    if np.any(corridor):
        d_obs = float(np.min(x_body[corridor]))
        v_obs = math.sqrt(2.0 * max_decel * max(0.0, d_obs - safety_margin_dist))
        v_target = min(v_target, max(min_corner_speed, v_obs))

    min_scan_all = float(np.min(ranges[valid]))
    if min_scan_all < 0.35:
        v_target *= max(0.70, min_scan_all / 0.35)

    return v_target


def compute_target_velocity(
    base_speed: float,
    steer_cmd: float,
    max_steer: float,
    heading_err: float,
    last_speed_cmd: float,
    dt_actual: float,
    max_accel: float,
    max_decel: float,
    max_straight_speed: float,
    min_corner_speed: float,
    scan: Optional[LaserScan] = None,
    enable_adaptive_speed: bool = True,
    safety_margin_dist: float = 0.50
) -> float:
    """Compute rate-limited, corridor-safe longitudinal velocity command."""
    v_target = base_speed

    if enable_adaptive_speed and scan is not None:
        v_target = evaluate_lidar_corridor(
            scan, v_target, max_decel, safety_margin_dist, min_corner_speed
        )

    steer_ratio = abs(steer_cmd) / max(max_steer, 1e-3)
    if steer_ratio > 0.45:
        v_target *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)

    if abs(heading_err) > 0.35:
        v_target *= 0.85

    v_cmd_max = last_speed_cmd + max_accel * dt_actual
    v_cmd_min = last_speed_cmd - max_decel * dt_actual
    if last_speed_cmd < 0.2 and v_target > 0.5:
        v_cmd_max = max(v_cmd_max, 0.8)

    return float(np.clip(v_target, max(0.0, v_cmd_min), min(max_straight_speed, v_cmd_max)))
