import math
import numpy as np
from typing import Tuple, Optional


def find_reference_waypoint(
    ref_x: float,
    ref_y: float,
    current_yaw: float,
    waypoints: np.ndarray,
    path_headings: np.ndarray,
    last_idx: int,
    num_waypoints: int
) -> int:
    """Find closest waypoint index using windowed forward search with heading penalty."""
    search_indices = (np.arange(-20, 100) + last_idx) % num_waypoints
    cand_pts = waypoints[search_indices]

    dx = cand_pts[:, 0] - ref_x
    dy = cand_pts[:, 1] - ref_y
    dist_sq = dx * dx + dy * dy

    dpsi = (path_headings[search_indices] - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
    cost = dist_sq + 4.0 * (dpsi ** 2)

    min_local = int(np.argmin(cost))
    idx = int(search_indices[min_local])

    if math.sqrt(dist_sq[min_local]) > 3.0:
        all_dx = waypoints[:, 0] - ref_x
        all_dy = waypoints[:, 1] - ref_y
        all_dist_sq = all_dx * all_dx + all_dy * all_dy
        all_dpsi = (path_headings - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
        valid_mask = np.cos(all_dpsi) > 0.0
        if np.any(valid_mask):
            all_cost = np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)
            idx = int(np.argmin(all_cost))
        else:
            idx = int(np.argmin(all_dist_sq))

    return idx


def compute_stanley_steering(
    ref_x: float,
    ref_y: float,
    current_yaw: float,
    current_vel: float,
    wpt_x: float,
    wpt_y: float,
    path_heading: float,
    kappa_val: Optional[float],
    gain_k: float,
    software_k: float,
    gain_ff: float,
    wheelbase: float,
    max_steer: float
) -> Tuple[float, float, float]:
    """Compute Stanley cross-track steering angle with curvature feedforward."""
    tx = math.cos(path_heading)
    ty = math.sin(path_heading)
    ex = ref_x - wpt_x
    ey = ref_y - wpt_y

    lateral_error = tx * ey - ty * ex
    cross_track_error = -lateral_error

    heading_error = (path_heading - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
    cross_track_steering = math.atan2(gain_k * cross_track_error, current_vel + software_k)
    curvature_ff = gain_ff * math.atan(wheelbase * float(kappa_val)) if kappa_val is not None else 0.0

    raw_steering = heading_error + cross_track_steering + curvature_ff
    steering_angle = float(np.clip(raw_steering, -max_steer, max_steer))

    return steering_angle, cross_track_error, heading_error


def compute_corner_speed_limit(
    s_cur: float,
    speed: float,
    s_arr: Optional[np.ndarray],
    kappa: Optional[np.ndarray],
    track_length: float,
    brake_decel: float,
    min_lookahead_horizon: float,
    lat_accel_max: float
) -> float:
    """Calculate maximum safe cornering speed within upcoming braking horizon."""
    if s_arr is None or kappa is None or brake_decel <= 0.0:
        return float('inf')

    horizon = (speed * speed) / (2.0 * brake_decel) + min_lookahead_horizon
    s_rel = (s_arr - s_cur) % track_length
    mask = s_rel <= horizon

    if not np.any(mask):
        return float('inf')

    k_max = float(np.max(np.abs(kappa[mask])))
    if k_max < 1e-4:
        return float('inf')

    return math.sqrt(lat_accel_max / k_max)
