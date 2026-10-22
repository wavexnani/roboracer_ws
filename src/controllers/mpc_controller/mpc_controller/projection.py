import math
import numpy as np
from typing import Tuple


def find_closest_waypoint(
    cur_x: float,
    cur_y: float,
    cur_yaw: float,
    waypoints: np.ndarray,
    headings: np.ndarray,
    last_idx: int,
    num_waypoints: int,
    pos_jump: bool = False
) -> Tuple[int, float]:
    """Find the closest waypoint using a forward-biased local search with global fallback."""
    search_offsets = np.arange(-15, 80)
    search_indices = (search_offsets + last_idx) % num_waypoints
    cand_pts = waypoints[search_indices]

    dx = cand_pts[:, 0] - cur_x
    dy = cand_pts[:, 1] - cur_y
    dist_sq = dx * dx + dy * dy

    dpsi = (headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
    forward_mask = np.cos(dpsi) > 0.17

    index_bias = np.where(search_offsets < 0, 0.4 * (-search_offsets), 0.0)
    cost = dist_sq + 2.0 * (dpsi ** 2) + index_bias

    if np.any(forward_mask):
        best_local = int(np.argmin(np.where(forward_mask, cost, np.inf)))
    else:
        best_local = int(np.argmin(cost))

    closest_idx = int(search_indices[best_local])
    min_dist = math.sqrt(dist_sq[best_local])

    if min_dist > 3.0 or pos_jump:
        all_dx = waypoints[:, 0] - cur_x
        all_dy = waypoints[:, 1] - cur_y
        all_dist_sq = all_dx * all_dx + all_dy * all_dy
        all_dpsi = (headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        valid_mask = np.cos(all_dpsi) > 0.0
        if np.any(valid_mask):
            closest_idx = int(np.argmin(np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)))
        else:
            closest_idx = int(np.argmin(all_dist_sq))
        min_dist = math.sqrt(all_dist_sq[closest_idx])

    return closest_idx, min_dist


def project_to_continuous_arc(
    cur_x: float,
    cur_y: float,
    closest_idx: int,
    waypoints: np.ndarray,
    s_arr: np.ndarray,
    track_length: float,
    num_waypoints: int
) -> float:
    """Project position onto neighboring segments [-1, 0, 1] to obtain continuous arc length."""
    p = np.array([cur_x, cur_y], dtype=np.float64)
    best_dist_sq = float('inf')
    best_s_cont = s_arr[closest_idx]
    best_t = 0.0

    for offset in (-1, 0, 1):
        ia = (closest_idx + offset) % num_waypoints
        ib = (ia + 1) % num_waypoints
        pa = waypoints[ia]
        pb = waypoints[ib]
        v = pb - pa
        l2 = float(np.dot(v, v))
        t = 0.0 if l2 < 1e-12 else float(np.clip(np.dot(p - pa, v) / l2, 0.0, 1.0))
        p_proj = pa + t * v
        dist_sq = float(np.dot(p - p_proj, p - p_proj))

        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_t = t
            seg_len = (track_length - s_arr[ia]) if ia == num_waypoints - 1 else (s_arr[ib] - s_arr[ia])
            best_s_cont = (s_arr[ia] + t * seg_len) % track_length

    assert np.isfinite(best_s_cont), f"s_cont is not finite: {best_s_cont}"
    assert 0.0 <= best_s_cont < track_length, f"s_cont={best_s_cont} outside [0, {track_length})"
    assert 0.0 <= best_t <= 1.0, f"Projection t={best_t} outside [0, 1]"

    return best_s_cont


def extract_reference_horizon(
    s_cont: float,
    cur_v: float,
    horizon_n: int,
    dt: float,
    track_length: float,
    waypoints: np.ndarray,
    headings: np.ndarray,
    s_arr: np.ndarray,
    scaled_target_speeds: np.ndarray,
    num_waypoints: int
) -> np.ndarray:
    """Sample N+1 reference states along the trajectory starting from s_cont."""
    ref_horizon = np.zeros((horizon_n + 1, 4))
    speed_est = max(1.5, cur_v)
    s_targets = np.zeros(horizon_n + 1)

    for k in range(horizon_n + 1):
        s_target = (s_cont + k * speed_est * dt) % track_length
        s_targets[k] = s_target

        idx_a = int(np.searchsorted(s_arr, s_target, side='right') - 1)
        idx_a = max(0, min(num_waypoints - 1, idx_a))
        idx_b = (idx_a + 1) % num_waypoints

        s_a = s_arr[idx_a]
        if idx_a == num_waypoints - 1:
            seg_len = track_length - s_a
            ds = s_target - s_a if s_target >= s_a else (s_target + track_length - s_a)
        else:
            s_b = s_arr[idx_b]
            seg_len = s_b - s_a
            ds = s_target - s_a

        t = float(np.clip(ds / max(seg_len, 1e-4), 0.0, 1.0))

        ref_horizon[k, 0] = (1.0 - t) * waypoints[idx_a, 0] + t * waypoints[idx_b, 0]
        ref_horizon[k, 1] = (1.0 - t) * waypoints[idx_a, 1] + t * waypoints[idx_b, 1]

        dpsi_seg = (headings[idx_b] - headings[idx_a] + math.pi) % (2.0 * math.pi) - math.pi
        ref_horizon[k, 2] = (headings[idx_a] + t * dpsi_seg + math.pi) % (2.0 * math.pi) - math.pi
        ref_horizon[k, 3] = (1.0 - t) * scaled_target_speeds[idx_a] + t * scaled_target_speeds[idx_b]

    assert abs(s_targets[0] - s_cont) < 1e-9, f"s_ref[0] mismatch: {s_targets[0]} vs {s_cont}"
    for k in range(horizon_n):
        ds_k = (s_targets[k + 1] - s_targets[k] + track_length) % track_length
        assert ds_k > 0.0, f"Non-monotonic horizon step {k}: ds={ds_k}"
    assert np.all(np.isfinite(ref_horizon)), "ref_horizon contains NaN or Inf"

    return ref_horizon
