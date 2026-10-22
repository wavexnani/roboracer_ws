"""Pure Pursuit Path-Tracking Controller Adapter."""

import math
from typing import Tuple, Optional
import numpy as np

from .base_controller import BaseBenchmarkController, compute_cross_track_error, wrap_angle


class PurePursuitAdapter(BaseBenchmarkController):
    """
    Pure Pursuit Geometric Path-Tracking Controller.
    Computes steering angle based on a lookahead goal point on the raceline.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        headings: np.ndarray,
        target_speeds: np.ndarray,
        lookahead_distance: float = 1.60,
        lookahead_gain: float = 0.12,
        min_lookahead: float = 0.80,
        max_lookahead: float = 3.20,
        speed_scale: float = 0.70,
        max_speed: float = 7.00
    ):
        super().__init__("Pure_Pursuit")
        self.waypoints = waypoints
        self.headings = headings
        self.target_speeds = target_speeds * speed_scale
        self.num_pts = len(waypoints)
        self.last_idx = 0

        self.lookahead_distance = lookahead_distance
        self.lookahead_gain = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.max_speed = max_speed

    def reset(self):
        self.last_idx = 0

    def compute_control(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        dt: float,
        scans: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        # Effective lookahead distance (speed adaptive or fixed)
        L_d = self.lookahead_distance + self.lookahead_gain * v
        L_d = float(np.clip(L_d, self.min_lookahead, self.max_lookahead))

        # Find closest point
        e_ct, e_psi, closest_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
        self.last_idx = closest_idx

        # Search forward along the raceline for goal point at distance >= L_d
        goal_idx = closest_idx
        for offset in range(1, min(150, self.num_pts)):
            idx_cand = (closest_idx + offset) % self.num_pts
            dist = math.hypot(self.waypoints[idx_cand, 0] - x, self.waypoints[idx_cand, 1] - y)
            if dist >= L_d:
                goal_idx = idx_cand
                break

        goal_x = self.waypoints[goal_idx, 0]
        goal_y = self.waypoints[goal_idx, 1]

        # Transform goal point to vehicle frame
        dx = goal_x - x
        dy = goal_y - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Pure pursuit curvature: kappa = 2 * local_y / L_d^2
        dist_sq = local_x ** 2 + local_y ** 2
        if dist_sq < 1e-4:
            steer = 0.0
        else:
            steer = math.atan2(2.0 * self.wheelbase * local_y, dist_sq)

        steer_cmd = float(np.clip(steer, -self.max_steer, self.max_steer))

        # Speed profiling: target speed with cornering slowdown
        v_target = float(self.target_speeds[goal_idx])
        if abs(local_y) > 0.30 * L_d:
            v_target *= 0.75

        speed_cmd = float(np.clip(v_target, 1.20, self.max_speed))

        return steer_cmd, speed_cmd, e_ct, e_psi
