"""
Pure Pursuit Path Tracking Controller for F1TENTH / ROBORACER.
Implements speed-adaptive lookahead geometric tracking.
"""

import math
from typing import Tuple
import numpy as np
from .track_manager import TrackInfo


class PurePursuitController:
    """Speed-adaptive geometric Pure Pursuit controller."""

    def __init__(
        self,
        track: TrackInfo,
        wheelbase: float = 0.33,
        lookahead_ratio: float = 0.25,
        min_lookahead: float = 0.8,
        max_lookahead: float = 2.5,
        max_steer: float = 0.4189
    ):
        self.track = track
        self.waypoints = np.copy(track.waypoints)
        self.headings = np.copy(track.headings)
        self.s_arr = np.copy(track.s_arr)
        self.target_speeds = np.copy(track.target_speeds)
        self.num_pts = len(self.waypoints)

        self.wheelbase = wheelbase
        self.lookahead_ratio = lookahead_ratio
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.max_steer = max_steer
        self.last_idx = 0

    def compute_control(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float]:
        """
        Computes (steering_angle, target_speed, cross_track_error, heading_error).
        """
        lookahead_dist = float(np.clip(self.lookahead_ratio * max(v, 1.0), self.min_lookahead, self.max_lookahead))

        # Closest waypoint to rear axle
        search_window = 120
        indices = (np.arange(-20, search_window) + self.last_idx) % self.num_pts
        dists_sq = (self.waypoints[indices, 0] - x)**2 + (self.waypoints[indices, 1] - y)**2
        closest_local = int(np.argmin(dists_sq))
        closest_idx = int(indices[closest_local])
        self.last_idx = closest_idx

        # Search forward for lookahead target point
        target_idx = closest_idx
        for step in range(search_window):
            idx_cand = (closest_idx + step) % self.num_pts
            d_cand = math.hypot(self.waypoints[idx_cand, 0] - x, self.waypoints[idx_cand, 1] - y)
            if d_cand >= lookahead_dist:
                target_idx = idx_cand
                break

        # Transform target point to vehicle frame
        dx = self.waypoints[target_idx, 0] - x
        dy = self.waypoints[target_idx, 1] - y

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        # Pure pursuit curvature: gamma = 2 * y / Ld^2
        dist_actual = math.hypot(local_x, local_y)
        if dist_actual < 1e-3:
            dist_actual = 1e-3

        steer = math.atan2(2.0 * self.wheelbase * local_y, dist_actual * dist_actual)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # Cross track error and heading error
        tx = math.cos(self.headings[closest_idx])
        ty = math.sin(self.headings[closest_idx])
        ex = x - self.waypoints[closest_idx, 0]
        ey = y - self.waypoints[closest_idx, 1]
        e_ct = -(tx * ey - ty * ex)
        e_psi = (self.headings[closest_idx] - yaw + math.pi) % (2.0 * math.pi) - math.pi

        speed = float(self.target_speeds[closest_idx])
        return steer, speed, e_ct, e_psi
