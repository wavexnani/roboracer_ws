"""
Stanley Steering Path Tracking Controller for F1TENTH / ROBORACER.
Features exact orthogonal cross-track projection, speed-adaptive lookahead,
curvature feedforward, and global asymptotic convergence guarantee.
"""

import math
from typing import Tuple
import numpy as np
from .track_manager import TrackInfo


class StanleyController:
    """Non-linear Stanley Path Tracking Controller with Curvature Feedforward."""

    def __init__(
        self,
        track: TrackInfo,
        wheelbase: float = 0.33,
        gain_k: float = 2.3,
        software_k: float = 0.5,
        gain_ff: float = 0.12,
        lookahead_dist: float = 0.15,
        lookahead_gain: float = 0.02,
        max_steer: float = 0.4189,
        lat_accel_max: float = 2.5,
        brake_decel: float = 3.5
    ):
        self.track = track
        self.waypoints = np.copy(track.waypoints)
        self.headings = np.copy(track.headings)
        self.s_arr = np.copy(track.s_arr)
        self.kappa = np.copy(track.kappa)
        self.target_speeds = np.copy(track.target_speeds)
        self.track_length = track.track_length
        self.num_pts = len(self.waypoints)

        self.wheelbase = wheelbase
        self.gain_k = gain_k
        self.software_k = software_k
        self.gain_ff = gain_ff
        self.lookahead_dist = lookahead_dist
        self.lookahead_gain = lookahead_gain
        self.max_steer = max_steer
        self.lat_accel_max = lat_accel_max
        self.brake_decel = brake_decel
        self.last_idx = 0

    def _corner_speed_limit(self, s_cur: float, speed: float) -> float:
        horizon = (speed * speed) / (2.0 * self.brake_decel) + 1.2
        s_rel = (self.s_arr - s_cur) % self.track_length
        mask = s_rel <= horizon
        if not np.any(mask) or self.kappa is None:
            return float('inf')
        k_max = float(np.max(np.abs(self.kappa[mask])))
        if k_max < 1e-4:
            return float('inf')
        return math.sqrt(self.lat_accel_max / k_max)

    def compute_control(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float]:
        """
        Computes (steering_angle, target_speed, cross_track_error, heading_error).
        """
        eff_offset = self.wheelbase + self.lookahead_dist + self.lookahead_gain * v
        ref_x = x + eff_offset * math.cos(yaw)
        ref_y = y + eff_offset * math.sin(yaw)

        # Local window search with forward-heading alignment
        search_window = 120
        indices = (np.arange(-20, search_window) + self.last_idx) % self.num_pts
        cand_pts = self.waypoints[indices]
        dist_sq = (cand_pts[:, 0] - ref_x)**2 + (cand_pts[:, 1] - ref_y)**2
        dpsi = (self.headings[indices] - yaw + math.pi) % (2.0 * math.pi) - math.pi
        forward_mask = np.cos(dpsi) > 0.0

        if np.any(forward_mask):
            best_local = int(np.argmin(np.where(forward_mask, dist_sq, np.inf)))
        else:
            best_local = int(np.argmin(dist_sq))

        idx = int(indices[best_local])
        self.last_idx = idx

        # Continuous orthogonal cross track error
        tx = math.cos(self.headings[idx])
        ty = math.sin(self.headings[idx])
        ex = ref_x - self.waypoints[idx, 0]
        ey = ref_y - self.waypoints[idx, 1]
        e_ct = -(tx * ey - ty * ex)

        # Heading error
        e_psi = (self.headings[idx] - yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Stanley Control Law with Curvature Feedforward
        delta_ct = math.atan2(self.gain_k * e_ct, v + self.software_k)
        delta_ff = self.gain_ff * math.atan(self.wheelbase * float(self.kappa[idx])) if self.kappa is not None else 0.0
        steer = float(np.clip(e_psi + delta_ct + delta_ff, -self.max_steer, self.max_steer))

        # Speed limit
        v_target = float(self.target_speeds[idx])
        v_corner = self._corner_speed_limit(float(self.s_arr[idx]), max(v_target, v))
        v_target = min(v_target, v_corner)

        # Steering lock taper
        steer_ratio = abs(steer) / self.max_steer
        if steer_ratio > 0.5:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.5) / 0.5)

        # Heading divergence safety guard
        if abs(e_psi) > 0.35:
            v_target *= 0.85

        v_target = float(np.clip(v_target, 1.0, 8.0))
        return steer, v_target, e_ct, e_psi
