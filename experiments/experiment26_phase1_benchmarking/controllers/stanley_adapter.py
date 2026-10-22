"""Stanley Path-Tracking Controller Adapter."""

import math
from typing import Tuple, Optional
import numpy as np

from .base_controller import BaseBenchmarkController, compute_cross_track_error, wrap_angle


class StanleyAdapter(BaseBenchmarkController):
    """
    Stanley Path-Tracking Controller with Front-Axle Reference and Curvature Feedforward.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        headings: np.ndarray,
        target_speeds: np.ndarray,
        kappa: Optional[np.ndarray] = None,
        gain_k: float = 2.20,
        software_k: float = 0.50,
        gain_ff: float = 0.10,
        lookahead_preview: float = 0.10,
        speed_scale: float = 0.70,
        max_speed: float = 7.00
    ):
        super().__init__("Stanley_Controller")
        self.waypoints = waypoints
        self.headings = headings
        self.target_speeds = target_speeds * speed_scale
        self.kappa = kappa
        self.num_pts = len(waypoints)
        self.last_idx = 0

        self.gain_k = gain_k
        self.software_k = software_k
        self.gain_ff = gain_ff
        self.lookahead_preview = lookahead_preview
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
        # Reference point at front axle plus preview offset
        ref_offset = self.wheelbase + self.lookahead_preview
        ref_x = x + ref_offset * math.cos(yaw)
        ref_y = y + ref_offset * math.sin(yaw)

        # Ground-truth cross track error from rear/center
        e_ct, e_psi, closest_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
        self.last_idx = closest_idx

        # Local error at front axle reference point
        tx = math.cos(self.headings[closest_idx])
        ty = math.sin(self.headings[closest_idx])
        ex_ref = ref_x - self.waypoints[closest_idx, 0]
        ey_ref = ref_y - self.waypoints[closest_idx, 1]
        front_e_ct = -(tx * ey_ref - ty * ex_ref)

        # Stanley steering law: delta = e_psi + arctan(k * e_ct / (v + k_soft)) + delta_ff
        delta_ct = math.atan2(self.gain_k * front_e_ct, v + self.software_k)

        delta_ff = 0.0
        if self.kappa is not None and self.gain_ff > 0.0:
            k_val = float(self.kappa[closest_idx])
            delta_ff = self.gain_ff * math.atan(self.wheelbase * k_val)

        raw_steer = e_psi + delta_ct + delta_ff
        steer_cmd = float(np.clip(raw_steer, -self.max_steer, self.max_steer))

        # Target speed
        v_target = float(self.target_speeds[closest_idx])
        steer_ratio = abs(steer_cmd) / self.max_steer
        if steer_ratio > 0.50:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.50) / 0.50)

        speed_cmd = float(np.clip(v_target, 1.20, self.max_speed))

        return steer_cmd, speed_cmd, e_ct, e_psi
