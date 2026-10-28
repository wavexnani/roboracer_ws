"""Wall Following (PID) Controller Adapter."""

import math
from typing import Tuple, Optional
import numpy as np

# pyrefly: ignore[missing-import]
from .base_controller import BaseBenchmarkController, compute_cross_track_error


class WallFollowAdapter(BaseBenchmarkController):
    """
    Reactive PID Left-Wall Following Controller.
    Computes wall orientation and projected lateral distance using two LiDAR beams.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        headings: np.ndarray,
        target_speeds: np.ndarray,
        desired_distance: float = 0.80,
        lookahead_distance: float = 0.80,
        kp: float = 1.50,
        kd: float = 0.20,
        ki: float = 0.0,
        base_speed: float = 3.50,
        min_speed: float = 1.50,
    ):
        super().__init__("Wall_Following")
        self.waypoints = waypoints
        self.headings = headings
        self.target_speeds = target_speeds
        self.last_idx = 0

        self.desired_distance = desired_distance
        self.lookahead_distance = lookahead_distance
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.base_speed = base_speed
        self.min_speed = min_speed

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0
        self.theta = math.radians(45.0)  # angle between beam a and beam b
        self.beam_b_angle = math.pi / 2.0  # 90 deg left
        self.beam_a_angle = math.pi / 2.0 - self.theta  # 45 deg forward-left

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0
        self.last_idx = 0

    def _get_range(self, ranges: np.ndarray, angle: float, angle_min: float, angle_inc: float) -> float:
        # pyrefly: ignore[unnecessary-type-conversion]
        idx = int(round((angle - angle_min) / angle_inc))
        idx = np.clip(idx, 0, len(ranges) - 1)
        r = ranges[idx]
        if not np.isfinite(r) or r <= 0.05:
            # Search adjacent window for valid range
            window = ranges[max(0, idx - 5): min(len(ranges), idx + 6)]
            valid = window[np.isfinite(window) & (window > 0.05)]
            return float(np.mean(valid)) if len(valid) > 0 else 4.0
        return float(r)

    def compute_control(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        dt: float,
        scans: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        if scans is None or len(scans) == 0:
            return 0.0, self.min_speed, 0.0, 0.0

        n_beams = len(scans)
        # F1TENTH typical scan: [-3*pi/4, 3*pi/4] across 1080 beams
        angle_min = -2.35619449
        angle_max = 2.35619449
        angle_inc = (angle_max - angle_min) / (n_beams - 1)

        b = self._get_range(scans, self.beam_b_angle, angle_min, angle_inc)
        a = self._get_range(scans, self.beam_a_angle, angle_min, angle_inc)

        # Official wall_follow formula
        alpha = math.atan2(a * math.cos(self.theta) - b, a * math.sin(self.theta))
        current_distance = b * math.cos(alpha)
        future_distance = current_distance + self.lookahead_distance * math.sin(alpha)
        error = future_distance - self.desired_distance

        # PID with smoothing
        self.integral = float(np.clip(self.integral + error * dt, -1.0, 1.0))
        derivative = (error - self.prev_error) / max(dt, 1e-4)
        self.prev_error = error

        raw_steer = self.kp * error + self.kd * derivative + self.ki * self.integral
        steer_cmd = float(np.clip(raw_steer, -self.max_steer, self.max_steer))

        # Exponential smoothing to eliminate chatter
        steer_cmd = 0.70 * steer_cmd + 0.30 * self.prev_steer
        self.prev_steer = steer_cmd

        # Lab speed schedule based on steering magnitude
        deg = abs(math.degrees(steer_cmd))
        if deg <= 10.0:
            speed_cmd = self.base_speed
        elif deg <= 20.0:
            speed_cmd = max(self.min_speed, self.base_speed * 0.70)
        else:
            speed_cmd = self.min_speed

        speed_cmd = float(np.clip(speed_cmd, self.min_speed, self.base_speed))

        # Ground-truth cross-track error relative to raceline
        e_ct, e_psi, closest_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
        self.last_idx = closest_idx

        return steer_cmd, speed_cmd, e_ct, e_psi
