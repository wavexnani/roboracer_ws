"""Follow-the-Gap (FTG) / Disparity Extender Controller Adapter."""

import math
from typing import Tuple, Optional, List
import numpy as np

from .base_controller import BaseBenchmarkController, compute_cross_track_error


class FollowTheGapAdapter(BaseBenchmarkController):
    """
    Reactive Follow-the-Gap Controller with Disparity Extension and Bubble Elimination.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        headings: np.ndarray,
        target_speeds: np.ndarray,
        safety_margin: float = 0.22,
        bubble_radius: float = 0.35,
        disparity_threshold: float = 0.30,
        max_range: float = 4.50,
        best_point_fraction: float = 0.90,
        fov_deg: float = 170.0,
        max_speed: float = 4.20,
        min_speed: float = 1.30,
        car_width: float = 0.30,
        smoothing_window: int = 5
    ):
        super().__init__("Follow_The_Gap")
        self.waypoints = waypoints
        self.headings = headings
        self.target_speeds = target_speeds
        self.last_idx = 0

        self.safety_margin = safety_margin
        self.bubble_radius = bubble_radius
        self.disparity_threshold = disparity_threshold
        self.max_range = max_range
        self.best_point_fraction = best_point_fraction
        self.fov_deg = fov_deg
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.car_width = car_width
        self.smoothing_window = smoothing_window

    def reset(self):
        self.last_idx = 0

    def preprocess_lidar(self, ranges: np.ndarray) -> np.ndarray:
        proc = np.copy(ranges)
        # 1. Clean invalid
        proc[~np.isfinite(proc) | (proc < 0.0)] = 0.0
        proc[proc > self.max_range] = self.max_range

        # 2. Moving average smoothing
        if self.smoothing_window > 1:
            pad = self.smoothing_window // 2
            padded = np.pad(proc, pad, mode='edge')
            kernel = np.ones(self.smoothing_window) / self.smoothing_window
            smoothed = np.convolve(padded, kernel, mode='valid')
            proc = smoothed[:len(proc)]

        return proc

    def extend_disparities(self, ranges: np.ndarray, angle_inc: float) -> np.ndarray:
        out = np.copy(ranges)
        n = len(ranges)
        half_width = (self.car_width / 2.0) + self.safety_margin

        for i in range(n - 1):
            a = ranges[i]
            b = ranges[i + 1]
            diff = b - a
            if abs(diff) < self.disparity_threshold:
                continue

            near_val = min(a, b)
            if near_val < 0.05:
                continue

            extend_idx = int(math.atan2(half_width, near_val) / angle_inc)
            if extend_idx < 1:
                continue

            if diff > 0:
                end = min(n - 1, i + extend_idx)
                out[i + 1:end + 1] = np.minimum(out[i + 1:end + 1], near_val)
            else:
                start = max(0, i - extend_idx + 1)
                out[start:i + 1] = np.minimum(out[start:i + 1], near_val)

        return out

    def eliminate_bubble(self, ranges: np.ndarray, closest_i: int, angle_inc: float) -> None:
        closest_dist = max(ranges[closest_i], 0.05)
        half_width = (self.car_width / 2.0) + self.bubble_radius
        bubble_angle = math.atan2(half_width, closest_dist)
        bubble_idx = int(bubble_angle / angle_inc)

        n = len(ranges)
        start = max(0, closest_i - bubble_idx)
        end = min(n - 1, closest_i + bubble_idx)
        ranges[start:end + 1] = 0.0

    def find_max_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        n = len(ranges)
        best_start = 0
        best_len = 0
        cur_start = -1

        for i in range(n):
            if ranges[i] > 0.10:
                if cur_start == -1:
                    cur_start = i
                cur_len = i - cur_start + 1
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
            else:
                cur_start = -1

        best_end = best_start + best_len - 1
        return best_start, best_end

    def find_best_point(self, ranges: np.ndarray, start_i: int, end_i: int) -> int:
        if start_i > end_i or start_i < 0:
            return len(ranges) // 2

        gap_slice = ranges[start_i:end_i + 1]
        if len(gap_slice) == 0:
            return len(ranges) // 2

        max_val = np.max(gap_slice)
        thresh = max_val * self.best_point_fraction
        near_best = np.where(gap_slice >= thresh)[0]

        if len(near_best) == 0:
            return (start_i + end_i) // 2

        return int(start_i + np.mean(near_best))

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
        angle_min = -2.35619449
        angle_max = 2.35619449
        angle_inc = (angle_max - angle_min) / (n_beams - 1)

        # 1. Preprocess
        proc = self.preprocess_lidar(np.array(scans))

        # 2. Limit FOV
        fov_rad = math.radians(self.fov_deg)
        mid_idx = n_beams // 2
        half_fov_idx = int((fov_rad / 2.0) / angle_inc)
        fov_start = max(0, mid_idx - half_fov_idx)
        fov_end = min(n_beams - 1, mid_idx + half_fov_idx)

        proc[:fov_start] = 0.0
        proc[fov_end + 1:] = 0.0

        # 3. Disparity Extension
        proc = self.extend_disparities(proc, angle_inc)

        # 4. Find closest point in FOV
        fov_valid = np.where(proc[fov_start:fov_end + 1] > 0.10)[0] + fov_start
        if len(fov_valid) == 0:
            # Emergency stop
            e_ct, e_psi, c_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
            self.last_idx = c_idx
            return 0.0, 0.0, e_ct, e_psi

        closest_i = fov_valid[np.argmin(proc[fov_valid])]
        self.eliminate_bubble(proc, closest_i, angle_inc)

        # 5. Max Gap & Best Point
        gap_start, gap_end = self.find_max_gap(proc)
        best_i = self.find_best_point(proc, gap_start, gap_end)

        steering_angle = angle_min + best_i * angle_inc
        steer_cmd = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        # 6. Speed profile
        steer_ratio = abs(steer_cmd) / self.max_steer
        speed_cmd = self.max_speed - steer_ratio * (self.max_speed - self.min_speed)

        target_depth = proc[best_i]
        if target_depth < 1.50:
            depth_scale = max(0.0, target_depth / 1.50)
            speed_cmd = min(speed_cmd, self.min_speed + depth_scale * (self.max_speed - self.min_speed))

        speed_cmd = float(np.clip(speed_cmd, self.min_speed, self.max_speed))

        e_ct, e_psi, closest_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
        self.last_idx = closest_idx

        return steer_cmd, speed_cmd, e_ct, e_psi
