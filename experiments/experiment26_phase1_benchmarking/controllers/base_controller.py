"""Base interface and common utilities for Phase 1 controller benchmarking."""

import math
from abc import ABC, abstractmethod
from typing import Tuple, Optional
import numpy as np


def wrap_angle(angle: float) -> float:
    """Wraps an angle to the range [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_cross_track_error(
    cur_x: float,
    cur_y: float,
    cur_yaw: float,
    waypoints: np.ndarray,
    headings: np.ndarray,
    last_idx: int = 0
) -> Tuple[float, float, int]:
    """
    Computes ground-truth cross-track error and heading error relative to waypoints.
    Returns: (cross_track_error, heading_error, closest_index)
    """
    n_pts = len(waypoints)
    search_indices = (np.arange(-30, 80) + last_idx) % n_pts
    cand_pts = waypoints[search_indices]
    dist_sq = (cand_pts[:, 0] - cur_x) ** 2 + (cand_pts[:, 1] - cur_y) ** 2
    best_loc = int(np.argmin(dist_sq))
    closest_idx = int(search_indices[best_loc])

    if dist_sq[best_loc] > 9.0:
        all_dsq = (waypoints[:, 0] - cur_x) ** 2 + (waypoints[:, 1] - cur_y) ** 2
        closest_idx = int(np.argmin(all_dsq))

    tx = math.cos(headings[closest_idx])
    ty = math.sin(headings[closest_idx])
    ex = cur_x - waypoints[closest_idx, 0]
    ey = cur_y - waypoints[closest_idx, 1]

    cross_track_error = -(tx * ey - ty * ex)
    heading_error = wrap_angle(headings[closest_idx] - cur_yaw)

    return float(cross_track_error), float(heading_error), closest_idx


class BaseBenchmarkController(ABC):
    """Abstract base class for all benchmarked racing controllers."""

    def __init__(self, name: str):
        self.name = name
        self.wheelbase = 0.33024
        self.max_steer = 0.4189  # ~24.0 deg

    @abstractmethod
    def compute_control(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        dt: float,
        scans: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        """
        Compute control commands.
        Returns:
            steer_cmd (float): Steering angle command (rad)
            speed_cmd (float): Speed command (m/s)
            e_ct (float): Cross-track error (m)
            e_psi (float): Heading error (rad)
        """
        pass

    def reset(self):
        """Reset internal state between runs/laps."""
        pass
