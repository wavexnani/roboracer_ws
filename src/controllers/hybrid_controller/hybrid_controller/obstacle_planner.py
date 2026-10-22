import math
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from .clustering import ObstacleCluster, cluster_corridor_points

__all__ = ['ObstacleCluster', 'BypassPath', 'ObstaclePlanner']


@dataclass
class BypassPath:
    active: bool
    direction: str          # 'LEFT', 'RIGHT', or 'NONE'
    offset_dist: float      # Peak lateral deviation [m]
    start_s: float          # Arc length where avoidance begins [m]
    length_s: float         # Arc length of bypass maneuver [m]
    safe_speed_cap: float   # Recommended speed during bypass [m/s]


class ObstaclePlanner:
    """Reactive Frenet avoidance planner for forward obstacles."""

    def __init__(
        self,
        vehicle_width: float = 0.30,
        vehicle_length: float = 0.55,
        safety_margin: float = 0.22,
        min_cluster_pts: int = 5,
        cluster_dist_thresh: float = 0.20
    ):
        self.vehicle_width = vehicle_width
        self.vehicle_length = vehicle_length
        self.safety_margin = safety_margin
        self.min_cluster_pts = min_cluster_pts
        self.cluster_dist_thresh = cluster_dist_thresh

        self.current_bypass: Optional[BypassPath] = None
        self.bypass_completion_s: float = 0.0

    def parse_lidar_scans(
        self,
        ranges: List[float],
        angle_min: float = -2.356194,
        angle_max: float = 2.356194,
        cur_x: float = 0.0,
        cur_y: float = 0.0,
        cur_yaw: float = 0.0,
        sdf_fn = None
    ) -> List[ObstacleCluster]:
        return cluster_corridor_points(
            ranges=ranges,
            angle_min=angle_min,
            angle_max=angle_max,
            cur_x=cur_x,
            cur_y=cur_y,
            cur_yaw=cur_yaw,
            sdf_fn=sdf_fn,
            min_cluster_pts=self.min_cluster_pts,
            cluster_dist_thresh=self.cluster_dist_thresh
        )

    def plan_avoidance(
        self,
        clusters: List[ObstacleCluster],
        current_s: float,
        track_length: float,
        cur_x: float = 0.0,
        cur_y: float = 0.0,
        cur_yaw: float = 0.0,
        sdf_fn = None,
        lane_width_est: float = 1.2
    ) -> BypassPath:
        # Check active bypass completion
        if self.current_bypass and self.current_bypass.active:
            s_traveled = (current_s - self.current_bypass.start_s) % track_length
            if s_traveled >= self.current_bypass.length_s:
                self.current_bypass = None
            else:
                return self.current_bypass

        if not clusters:
            return BypassPath(active=False, direction='NONE', offset_dist=0.0, start_s=current_s, length_s=0.0, safe_speed_cap=7.5)

        # Closest forward obstacle directly in driving envelope
        critical_obs = None
        min_dist = float('inf')
        for c in clusters:
            if abs(c.center_y) <= 0.32 and c.center_x < min_dist:
                min_dist = c.center_x
                critical_obs = c

        if critical_obs is None or critical_obs.center_x > 8.5:
            return BypassPath(active=False, direction='NONE', offset_dist=0.0, start_s=current_s, length_s=0.0, safe_speed_cap=7.5)

        # Compute obstacle world position
        cos_y = math.cos(cur_yaw)
        sin_y = math.sin(cur_yaw)
        obs_xw = cur_x + critical_obs.center_x * cos_y - critical_obs.center_y * sin_y
        obs_yw = cur_y + critical_obs.center_x * sin_y + critical_obs.center_y * cos_y

        # Track left normal: [-sin(cur_yaw), cos(cur_yaw)]
        nx = -sin_y
        ny = cos_y

        test_offset = 0.52
        direction = 'LEFT'
        offset_dist = test_offset

        if sdf_fn is not None:
            sdf_left = sdf_fn(obs_xw + test_offset * nx, obs_yw + test_offset * ny)
            sdf_right = sdf_fn(obs_xw - test_offset * nx, obs_yw - test_offset * ny)

            if sdf_left >= sdf_right:
                direction = 'LEFT'
                offset_dist = min(0.55, max(0.35, sdf_left - 0.22))
            else:
                direction = 'RIGHT'
                offset_dist = -min(0.55, max(0.35, sdf_right - 0.22))
        else:
            if critical_obs.center_y <= 0.0:
                direction = 'LEFT'
                offset_dist = test_offset
            else:
                direction = 'RIGHT'
                offset_dist = -test_offset

        bypass_length = max(7.0, 2.0 * critical_obs.center_x + 2.0)
        safe_speed = min(2.8, max(1.8, 0.45 * critical_obs.center_x))

        bypass = BypassPath(
            active=True,
            direction=direction,
            offset_dist=offset_dist,
            start_s=current_s,
            length_s=bypass_length,
            safe_speed_cap=safe_speed
        )
        self.current_bypass = bypass
        return bypass

    def modulate_reference_waypoints(
        self,
        ref_horizon: np.ndarray,
        bypass: BypassPath,
        current_s: float,
        track_length: float,
        step_ds: float = 0.35
    ) -> np.ndarray:
        if not bypass.active or abs(bypass.offset_dist) < 1e-3:
            return ref_horizon

        mod_horizon = np.copy(ref_horizon)
        N_pts = len(ref_horizon)

        for k in range(N_pts):
            s_k = (current_s + k * step_ds) % track_length
            s_rel = (s_k - bypass.start_s) % track_length

            if 0.0 <= s_rel <= bypass.length_s:
                tau = s_rel / bypass.length_s
                bell = 0.5 * (1.0 - math.cos(2.0 * math.pi * tau))
                lat_shift = bypass.offset_dist * bell

                d_bell_ds = (math.pi / bypass.length_s) * math.sin(2.0 * math.pi * tau)
                d_lat_ds = bypass.offset_dist * d_bell_ds

                psi = mod_horizon[k, 2]
                mod_horizon[k, 0] += -math.sin(psi) * lat_shift
                mod_horizon[k, 1] += math.cos(psi) * lat_shift
                mod_horizon[k, 2] += math.atan2(d_lat_ds, 1.0)
                mod_horizon[k, 3] = min(mod_horizon[k, 3], bypass.safe_speed_cap)

        return mod_horizon
