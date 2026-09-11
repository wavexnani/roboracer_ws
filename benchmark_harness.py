#!/usr/bin/env python3
"""
Systematic Benchmark Harness for F1TENTH Controllers:
1. Original Stanley Controller (Classic, a34b5db)
2. Modified Stanley Controller (Augmented, 37b6afd)
3. Baseline MPC Controller (b116494 / baseline_results.json)
4. Latest MPC + Curvature-Aware Velocity Controller (1e57c15 - HEAD)

Evaluates headlessly in f110_gym across 5 primary circuits + 2 generalization circuits,
logging performance metrics and full time-series telemetry.
"""

import os
import sys
import math
import time
import json
import warnings
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

warnings.filterwarnings('ignore')

WS_ROOT = '/home/yeswanth/roboracer_ws'
for p in [f'{WS_ROOT}/src/mpc_controller', f'{WS_ROOT}/src/stanley_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import gym
_orig_make = gym.make
def _patched_make(id, **kwargs):
    kwargs['disable_env_checker'] = True
    env = _orig_make(id, **kwargs)
    return env.unwrapped
gym.make = _patched_make

import f110_gym
from mpc_controller.track_manager import TrackManager, TrackInfo
from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def find_closest_waypoint(
    waypoints: np.ndarray,
    headings: np.ndarray,
    cur_x: float,
    cur_y: float,
    cur_yaw: float,
    last_idx: int,
    num_pts: int,
    window_back: int = 20,
    window_fwd: int = 100
) -> Tuple[int, float]:
    """Finds closest waypoint with local window search and forward-heading preference."""
    search_indices = (np.arange(-window_back, window_fwd) + last_idx) % num_pts
    cand_pts = waypoints[search_indices]
    dist_sq = (cand_pts[:, 0] - cur_x)**2 + (cand_pts[:, 1] - cur_y)**2
    dpsi = (headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
    forward_mask = np.cos(dpsi) > 0.0

    if np.any(forward_mask):
        best_local = int(np.argmin(np.where(forward_mask, dist_sq, np.inf)))
    else:
        best_local = int(np.argmin(dist_sq))

    closest_idx = int(search_indices[best_local])
    min_dist = math.sqrt(dist_sq[best_local])

    # Global search fallback if diverged
    if min_dist > 3.0:
        all_dx = waypoints[:, 0] - cur_x
        all_dy = waypoints[:, 1] - cur_y
        all_dist_sq = all_dx * all_dx + all_dy * all_dy
        all_dpsi = (headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        valid_mask = np.cos(all_dpsi) > 0.0
        if np.any(valid_mask):
            all_cost = np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)
            closest_idx = int(np.argmin(all_cost))
        else:
            closest_idx = int(np.argmin(all_dist_sq))
        min_dist = math.sqrt(all_dist_sq[closest_idx])

    return closest_idx, min_dist


# ---------------------------------------------------------
# Controller 1: Original Stanley Controller (Classic, a34b5db)
# ---------------------------------------------------------
class OriginalStanleyController:
    """
    Classic Stanley controller implementation:
    - Fixed raceline speed scaling (0.68)
    - Front axle reference point (no speed-dependent preview)
    - Zero curvature feedforward (gain_ff = 0.0)
    - No braking lookahead horizon
    - No steering lock speed taper or heading error guard
    """
    def __init__(self, track: TrackInfo, speed_scale: float = 0.68):
        self.track = track
        self.waypoints = track.waypoints
        self.headings = track.headings
        self.s_arr = track.s_arr
        self.target_speeds = track.target_speeds * speed_scale
        self.num_pts = len(self.waypoints)
        self.last_idx = 0

        self.wheelbase = 0.33
        self.gain_k = 2.3
        self.software_k = 0.5
        self.max_steer = 0.4189  # ~24 deg

    def compute_control(self, x: float, y: float, yaw: float, v: float, dt: float) -> Tuple[float, float, float, float]:
        # Classic reference point at front axle
        ref_x = x + self.wheelbase * math.cos(yaw)
        ref_y = y + self.wheelbase * math.sin(yaw)

        idx, _ = find_closest_waypoint(self.waypoints, self.headings, ref_x, ref_y, yaw, self.last_idx, self.num_pts)
        self.last_idx = idx

        tx = math.cos(self.headings[idx])
        ty = math.sin(self.headings[idx])
        ex = ref_x - self.waypoints[idx, 0]
        ey = ref_y - self.waypoints[idx, 1]
        e_ct = -(tx * ey - ty * ex)
        e_psi = wrap_angle(self.headings[idx] - yaw)

        delta_ct = math.atan2(self.gain_k * e_ct, v + self.software_k)
        steer = float(np.clip(e_psi + delta_ct, -self.max_steer, self.max_steer))
        speed = float(self.target_speeds[idx])

        return steer, speed, e_ct, e_psi


# ---------------------------------------------------------
# Controller 2: Modified Stanley Controller (Augmented, 37b6afd)
# ---------------------------------------------------------
class ModifiedStanleyController:
    """
    Augmented Stanley controller:
    - Speed-adaptive preview lookahead (L_preview = 0.15 + 0.02 * v)
    - Curvature feedforward (gain_ff = 0.12)
    - Braking lookahead corner velocity profiling
    - Dynamic steering lock speed reduction (up to 25% taper)
    - Heading error divergence safety guard (15% reduction)
    """
    def __init__(self, track: TrackInfo, speed_scale: float = 0.68, lat_accel_max: float = 3.5, brake_decel: float = 3.5):
        self.track = track
        self.waypoints = track.waypoints
        self.headings = track.headings
        self.s_arr = track.s_arr
        self.kappa = track.kappa
        self.track_length = track.track_length
        self.target_speeds = track.target_speeds * speed_scale
        self.num_pts = len(self.waypoints)
        self.last_idx = 0

        self.wheelbase = 0.33
        self.gain_k = 2.3
        self.software_k = 0.5
        self.gain_ff = 0.12
        self.lookahead_dist = 0.15
        self.lookahead_gain = 0.02
        self.max_steer = 0.4189
        self.lat_accel_max = lat_accel_max
        self.brake_decel = brake_decel

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

    def compute_control(self, x: float, y: float, yaw: float, v: float, dt: float) -> Tuple[float, float, float, float]:
        eff_offset = self.wheelbase + self.lookahead_dist + self.lookahead_gain * v
        ref_x = x + eff_offset * math.cos(yaw)
        ref_y = y + eff_offset * math.sin(yaw)

        idx, _ = find_closest_waypoint(self.waypoints, self.headings, ref_x, ref_y, yaw, self.last_idx, self.num_pts)
        self.last_idx = idx

        tx = math.cos(self.headings[idx])
        ty = math.sin(self.headings[idx])
        ex = ref_x - self.waypoints[idx, 0]
        ey = ref_y - self.waypoints[idx, 1]
        e_ct = -(tx * ey - ty * ex)
        e_psi = wrap_angle(self.headings[idx] - yaw)

        delta_ct = math.atan2(self.gain_k * e_ct, v + self.software_k)
        delta_ff = self.gain_ff * math.atan(self.wheelbase * float(self.kappa[idx])) if self.kappa is not None else 0.0
        steer = float(np.clip(e_psi + delta_ct + delta_ff, -self.max_steer, self.max_steer))

        v_target = float(self.target_speeds[idx])
        v_corner = self._corner_speed_limit(float(self.s_arr[idx]), max(v_target, v))
        v_target = min(v_target, v_corner)

        # Dynamic steering lock attenuation
        steer_ratio = abs(steer) / self.max_steer
        if steer_ratio > 0.5:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.5) / 0.5)

        # Heading divergence safety guard
        if abs(e_psi) > 0.35:
            v_target *= 0.85

        v_target = float(np.clip(v_target, 1.0, 8.0))
        return steer, v_target, e_ct, e_psi


# ---------------------------------------------------------
# Controller 3: Baseline MPC Controller (b116494)
# ---------------------------------------------------------
class BaselineMPCController:
    """
    Baseline LTV-MPC controller (b116494 / baseline_results.json):
    - Initial weights: w_x=2.5, w_y=2.5, w_psi=1.8, w_v=0.5, w_delta=0.8, w_ddelta=2.5
    - Fixed speed scaling (0.60)
    - max_steer_rate = 0.18 rad/step
    - No curvature lookahead pre-braking or LiDAR obstacle protection
    """
    def __init__(self, track: TrackInfo, speed_scale: float = 0.60):
        self.track = track
        self.waypoints = track.waypoints
        self.headings = track.headings
        self.s_arr = track.s_arr
        self.track_length = track.track_length
        self.target_speeds = track.target_speeds * speed_scale
        self.num_pts = len(self.waypoints)
        self.last_idx = 0

        self.cfg = MPCConfig(
            wheelbase=0.33,
            dt=0.08,
            N=10,
            w_x=2.5,
            w_y=2.5,
            w_psi=1.8,
            w_v=0.5,
            w_a=0.1,
            w_delta=0.8,
            w_da=0.2,
            w_ddelta=2.5,
            max_steer=0.4189,
            max_steer_rate=0.18
        )
        self.optimizer = MPCOptimizer(self.cfg)
        self.last_control = (0.0, 0.0)

    def compute_control(self, x: float, y: float, yaw: float, v: float, dt: float) -> Tuple[float, float, float, float]:
        idx, min_dist = find_closest_waypoint(self.waypoints, self.headings, x, y, yaw, self.last_idx, self.num_pts)
        self.last_idx = idx

        # Arc-length parameterised reference horizon
        ref_horizon = np.zeros((self.cfg.N + 1, 4))
        cur_s = self.s_arr[idx]
        speed_est = max(1.5, v)

        for k in range(self.cfg.N + 1):
            s_target = (cur_s + k * speed_est * self.cfg.dt) % self.track_length
            s_diff = np.abs(self.s_arr - s_target)
            idx_k = int(np.argmin(s_diff))
            ref_horizon[k, 0] = self.waypoints[idx_k, 0]
            ref_horizon[k, 1] = self.waypoints[idx_k, 1]
            ref_horizon[k, 2] = self.headings[idx_k]
            ref_horizon[k, 3] = self.target_speeds[idx_k]

        heading_err = wrap_angle(self.headings[idx] - yaw)
        wrong_way = abs(heading_err) > math.radians(85)

        if wrong_way:
            steer_cmd = float(np.clip(heading_err, -self.cfg.max_steer, self.cfg.max_steer))
            target_v = 1.0
            self.last_control = (0.0, steer_cmd)
        else:
            res = self.optimizer.solve(
                current_state=np.array([x, y, yaw, v]),
                ref_trajectory=ref_horizon,
                prev_control=self.last_control
            )
            steer_cmd = float(res.steering)
            target_v = float(self.target_speeds[idx])
            self.last_control = (res.accel, res.steering)

            steer_ratio = abs(steer_cmd) / max(self.cfg.max_steer, 1e-3)
            if steer_ratio > 0.45:
                target_v *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)
            if abs(heading_err) > 0.35:
                target_v *= 0.85

        tx = math.cos(self.headings[idx])
        ty = math.sin(self.headings[idx])
        ex = x - self.waypoints[idx, 0]
        ey = y - self.waypoints[idx, 1]
        e_ct = -(tx * ey - ty * ex)

        return steer_cmd, target_v, e_ct, heading_err


# ---------------------------------------------------------
# Controller 4: Latest MPC + Curvature-Aware Velocity Controller (1e57c15)
# ---------------------------------------------------------
class LatestMPCController:
    """
    Latest State-of-the-Art MPC Controller (1e57c15 - HEAD):
    - Balanced anti-flutter weights: w_x=8.0, w_y=8.0, w_psi=3.0, w_v=0.8, w_delta=0.25, w_ddelta=1.5
    - Backward pre-braking calculation in TrackManager (4m lookahead)
    - Max speed 7.5 m/s on straights, min corner speed 1.8 m/s
    - Real-time Cartesian forward corridor LiDAR obstacle detection (0.50m margin)
    - Longitudinal acceleration/deceleration slew rate limits (2.5 / 3.2 m/s^2)
    - Steering micro-deadband (0.0035 rad) to suppress chatter
    """
    def __init__(self, track_name: str, max_straight_speed: float = 7.5, min_corner_speed: float = 1.8):
        self.track = TrackManager.load_track(
            track_name=track_name,
            waypoint_type='raceline',
            max_straight_speed=max_straight_speed,
            min_corner_speed=min_corner_speed,
            lat_accel_max=2.5,
            a_brake_max=2.0,
            a_accel_max=2.5
        )
        self.waypoints = np.copy(self.track.waypoints)
        self.headings = np.copy(self.track.headings)
        self.s_arr = np.copy(self.track.s_arr)
        self.track_length = self.track.track_length
        self.target_speeds = np.copy(self.track.target_speeds)
        self.num_pts = len(self.waypoints)
        self.last_idx = 0

        self.cfg = MPCConfig(
            max_steer_rate=2.8,
            w_x=8.0,
            w_y=8.0,
            w_psi=3.0,
            w_delta=0.25,
            w_ddelta=1.5,
            w_v=0.8
        )
        self.optimizer = MPCOptimizer(self.cfg)
        self.last_control = (0.0, 0.0)
        self.last_steer_cmd = 0.0
        self.last_speed_cmd = 0.0

        self.max_straight_speed = max_straight_speed
        self.min_corner_speed = min_corner_speed
        self.max_accel = 2.5
        self.max_decel = 3.2
        self.safety_margin_dist = 0.50
        self.steer_deadband = 0.0035

    def compute_control(
        self, x: float, y: float, yaw: float, v: float, dt: float, scans: Optional[List[float]] = None
    ) -> Tuple[float, float, float, float]:
        idx, min_dist = find_closest_waypoint(self.waypoints, self.headings, x, y, yaw, self.last_idx, self.num_pts)
        self.last_idx = idx

        # Arc-length parameterised reference horizon
        ref_horizon = np.zeros((self.cfg.N + 1, 4))
        cur_s = self.s_arr[idx]
        speed_est = max(1.5, v)

        for k in range(self.cfg.N + 1):
            s_target = (cur_s + k * speed_est * self.cfg.dt) % self.track_length
            idx_k = int(np.argmin(np.abs(self.s_arr - s_target)))
            ref_horizon[k] = [self.waypoints[idx_k, 0], self.waypoints[idx_k, 1], self.headings[idx_k], self.target_speeds[idx_k]]

        heading_err = wrap_angle(self.headings[idx] - yaw)
        wrong_way = abs(heading_err) > math.radians(85)

        if wrong_way:
            steer_cmd = float(np.clip(heading_err, -self.cfg.max_steer, self.cfg.max_steer))
            target_v = self.min_corner_speed
            self.last_control = (0.0, steer_cmd)
            self.last_speed_cmd = target_v
        else:
            res = self.optimizer.solve(
                current_state=np.array([x, y, yaw, v]),
                ref_trajectory=ref_horizon,
                prev_control=self.last_control
            )
            steer_cmd = float(res.steering)
            self.last_control = (res.accel, res.steering)

            # 1. Base target speed from pre-braked curvature lookahead profile
            v_target = float(self.target_speeds[idx])

            # 2. Dynamic LiDAR Cartesian forward corridor evaluation
            if scans is not None and len(scans) > 0:
                ranges = np.array(scans)
                n_beams = len(ranges)
                angles = np.linspace(-2.356194, 2.356194, n_beams) - steer_cmd
                valid = np.isfinite(ranges) & (ranges >= 0.10) & (ranges <= 25.0)
                if np.any(valid):
                    r_val = ranges[valid]
                    th_val = angles[valid]
                    x_body = r_val * np.cos(th_val)
                    y_body = r_val * np.sin(th_val)
                    corridor = (x_body > 0.35) & (x_body < 10.0) & (np.abs(y_body) <= 0.28)
                    if np.any(corridor):
                        d_obs = float(np.min(x_body[corridor]))
                        v_obs = math.sqrt(2.0 * self.max_decel * max(0.0, d_obs - self.safety_margin_dist))
                        v_target = min(v_target, max(self.min_corner_speed, v_obs))

                    min_scan_all = float(np.min(ranges[valid]))
                    if min_scan_all < 0.35:
                        v_target *= max(0.70, min_scan_all / 0.35)

            # 3. Dynamic steering lock attenuation
            steer_ratio = abs(steer_cmd) / max(self.cfg.max_steer, 1e-3)
            if steer_ratio > 0.45:
                v_target *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)

            # 4. Heading error safety guard
            if abs(heading_err) > 0.35:
                v_target *= 0.85

            # 5. Smooth longitudinal rate-limiting
            v_cmd_max = self.last_speed_cmd + self.max_accel * dt
            v_cmd_min = self.last_speed_cmd - self.max_decel * dt
            if self.last_speed_cmd < 0.2 and v_target > 0.5:
                v_cmd_max = max(v_cmd_max, 0.8)

            target_v = float(np.clip(v_target, max(0.0, v_cmd_min), min(self.max_straight_speed, v_cmd_max)))
            self.last_speed_cmd = target_v

        # 6. Micro-deadband filter on steering
        if abs(steer_cmd - self.last_steer_cmd) < self.steer_deadband:
            steer_cmd = self.last_steer_cmd
        else:
            self.last_steer_cmd = steer_cmd

        tx = math.cos(self.headings[idx])
        ty = math.sin(self.headings[idx])
        ex = x - self.waypoints[idx, 0]
        ey = y - self.waypoints[idx, 1]
        e_ct = -(tx * ey - ty * ex)

        return steer_cmd, target_v, e_ct, heading_err


# ---------------------------------------------------------
# Simulation Runner
# ---------------------------------------------------------
def run_simulation(
    controller_type: str,
    track_name: str,
    max_sim_time: Optional[float] = None,
    mpc_rate_hz: float = 25.0,
    sim_dt: float = 0.01,
    record_telemetry: bool = True
) -> Dict[str, Any]:
    """Runs a complete simulation lap for a given controller and racetrack."""
    base_track = TrackManager.load_track(track_name, 'raceline')
    track_len = base_track.track_length
    start_pose = base_track.start_pose
    map_path_no_ext = base_track.map_path_no_ext

    if max_sim_time is None:
        max_sim_time = max(180.0, track_len * 0.75 + 40.0)

    # Instantiate Controller
    if controller_type == 'Original_Stanley':
        ctrl = OriginalStanleyController(base_track, speed_scale=0.68)
    elif controller_type == 'Modified_Stanley':
        ctrl = ModifiedStanleyController(base_track, speed_scale=0.68)
    elif controller_type == 'Baseline_MPC':
        ctrl = BaselineMPCController(base_track, speed_scale=0.60)
    elif controller_type == 'Latest_MPC':
        ctrl = LatestMPCController(track_name, max_straight_speed=7.5, min_corner_speed=1.8)
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")

    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=1)
    obs, _, done, _ = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))

    control_interval = int(round((1.0 / mpc_rate_hz) / sim_dt))
    control_dt = 1.0 / mpc_rate_hz

    telemetry = {
        'time': [],
        'distance': [],
        'speed': [],
        'steer': [],
        'steer_rate': [],
        'cross_track_err': [],
        'heading_err': [],
        'curvature': [],
        'x': [],
        'y': []
    }

    sim_time = 0.0
    total_dist = 0.0
    prev_xy = (start_pose[0], start_pose[1])
    current_steer_cmd = 0.0
    current_speed_cmd = 0.0
    last_steer = 0.0

    completed = False
    crashed = False
    crash_reason = "None"
    crash_location = [0.0, 0.0, 0.0]
    interventions = 0

    sharp_errors = []
    sharp_speeds = []
    smooth_errors = []
    smooth_speeds = []

    step_count = 0
    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))

        is_col = bool(obs['collisions'][0])
        min_scan = float(min(obs['scans'][0])) if len(obs['scans'][0]) > 0 else 10.0

        if is_col or min_scan < 0.15:
            crashed = True
            interventions = 1
            crash_reason = f"Collision at t={sim_time:.2f}s, dist={total_dist:.1f}m (min_scan={min_scan:.2f}m)"
            crash_location = [round(cur_x, 2), round(cur_y, 2), round(total_dist, 1)]
            break

        if obs['lap_counts'][0] >= 1 or (total_dist > track_len * 0.95 and math.hypot(cur_x - start_pose[0], cur_y - start_pose[1]) < 3.0):
            completed = True
            break

        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)

        if step_count > 500 and total_dist < 1.0:
            crashed = True
            crash_reason = "Vehicle stalled at start line"
            break

        # Compute Control at control rate
        if step_count % control_interval == 0:
            if controller_type == 'Latest_MPC':
                steer_cmd, speed_cmd, e_ct, e_psi = ctrl.compute_control(
                    cur_x, cur_y, cur_yaw, cur_v, control_dt, scans=obs['scans'][0]
                )
            else:
                steer_cmd, speed_cmd, e_ct, e_psi = ctrl.compute_control(
                    cur_x, cur_y, cur_yaw, cur_v, control_dt
                )

            current_steer_cmd = steer_cmd
            current_speed_cmd = speed_cmd

            steer_rate = abs(steer_cmd - last_steer) / control_dt
            last_steer = steer_cmd

            # Curvature classification
            w_idx = ctrl.last_idx
            cur_kappa = abs(float(base_track.kappa[w_idx])) if base_track.kappa is not None else 0.0

            if cur_kappa >= 0.15:
                sharp_errors.append(abs(e_ct))
                sharp_speeds.append(cur_v)
            elif cur_kappa < 0.05:
                smooth_errors.append(abs(e_ct))
                smooth_speeds.append(cur_v)

            if record_telemetry:
                telemetry['time'].append(round(sim_time, 3))
                telemetry['distance'].append(round(total_dist, 2))
                telemetry['speed'].append(round(cur_v, 3))
                telemetry['steer'].append(round(steer_cmd, 4))
                telemetry['steer_rate'].append(round(steer_rate, 4))
                telemetry['cross_track_err'].append(round(e_ct, 4))
                telemetry['heading_err'].append(round(e_psi, 4))
                telemetry['curvature'].append(round(cur_kappa, 4))
                telemetry['x'].append(round(cur_x, 3))
                telemetry['y'].append(round(cur_y, 3))

        # Simulator physics step
        obs, reward, done, info = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        sim_time += sim_dt
        step_count += 1

    completion_pct = min(100.0, round((total_dist / track_len) * 100.0, 1))
    speeds_arr = np.array(telemetry['speed']) if telemetry['speed'] else np.array([0.0])
    steer_rates_arr = np.array(telemetry['steer_rate']) if telemetry['steer_rate'] else np.array([0.0])
    ctes_arr = np.array(telemetry['cross_track_err']) if telemetry['cross_track_err'] else np.array([0.0])

    avg_speed = float(np.mean(speeds_arr)) if len(speeds_arr) > 0 else 0.0
    max_speed = float(np.max(speeds_arr)) if len(speeds_arr) > 0 else 0.0
    steer_jitter_std = float(np.std(steer_rates_arr)) if len(steer_rates_arr) > 0 else 0.0
    mean_steer_rate = float(np.mean(steer_rates_arr)) if len(steer_rates_arr) > 0 else 0.0
    avg_cte = float(np.mean(np.abs(ctes_arr))) if len(ctes_arr) > 0 else 0.0
    max_cte = float(np.max(np.abs(ctes_arr))) if len(ctes_arr) > 0 else 0.0

    return {
        'controller': controller_type,
        'track_name': track_name,
        'completed': completed,
        'crashed': crashed,
        'interventions': interventions,
        'lap_time': round(sim_time, 2) if completed else None,
        'completion_pct': completion_pct,
        'distance_traveled': round(total_dist, 1),
        'track_length': round(track_len, 1),
        'avg_speed': round(avg_speed, 2),
        'max_speed': round(max_speed, 2),
        'avg_cte': round(avg_cte, 3),
        'max_cte': round(max_cte, 3),
        'steer_jitter_std': round(steer_jitter_std, 3),
        'mean_steer_rate': round(mean_steer_rate, 3),
        'sharp_curve_avg_cte': round(float(np.mean(sharp_errors)), 3) if sharp_errors else 0.0,
        'sharp_curve_avg_speed': round(float(np.mean(sharp_speeds)), 2) if sharp_speeds else 0.0,
        'smooth_curve_avg_cte': round(float(np.mean(smooth_errors)), 3) if smooth_errors else 0.0,
        'smooth_curve_avg_speed': round(float(np.mean(smooth_speeds)), 2) if smooth_speeds else 0.0,
        'crash_reason': crash_reason,
        'crash_location': crash_location,
        'telemetry': telemetry
    }


def main():
    tracks_to_evaluate = [
        'Spielberg',
        'Austin',
        'Monza',
        'BrandsHatch',
        'Silverstone',
        'Catalunya',  # Generalization track
        'Spa'          # Generalization track
    ]

    controllers = [
        'Original_Stanley',
        'Modified_Stanley',
        'Baseline_MPC',
        'Latest_MPC'
    ]

    print("================================================================================")
    print("STARTING SYSTEMATIC BENCHMARK EVALUATION ACROSS ALL 4 CONTROLLERS")
    print(f"Circuits: {tracks_to_evaluate}")
    print(f"Controllers: {controllers}")
    print("================================================================================\n")

    summary_results = []
    telemetry_database = {}

    for t_idx, track in enumerate(tracks_to_evaluate):
        print(f"\n--------------------------------------------------------------------------------")
        print(f"[{t_idx+1}/{len(tracks_to_evaluate)}] BENCHMARKING CIRCUIT: {track}")
        print(f"--------------------------------------------------------------------------------")
        telemetry_database[track] = {}

        for c_idx, ctrl in enumerate(controllers):
            print(f"  --> Running {ctrl} on {track}...", end="", flush=True)
            t_start = time.time()
            res = run_simulation(ctrl, track, record_telemetry=True)
            elapsed = time.time() - t_start

            status_str = "COMPLETED" if res['completed'] else f"FAILED ({res['crash_reason']})"
            lap_str = f"{res['lap_time']}s" if res['completed'] else f"timed out/crashed at {res['completion_pct']}%"
            print(f" {status_str} in {lap_str} (sim wall: {elapsed:.2f}s, avg_v={res['avg_speed']}m/s, max_v={res['max_speed']}m/s)")

            telemetry_data = res.pop('telemetry')
            telemetry_database[track][ctrl] = telemetry_data
            summary_results.append(res)

    out_summary = os.path.join(WS_ROOT, 'benchmark_results.json')
    with open(out_summary, 'w') as f:
        json.dump(summary_results, f, indent=2)
    print(f"\n[INFO] Saved summary results to: {out_summary}")

    out_telem = os.path.join(WS_ROOT, 'benchmark_telemetry.json')
    with open(out_telem, 'w') as f:
        json.dump(telemetry_database, f)
    print(f"[INFO] Saved telemetry datasets to: {out_telem}")
    print("\nBenchmark successfully finished!")


if __name__ == '__main__':
    main()
