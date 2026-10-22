#!/usr/bin/env python3
"""
Supervisory Context-Aware Hybrid Controller Engine for F1TENTH / ROBORACER.
ROBORACER IFAC 2026 Regulations Compliant (BEXCO, Busan).

Unifies 5 operational regimes:
1. Low-Jitter Wall-Damped MPC (Narrow Corridors: std(d_dot) <= 0.10 rad/s)
2. Reactive Frenet Lattice Bypass (Q2 & Finals Obstacle Avoidance)
3. High-Speed Straightaway MPC (Q3 Time Trial Attack)
4. High-Authority Apex MPC (Sharp Hairpins & Backward Pre-Braking)
5. Fail-Safe Stanley Path Tracking & Recovery (Global Asymptotic Convergence)
+ Anti-Deadlock Autonomous Watchdog (<5s Fully Autonomous Rule)
"""

import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

from .track_manager import TrackInfo
from .mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
from .stanley_controller import StanleyController
from .pure_pursuit import PurePursuitController
from .obstacle_planner import ObstaclePlanner, BypassPath


class DrivingRegime(Enum):
    WALL_DAMPED = 'WALL_DAMPED'
    OBSTACLE_BYPASS = 'OBSTACLE_BYPASS'
    HIGH_SPEED_STRAIGHT = 'HIGH_SPEED_STRAIGHT'
    CORNER_APEX = 'CORNER_APEX'
    STANLEY_RECOVERY = 'STANLEY_RECOVERY'


@dataclass
class HybridControlOutput:
    steering: float
    speed: float
    active_regime: DrivingRegime
    cross_track_err: float
    heading_err: float
    bypass_active: bool
    predicted_horizon: Optional[np.ndarray] = None


class HybridSupervisor:
    """Supervisory manager handling dynamic gain-scheduling, regime classification, and bumpless transfer."""

    def __init__(
        self,
        track: TrackInfo,
        mode: str = 'Q1',
        low_friction_mode: bool = False,
        mpc_rate_hz: float = 25.0
    ):
        self.track = track
        self.mode = mode.upper()  # 'Q1', 'Q2', 'Q3', or 'FINALS'
        self.low_friction_mode = low_friction_mode
        self.dt = 1.0 / mpc_rate_hz

        # Kinematic limits
        if low_friction_mode:
            self.max_straight_speed = 5.0
            self.min_corner_speed = 1.6
            self.max_accel = 2.0
            self.max_decel = 2.8
            self.steer_deadband = 0.0035
        else:
            self.max_straight_speed = 7.5
            self.min_corner_speed = 1.8
            self.max_accel = 2.5
            self.max_decel = 3.2
            self.steer_deadband = 0.0035

        # Sub-controllers
        self.mpc_cfg = MPCConfig(
            wheelbase=0.33,
            dt=0.08,
            N=10,
            w_x=8.0,
            w_y=8.0,
            w_psi=3.0,
            w_v=0.8,
            w_delta=0.25,
            w_ddelta=1.5,
            max_steer=0.4189,
            max_steer_rate=2.8
        )
        self.mpc = MPCOptimizer(self.mpc_cfg)
        self.stanley = StanleyController(track, lat_accel_max=2.0 if low_friction_mode else 2.5)
        self.pure_pursuit = PurePursuitController(track)
        self.obstacle_planner = ObstaclePlanner()

        # State memory for bumpless transfer and rate limiting
        self.last_steer_cmd = 0.0
        self.last_speed_cmd = 0.0
        self.last_mpc_control = (0.0, 0.0)
        self.last_idx = 0
        self.active_regime = DrivingRegime.HIGH_SPEED_STRAIGHT

        # Anti-deadlock watchdog (§ 5.5 Fully Autonomous <5s rule)
        self.stall_start_time: Optional[float] = None
        self.reversing_until: Optional[float] = None

    def classify_regime(
        self,
        e_ct: float,
        e_psi: float,
        min_scan_side: float,
        current_kappa: float,
        upcoming_kappa: float,
        bypass_active: bool
    ) -> DrivingRegime:
        """
        Determines the active operational regime from sensory state.
        Priority:
        1. Large Error / Spinout -> Stanley Recovery (asymptotic convergence)
        2. Obstacle in corridor -> Obstacle Bypass (Q2 & Finals)
        3. Corner approach or apex -> Corner Apex (high authority)
        4. Straight near wall -> Wall Damped (eliminates flutter)
        5. Default -> High-Speed Straight
        """
        if self.mode in ['Q2', 'FINALS'] and bypass_active:
            if abs(e_psi) > math.radians(65):
                return DrivingRegime.STANLEY_RECOVERY
            return DrivingRegime.OBSTACLE_BYPASS

        if abs(e_ct) > 0.50 or abs(e_psi) > math.radians(55):
            return DrivingRegime.STANLEY_RECOVERY

        # Turn entry or sharp curve: Prioritize cornering authority!
        if upcoming_kappa >= 0.05 or current_kappa >= 0.05:
            return DrivingRegime.CORNER_APEX

        # Straight or gentle sweeper close to side walls (< 0.60m)
        if min_scan_side < 0.60:
            return DrivingRegime.WALL_DAMPED

        return DrivingRegime.HIGH_SPEED_STRAIGHT

    def compute_control(
        self,
        cur_x: float,
        cur_y: float,
        cur_yaw: float,
        cur_v: float,
        scans: Optional[List[float]] = None,
        sim_time: float = 0.0
    ) -> HybridControlOutput:
        """Main execution pipeline executed at 25 Hz."""
        # Normalize yaw to [-pi, pi]
        cur_yaw = (cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        track = self.track
        num_pts = len(track.waypoints)

        # 1. Anti-Deadlock Watchdog (<5s rule)
        if self.reversing_until is not None:
            if sim_time < self.reversing_until:
                return HybridControlOutput(
                    steering=-self.last_steer_cmd,
                    speed=-1.0,
                    active_regime=DrivingRegime.STANLEY_RECOVERY,
                    cross_track_err=0.0,
                    heading_err=0.0,
                    bypass_active=False
                )
            else:
                self.reversing_until = None
                self.stall_start_time = None

        if cur_v < 0.15:
            if self.stall_start_time is None:
                self.stall_start_time = sim_time
            elif (sim_time - self.stall_start_time) > 1.5:
                self.reversing_until = sim_time + 0.8
                return HybridControlOutput(
                    steering=-self.last_steer_cmd,
                    speed=-1.0,
                    active_regime=DrivingRegime.STANLEY_RECOVERY,
                    cross_track_err=0.0,
                    heading_err=0.0,
                    bypass_active=False
                )
        else:
            self.stall_start_time = None

        # 2. Windowed Waypoint Search with Forward-Heading Preference
        search_window = 120
        indices = (np.arange(-20, search_window) + self.last_idx) % num_pts
        cand_pts = track.waypoints[indices]
        dist_sq = (cand_pts[:, 0] - cur_x)**2 + (cand_pts[:, 1] - cur_y)**2
        dpsi = (track.headings[indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        forward_mask = np.cos(dpsi) > 0.0

        if np.any(forward_mask):
            best_local = int(np.argmin(np.where(forward_mask, dist_sq, np.inf)))
        else:
            best_local = int(np.argmin(dist_sq))

        closest_idx = int(indices[best_local])
        min_dist = math.sqrt(dist_sq[best_local])

        # Global search fallback if vehicle diverged
        if min_dist > 3.0:
            all_dx = track.waypoints[:, 0] - cur_x
            all_dy = track.waypoints[:, 1] - cur_y
            all_dist_sq = all_dx * all_dx + all_dy * all_dy
            all_dpsi = (track.headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            valid_mask = np.cos(all_dpsi) > 0.0
            if np.any(valid_mask):
                closest_idx = int(np.argmin(np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)))
            else:
                closest_idx = int(np.argmin(all_dist_sq))

        self.last_idx = closest_idx

        # Continuous orthogonal cross-track error
        tx = math.cos(track.headings[closest_idx])
        ty = math.sin(track.headings[closest_idx])
        ex = cur_x - track.waypoints[closest_idx, 0]
        ey = cur_y - track.waypoints[closest_idx, 1]
        e_ct = -(tx * ey - ty * ex)
        e_psi = (track.headings[closest_idx] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # 3. Analyze LiDAR Scans
        min_scan_all = 10.0
        min_scan_side = 10.0
        bypass = BypassPath(active=False, direction='NONE', offset_dist=0.0, start_s=0.0, length_s=0.0, safe_speed_cap=7.5)

        if scans is not None and len(scans) > 0:
            r_arr = np.array(scans)
            valid = np.isfinite(r_arr) & (r_arr >= 0.10)
            if np.any(valid):
                min_scan_all = float(np.min(r_arr[valid]))
                n_beams = len(r_arr)
                left_idx = slice(int(n_beams * 0.65), int(n_beams * 0.85))
                right_idx = slice(int(n_beams * 0.15), int(n_beams * 0.35))
                side_scans = np.concatenate([r_arr[left_idx], r_arr[right_idx]])
                valid_side = side_scans[np.isfinite(side_scans) & (side_scans >= 0.10)]
                if len(valid_side) > 0:
                    min_scan_side = float(np.min(valid_side))

            if self.mode in ['Q2', 'FINALS']:
                clusters = self.obstacle_planner.parse_lidar_scans(
                    scans,
                    cur_x=cur_x,
                    cur_y=cur_y,
                    cur_yaw=cur_yaw,
                    sdf_fn=track.sdf_fn
                )
                current_s = float(track.s_arr[closest_idx])
                bypass = self.obstacle_planner.plan_avoidance(
                    clusters, current_s, track.track_length,
                    cur_x=cur_x, cur_y=cur_y, cur_yaw=cur_yaw,
                    sdf_fn=track.sdf_fn
                )

        # 4. Curvature checks
        current_kappa = abs(float(track.kappa[closest_idx])) if track.kappa is not None else 0.0
        lookahead_pts = max(1, int(round(3.5 / max(0.05, track.track_length / num_pts))))
        lookahead_indices = (np.arange(0, lookahead_pts) + closest_idx) % num_pts
        upcoming_kappa = float(np.max(np.abs(track.kappa[lookahead_indices]))) if track.kappa is not None else 0.0

        # 5. Classify regime
        regime = self.classify_regime(
            e_ct, e_psi, min_scan_side, current_kappa, upcoming_kappa, bypass.active
        )
        self.active_regime = regime

        # 6. Branch into active control law
        predicted_horizon = None

        if regime == DrivingRegime.STANLEY_RECOVERY:
            steer_raw, v_target, _, _ = self.stanley.compute_control(cur_x, cur_y, cur_yaw, cur_v)
            v_target = min(v_target, 2.5)

        elif regime == DrivingRegime.WALL_DAMPED:
            # Laser-smooth corridor mode: Higher damping on steering rate to eradicate flutter
            self.mpc.update_weights(
                w_x=5.0, w_y=5.0, w_psi=2.5, w_delta=0.45, w_ddelta=2.5, w_v=0.8
            )
            ref_horizon = self._build_ref_horizon(closest_idx, cur_v)
            res = self.mpc.solve(
                current_state=np.array([cur_x, cur_y, cur_yaw, cur_v]),
                ref_trajectory=ref_horizon,
                prev_control=self.last_mpc_control
            )
            steer_raw = res.steering
            self.last_mpc_control = (res.accel, res.steering)
            predicted_horizon = res.predicted_trajectory
            # Adhere to pre-braked profile
            v_target = float(track.target_speeds[closest_idx])

        elif regime == DrivingRegime.OBSTACLE_BYPASS:
            # Modulate reference trajectory around obstacle
            self.mpc.update_weights(
                w_x=7.5, w_y=7.5, w_psi=3.0, w_delta=0.25, w_ddelta=1.8, w_v=0.8
            )
            current_s = float(track.s_arr[closest_idx])
            base_horizon = self._build_ref_horizon(closest_idx, cur_v)
            step_ds = max(1.5, cur_v) * self.mpc_cfg.dt
            mod_horizon = self.obstacle_planner.modulate_reference_waypoints(
                base_horizon, bypass, current_s, track.track_length, step_ds=step_ds
            )
            res = self.mpc.solve(
                current_state=np.array([cur_x, cur_y, cur_yaw, cur_v]),
                ref_trajectory=mod_horizon,
                prev_control=self.last_mpc_control
            )
            steer_raw = res.steering
            self.last_mpc_control = (res.accel, res.steering)
            predicted_horizon = res.predicted_trajectory
            v_target = bypass.safe_speed_cap

        elif regime == DrivingRegime.CORNER_APEX:
            # High-authority apex mode: Relax steering penalty to allow full 24° lock into turn
            self.mpc.update_weights(
                w_x=8.5, w_y=8.5, w_psi=3.2, w_delta=0.20, w_ddelta=1.4, w_v=0.8
            )
            ref_horizon = self._build_ref_horizon(closest_idx, cur_v)
            res = self.mpc.solve(
                current_state=np.array([cur_x, cur_y, cur_yaw, cur_v]),
                ref_trajectory=ref_horizon,
                prev_control=self.last_mpc_control
            )
            steer_raw = res.steering
            self.last_mpc_control = (res.accel, res.steering)
            predicted_horizon = res.predicted_trajectory
            v_target = float(track.target_speeds[closest_idx])

        else:  # HIGH_SPEED_STRAIGHT
            self.mpc.update_weights(
                w_x=7.0, w_y=7.0, w_psi=2.8, w_delta=0.25, w_ddelta=1.5, w_v=0.8
            )
            ref_horizon = self._build_ref_horizon(closest_idx, cur_v)
            res = self.mpc.solve(
                current_state=np.array([cur_x, cur_y, cur_yaw, cur_v]),
                ref_trajectory=ref_horizon,
                prev_control=self.last_mpc_control
            )
            steer_raw = res.steering
            self.last_mpc_control = (res.accel, res.steering)
            predicted_horizon = res.predicted_trajectory
            v_target = float(track.target_speeds[closest_idx])

        # 7. Dynamic LiDAR Cartesian forward corridor safety (emergency stop when not in active bypass)
        if regime != DrivingRegime.OBSTACLE_BYPASS and scans is not None and len(scans) > 0:
            ranges = np.array(scans)
            n_beams = len(ranges)
            angles = np.linspace(-2.356194, 2.356194, n_beams)
            valid = np.isfinite(ranges) & (ranges >= 0.10) & (ranges <= 25.0)
            if np.any(valid):
                r_val = ranges[valid]
                th_val = angles[valid]
                x_body = r_val * np.cos(th_val)
                y_body = r_val * np.sin(th_val)
                corridor = (x_body > 0.35) & (x_body < 10.0) & (np.abs(y_body) <= 0.28)
                if np.any(corridor):
                    d_obs = float(np.min(x_body[corridor]))
                    v_obs = math.sqrt(2.0 * self.max_decel * max(0.0, d_obs - 0.50))
                    v_target = min(v_target, max(self.min_corner_speed, v_obs))

                min_scan_all = float(np.min(ranges[valid]))
                if min_scan_all < 0.35:
                    v_target *= max(0.70, min_scan_all / 0.35)

        # 8. Dynamic Steering Lock Attenuation
        steer_ratio = abs(steer_raw) / self.mpc_cfg.max_steer
        if steer_ratio > 0.45:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)

        # 9. Heading divergence safety guard
        if abs(e_psi) > 0.35:
            v_target *= 0.85

        # 10. Micro-deadband on steering to suppress chatter in straight corridors
        if regime in [DrivingRegime.WALL_DAMPED, DrivingRegime.HIGH_SPEED_STRAIGHT]:
            if abs(steer_raw - self.last_steer_cmd) < self.steer_deadband:
                steer_cmd = self.last_steer_cmd
            else:
                steer_cmd = steer_raw
        else:
            steer_cmd = steer_raw

        self.last_steer_cmd = steer_cmd

        # 11. Smooth Longitudinal Acceleration / Deceleration Rate Limiting
        v_cmd_max = self.last_speed_cmd + self.max_accel * self.dt
        v_cmd_min = self.last_speed_cmd - self.max_decel * self.dt
        if self.last_speed_cmd < 0.2 and v_target > 0.5:
            v_cmd_max = max(v_cmd_max, 0.8)

        v_cmd = float(np.clip(v_target, max(0.0, v_cmd_min), min(self.max_straight_speed, v_cmd_max)))
        self.last_speed_cmd = v_cmd

        return HybridControlOutput(
            steering=steer_cmd,
            speed=v_cmd,
            active_regime=regime,
            cross_track_err=e_ct,
            heading_err=e_psi,
            bypass_active=bypass.active,
            predicted_horizon=predicted_horizon
        )

    def _build_ref_horizon(self, closest_idx: int, cur_v: float) -> np.ndarray:
        """Constructs an arc-length parameterized reference horizon ahead of current position."""
        track = self.track
        N = self.mpc_cfg.N
        dt = self.mpc_cfg.dt
        ref = np.zeros((N + 1, 4))

        cur_s = track.s_arr[closest_idx]
        speed_est = max(1.5, cur_v)

        for k in range(N + 1):
            s_target = (cur_s + k * speed_est * dt) % track.track_length
            idx_k = int(np.argmin(np.abs(track.s_arr - s_target)))
            ref[k] = [
                track.waypoints[idx_k, 0],
                track.waypoints[idx_k, 1],
                track.headings[idx_k],
                track.target_speeds[idx_k]
            ]
        return ref
