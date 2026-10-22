#!/usr/bin/env python3
"""
Diagnostic Script for MPC Controller Steering Jitter Analysis.
Evaluates the exact MPC controller node pipeline on Spielberg Turn 1:
- Straight before corner (s in [2, 22]m)
- Corner approach (s in [22, 29.5]m)
- Corner (s in [29.5, 41.5]m)
- Corner exit (s in [41.5, 55]m)

Logs all required diagnostic variables at every 25 Hz MPC control cycle.
"""

import os
import sys
import math
import time
import json
from typing import Optional, List, Dict, Tuple
import numpy as np
import scipy.sparse as sp

for p in [
    '/home/yeswanth/roboracer_ws/src/controllers/mpc_controller',
    '/sim_ws/src/controllers/mpc_controller',
    '/sim_ws/src/mpc_controller',
    '/home/yeswanth/roboracer_ws/src/mpc_controller',
]:
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
from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DiagnosticMPCController:
    """
    Exact reproduction of MPCControllerNode logic with comprehensive telemetry logging.
    Does NOT modify the original package code.
    """
    def __init__(self, track: TrackInfo, speed_scale: float = 1.0, straight_w_psi: float = 2.0, actuator_delay_s: float = 0.0, model_type: str = "kinematic"):
        self.track = track
        self.straight_w_psi = float(straight_w_psi)
        self.actuator_delay_s = float(actuator_delay_s)
        self.model_type = model_type
        self.waypoints = np.copy(track.waypoints)
        self.path_headings = np.copy(track.headings)
        self.s_arr = np.copy(track.s_arr)
        self.kappa = np.copy(track.kappa)
        self.track_length = track.track_length
        self.scaled_target_speeds = np.copy(track.target_speeds) * speed_scale
        self.num_waypoints = len(self.waypoints)

        # Vehicle & MPC configuration matching mpc_controller_node.py
        # For Experiment 11: model_type="yaw_first_order", using:
        # "exact ZOH discretization of the identified first-order yaw model combined with the Experiment-8 effective 20 ms transport-delay approximation."
        self.cfg = MPCConfig(
            wheelbase=0.33,
            dt=0.08,
            N=10,
            w_x=4.0,
            w_y=4.0,
            w_psi=self.straight_w_psi,
            w_v=0.8,
            w_delta=0.60,
            w_ddelta=3.0,
            w_a=0.1,
            max_steer=0.4189,
            max_steer_rate=2.0,
            actuator_delay_s=self.actuator_delay_s,
            model_type=self.model_type,
            yaw_K=15.4016,
            yaw_tau=0.0363,
            w_r=0.0
        )
        self.optimizer = MPCOptimizer(self.cfg)
        self.N = self.cfg.N
        self.dt = self.cfg.dt

        self._last_idx = 0
        self._last_control = (0.0, 0.0)
        self._last_pos = None
        self._last_odom_time = None
        self.steer_deadband_rad = 0.0035
        self._last_steer_cmd = 0.0
        self._last_speed_cmd = 0.0
        self._current_regime = 'STRAIGHT'

        self.max_straight_speed = 7.5
        self.min_corner_speed = 1.8
        self.max_lat_accel = 2.5
        self.max_accel = 2.5
        self.max_decel = 3.2
        self.safety_margin_dist = 0.50

        # Optimizer initial weights for STRAIGHT
        self.optimizer.update_weights(
            w_x=4.0, w_y=4.0, w_psi=self.straight_w_psi, w_v=0.8, w_delta=0.60, w_ddelta=3.0
        )

        self.regime_transitions = 0
        self._last_s_cont = None

    def _extract_reference_horizon(
        self, cur_x: float, cur_y: float, cur_yaw: float, cur_v: float
    ):
        """Exact logic from mpc_controller_node.py lines 264-360."""
        pos_jump = False
        if self._last_pos is not None:
            if math.hypot(cur_x - self._last_pos[0], cur_y - self._last_pos[1]) > 3.0:
                pos_jump = True
        self._last_pos = (cur_x, cur_y)

        window_backward = 15
        window_forward = 80
        search_offsets = np.arange(-window_backward, window_forward)
        search_indices = (search_offsets + self._last_idx) % self.num_waypoints
        cand_pts = self.waypoints[search_indices]

        dx = cand_pts[:, 0] - cur_x
        dy = cand_pts[:, 1] - cur_y
        dist_sq = dx * dx + dy * dy

        dpsi = (self.path_headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        forward_mask = np.cos(dpsi) > 0.17

        index_bias = np.where(search_offsets < 0, 0.4 * (-search_offsets), 0.0)
        cost = dist_sq + 2.0 * (dpsi ** 2) + index_bias

        if np.any(forward_mask):
            best_local = int(np.argmin(np.where(forward_mask, cost, np.inf)))
        else:
            best_local = int(np.argmin(cost))

        closest_idx = int(search_indices[best_local])
        min_dist = math.sqrt(dist_sq[best_local])

        if min_dist > 3.0 or pos_jump:
            all_dx = self.waypoints[:, 0] - cur_x
            all_dy = self.waypoints[:, 1] - cur_y
            all_dist_sq = all_dx * all_dx + all_dy * all_dy
            all_dpsi = (self.path_headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            valid_mask = np.cos(all_dpsi) > 0.0
            if np.any(valid_mask):
                closest_idx = int(np.argmin(np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)))
            else:
                closest_idx = int(np.argmin(all_dist_sq))

        self._last_idx = closest_idx

        # Determine closest track segment robustly using offsets [-1, 0, 1] relative to closest_idx
        cand_offsets = [-1, 0, 1]
        best_dist_sq = float('inf')
        best_seg_a = closest_idx
        best_seg_b = (closest_idx + 1) % self.num_waypoints
        best_t = 0.0
        best_s_cont = self.s_arr[closest_idx]

        p = np.array([cur_x, cur_y], dtype=np.float64)

        for offset in cand_offsets:
            ia = (closest_idx + offset) % self.num_waypoints
            ib = (ia + 1) % self.num_waypoints
            pa = self.waypoints[ia]
            pb = self.waypoints[ib]
            v = pb - pa
            L2 = float(np.dot(v, v))
            if L2 < 1e-12:
                t = 0.0
            else:
                t = float(np.clip(np.dot(p - pa, v) / L2, 0.0, 1.0))
            p_proj = pa + t * v
            dist_sq = float(np.dot(p - p_proj, p - p_proj))

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_seg_a = ia
                best_seg_b = ib
                best_t = t
                if ia == self.num_waypoints - 1:
                    seg_len = self.track_length - self.s_arr[ia]
                else:
                    seg_len = self.s_arr[ib] - self.s_arr[ia]
                best_s_cont = (self.s_arr[ia] + t * seg_len) % self.track_length

        s_cont = best_s_cont

        # Diagnostics and assertions for projection integrity
        assert np.isfinite(s_cont), f"s_cont is not finite: {s_cont}"
        assert 0.0 <= s_cont < self.track_length, f"s_cont={s_cont} outside [0, {self.track_length})"
        assert 0.0 <= best_t <= 1.0, f"Projection parameter t={best_t} outside [0, 1]"

        # Track and log progress between cycles, flagging anomalous backward motion without hard failure
        if hasattr(self, '_last_s_cont') and self._last_s_cont is not None and not pos_jump:
            ds_progress = s_cont - self._last_s_cont
            if ds_progress < -self.track_length / 2.0:
                ds_progress += self.track_length
            elif ds_progress > self.track_length / 2.0:
                ds_progress -= self.track_length
            if ds_progress < -0.20:
                print(f"[WARN] Anomalous backward s_cont motion: ds={ds_progress:.4f}m (from {self._last_s_cont:.4f} to {s_cont:.4f})")
        self._last_s_cont = s_cont

        # Build N+1 horizon references anchored at continuous s_cont
        ref_horizon = np.zeros((self.N + 1, 4))
        ref_kappas = np.zeros(self.N + 1)
        cur_s = s_cont
        speed_est = max(1.5, cur_v)
        s_targets = np.zeros(self.N + 1)

        for k in range(self.N + 1):
            s_target = (cur_s + k * speed_est * self.dt) % self.track_length
            s_targets[k] = s_target

            idx_a = int(np.searchsorted(self.s_arr, s_target, side='right') - 1)
            idx_a = max(0, min(self.num_waypoints - 1, idx_a))
            idx_b = (idx_a + 1) % self.num_waypoints

            s_a = self.s_arr[idx_a]
            if idx_a == self.num_waypoints - 1:
                seg_len = self.track_length - s_a
                ds = s_target - s_a if s_target >= s_a else (s_target + self.track_length - s_a)
            else:
                s_b = self.s_arr[idx_b]
                seg_len = s_b - s_a
                ds = s_target - s_a

            t = float(np.clip(ds / max(seg_len, 1e-4), 0.0, 1.0))

            ref_horizon[k, 0] = (1.0 - t) * self.waypoints[idx_a, 0] + t * self.waypoints[idx_b, 0]
            ref_horizon[k, 1] = (1.0 - t) * self.waypoints[idx_a, 1] + t * self.waypoints[idx_b, 1]

            dpsi_seg = (self.path_headings[idx_b] - self.path_headings[idx_a] + math.pi) % (2.0 * math.pi) - math.pi
            ref_horizon[k, 2] = (self.path_headings[idx_a] + t * dpsi_seg + math.pi) % (2.0 * math.pi) - math.pi
            ref_horizon[k, 3] = (1.0 - t) * self.scaled_target_speeds[idx_a] + t * self.scaled_target_speeds[idx_b]
            ref_kappas[k] = (1.0 - t) * self.kappa[idx_a] + t * self.kappa[idx_b]

        # Verify reference horizon integrity
        assert abs(s_targets[0] - s_cont) < 1e-9, f"s_ref[0] mismatch: {s_targets[0]} vs {s_cont}"
        for k in range(self.N):
            ds_k = (s_targets[k + 1] - s_targets[k] + self.track_length) % self.track_length
            assert ds_k > 0.0, f"Reference horizon arc-length non-monotonic at step {k}: ds={ds_k}"
        assert np.all(np.isfinite(ref_horizon)), "ref_horizon contains NaN or Inf"

        return ref_horizon, closest_idx, ref_kappas, s_cont, s_targets

    def step(
        self,
        cur_x: float,
        cur_y: float,
        cur_yaw: float,
        cur_v: float,
        dt_actual: float = 0.04,
        scan: Optional[np.ndarray] = None,
        cur_r: float = 0.0
    ):
        """Executes one MPC cycle with full telemetry capture."""
        if self.model_type == "yaw_first_order":
            # Initial yaw rate is taken directly from measured yaw rate (never initialized using v/L*tan(delta))
            current_state = np.array([cur_x, cur_y, cur_yaw, cur_v, cur_r])
        else:
            current_state = np.array([cur_x, cur_y, cur_yaw, cur_v])

        ref_horizon, closest_idx, ref_kappas, s_cont, s_targets = self._extract_reference_horizon(cur_x, cur_y, cur_yaw, cur_v)

        # Dynamic curvature lookahead for regime-adaptive weight scheduling
        k_lookahead = max(abs(float(self.kappa[(closest_idx + w) % self.num_waypoints])) for w in range(15))

        prev_regime = self._current_regime
        # Regime classification with hysteresis
        if self._current_regime == 'CORNER':
            if k_lookahead < 0.032:
                self._current_regime = 'STRAIGHT'
                self.optimizer.update_weights(w_x=4.0, w_y=4.0, w_psi=self.straight_w_psi, w_delta=0.60, w_ddelta=3.0)
                self.regime_transitions += 1
        else:
            if k_lookahead >= 0.040:
                self._current_regime = 'CORNER'
                self.optimizer.update_weights(w_x=8.0, w_y=8.0, w_psi=3.2, w_delta=0.20, w_ddelta=1.5)
                self.regime_transitions += 1

        heading_err = wrap_angle(self.path_headings[closest_idx] - cur_yaw)

        # Tangent orthogonal lateral error
        tx = math.cos(self.path_headings[closest_idx])
        ty = math.sin(self.path_headings[closest_idx])
        ex = cur_x - self.waypoints[closest_idx, 0]
        ey = cur_y - self.waypoints[closest_idx, 1]
        lat_err = -(tx * ey - ty * ex)

        # Solve MPC Optimization
        prev_steer_cmd = self._last_steer_cmd
        res: MPCResult = self.optimizer.solve(
            current_state=current_state,
            ref_trajectory=ref_horizon,
            prev_control=self._last_control
        )
        opt_raw_steer = float(res.steering)
        self._last_control = (res.accel, res.steering)

        # Extract full predicted sequences
        x_sol = self.optimizer._last_sol_x
        N = self.N
        nx = self.optimizer.nx
        nu = self.optimizer.nu
        u_offset = (N + 1) * nx

        if x_sol is not None and len(x_sol) == (N + 1) * nx + N * nu:
            pred_deltas = [float(x_sol[u_offset + k * nu + 1]) for k in range(N)]
            pred_xs = [float(x_sol[k * nx]) for k in range(N + 1)]
            pred_ys = [float(x_sol[k * nx + 1]) for k in range(N + 1)]
            pred_psis = [float(x_sol[k * nx + 2]) for k in range(N + 1)]
            pred_rs = [float(x_sol[k * nx + 4]) for k in range(N + 1)] if nx == 5 else []

            # Predicted lateral and heading errors along the horizon
            pred_lat_errs = []
            pred_heading_errs = []
            for k in range(N + 1):
                pk_tx = math.cos(ref_horizon[k, 2])
                pk_ty = math.sin(ref_horizon[k, 2])
                pk_ex = pred_xs[k] - ref_horizon[k, 0]
                pk_ey = pred_ys[k] - ref_horizon[k, 1]
                pred_lat_errs.append(-(pk_tx * pk_ey - pk_ty * pk_ex))
                pred_heading_errs.append(wrap_angle(pred_psis[k] - ref_horizon[k, 2]))
        else:
            pred_deltas = [opt_raw_steer] * N
            pred_xs = [cur_x] * (N + 1)
            pred_ys = [cur_y] * (N + 1)
            pred_psis = [cur_yaw] * (N + 1)
            pred_rs = [cur_r] * (N + 1) if nx == 5 else []
            pred_lat_errs = [lat_err] * (N + 1)
            pred_heading_errs = [heading_err] * (N + 1)

        # Target speed computation
        v_target = float(self.scaled_target_speeds[closest_idx])
        steer_ratio = abs(opt_raw_steer) / max(self.optimizer.cfg.max_steer, 1e-3)
        if steer_ratio > 0.45:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)
        if abs(heading_err) > 0.35:
            v_target *= 0.85

        v_cmd_max = self._last_speed_cmd + self.max_accel * dt_actual
        v_cmd_min = self._last_speed_cmd - self.max_decel * dt_actual
        if self._last_speed_cmd < 0.2 and v_target > 0.5:
            v_cmd_max = max(v_cmd_max, 0.8)

        target_v = float(np.clip(v_target, max(0.0, v_cmd_min), min(self.max_straight_speed, v_cmd_max)))
        self._last_speed_cmd = target_v

        # Slew rate limit
        max_steer_rate = float(self.optimizer.cfg.max_steer_rate)  # 2.0 rad/s
        max_dsteer = max_steer_rate * dt_actual
        steer_slew_limited = float(np.clip(opt_raw_steer, self._last_steer_cmd - max_dsteer, self._last_steer_cmd + max_dsteer))

        # Deadband filter
        if abs(steer_slew_limited - self._last_steer_cmd) < self.steer_deadband_rad:
            steer_final = self._last_steer_cmd
        else:
            steer_final = steer_slew_limited
            self._last_steer_cmd = steer_final

        # Steering rate actually applied
        steer_rate_actual = (steer_final - prev_steer_cmd) / dt_actual
        raw_steer_rate = (opt_raw_steer - prev_steer_cmd) / dt_actual

        # Reference horizon point 0 tracking errors
        tx0 = math.cos(ref_horizon[0, 2])
        ty0 = math.sin(ref_horizon[0, 2])
        ex0 = cur_x - ref_horizon[0, 0]
        ey0 = cur_y - ref_horizon[0, 1]
        e_y_0 = -(tx0 * ey0 - ty0 * ex0)
        e_psi_0 = wrap_angle(cur_yaw - ref_horizon[0, 2])

        # Effective transport-delay approximation (Experiment 8 & 11)
        alpha = float(np.clip(self.actuator_delay_s / 0.040, 0.0, 1.0))
        delta_delayed = float((1.0 - alpha) * opt_raw_steer + alpha * prev_steer_cmd)

        telemetry_point = {
            'cur_s': float(s_cont),
            'cur_s_discrete': float(self.s_arr[closest_idx]),
            's_cont': float(s_cont),
            's_ref_0': float(s_targets[0]),
            'psi_ref_0': float(ref_horizon[0, 2]),
            'e_psi_0': float(e_psi_0),
            'e_y_0': float(e_y_0),
            'delta0': float(pred_deltas[0]),
            'delta1': float(pred_deltas[1]),
            'delta0_minus_prev': float(pred_deltas[0] - prev_steer_cmd),
            'delta1_minus_delta0': float(pred_deltas[1] - pred_deltas[0]),
            'speed': float(cur_v),
            'regime': self._current_regime,
            'k_lookahead': float(k_lookahead),
            'closest_idx': int(closest_idx),
            'lateral_error': float(lat_err),
            'heading_error': float(heading_err),
            'steer_cmd': float(steer_final),
            'prev_steer_cmd': float(prev_steer_cmd),
            'opt_raw_steer': float(opt_raw_steer),
            'delta_cmd': float(opt_raw_steer),
            'delta_delayed': float(delta_delayed),
            'measured_yaw_rate': float(cur_r),
            'steer_rate': float(steer_rate_actual),
            'raw_steer_rate': float(raw_steer_rate),
            'optimizer_success': bool(res.success),
            'solve_time_ms': float(res.solve_time_ms),
            'predicted_deltas': pred_deltas,
            'predicted_xs': pred_xs,
            'predicted_ys': pred_ys,
            'predicted_psis': pred_psis,
            'predicted_yaw_rates': pred_rs,
            'predicted_lateral_errors': pred_lat_errs,
            'predicted_heading_errors': pred_heading_errs,
            'reference_kappas': list(ref_kappas),
            's_ref': list(s_targets),
            'ref_horizon': ref_horizon.tolist(),
            'target_speed': float(target_v)
        }

        return steer_final, target_v, telemetry_point


def run_controlled_baseline(straight_w_psi: float = 2.0, actuator_delay_s: float = 0.0, model_type: str = "kinematic"):
    print(f"Loading Spielberg track (straight_w_psi = {straight_w_psi}, actuator_delay_s = {actuator_delay_s}, model_type = {model_type})...")
    track = TrackManager.load_track('Spielberg')
    start_pose = track.start_pose
    map_path_no_ext = track.map_path_no_ext

    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret

    ctrl = DiagnosticMPCController(track, speed_scale=1.0, straight_w_psi=straight_w_psi, actuator_delay_s=actuator_delay_s, model_type=model_type)
    agent = env.sim.agents[0]

    mpc_rate_hz = 25.0
    sim_dt = 0.01
    control_interval = int(round((1.0 / mpc_rate_hz) / sim_dt))
    control_dt = 1.0 / mpc_rate_hz

    sim_time = 0.0
    max_sim_time = 20.0  # Sufficient to cover straight, entry, apex, and exit of Turn 1
    total_dist = 0.0
    prev_xy = (start_pose[0], start_pose[1])

    cycle_logs = []
    crashed = False
    step_count = 0

    current_steer = 0.0
    current_speed = 0.0

    print(f"Running simulation through Turn 1 [{model_type}]...")
    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        cur_r = float(agent.state[5])
        actual_steer = float(agent.state[2])
        actual_beta = float(agent.state[6])

        is_col = bool(obs['collisions'][0])
        min_scan = float(min(obs['scans'][0])) if len(obs['scans'][0]) > 0 else 10.0

        if is_col:
            crashed = True
            print(f"Collision at t={sim_time:.2f}s, dist={total_dist:.2f}m!")
            break

        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)

        # Control cycle at 25 Hz
        if step_count % control_interval == 0:
            current_steer, current_speed, pt = ctrl.step(
                cur_x, cur_y, cur_yaw, cur_v, dt_actual=control_dt, scan=obs['scans'][0], cur_r=cur_r
            )
            pt['timestamp'] = float(sim_time)
            pt['min_scan'] = float(min_scan)
            pt['cur_x'] = float(cur_x)
            pt['cur_y'] = float(cur_y)
            pt['cur_yaw'] = float(cur_yaw)
            pt['actual_steer'] = float(actual_steer)
            pt['actual_yaw_rate'] = float(cur_r)
            pt['actual_slip_beta'] = float(actual_beta)
            cycle_logs.append(pt)

            # Stop once vehicle completes corner exit and is well onto the next straight (s >= 56m)
            if pt['cur_s'] > 58.0 and pt['cur_s'] < 100.0:
                print(f"Reached s={pt['cur_s']:.2f}m (past Turn 1 exit). Stopping scenario.")
                break

        # Step physics simulator
        obs, _, _, _ = env.step(np.array([[current_steer, current_speed]]))
        sim_time += sim_dt
        step_count += 1

    print(f"Simulation completed. Logged {len(cycle_logs)} MPC cycles.")
    return cycle_logs, ctrl.regime_transitions, crashed


if __name__ == '__main__':
    out_file = sys.argv[1] if len(sys.argv) > 1 else '/home/yeswanth/roboracer_ws/diagnostic_experiment1_telemetry.json'
    w_psi_val = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    delay_val = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    model_val = sys.argv[4] if len(sys.argv) > 4 else 'kinematic'
    logs, transitions, crashed = run_controlled_baseline(straight_w_psi=w_psi_val, actuator_delay_s=delay_val, model_type=model_val)
    with open(out_file, 'w') as f:
        json.dump(logs, f, indent=2)
    print(f"Saved telemetry to {out_file}")
