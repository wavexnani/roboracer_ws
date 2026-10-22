#!/usr/bin/env python3
"""
Phase 1: Parameter Sensitivity Analysis (Experiment 1.2)

One-Factor-At-A-Time (OFAT) parameter sweep across all 5 controllers:
1. Pure Pursuit: Lookahead distance (L_d)
2. Stanley: Cross-track gain (k), Soft damping (k_soft), Curvature feedforward (k_ff)
3. MPC: Prediction horizon (N), Steering rate weight (w_ddelta), Heading weight (w_psi)
4. Follow-the-Gap: Safety bubble radius (r_bubble), Best point fraction, Disparity threshold
5. Wall Following: Target wall distance (d_desired), Steering gain (Kp), Derivative gain (Kd)

Evaluates on a standardized circuit (Spielberg / Levine) to isolate and understand
WHY parameters affect performance, stability, and lap times.
"""

import os
import sys
import math
import time
import json
import csv
import argparse
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(EXP_DIR, '../..'))
for p in [
    EXP_DIR,
    f'{WS_ROOT}/src/controllers/mpc_controller',
    f'{WS_ROOT}/src/controllers/stanley_controller',
    f'{WS_ROOT}/src/controllers/pure_pursuit',
    WS_ROOT,
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

from controllers.wall_follow_adapter import WallFollowAdapter
from controllers.gap_follow_adapter import FollowTheGapAdapter
from controllers.pure_pursuit_adapter import PurePursuitAdapter
from controllers.stanley_adapter import StanleyAdapter
from controllers.mpc_adapter import MPCAdapter


def evaluate_controller_run(
    controller,
    track_info: TrackInfo,
    target_laps: int = 3,
    control_rate_hz: float = 25.0,
    sim_dt: float = 0.010
) -> Dict[str, Any]:
    """Runs a trial with a customized controller instance and returns performance metrics."""
    start_pose = track_info.start_pose
    map_path = track_info.map_path_no_ext
    track_length = track_info.track_length
    controller.reset()

    est_lap_time = max(18.0, track_length / 3.0)
    max_sim_time = target_laps * est_lap_time * 2.0 + 30.0

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret

    control_interval = int(round((1.0 / control_rate_hz) / sim_dt))
    control_dt = 1.0 / control_rate_hz

    sim_time = 0.0
    total_dist = 0.0
    prev_xy = (start_pose[0], start_pose[1])
    current_steer_cmd = 0.0
    current_speed_cmd = 0.0
    last_steer = 0.0

    completed_laps = 0
    lap_start_time = 0.0
    lap_times = []
    crashed = False
    crash_reason = "None"
    failure_location = None

    all_speeds = []
    all_cross_track_errors = []
    all_steer_rates = []
    steer_sign_reversals = 0
    prev_steer_sign = 0
    min_scans_recorded = []

    step_count = 0
    last_lap_count = int(obs['lap_counts'][0])

    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        current_lap_count = int(obs['lap_counts'][0])

        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)

        is_collision = bool(obs['collisions'][0])
        scans = obs['scans'][0] if len(obs['scans'][0]) > 0 else []
        min_scan = float(np.min(scans)) if len(scans) > 0 else 10.0
        min_scans_recorded.append(min_scan)

        if is_collision or min_scan < 0.12:
            crashed = True
            crash_reason = f"Collision at t={sim_time:.2f}s, min_scan={min_scan:.3f}m"
            failure_location = {"x": round(cur_x, 3), "y": round(cur_y, 3), "distance_m": round(total_dist, 2), "lap": completed_laps + 1}
            break

        if current_lap_count > last_lap_count:
            lap_t = sim_time - lap_start_time
            lap_times.append(round(lap_t, 3))
            completed_laps += 1
            lap_start_time = sim_time
            last_lap_count = current_lap_count
            if completed_laps >= target_laps:
                break

        if step_count % control_interval == 0:
            steer_cmd, speed_cmd, e_ct, e_psi = controller.compute_control(
                cur_x, cur_y, cur_yaw, cur_v, control_dt, scans=scans
            )
            current_steer_cmd = steer_cmd
            current_speed_cmd = speed_cmd

            steer_rate = abs(steer_cmd - last_steer) / control_dt
            all_steer_rates.append(steer_rate)

            cur_sign = 1 if steer_cmd > 0.01 else (-1 if steer_cmd < -0.01 else 0)
            if cur_sign != 0 and prev_steer_sign != 0 and cur_sign != prev_steer_sign:
                steer_sign_reversals += 1
            if cur_sign != 0:
                prev_steer_sign = cur_sign

            last_steer = steer_cmd
            all_speeds.append(cur_v)
            all_cross_track_errors.append(abs(e_ct))

        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        sim_time += sim_dt
        step_count += 1

    completion_rate = round((completed_laps / target_laps) * 100.0, 1)
    mean_speed = round(float(np.mean(all_speeds)), 3) if len(all_speeds) > 0 else 0.0
    max_speed = round(float(np.max(all_speeds)), 3) if len(all_speeds) > 0 else 0.0
    rmse_ct_error = round(float(np.sqrt(np.mean(np.square(all_cross_track_errors)))), 4) if len(all_cross_track_errors) > 0 else 0.0
    max_ct_error = round(float(np.max(all_cross_track_errors)), 4) if len(all_cross_track_errors) > 0 else 0.0
    steer_rate_rms = round(float(np.sqrt(np.mean(np.square(all_steer_rates)))), 4) if len(all_steer_rates) > 0 else 0.0
    min_lidar_distance = round(float(np.min(min_scans_recorded)), 3) if len(min_scans_recorded) > 0 else 0.0

    return {
        "completed_laps": completed_laps,
        "target_laps": target_laps,
        "completion_rate_pct": completion_rate,
        "mean_lap_s": round(float(np.mean(lap_times)), 3) if len(lap_times) > 0 else None,
        "best_lap_s": min(lap_times) if len(lap_times) > 0 else None,
        "avg_speed_mps": mean_speed,
        "max_speed_mps": max_speed,
        "collision_count": 1 if crashed else 0,
        "cross_track_rmse_m": rmse_ct_error,
        "cross_track_max_m": max_ct_error,
        "steer_rate_rms_radps": steer_rate_rms,
        "steer_reversals": steer_sign_reversals,
        "min_lidar_m": min_lidar_distance,
        "crash_reason": crash_reason,
        "failure_location": failure_location
    }


def run_parameter_sweeps(output_dir: str = f'{EXP_DIR}/data/exp1_2_parameters'):
    os.makedirs(output_dir, exist_ok=True)
    track_name = 'Spielberg'
    track_info = TrackManager.load_track(track_name, 'raceline')
    levine_info = TrackManager.load_track('Levine', 'raceline')

    all_sweep_results = []
    summary_rows = []

    print("=== Starting Experiment 1.2: Parameter Sensitivity Analysis ===")

    # -------------------------------------------------------------
    # 1. Pure Pursuit: Lookahead Distance (L_d) Sweep (on Levine)
    # -------------------------------------------------------------
    print("\n--- [1/5] Pure Pursuit: Lookahead Distance Sweep (on Levine) ---")
    pp_lookaheads = [0.60, 0.90, 1.20, 1.60, 2.20]
    for L_d in pp_lookaheads:
        ctrl = PurePursuitAdapter(
            waypoints=levine_info.waypoints,
            headings=levine_info.headings,
            target_speeds=levine_info.target_speeds,
            lookahead_distance=L_d,
            lookahead_gain=0.08,
            min_lookahead=0.50,
            max_lookahead=3.0,
            speed_scale=0.85,
            max_speed=4.20
        )
        res = evaluate_controller_run(ctrl, levine_info, target_laps=3)
        record = {
            "controller": "Pure_Pursuit",
            "parameter": "lookahead_distance",
            "value": L_d,
            "track": "Levine",
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  L_d={L_d:.2f}m -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']} rad/s")


    # -------------------------------------------------------------
    # 2. Stanley: Cross-track Gain (k) and Soft Damping (k_soft)
    # -------------------------------------------------------------
    print("\n--- [2/5] Stanley Controller: Gain (k) & Damping (k_soft) Sweep ---")
    stanley_k_vals = [0.80, 1.50, 2.20, 3.20, 4.50]
    for k in stanley_k_vals:
        ctrl = StanleyAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            kappa=track_info.kappa,
            gain_k=k,
            software_k=0.50,
            gain_ff=0.10,
            speed_scale=0.70,
            max_speed=7.0
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "Stanley_Controller",
            "parameter": "gain_k",
            "value": k,
            "track": track_name,
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  k={k:.2f} -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Reversals: {res['steer_reversals']}")

    stanley_ksoft_vals = [0.10, 0.30, 0.50, 1.20, 2.50]
    for k_soft in stanley_ksoft_vals:
        ctrl = StanleyAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            kappa=track_info.kappa,
            gain_k=2.20,
            software_k=k_soft,
            gain_ff=0.10,
            speed_scale=0.70,
            max_speed=7.0
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "Stanley_Controller",
            "parameter": "software_k",
            "value": k_soft,
            "track": track_name,
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  k_soft={k_soft:.2f} -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Reversals: {res['steer_reversals']}")

    # -------------------------------------------------------------
    # 3. MPC: Horizon (N) & Slew Rate Weight (w_ddelta)
    # -------------------------------------------------------------
    print("\n--- [3/5] MPC Controller: Horizon (N) & Weight (w_ddelta) Sweep ---")
    mpc_horizons = [5, 8, 10, 14, 18]
    for N in mpc_horizons:
        ctrl = MPCAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            s_arr=track_info.s_arr,
            track_length=track_info.track_length,
            N=N,
            dt_mpc=0.08,
            w_x=2.50,
            w_y=2.50,
            w_psi=1.80,
            w_v=0.50,
            w_delta=0.80,
            w_ddelta=2.50,
            speed_scale=0.70,
            max_speed=7.0
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "MPC_Controller",
            "parameter": "horizon_N",
            "value": N,
            "track": track_name,
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  N={N} -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']}")

    mpc_w_ddelta_vals = [0.50, 1.50, 2.50, 6.00, 15.00]
    for w_dd in mpc_w_ddelta_vals:
        ctrl = MPCAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            s_arr=track_info.s_arr,
            track_length=track_info.track_length,
            N=10,
            dt_mpc=0.08,
            w_x=2.50,
            w_y=2.50,
            w_psi=1.80,
            w_v=0.50,
            w_delta=0.80,
            w_ddelta=w_dd,
            speed_scale=0.70,
            max_speed=7.0
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "MPC_Controller",
            "parameter": "w_ddelta",
            "value": w_dd,
            "track": track_name,
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  w_ddelta={w_dd:.1f} -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']}")

    # -------------------------------------------------------------
    # 4. Follow-the-Gap: Bubble Radius & Best Point Fraction
    # -------------------------------------------------------------
    print("\n--- [4/5] Follow-the-Gap: Safety Bubble & Selection Sweep (on Levine) ---")
    ftg_bubbles = [0.15, 0.25, 0.35, 0.45, 0.55]
    for bubble in ftg_bubbles:
        ctrl = FollowTheGapAdapter(
            waypoints=levine_info.waypoints,
            headings=levine_info.headings,
            target_speeds=levine_info.target_speeds,
            safety_margin=0.20,
            bubble_radius=bubble,
            disparity_threshold=0.30,
            best_point_fraction=0.90,
            max_speed=4.00,
            min_speed=1.30
        )
        res = evaluate_controller_run(ctrl, levine_info, target_laps=3)
        record = {
            "controller": "Follow_The_Gap",
            "parameter": "bubble_radius",
            "value": bubble,
            "track": "Levine",
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  bubble_radius={bubble:.2f}m -> {status} | Lap: {res['mean_lap_s']}s | Min LiDAR: {res['min_lidar_m']}m")

    ftg_best_pt_fracs = [0.70, 0.80, 0.90, 0.98]
    for frac in ftg_best_pt_fracs:
        ctrl = FollowTheGapAdapter(
            waypoints=levine_info.waypoints,
            headings=levine_info.headings,
            target_speeds=levine_info.target_speeds,
            safety_margin=0.20,
            bubble_radius=0.32,
            disparity_threshold=0.30,
            best_point_fraction=frac,
            max_speed=4.00,
            min_speed=1.30
        )
        res = evaluate_controller_run(ctrl, levine_info, target_laps=3)
        record = {
            "controller": "Follow_The_Gap",
            "parameter": "best_point_fraction",
            "value": frac,
            "track": "Levine",
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  best_pt_frac={frac:.2f} -> {status} | Lap: {res['mean_lap_s']}s | Steer RMS: {res['steer_rate_rms_radps']}")

    # -------------------------------------------------------------
    # 5. Wall Following: Desired Distance & Gain Kp (on Spielberg)
    # -------------------------------------------------------------
    print("\n--- [5/5] Wall Following: Desired Distance & Gain Kp Sweep (on Spielberg) ---")
    wf_desired_distances = [0.80, 1.00, 1.20, 1.50, 1.80]
    for d_des in wf_desired_distances:
        ctrl = WallFollowAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            desired_distance=d_des,
            lookahead_distance=1.20,
            kp=1.60,
            kd=0.25,
            base_speed=3.20,
            min_speed=0.80
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "Wall_Following",
            "parameter": "desired_distance",
            "value": d_des,
            "track": "Spielberg",
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  d_desired={d_des:.2f}m -> {status} | Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']}")

    wf_kps = [0.80, 1.20, 1.60, 2.40, 3.20]
    for kp in wf_kps:
        ctrl = WallFollowAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            desired_distance=1.20,
            lookahead_distance=1.20,
            kp=kp,
            kd=0.25,
            base_speed=3.20,
            min_speed=0.80
        )
        res = evaluate_controller_run(ctrl, track_info, target_laps=3)
        record = {
            "controller": "Wall_Following",
            "parameter": "kp",
            "value": kp,
            "track": "Spielberg",
            **res
        }
        all_sweep_results.append(record)
        summary_rows.append(record)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f"  kp={kp:.2f} -> {status} | Lap: {res['mean_lap_s']}s | Steer RMS: {res['steer_rate_rms_radps']} rad/s | Reversals: {res['steer_reversals']}")


    # Save Results
    json_path = f"{output_dir}/parameter_sweep_results.json"
    with open(json_path, 'w') as f:
        json.dump(all_sweep_results, f, indent=2)

    csv_path = f"{output_dir}/parameter_sweep_summary.csv"
    if summary_rows:
        keys = list(summary_rows[0].keys())
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(summary_rows)

    print(f"\n=== Parameter Sensitivity Sweep Completed ===")
    print(f"Results saved to: {json_path} and {csv_path}")


if __name__ == '__main__':
    run_parameter_sweeps()
