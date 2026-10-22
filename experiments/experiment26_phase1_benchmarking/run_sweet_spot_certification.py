#!/usr/bin/env python3
"""
Phase 1: Sweet Spot Parameter Certification & Robustness Suite

Runs multi-track verification, fine grid validation, and friction stress testing
to certify the standardized optimal parameters for autonomous racing.
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


def evaluate_trial(
    controller,
    track_info: TrackInfo,
    target_laps: int = 10,
    mu: float = 1.04,
    control_rate_hz: float = 25.0,
    sim_dt: float = 0.010
) -> Dict[str, Any]:
    """Runs a multi-lap validation trial and records comprehensive telemetry."""
    start_pose = track_info.start_pose
    map_path = track_info.map_path_no_ext
    track_length = track_info.track_length
    controller.reset()

    est_lap_time = max(18.0, track_length / 3.0)
    max_sim_time = target_laps * est_lap_time * 2.0 + 40.0

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    # Set tyre friction parameter
    if hasattr(env, 'params') and 'mu' in env.params:
        env.params['mu'] = mu

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

    while sim_time < max_sim_time and completed_laps < target_laps and not crashed:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_theta = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        current_lap_count = int(obs['lap_counts'][0])
        scan = obs['scans'][0] if len(obs['scans'][0]) > 0 else []

        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)

        # Check collision
        is_collision = bool(obs['collisions'][0])
        min_scan = float(np.min(scan)) if len(scan) > 0 else 10.0
        min_scans_recorded.append(min_scan)
        if is_collision or min_scan < 0.120:
            crashed = True
            crash_reason = f"Collision at t={sim_time:.2f}s, min_scan={min_scan:.3f}m"
            failure_location = {
                "x": round(cur_x, 3),
                "y": round(cur_y, 3),
                "distance_m": round(total_dist, 1),
                "lap": completed_laps + 1
            }
            break

        # Vehicle stall check
        if step_count > 600 and total_dist < 1.0:
            crashed = True
            crash_reason = "Vehicle stalled at start line"
            failure_location = {"x": round(cur_x, 3), "y": round(cur_y, 3), "distance_m": round(total_dist, 2), "lap": 1}
            break

        # Native Gym lap completion detection
        if current_lap_count > last_lap_count:
            lap_t = sim_time - lap_start_time
            completed_laps += 1
            lap_times.append(round(lap_t, 3))
            lap_start_time = sim_time
            last_lap_count = current_lap_count
            if completed_laps >= target_laps:
                break

        # Control update
        if step_count % control_interval == 0:
            ctrl_ret = controller.compute_control(
                cur_x, cur_y, cur_theta, cur_v, control_dt, scans=scan
            )
            steer_cmd = float(ctrl_ret[0])
            speed_cmd = float(ctrl_ret[1])
            e_ct = float(ctrl_ret[2])

            current_steer_cmd = steer_cmd
            current_speed_cmd = speed_cmd

            steer_rate = abs(steer_cmd - last_steer) / control_dt
            all_steer_rates.append(steer_rate)

            sign = 1 if steer_cmd > 0.02 else (-1 if steer_cmd < -0.02 else 0)
            if sign != 0 and prev_steer_sign != 0 and sign != prev_steer_sign:
                steer_sign_reversals += 1
            if sign != 0:
                prev_steer_sign = sign

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


def run_certification_suite():
    out_dir = f"{EXP_DIR}/data/exp1_3_sweet_spot"
    os.makedirs(out_dir, exist_ok=True)
    json_path = f"{out_dir}/certification_results.json"
    csv_path = f"{out_dir}/certification_summary.csv"

    # Preload tracks
    tracks = {
        'Levine': TrackManager.load_track('Levine', 'raceline'),
        'Spielberg': TrackManager.load_track('Spielberg', 'raceline'),
        'Monza': TrackManager.load_track('Monza', 'raceline')
    }

    results = []
    summary_rows = []

    def log_and_record(rec):
        results.append(rec)
        summary_rows.append(rec)
        with open(json_path, 'w') as f:
            json.dump(results, f, indent=2)
        if summary_rows:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
                writer.writeheader()
                writer.writerows(summary_rows)

    print("======================================================================")
    print("      PHASE 1: SWEET SPOT PARAMETER CERTIFICATION SUITE               ")
    print("======================================================================")

    # =========================================================================
    # TIER 1: MULTI-TRACK 10-LAP ENDURANCE WITH SWEET SPOT CONFIGURATIONS
    # =========================================================================
    print("\n>>> TIER 1: Multi-Track Generalization & Endurance (10 Laps Each) <<<")

    # 1. MPC Sweet Spot
    for trk_name, trk_info in tracks.items():
        v_max = 4.20 if trk_name == 'Levine' else (7.50 if trk_name == 'Monza' else 7.00)
        scale = 0.85 if trk_name == 'Levine' else (0.75 if trk_name == 'Monza' else 0.70)
        ctrl = MPCAdapter(
            waypoints=trk_info.waypoints, headings=trk_info.headings, target_speeds=trk_info.target_speeds,
            s_arr=trk_info.s_arr, track_length=trk_info.track_length,
            N=10, dt_mpc=0.08, w_x=2.5, w_y=2.5, w_psi=1.8, w_v=0.5, w_delta=0.8, w_ddelta=2.5,
            speed_scale=scale, max_speed=v_max
        )
        print(f"  [MPC] Certifying 10 laps on {trk_name}...", end='', flush=True)
        res = evaluate_trial(ctrl, trk_info, target_laps=10, mu=1.04)
        rec = {"tier": "Tier1_Generalization", "controller": "MPC_Controller", "track": trk_name, "config": "N=10,w_dd=2.5", "mu": 1.04, **res}
        log_and_record(rec)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f" -> {status} | Mean Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']}")

    # 2. Stanley Sweet Spot
    for trk_name, trk_info in tracks.items():
        v_max = 4.20 if trk_name == 'Levine' else (7.50 if trk_name == 'Monza' else 7.00)
        scale = 0.85 if trk_name == 'Levine' else (0.75 if trk_name == 'Monza' else 0.70)
        ctrl = StanleyAdapter(
            waypoints=trk_info.waypoints, headings=trk_info.headings, target_speeds=trk_info.target_speeds,
            kappa=trk_info.kappa, gain_k=2.20, software_k=0.50, gain_ff=0.10, lookahead_preview=0.10,
            speed_scale=scale, max_speed=v_max
        )
        print(f"  [Stanley] Certifying 10 laps on {trk_name}...", end='', flush=True)
        res = evaluate_trial(ctrl, trk_info, target_laps=10, mu=1.04)
        rec = {"tier": "Tier1_Generalization", "controller": "Stanley_Controller", "track": trk_name, "config": "k=2.2,ks=0.5", "mu": 1.04, **res}
        log_and_record(rec)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f" -> {status} | Mean Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m | Reversals: {res['steer_reversals']}")

    # 3. Pure Pursuit (Speed-Adaptive Lookahead Calibration: Ld0=0.80m, kv=0.12)
    for trk_name, trk_info in tracks.items():
        v_max = 4.20 if trk_name == 'Levine' else (7.50 if trk_name == 'Monza' else 7.00)
        scale = 0.85 if trk_name == 'Levine' else (0.75 if trk_name == 'Monza' else 0.70)
        ctrl = PurePursuitAdapter(
            waypoints=trk_info.waypoints, headings=trk_info.headings, target_speeds=trk_info.target_speeds,
            lookahead_distance=0.80, lookahead_gain=0.12, min_lookahead=0.60, max_lookahead=2.20,
            speed_scale=scale, max_speed=v_max
        )
        print(f"  [Pure Pursuit] Certifying 10 laps on {trk_name} (speed-adaptive)...", end='', flush=True)
        res = evaluate_trial(ctrl, trk_info, target_laps=10, mu=1.04)
        rec = {"tier": "Tier1_Generalization", "controller": "Pure_Pursuit", "track": trk_name, "config": "Ld=0.80,kv=0.12", "mu": 1.04, **res}
        log_and_record(rec)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f" -> {status} | Mean Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m")

    # 4. Follow-the-Gap Sweet Spot
    for trk_name, trk_info in tracks.items():
        v_max = 4.00
        ctrl = FollowTheGapAdapter(
            waypoints=trk_info.waypoints, headings=trk_info.headings, target_speeds=trk_info.target_speeds,
            safety_margin=0.20, bubble_radius=0.28, disparity_threshold=0.30, max_range=4.50,
            best_point_fraction=0.90, max_speed=v_max, min_speed=1.30
        )
        print(f"  [Follow-the-Gap] Certifying 10 laps on {trk_name}...", end='', flush=True)
        res = evaluate_trial(ctrl, trk_info, target_laps=10, mu=1.04)
        rec = {"tier": "Tier1_Generalization", "controller": "Follow_The_Gap", "track": trk_name, "config": "rb=0.28,frac=0.90", "mu": 1.04, **res}
        log_and_record(rec)
        status = "PASS" if res["collision_count"] == 0 else "FAIL"
        print(f" -> {status} | Mean Lap: {res['mean_lap_s']}s | CT RMSE: {res['cross_track_rmse_m']}m")

    # =========================================================================
    # TIER 2: FINE GRID NEIGHBORHOOD VALIDATION (5 Laps Each on Spielberg)
    # =========================================================================
    print("\n>>> TIER 2: Fine Grid Neighborhood Validation (on Spielberg) <<<")
    sp_info = tracks['Spielberg']

    # MPC Fine Grid
    for N_cand in [8, 10, 12]:
        for w_cand in [2.0, 4.0]:
            ctrl = MPCAdapter(
                waypoints=sp_info.waypoints, headings=sp_info.headings, target_speeds=sp_info.target_speeds,
                s_arr=sp_info.s_arr, track_length=sp_info.track_length,
                N=N_cand, dt_mpc=0.08, w_x=2.5, w_y=2.5, w_psi=1.8, w_v=0.5, w_delta=0.8, w_ddelta=w_cand,
                speed_scale=0.70, max_speed=7.00
            )
            print(f"  [MPC Grid] N={N_cand}, w_dd={w_cand}...", end='', flush=True)
            res = evaluate_trial(ctrl, sp_info, target_laps=5, mu=1.04)
            rec = {"tier": "Tier2_FineGrid", "controller": "MPC_Controller", "track": "Spielberg", "config": f"N={N_cand},w_dd={w_cand}", "mu": 1.04, **res}
            log_and_record(rec)
            print(f" -> CT RMSE: {res['cross_track_rmse_m']}m | Steer RMS: {res['steer_rate_rms_radps']}")

    # Stanley Fine Grid
    for k_cand in [2.0, 2.6, 3.2]:
        for ks_cand in [0.40, 0.70]:
            ctrl = StanleyAdapter(
                waypoints=sp_info.waypoints, headings=sp_info.headings, target_speeds=sp_info.target_speeds,
                kappa=sp_info.kappa, gain_k=k_cand, software_k=ks_cand, gain_ff=0.10, lookahead_preview=0.10,
                speed_scale=0.70, max_speed=7.00
            )
            print(f"  [Stanley Grid] k={k_cand}, ks={ks_cand}...", end='', flush=True)
            res = evaluate_trial(ctrl, sp_info, target_laps=5, mu=1.04)
            rec = {"tier": "Tier2_FineGrid", "controller": "Stanley_Controller", "track": "Spielberg", "config": f"k={k_cand},ks={ks_cand}", "mu": 1.04, **res}
            log_and_record(rec)
            print(f" -> CT RMSE: {res['cross_track_rmse_m']}m | Reversals: {res['steer_reversals']}")

    # =========================================================================
    # TIER 3: FRICTION STRESS TEST (Low Grip mu=0.80, Nominal mu=1.04, High mu=1.20)
    # =========================================================================
    print("\n>>> TIER 3: Tyre Friction Stress Testing (10 Laps on Spielberg) <<<")
    friction_levels = [0.80, 1.04, 1.20]

    for mu_val in friction_levels:
        # MPC
        ctrl = MPCAdapter(
            waypoints=sp_info.waypoints, headings=sp_info.headings, target_speeds=sp_info.target_speeds,
            s_arr=sp_info.s_arr, track_length=sp_info.track_length,
            N=10, dt_mpc=0.08, w_x=2.5, w_y=2.5, w_psi=1.8, w_v=0.5, w_delta=0.8, w_ddelta=2.5,
            speed_scale=0.70, max_speed=7.00
        )
        print(f"  [MPC Friction] mu={mu_val}...", end='', flush=True)
        res = evaluate_trial(ctrl, sp_info, target_laps=10, mu=mu_val)
        rec = {"tier": "Tier3_FrictionStress", "controller": "MPC_Controller", "track": "Spielberg", "config": "N=10,w_dd=2.5", "mu": mu_val, **res}
        log_and_record(rec)
        print(f" -> CT RMSE: {res['cross_track_rmse_m']}m | Mean Lap: {res['mean_lap_s']}s")

        # Stanley
        ctrl = StanleyAdapter(
            waypoints=sp_info.waypoints, headings=sp_info.headings, target_speeds=sp_info.target_speeds,
            kappa=sp_info.kappa, gain_k=2.20, software_k=0.50, gain_ff=0.10, lookahead_preview=0.10,
            speed_scale=0.70, max_speed=7.00
        )
        print(f"  [Stanley Friction] mu={mu_val}...", end='', flush=True)
        res = evaluate_trial(ctrl, sp_info, target_laps=10, mu=mu_val)
        rec = {"tier": "Tier3_FrictionStress", "controller": "Stanley_Controller", "track": "Spielberg", "config": "k=2.2,ks=0.5", "mu": mu_val, **res}
        log_and_record(rec)
        print(f" -> CT RMSE: {res['cross_track_rmse_m']}m | Mean Lap: {res['mean_lap_s']}s")

    print("\n======================================================================")
    print("      CERTIFICATION SUITE COMPLETED SUCCESSFULLY!                     ")
    print(f"Results saved to: {json_path} and {csv_path}")
    print("======================================================================")


if __name__ == '__main__':
    run_certification_suite()
