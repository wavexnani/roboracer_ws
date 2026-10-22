#!/usr/bin/env python3
"""
Phase 1: Controller Benchmarking (Experiment 1.1)

Conducts multi-lap baseline comparison across 5 controllers:
1. Wall Following
2. Follow-the-Gap (FTG)
3. Pure Pursuit
4. Stanley Controller
5. Model Predictive Control (MPC)

Across 3 different map archetypes:
- Levine (Narrow indoor corridor - favors reactive Wall Follow / FTG)
- Monza (Long straights & chicanes - favors Stanley / MPC)
- Spielberg (Balanced GP circuit - neutral benchmark)

Across multi-lap durations:
- 3 consecutive laps
- 5 consecutive laps
- 10 consecutive laps

Records all required metrics:
- Lap completion rate (%)
- Lap times (per-lap and total/mean)
- Average speed and Maximum speed
- Collision count and minimum LiDAR distance
- Steering oscillation / chatter RMS and sign reversal count
- Cross-track error (mean, max, std/RMSE) relative to raceline
- Failure location (x, y, dist) if collision occurs
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

# Ensure repository paths
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


CONTROLLERS = [
    'Wall_Following',
    'Follow_The_Gap',
    'Pure_Pursuit',
    'Stanley_Controller',
    'MPC_Controller'
]

TRACKS = ['Levine', 'Monza', 'Spielberg']
LAP_COUNTS = [3, 5, 10]


def instantiate_controller(controller_name: str, track_info: TrackInfo, track_name: str):
    """Instantiate a controller with standardized parameters for the given track."""
    # Scale speeds sensibly per track
    if track_name == 'Levine':
        max_v = 4.20
        speed_scale = 0.85
    elif track_name == 'Monza':
        max_v = 7.50
        speed_scale = 0.75
    else:  # Spielberg
        max_v = 7.00
        speed_scale = 0.70

    if controller_name == 'Wall_Following':
        return WallFollowAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            desired_distance=0.70 if track_name == 'Levine' else 1.20,
            lookahead_distance=0.80 if track_name == 'Levine' else 1.20,
            kp=1.60,
            kd=0.25,
            base_speed=2.00 if track_name == 'Levine' else 3.20,
            min_speed=0.80
        )
    elif controller_name == 'Follow_The_Gap':
        return FollowTheGapAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            safety_margin=0.20,
            bubble_radius=0.32,
            disparity_threshold=0.30,
            max_range=4.50,
            best_point_fraction=0.90,
            max_speed=min(4.00, max_v),
            min_speed=1.30
        )
    elif controller_name == 'Pure_Pursuit':
        return PurePursuitAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            lookahead_distance=1.20 if track_name == 'Levine' else 1.70,
            lookahead_gain=0.10,
            min_lookahead=0.70 if track_name == 'Levine' else 1.00,
            max_lookahead=2.80,
            speed_scale=speed_scale,
            max_speed=max_v
        )
    elif controller_name == 'Stanley_Controller':
        return StanleyAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            kappa=track_info.kappa,
            gain_k=2.20,
            software_k=0.50,
            gain_ff=0.10,
            lookahead_preview=0.10,
            speed_scale=speed_scale,
            max_speed=max_v
        )
    elif controller_name == 'MPC_Controller':
        return MPCAdapter(
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
            w_ddelta=2.50,
            speed_scale=speed_scale,
            max_speed=max_v
        )
    else:
        raise ValueError(f"Unknown controller: {controller_name}")


def run_multi_lap_trial(
    controller_name: str,
    track_name: str,
    target_laps: int,
    control_rate_hz: float = 25.0,
    sim_dt: float = 0.010,
    save_telemetry: bool = True
) -> Dict[str, Any]:
    """Runs a multi-lap trial and returns comprehensive telemetry and metrics."""
    track_info = TrackManager.load_track(track_name, 'raceline')
    start_pose = track_info.start_pose
    map_path = track_info.map_path_no_ext
    track_length = track_info.track_length

    # Instantiate Controller
    controller = instantiate_controller(controller_name, track_info, track_name)
    controller.reset()

    # Time limit: give ample time per lap plus buffer
    est_lap_time = max(18.0, track_length / 3.0)
    max_sim_time = target_laps * est_lap_time * 2.5 + 30.0

    # Gym Environment Setup
    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret

    control_interval = int(round((1.0 / control_rate_hz) / sim_dt))
    control_dt = 1.0 / control_rate_hz

    # State tracking
    sim_time = 0.0
    total_dist = 0.0
    prev_xy = (start_pose[0], start_pose[1])
    current_steer_cmd = 0.0
    current_speed_cmd = 0.0
    last_steer = 0.0

    completed_laps = 0
    lap_start_time = 0.0
    lap_times = []
    lap_start_dist = 0.0

    crashed = False
    crash_reason = "None"
    failure_location = None

    # Telemetry buffers
    all_speeds = []
    all_cross_track_errors = []
    all_heading_errors = []
    all_steer_rates = []
    min_scans_recorded = []
    steer_sign_reversals = 0
    prev_steer_sign = 0

    step_count = 0
    last_lap_count = int(obs['lap_counts'][0])

    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        current_lap_count = int(obs['lap_counts'][0])

        # Step distance
        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)

        # Collision check
        is_collision = bool(obs['collisions'][0])
        scans = obs['scans'][0] if len(obs['scans'][0]) > 0 else []
        min_scan = float(np.min(scans)) if len(scans) > 0 else 10.0
        min_scans_recorded.append(min_scan)

        if is_collision or min_scan < 0.12:
            crashed = True
            crash_reason = f"Collision at t={sim_time:.2f}s, min_scan={min_scan:.3f}m"
            failure_location = {
                "x": round(cur_x, 3),
                "y": round(cur_y, 3),
                "distance_m": round(total_dist, 2),
                "lap": completed_laps + 1
            }
            break

        # Vehicle stall check
        if step_count > 600 and total_dist < 1.0:
            crashed = True
            crash_reason = "Vehicle stalled at start line"
            failure_location = {"x": round(cur_x, 3), "y": round(cur_y, 3), "distance_m": round(total_dist, 2), "lap": 1}
            break

        # Lap completion detection
        if current_lap_count > last_lap_count:
            lap_t = sim_time - lap_start_time
            lap_times.append(round(lap_t, 3))
            completed_laps += 1
            lap_start_time = sim_time
            lap_start_dist = total_dist
            last_lap_count = current_lap_count

            if completed_laps >= target_laps:
                break

        # Compute Control at control interval
        if step_count % control_interval == 0:
            steer_cmd, speed_cmd, e_ct, e_psi = controller.compute_control(
                cur_x, cur_y, cur_yaw, cur_v, control_dt, scans=scans
            )

            current_steer_cmd = steer_cmd
            current_speed_cmd = speed_cmd

            steer_rate = abs(steer_cmd - last_steer) / control_dt
            all_steer_rates.append(steer_rate)

            # Steering reversal count (sign changes)
            cur_sign = 1 if steer_cmd > 0.01 else (-1 if steer_cmd < -0.01 else 0)
            if cur_sign != 0 and prev_steer_sign != 0 and cur_sign != prev_steer_sign:
                steer_sign_reversals += 1
            if cur_sign != 0:
                prev_steer_sign = cur_sign

            last_steer = steer_cmd
            all_speeds.append(cur_v)
            all_cross_track_errors.append(abs(e_ct))
            all_heading_errors.append(abs(e_psi))

        # Simulator step
        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        sim_time += sim_dt
        step_count += 1

    # Final metric calculations
    completion_rate = round((completed_laps / target_laps) * 100.0, 1)
    mean_speed = round(float(np.mean(all_speeds)), 3) if len(all_speeds) > 0 else 0.0
    max_speed = round(float(np.max(all_speeds)), 3) if len(all_speeds) > 0 else 0.0
    mean_ct_error = round(float(np.mean(all_cross_track_errors)), 4) if len(all_cross_track_errors) > 0 else 0.0
    max_ct_error = round(float(np.max(all_cross_track_errors)), 4) if len(all_cross_track_errors) > 0 else 0.0
    rmse_ct_error = round(float(np.sqrt(np.mean(np.square(all_cross_track_errors)))), 4) if len(all_cross_track_errors) > 0 else 0.0
    steer_rate_rms = round(float(np.sqrt(np.mean(np.square(all_steer_rates)))), 4) if len(all_steer_rates) > 0 else 0.0
    min_lidar_distance = round(float(np.min(min_scans_recorded)), 3) if len(min_scans_recorded) > 0 else 0.0

    result = {
        "controller": controller_name,
        "track": track_name,
        "target_laps": target_laps,
        "completed_laps": completed_laps,
        "completion_rate_pct": completion_rate,
        "total_sim_time_s": round(sim_time, 2),
        "lap_times_s": lap_times,
        "best_lap_s": min(lap_times) if len(lap_times) > 0 else None,
        "mean_lap_s": round(float(np.mean(lap_times)), 3) if len(lap_times) > 0 else None,
        "average_speed_mps": mean_speed,
        "max_speed_mps": max_speed,
        "collision_count": 1 if crashed else 0,
        "min_lidar_distance_m": min_lidar_distance,
        "steering_rate_rms_radps": steer_rate_rms,
        "steering_reversals": steer_sign_reversals,
        "cross_track_mae_m": mean_ct_error,
        "cross_track_max_m": max_ct_error,
        "cross_track_rmse_m": rmse_ct_error,
        "crash_reason": crash_reason,
        "failure_location": failure_location
    }

    return result


def main():
    parser = argparse.ArgumentParser(description="Phase 1: Controller Benchmarking Harness")
    parser.add_argument('--controller', type=str, choices=CONTROLLERS + ['all'], default='all')
    parser.add_argument('--track', type=str, choices=TRACKS + ['all'], default='all')
    parser.add_argument('--laps', type=int, choices=[3, 5, 10, 0], default=0, help="0 means all [3, 5, 10]")
    parser.add_argument('--output-dir', type=str, default=f'{EXP_DIR}/data/exp1_1_baseline')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    controllers_to_run = CONTROLLERS if args.controller == 'all' else [args.controller]
    tracks_to_run = TRACKS if args.track == 'all' else [args.track]
    laps_to_run = LAP_COUNTS if args.laps == 0 else [args.laps]

    print(f"=== Starting Phase 1 Benchmarking ===")
    print(f"Controllers: {controllers_to_run}")
    print(f"Tracks: {tracks_to_run}")
    print(f"Laps: {laps_to_run}")
    print(f"Total Trials: {len(controllers_to_run) * len(tracks_to_run) * len(laps_to_run)}\n")

    json_path = f"{args.output_dir}/baseline_results.json"
    csv_path = f"{args.output_dir}/baseline_summary.csv"
    all_results = []
    summary_rows = []

    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                all_results = json.load(f)
        except Exception:
            all_results = []

    if os.path.exists(csv_path):
        try:
            with open(csv_path, 'r') as f:
                summary_rows = list(csv.DictReader(f))
        except Exception:
            summary_rows = []

    trial_idx = 1
    total_trials = len(controllers_to_run) * len(tracks_to_run) * len(laps_to_run)

    for track in tracks_to_run:
        for ctrl in controllers_to_run:
            for laps in laps_to_run:
                print(f"[{trial_idx}/{total_trials}] Running {ctrl} on {track} ({laps} laps)...")
                t0 = time.time()
                res = run_multi_lap_trial(ctrl, track, laps)
                dt_real = time.time() - t0

                # Update or append in all_results
                matched_json = False
                for i, r in enumerate(all_results):
                    if r.get("controller") == res["controller"] and r.get("track") == res["track"] and r.get("target_laps") == res["target_laps"]:
                        all_results[i] = res
                        matched_json = True
                        break
                if not matched_json:
                    all_results.append(res)

                summary_entry = {
                    "Controller": res["controller"],
                    "Track": res["track"],
                    "Target_Laps": res["target_laps"],
                    "Completed_Laps": res["completed_laps"],
                    "Completion_Rate_%": res["completion_rate_pct"],
                    "Mean_Lap_s": res["mean_lap_s"],
                    "Best_Lap_s": res["best_lap_s"],
                    "Avg_Speed_mps": res["average_speed_mps"],
                    "Max_Speed_mps": res["max_speed_mps"],
                    "Collisions": res["collision_count"],
                    "Min_LiDAR_m": res["min_lidar_distance_m"],
                    "Steer_RMS_radps": res["steering_rate_rms_radps"],
                    "Steer_Reversals": res["steering_reversals"],
                    "CrossTrack_RMSE_m": res["cross_track_rmse_m"],
                    "CrossTrack_Max_m": res["cross_track_max_m"],
                    "Status": "PASS" if res["collision_count"] == 0 and res["completion_rate_pct"] == 100.0 else "FAIL"
                }

                matched_csv = False
                for i, r in enumerate(summary_rows):
                    if r.get("Controller") == res["controller"] and r.get("Track") == res["track"] and int(r.get("Target_Laps", 0)) == res["target_laps"]:
                        summary_rows[i] = summary_entry
                        matched_csv = True
                        break
                if not matched_csv:
                    summary_rows.append(summary_entry)

                status_str = "SUCCESS" if res["collision_count"] == 0 else f"CRASH: {res['crash_reason']}"
                print(f"    --> {status_str} | Laps: {res['completed_laps']}/{res['target_laps']} | "
                      f"Mean Lap: {res['mean_lap_s']}s | Avg Spd: {res['average_speed_mps']} m/s | "
                      f"CT RMSE: {res['cross_track_rmse_m']}m | Wall Time: {dt_real:.1f}s")
                trial_idx += 1

                # Incremental save after each trial
                json_path = f"{args.output_dir}/baseline_results.json"
                with open(json_path, 'w') as f:
                    json.dump(all_results, f, indent=2)

                csv_path = f"{args.output_dir}/baseline_summary.csv"
                if summary_rows:
                    keys = list(summary_rows[0].keys())
                    with open(csv_path, 'w', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=keys)
                        writer.writeheader()
                        writer.writerows(summary_rows)

    print(f"\n=== Benchmarking Completed ===")
    print(f"Results saved to:")
    print(f"  - JSON: {json_path}")
    print(f"  - CSV:  {csv_path}")


if __name__ == '__main__':
    main()
