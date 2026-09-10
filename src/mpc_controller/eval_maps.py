#!/usr/bin/env python3
"""
Comprehensive Batch Evaluation Harness for Testing MPC Controller across all F1TENTH maps.
"""

import os
import sys
import math
import time
import json
import glob
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Tuple
import numpy as np

# Add package paths
for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import gym
import f110_gym
from mpc_controller.track_manager import TrackManager, TrackInfo
from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult


def get_all_raceline_maps() -> List[str]:
    """Returns sorted list of map names that have a raceline.csv file."""
    racetracks_root = TrackManager.get_racetracks_root()
    all_tracks = TrackManager.list_available_tracks()
    valid_tracks = []
    for t in all_tracks:
        tdir = os.path.join(racetracks_root, t)
        rl = glob.glob(os.path.join(tdir, '*raceline.csv'))
        if rl:
            valid_tracks.append(t)
    return sorted(valid_tracks)


def evaluate_single_map(
    track_name: str,
    speed_scale: float = 0.60,
    max_sim_time: float = 90.0,
    mpc_rate_hz: float = 25.0,
    sim_dt: float = 0.01
) -> dict:
    """Evaluates the MPC controller on a single track map."""
    track = TrackManager.load_track(track_name, waypoint_type='raceline')
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    
    start_pose = np.array([[track.start_pose[0], track.start_pose[1], track.start_pose[2]]])
    obs, _, done, _ = env.reset(start_pose)
    
    cfg = MPCConfig()
    optimizer = MPCOptimizer(cfg)
    
    scaled_speeds = track.target_speeds * speed_scale
    waypoints = track.waypoints
    path_headings = track.headings
    s_arr = track.s_arr
    num_pts = len(waypoints)
    track_length = track.track_length
    
    last_idx = 0
    last_control = (0.0, 0.0)
    last_pos = None
    
    sim_time = 0.0
    crashed = False
    completed = False
    crash_reason = "None"
    crash_loc = [0.0, 0.0, 0.0]
    
    sim_steps_per_mpc = max(1, int(round((1.0 / mpc_rate_hz) / sim_dt)))
    
    ct_errors = []
    speeds = []
    
    total_dist_traveled = 0.0
    prev_xy = (obs['poses_x'][0], obs['poses_y'][0])
    
    current_steer_cmd = 0.0
    current_speed_cmd = 0.0
    step_count = 0
    
    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        
        # Check collision
        is_collision = bool(obs['collisions'][0])
        min_scan = float(min(obs['scans'][0]))
        if is_collision or min_scan < 0.15:
            crashed = True
            closest_pt = int(np.argmin(np.hypot(waypoints[:, 0] - cur_x, waypoints[:, 1] - cur_y)))
            crash_s = float(s_arr[closest_pt])
            crash_loc = [round(cur_x, 2), round(cur_y, 2), round(crash_s, 1)]
            
            steer_err = abs(current_steer_cmd) / cfg.max_steer
            head_err = abs((path_headings[closest_pt] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi)
            if head_err > math.radians(45):
                crash_reason = f"Spinout/Misalignment (yaw_err={math.degrees(head_err):.1f}deg, steer={math.degrees(current_steer_cmd):.1f}deg, kappa={track.kappa[closest_pt]:.2f})"
            elif steer_err > 0.85 and cur_v > 3.0:
                crash_reason = f"Understeer into wall (steer saturated {math.degrees(current_steer_cmd):.1f}deg, v={cur_v:.1f}m/s, kappa={track.kappa[closest_pt]:.2f})"
            elif min_scan < 0.15:
                crash_reason = f"Track Boundary Impact (scan={min_scan:.2f}m, dist_to_ref={ct_errors[-1] if ct_errors else 0:.2f}m, kappa={track.kappa[closest_pt]:.2f})"
            else:
                crash_reason = f"Collision at s={crash_s:.1f}m (v={cur_v:.1f}m/s)"
            break
        
        # Check lap completion
        if obs['lap_counts'][0] >= 1 or (total_dist_traveled > track_length * 0.95 and math.hypot(cur_x - waypoints[0, 0], cur_y - waypoints[0, 1]) < 2.0):
            completed = True
            break
        
        # Distance accumulation
        d_step = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist_traveled += d_step
        prev_xy = (cur_x, cur_y)
        speeds.append(cur_v)
        
        # Stalling check
        if step_count > 500 and total_dist_traveled < 1.0:
            crashed = True
            crash_reason = "Vehicle stalled at start line (zero speed)"
            break
            
        # Run MPC at mpc_rate_hz
        if step_count % sim_steps_per_mpc == 0:
            pos_jump = False
            if last_pos is not None:
                if math.hypot(cur_x - last_pos[0], cur_y - last_pos[1]) > 3.0:
                    pos_jump = True
            last_pos = (cur_x, cur_y)
            
            window_backward = 20
            window_forward = 100
            search_indices = (np.arange(-window_backward, window_forward) + last_idx) % num_pts
            cand_pts = waypoints[search_indices]
            dx = cand_pts[:, 0] - cur_x
            dy = cand_pts[:, 1] - cur_y
            dist_sq = dx * dx + dy * dy
            dpsi = (path_headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            cost = dist_sq + 3.0 * (dpsi ** 2)
            
            best_local = int(np.argmin(cost))
            closest_idx = int(search_indices[best_local])
            min_dist = math.sqrt(dist_sq[best_local])
            
            if min_dist > 3.0 or pos_jump:
                all_dx = waypoints[:, 0] - cur_x
                all_dy = waypoints[:, 1] - cur_y
                all_dist_sq = all_dx * all_dx + all_dy * all_dy
                all_dpsi = (path_headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
                valid_mask = np.cos(all_dpsi) > 0.0
                if np.any(valid_mask):
                    all_cost = np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)
                    closest_idx = int(np.argmin(all_cost))
                else:
                    closest_idx = int(np.argmin(all_dist_sq))
            
            last_idx = closest_idx
            ct_errors.append(min_dist)
            
            # Reference horizon
            ref_horizon = np.zeros((cfg.N + 1, 4))
            cur_s = s_arr[closest_idx]
            speed_est = max(1.5, cur_v)
            
            for k in range(cfg.N + 1):
                s_target = (cur_s + k * speed_est * cfg.dt) % track_length
                s_diff = np.abs(s_arr - s_target)
                idx_k = int(np.argmin(s_diff))
                ref_horizon[k, 0] = waypoints[idx_k, 0]
                ref_horizon[k, 1] = waypoints[idx_k, 1]
                ref_horizon[k, 2] = path_headings[idx_k]
                ref_horizon[k, 3] = scaled_speeds[idx_k]
            
            heading_err = (path_headings[closest_idx] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            wrong_way = abs(heading_err) > math.radians(85)
            
            if wrong_way:
                steer_cmd = float(np.clip(heading_err, -cfg.max_steer, cfg.max_steer))
                target_v = 1.0
                last_control = (0.0, steer_cmd)
            else:
                res = optimizer.solve(
                    current_state=np.array([cur_x, cur_y, cur_yaw, cur_v]),
                    ref_trajectory=ref_horizon,
                    prev_control=last_control
                )
                steer_cmd = float(res.steering)
                target_v = float(res.target_speed)
                last_control = (res.accel, res.steering)
                
                # Corner speed modulation
                steer_ratio = abs(steer_cmd) / max(cfg.max_steer, 1e-3)
                if steer_ratio > 0.45:
                    target_v *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)
                if abs(heading_err) > 0.35:
                    target_v *= 0.85
            
            target_v = float(np.clip(target_v, 0.5, scaled_speeds[closest_idx]))
            current_steer_cmd = steer_cmd
            current_speed_cmd = target_v
            
        obs, _, done, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        sim_time += sim_dt
        step_count += 1
    
    completion_pct = min(100.0, (total_dist_traveled / track_length) * 100.0)
    avg_speed = float(np.mean(speeds)) if speeds else 0.0
    max_ct = float(np.max(ct_errors)) if ct_errors else 0.0
    avg_ct = float(np.mean(ct_errors)) if ct_errors else 0.0
    
    return {
        'track_name': track_name,
        'completed': completed,
        'crashed': crashed,
        'crash_time': round(sim_time if crashed else 0.0, 2),
        'lap_time': round(sim_time if completed else 0.0, 2),
        'distance_traveled': round(total_dist_traveled, 1),
        'track_length': round(track_length, 1),
        'completion_pct': round(completion_pct, 1),
        'avg_speed': round(avg_speed, 2),
        'max_crosstrack_err': round(max_ct, 2),
        'avg_crosstrack_err': round(avg_ct, 2),
        'crash_reason': crash_reason,
        'crash_location': crash_loc
    }


if __name__ == '__main__':
    all_tracks = get_all_raceline_maps()
    if len(sys.argv) > 1 and sys.argv[1] == '--all':
        tracks_to_test = all_tracks
    elif len(sys.argv) > 1:
        tracks_to_test = sys.argv[1:]
    else:
        tracks_to_test = all_tracks

    print(f"============================================================")
    print(f"RUNNING BASELINE EVALUATION ACROSS {len(tracks_to_test)} RACELINE MAPS")
    print(f"============================================================")
    
    results = []
    crashes = 0
    completions = 0
    
    out_file = '/sim_ws/src/mpc_controller/baseline_results.json'
    
    for idx, tname in enumerate(tracks_to_test, 1):
        t0 = time.time()
        try:
            res = evaluate_single_map(tname, speed_scale=0.60, max_sim_time=80.0)
            elapsed = time.time() - t0
            status_str = "PASSED" if res['completed'] else ("CRASHED" if res['crashed'] else "TIMEOUT")
            print(f"[{idx:02d}/{len(tracks_to_test):02d}] {tname:15s} | {status_str:8s} | Progress: {res['completion_pct']:5.1f}% ({res['distance_traveled']:.1f}/{res['track_length']:.1f}m) | Time: {res['lap_time'] or res['crash_time']}s (eval {elapsed:.1f}s)")
            if res['crashed']:
                crashes += 1
                print(f"       -> Crash Reason: {res['crash_reason']}")
                print(f"       -> Location: s={res['crash_location'][2]}m (x={res['crash_location'][0]}, y={res['crash_location'][1]})")
            elif res['completed']:
                completions += 1
            results.append(res)
        except Exception as e:
            print(f"[{idx:02d}/{len(tracks_to_test):02d}] {tname:15s} | ERROR: {e}")
            results.append({'track_name': tname, 'error': str(e), 'completed': False, 'crashed': True, 'crash_reason': f'Exception: {e}'})
            crashes += 1

    with open(out_file, 'w') as f:
        json.dump(results, f, indent=2)
        
    print(f"\n============================================================")
    print(f"BASELINE SUMMARY: {completions}/{len(tracks_to_test)} Completed | {crashes}/{len(tracks_to_test)} Crashed/Failed")
    print(f"Results saved to {out_file}")
    print(f"============================================================")
