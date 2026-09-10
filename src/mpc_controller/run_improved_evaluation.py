#!/usr/bin/env python3
"""
Evaluation Harness for Resolved/Improved MPC Controller across all 22 Raceline Maps.
Includes:
- Realistic steering slew rate constraint (3.2 rad/s)
- Retuned state tracking weights (w_x=10, w_y=10, w_psi=3.5, w_delta=0.15, w_ddelta=0.8, w_v=0.8)
- Automated track obstacle clearance buffer (SDF + iterative centerline corridor projection)
- Dynamic curvature-adaptive speed profiling (lateral acceleration <= 2.8 m/s^2)
"""

import os
import sys
import glob
import yaml
import math
import time
import json
from typing import Tuple, List, Dict, Optional
from PIL import Image
import numpy as np

# Add package paths
for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import gym
import f110_gym
from mpc_controller.track_manager import TrackManager, TrackInfo
from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig


def evaluate_improved_map(
    track_name: str,
    speed_scale: float = 0.75,
    max_sim_time: Optional[float] = None,
    mpc_rate_hz: float = 25.0,
    sim_dt: float = 0.01
) -> dict:
    """Evaluates resolved MPC controller on a racetrack map."""
    track = TrackManager.load_track(track_name, waypoint_type='raceline', lat_accel_max=2.8)
    waypoints = np.copy(track.waypoints)
    path_headings = np.copy(track.headings)
    s_arr = np.copy(track.s_arr)
    scaled_speeds = np.copy(track.target_speeds) * speed_scale
    track_length = track.track_length
    start_pose = track.start_pose
    map_path_no_ext = track.map_path_no_ext
    
    if max_sim_time is None:
        max_sim_time = max(180.0, track_length * 0.70 + 40.0)
    
    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=1)
    obs, _, done, _ = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    
    # Tuned Balanced MPC optimizer (anti-shaking)
    cfg = MPCConfig(
        max_steer_rate=2.8,
        w_x=8.0,
        w_y=8.0,
        w_psi=3.0,
        w_delta=0.25,
        w_ddelta=1.5,
        w_v=0.8
    )
    optimizer = MPCOptimizer(cfg)
    num_pts = len(waypoints)
    
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
    steer_history = []
    
    total_dist_traveled = 0.0
    prev_xy = (obs['poses_x'][0], obs['poses_y'][0])
    current_steer_cmd = 0.0
    filtered_steer = 0.0
    steer_ema_alpha = 0.90
    current_speed_cmd = 0.0
    step_count = 0
    
    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = (float(obs['poses_theta'][0]) + math.pi) % (2.0 * math.pi) - math.pi
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        
        is_collision = bool(obs['collisions'][0])
        min_scan = float(min(obs['scans'][0]))
        if is_collision or min_scan < 0.15:
            crashed = True
            closest_pt = int(np.argmin(np.hypot(waypoints[:, 0] - cur_x, waypoints[:, 1] - cur_y)))
            crash_s = float(s_arr[closest_pt])
            crash_loc = [round(cur_x, 2), round(cur_y, 2), round(crash_s, 1)]
            crash_reason = f"Collision at s={crash_s:.1f}m (scan={min_scan:.2f}m, v={cur_v:.1f}m/s)"
            break
            
        if obs['lap_counts'][0] >= 1 or (total_dist_traveled > track_length * 0.95 and math.hypot(cur_x - waypoints[0, 0], cur_y - waypoints[0, 1]) < 3.0):
            completed = True
            break
            
        d_step = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist_traveled += d_step
        prev_xy = (cur_x, cur_y)
        speeds.append(cur_v)
        
        if step_count > 500 and total_dist_traveled < 1.0:
            crashed = True
            crash_reason = "Vehicle stalled at start line"
            break
            
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
            dist_sq = (cand_pts[:, 0] - cur_x)**2 + (cand_pts[:, 1] - cur_y)**2
            dpsi = (path_headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            forward_mask = np.cos(dpsi) > 0.0
            
            if np.any(forward_mask):
                best_local = int(np.argmin(np.where(forward_mask, dist_sq, np.inf)))
            else:
                best_local = int(np.argmin(dist_sq))
                
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
            
            ref_horizon = np.zeros((cfg.N + 1, 4))
            cur_s = s_arr[closest_idx]
            speed_est = max(1.5, cur_v)
            for k in range(cfg.N + 1):
                s_target = (cur_s + k * speed_est * cfg.dt) % track_length
                idx_k = int(np.argmin(np.abs(s_arr - s_target)))
                ref_horizon[k] = [waypoints[idx_k, 0], waypoints[idx_k, 1], path_headings[idx_k], scaled_speeds[idx_k]]
                
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
                
                steer_ratio = abs(steer_cmd) / max(cfg.max_steer, 1e-3)
                if steer_ratio > 0.45:
                    target_v *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)
                if abs(heading_err) > 0.35:
                    target_v *= 0.85
                    
            target_v = float(np.clip(target_v, 0.5, scaled_speeds[closest_idx]))
            current_steer_cmd = steer_cmd
            current_speed_cmd = target_v
            steer_history.append(current_steer_cmd)
            
        obs, _, done, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        sim_time += sim_dt
        step_count += 1
        
    completion_pct = min(100.0, (total_dist_traveled / track_length) * 100.0)
    avg_speed = float(np.mean(speeds)) if speeds else 0.0
    max_ct = float(np.max(ct_errors)) if ct_errors else 0.0
    avg_ct = float(np.mean(ct_errors)) if ct_errors else 0.0
    
    # Steering smoothness telemetry
    steer_diffs = np.abs(np.diff(steer_history)) if len(steer_history) > 1 else np.array([0.0])
    mean_steer_rate = float(np.mean(steer_diffs) * mpc_rate_hz)
    max_steer_rate = float(np.max(steer_diffs) * mpc_rate_hz)
    steer_std = float(np.std(steer_history)) if steer_history else 0.0
    
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
        'mean_steer_rate': round(mean_steer_rate, 3),
        'max_steer_rate': round(max_steer_rate, 3),
        'steer_std': round(steer_std, 3),
        'crash_reason': crash_reason,
        'crash_location': crash_loc
    }


if __name__ == '__main__':
    all_tracks = sorted([
        'Austin', 'BrandsHatch', 'Budapest', 'Catalunya', 'Hockenheim',
        'IMS', 'Levine', 'Melbourne', 'Mexico City', 'Monza',
        'MoscowRaceway', 'Nuerburgring', 'Oschersleben', 'Sakhir', 'SaoPaulo',
        'Sepang', 'Silverstone', 'Sochi', 'Spa', 'Spielberg',
        'YasMarina', 'Zandvoort'
    ])
    
    tracks_to_test = sys.argv[1:] if len(sys.argv) > 1 and sys.argv[1] != '--all' else all_tracks
    
    print(f"============================================================")
    print(f"RUNNING RESOLVED CONTROLLER BENCHMARK ACROSS {len(tracks_to_test)} RACELINE MAPS")
    print(f"============================================================")
    
    results = []
    completions = 0
    crashes = 0
    out_files = [
        '/sim_ws/src/mpc_controller/resolved_results.json',
        '/home/yeswanth/roboracer_ws/src/mpc_controller/resolved_results.json',
        '/sim_ws/src/mpc_controller/smooth_results.json',
        '/home/yeswanth/roboracer_ws/src/mpc_controller/smooth_results.json'
    ]
    
    for idx, tname in enumerate(tracks_to_test, 1):
        t0 = time.time()
        try:
            res = evaluate_improved_map(tname, speed_scale=0.75)
            elapsed = time.time() - t0
            status_str = "PASSED" if res['completed'] else ("CRASHED" if res['crashed'] else "TIMEOUT")
            print(f"[{idx:02d}/{len(tracks_to_test):02d}] {tname:15s} | {status_str:8s} | Progress: {res['completion_pct']:5.1f}% | Lap: {res['lap_time']}s | CTE: {res['avg_crosstrack_err']:.2f}m | Steer Rate: {res.get('mean_steer_rate', 0.0):.2f}/{res.get('max_steer_rate', 0.0):.2f} rad/s ({elapsed:.1f}s)")
            if res['completed']:
                completions += 1
            else:
                crashes += 1
                if res['crashed']:
                    print(f"       -> Crash: {res['crash_reason']} at {res['crash_location']}")
            results.append(res)
        except Exception as e:
            print(f"[{idx:02d}/{len(tracks_to_test):02d}] {tname:15s} | ERROR: {e}")
            results.append({'track_name': tname, 'completed': False, 'crashed': True, 'error': str(e)})
            crashes += 1
            
        # Save incrementally
        for of in out_files:
            try:
                with open(of, 'w') as f:
                    json.dump(results, f, indent=2)
            except Exception:
                pass
                
    print(f"\n============================================================")
    print(f"RESOLVED SUMMARY: {completions}/{len(tracks_to_test)} Completed | {crashes}/{len(tracks_to_test)} Failed")
    print(f"============================================================")
