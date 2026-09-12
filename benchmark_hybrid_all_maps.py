#!/usr/bin/env python3
"""
Benchmark ARM-HC Hybrid Controller across all F1TENTH circuits:
Primary: Spielberg, Austin, Monza, BrandsHatch, Silverstone
Generalization (Unseen): Catalunya, Spa
Corridor: Levine

Matches exact telemetry and metrics structure as benchmark_results.json.
"""

import os
import sys
import math
import time
import json
import warnings
import numpy as np

warnings.filterwarnings('ignore')

WS_ROOT = '/home/yeswanth/roboracer_ws'
for p in [f'{WS_ROOT}/install/hybrid_controller/lib/python3.12/site-packages', f'{WS_ROOT}/src/hybrid_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import gym
_orig_make = gym.make
def _patched_make(id, **kwargs):
    kwargs['disable_env_checker'] = True
    return _orig_make(id, **kwargs).unwrapped
gym.make = _patched_make
import f110_gym

from hybrid_controller.track_manager import TrackManager
from hybrid_controller.hybrid_supervisor import HybridSupervisor, DrivingRegime


def run_hybrid_benchmark():
    tracks = [
        ('Spielberg', False, 140.0),
        ('Austin', False, 180.0),
        ('Monza', False, 160.0),
        ('BrandsHatch', False, 140.0),
        ('Silverstone', False, 180.0),
        ('Catalunya', False, 160.0),
        ('Spa', False, 220.0),
        ('Levine', True, 60.0)
    ]

    all_results = []
    all_telemetry = {}

    sim_dt = 0.01
    mpc_rate_hz = 25.0
    control_interval = int(round((1.0 / mpc_rate_hz) / sim_dt))
    control_dt = 1.0 / mpc_rate_hz

    for track_name, low_fric, max_sim_time in tracks:
        print(f"--> Benchmarking Hybrid Controller on '{track_name}' (LowFric={low_fric})...", flush=True)
        track = TrackManager.load_track(track_name, waypoint_type='raceline', low_friction_mode=low_fric)
        track_len = track.track_length
        start_pose = track.start_pose
        num_pts = len(track.waypoints)

        env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
        obs, _, done, _ = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))

        supervisor = HybridSupervisor(track=track, mode='Q1', low_friction_mode=low_fric, mpc_rate_hz=mpc_rate_hz)

        sim_time = 0.0
        total_dist = 0.0
        prev_xy = (start_pose[0], start_pose[1])
        cur_steer = 0.0
        cur_speed = 0.0
        last_steer = 0.0

        completed = False
        crashed = False
        crash_reason = "None"
        crash_loc = [0.0, 0.0, 0.0]
        step_count = 0

        # Telemetry
        times, xs, ys, yaws, speeds, steers, steer_rates = [], [], [], [], [], [], []
        ct_errors, head_errors, regimes = [], [], []
        sharp_ctes, sharp_speeds = [], []
        smooth_ctes, smooth_speeds = [], []
        wall_steer_rates = []
        regimes_count = {}

        while sim_time < max_sim_time:
            cur_x = float(obs['poses_x'][0])
            cur_y = float(obs['poses_y'][0])
            cur_yaw = float(obs['poses_theta'][0])
            cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))

            is_col = bool(obs['collisions'][0])
            min_scan = float(min(obs['scans'][0])) if len(obs['scans'][0]) > 0 else 10.0

            if is_col or min_scan < 0.12:
                crashed = True
                crash_reason = f"Wall collision at t={sim_time:.2f}s, dist={total_dist:.1f}m (min_scan={min_scan:.2f}m)"
                crash_loc = [cur_x, cur_y, cur_yaw]
                break

            if obs['lap_counts'][0] >= 1 or (total_dist > track_len * 0.95 and math.hypot(cur_x - start_pose[0], cur_y - start_pose[1]) < 3.0):
                completed = True
                break

            step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
            total_dist += step_dist
            prev_xy = (cur_x, cur_y)

            # Control tick
            if step_count % control_interval == 0:
                out = supervisor.compute_control(
                    cur_x, cur_y, cur_yaw, cur_v, scans=obs['scans'][0], sim_time=sim_time
                )
                cur_steer = out.steering
                cur_speed = out.speed

                s_rate = abs(cur_steer - last_steer) / control_dt
                last_steer = cur_steer
                steer_rates.append(s_rate)

                r_str = out.active_regime.value
                regimes_count[r_str] = regimes_count.get(r_str, 0) + 1

                if out.active_regime == DrivingRegime.WALL_DAMPED:
                    wall_steer_rates.append(s_rate)

                # Segment into sharp (|kappa| >= 0.15) vs smooth (|kappa| < 0.05)
                c_idx = supervisor.last_idx
                k_val = abs(float(track.kappa[c_idx])) if track.kappa is not None else 0.0
                if k_val >= 0.15:
                    sharp_ctes.append(abs(out.cross_track_err))
                    sharp_speeds.append(cur_v)
                elif k_val < 0.05:
                    smooth_ctes.append(abs(out.cross_track_err))
                    smooth_speeds.append(cur_v)

                # Downsampled telemetry logging (25 Hz)
                times.append(round(sim_time, 3))
                xs.append(round(cur_x, 3))
                ys.append(round(cur_y, 3))
                yaws.append(round(cur_yaw, 4))
                speeds.append(round(cur_v, 3))
                steers.append(round(cur_steer, 4))
                ct_errors.append(round(out.cross_track_err, 4))
                head_errors.append(round(out.heading_err, 4))
                regimes.append(r_str)

            obs, reward, done, info = env.step(np.array([[cur_steer, cur_speed]]))
            sim_time += sim_dt
            step_count += 1

        completion_pct = min(100.0, round((total_dist / track_len) * 100.0, 1))
        avg_speed = float(np.mean(speeds)) if speeds else 0.0
        max_speed = float(np.max(speeds)) if speeds else 0.0
        avg_cte = float(np.mean(np.abs(ct_errors))) if ct_errors else 0.0
        max_cte = float(np.max(np.abs(ct_errors))) if ct_errors else 0.0
        overall_jitter_std = float(np.std(steer_rates)) if steer_rates else 0.0
        mean_steer_rate = float(np.mean(steer_rates)) if steer_rates else 0.0
        wall_jitter_std = float(np.std(wall_steer_rates)) if wall_steer_rates else 0.0

        sharp_avg_cte = float(np.mean(sharp_ctes)) if sharp_ctes else 0.0
        sharp_avg_spd = float(np.mean(sharp_speeds)) if sharp_speeds else 0.0
        smooth_avg_cte = float(np.mean(smooth_ctes)) if smooth_ctes else 0.0
        smooth_avg_spd = float(np.mean(smooth_speeds)) if smooth_speeds else 0.0

        res_item = {
            "controller": "Hybrid_Controller",
            "track_name": track_name,
            "completed": completed,
            "crashed": crashed,
            "interventions": 0,
            "lap_time": round(sim_time, 2) if completed else None,
            "completion_pct": completion_pct,
            "distance_traveled": round(total_dist, 1),
            "track_length": round(track_len, 1),
            "avg_speed": round(avg_speed, 2),
            "max_speed": round(max_speed, 2),
            "avg_cte": round(avg_cte, 3),
            "max_cte": round(max_cte, 3),
            "steer_jitter_std": round(overall_jitter_std, 3),
            "mean_steer_rate": round(mean_steer_rate, 3),
            "wall_jitter_std": round(wall_jitter_std, 3),
            "sharp_curve_avg_cte": round(sharp_avg_cte, 3),
            "sharp_curve_avg_speed": round(sharp_avg_spd, 2),
            "smooth_curve_avg_cte": round(smooth_avg_cte, 3),
            "smooth_curve_avg_speed": round(smooth_avg_spd, 2),
            "regimes_visited": regimes_count,
            "crash_reason": crash_reason,
            "crash_location": crash_loc
        }
        all_results.append(res_item)
        all_telemetry[track_name] = {
            "time": times,
            "x": xs,
            "y": ys,
            "yaw": yaws,
            "speed": speeds,
            "steer": steers,
            "cte": ct_errors,
            "heading_err": head_errors,
            "regimes": regimes
        }

        status_str = "COMPLETED" if completed else f"CRASH ({crash_reason})"
        print(f"    --> {status_str} in {res_item['lap_time']}s | Avg V: {avg_speed:.2f} m/s | Avg CTE: {avg_cte:.3f}m | Jitter Std: {overall_jitter_std:.3f} rad/s")

    with open('benchmark_hybrid_results.json', 'w') as f:
        json.dump(all_results, f, indent=2)

    with open('benchmark_hybrid_telemetry.json', 'w') as f:
        json.dump(all_telemetry, f, indent=2)

    print("\n[SUCCESS] Completed All-Map Benchmark for Hybrid Controller!")


if __name__ == '__main__':
    run_hybrid_benchmark()
