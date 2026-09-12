#!/usr/bin/env python3
"""
Automated Verification Harness for Adaptive Multi-Regime Hybrid Controller (ARM-HC).
Tests across:
1. Spielberg (High-Speed GP Circuit)
2. Levine (Tight Indoor Corridor Circuit, BEXCO style)
3. Catalunya & Spa (Unseen Generalization Circuits)
4. Wall Jitter Metric Verification (comparing std(d_dot) near walls)
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
    env = _orig_make(id, **kwargs)
    return env.unwrapped
gym.make = _patched_make

import f110_gym
from hybrid_controller.track_manager import TrackManager
from hybrid_controller.hybrid_supervisor import HybridSupervisor, DrivingRegime


def evaluate_hybrid_circuit(
    track_name: str,
    mode: str = 'Q1',
    low_friction_mode: bool = False,
    max_sim_time: float = 140.0,
    mpc_rate_hz: float = 25.0,
    sim_dt: float = 0.01
) -> dict:
    """Evaluates ARM-HC hybrid controller on a given track."""
    track = TrackManager.load_track(
        track_name=track_name,
        waypoint_type='raceline',
        low_friction_mode=low_friction_mode
    )
    track_len = track.track_length
    start_pose = track.start_pose
    map_path_no_ext = track.map_path_no_ext

    # Setup obstacles for Q2 mode (2 static obstacles)
    num_agents = 1
    obs_poses = []
    if mode == 'Q2':
        num_agents = 3
        s_targets = [70.0, 210.0] if 'spielberg' in track_name.lower() else [0.25 * track_len, 0.65 * track_len]
        for s_tar in s_targets:
            idx = int(np.argmin(np.abs(track.s_arr - s_tar)))
            obs_poses.append([
                float(track.waypoints[idx, 0]),
                float(track.waypoints[idx, 1]),
                float(track.headings[idx])
            ])

    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=num_agents)
    if num_agents > 1:
        init_poses = np.array([[start_pose[0], start_pose[1], start_pose[2]]] + obs_poses)
    else:
        init_poses = np.array([[start_pose[0], start_pose[1], start_pose[2]]])
    obs, _, done, _ = env.reset(init_poses)

    supervisor = HybridSupervisor(
        track=track,
        mode=mode,
        low_friction_mode=low_friction_mode,
        mpc_rate_hz=mpc_rate_hz
    )

    control_interval = int(round((1.0 / mpc_rate_hz) / sim_dt))
    sim_time = 0.0
    total_dist = 0.0
    prev_xy = (start_pose[0], start_pose[1])
    current_steer_cmd = 0.0
    current_speed_cmd = 0.0
    last_steer = 0.0

    completed = False
    crashed = False
    crash_reason = "None"
    step_count = 0

    speeds = []
    steer_rates = []
    wall_steer_rates = []  # Specifically when side walls < 0.65m
    ct_errors = []
    regimes_visited = {}

    while sim_time < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))

        is_col = bool(obs['collisions'][0])
        min_scan = float(min(obs['scans'][0])) if len(obs['scans'][0]) > 0 else 10.0

        if is_col or min_scan < 0.12:
            crashed = True
            crash_reason = f"Collision at t={sim_time:.2f}s, dist={total_dist:.1f}m (min_scan={min_scan:.2f}m)"
            break

        if obs['lap_counts'][0] >= 1 or (total_dist > track_len * 0.95 and math.hypot(cur_x - start_pose[0], cur_y - start_pose[1]) < 3.0):
            completed = True
            break

        step_dist = math.hypot(cur_x - prev_xy[0], cur_y - prev_xy[1])
        total_dist += step_dist
        prev_xy = (cur_x, cur_y)
        speeds.append(cur_v)

        if step_count % control_interval == 0:
            out = supervisor.compute_control(
                cur_x, cur_y, cur_yaw, cur_v, scans=obs['scans'][0], sim_time=sim_time
            )
            current_steer_cmd = out.steering
            current_speed_cmd = out.speed

            control_dt = 1.0 / mpc_rate_hz
            steer_rate = abs(out.steering - last_steer) / control_dt
            last_steer = out.steering
            steer_rates.append(steer_rate)

            # Check if wall regime or close side wall
            r_name = out.active_regime.value
            regimes_visited[r_name] = regimes_visited.get(r_name, 0) + 1

            if out.active_regime == DrivingRegime.WALL_DAMPED:
                wall_steer_rates.append(steer_rate)

            ct_errors.append(abs(out.cross_track_err))

        if num_agents > 1:
            action = np.array([[current_steer_cmd, current_speed_cmd]] + [[0.0, 0.0]] * (num_agents - 1))
        else:
            action = np.array([[current_steer_cmd, current_speed_cmd]])

        obs, reward, done, info = env.step(action)
        sim_time += sim_dt
        step_count += 1

    completion_pct = min(100.0, round((total_dist / track_len) * 100.0, 1))
    avg_speed = float(np.mean(speeds)) if speeds else 0.0
    max_speed = float(np.max(speeds)) if speeds else 0.0
    avg_cte = float(np.mean(ct_errors)) if ct_errors else 0.0
    max_cte = float(np.max(ct_errors)) if ct_errors else 0.0
    overall_jitter_std = float(np.std(steer_rates)) if steer_rates else 0.0
    wall_jitter_std = float(np.std(wall_steer_rates)) if wall_steer_rates else 0.0
    wall_mean_rate = float(np.mean(wall_steer_rates)) if wall_steer_rates else 0.0

    return {
        'track_name': track_name,
        'mode': mode,
        'completed': completed,
        'crashed': crashed,
        'lap_time': round(sim_time, 2) if completed else None,
        'completion_pct': completion_pct,
        'avg_speed': round(avg_speed, 2),
        'max_speed': round(max_speed, 2),
        'avg_cte': round(avg_cte, 3),
        'max_cte': round(max_cte, 3),
        'overall_jitter_std': round(overall_jitter_std, 3),
        'wall_jitter_std': round(wall_jitter_std, 3),
        'wall_mean_rate': round(wall_mean_rate, 3),
        'regimes_visited': regimes_visited,
        'crash_reason': crash_reason
    }


def main():
    print("================================================================================")
    print("ROBORACER IFAC 2026: HYBRID CONTROLLER (ARM-HC) VERIFICATION SUITE")
    print("================================================================================\n")

    test_circuits = [
        ('Spielberg', 'Q1', False),     # Primary GP circuit
        ('Levine', 'Q1', True),         # BEXCO-style indoor corridor, low friction urethane mode
        ('Spielberg', 'Q2', False),     # Q2 Obstacle Avoidance (2 static obstacles)
        ('Catalunya', 'Q1', False),     # Unseen Generalization circuit
        ('Spa', 'Q1', False)            # Unseen Generalization circuit (544.5m)
    ]

    for track, mode, low_fric in test_circuits:
        print(f"Testing {track} (Mode={mode}, LowFric={low_fric})...", end="", flush=True)
        t0 = time.time()
        res = evaluate_hybrid_circuit(track, mode=mode, low_friction_mode=low_fric)
        elapsed = time.time() - t0

        status_str = "PASSED" if res['completed'] else f"FAILED ({res['crash_reason']})"
        print(f" {status_str} in {res.get('lap_time', 'N/A')}s (wall: {elapsed:.2f}s, avg_v={res['avg_speed']}m/s, max_v={res['max_speed']}m/s)")
        print(f"   --> Wall Jitter Std: {res['wall_jitter_std']} rad/s | Overall Jitter Std: {res['overall_jitter_std']} rad/s | Avg CTE: {res['avg_cte']}m")
        print(f"   --> Regimes Visited: {res['regimes_visited']}")

    print("\n[SUCCESS] ARM-HC Hybrid Controller Verification Complete!")


if __name__ == '__main__':
    main()
