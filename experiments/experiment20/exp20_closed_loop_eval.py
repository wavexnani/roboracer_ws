#!/usr/bin/env python3
"""
Experiment 20: Closed Loop Validation on Spielberg Track.
Measures all 13 metrics across Policy A (Raw MPC), Policy B (Actuator-Aware Reachability),
and Policy C (Lattice Schmitt Hysteresis) against Baseline.
"""

import os
import sys
import time
import math
import json
import numpy as np

for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws']:
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
from mpc_controller.track_manager import TrackManager
from diagnose_mpc_jitter import DiagnosticMPCController, wrap_angle

SV_MAX = 3.2
DT_SIM = 0.010
DT_CTRL = 0.040
QUANTUM = SV_MAX * DT_SIM # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

def run_simulation(policy_name, duration_s=26.0):
    track = TrackManager.load_track('Spielberg')
    start_pose = track.start_pose
    map_path_no_ext = track.map_path_no_ext

    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret
    agent = env.sim.agents[0]

    ctrl = DiagnosticMPCController(
        track, speed_scale=1.0, straight_w_psi=2.0, actuator_delay_s=0.020, model_type="yaw_first_order"
    )

    max_steps = int(round(duration_s / DT_SIM))
    control_interval = 4

    current_steer_cmd = 0.0
    current_speed_cmd = 7.50
    prev_published_cmd = 0.0
    schmitt_state = 0.0

    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    act_deltas = []
    steer_rates = []
    yaw_rates = []
    lat_errs = []
    head_errs = []
    solve_times = []
    min_lidar_scans = []
    collisions = 0

    for step_i in range(max_steps):
        t_sim = step_i * DT_SIM

        # Control update at 25 Hz
        if step_i % control_interval == 0:
            cur_x = float(agent.state[0])
            cur_y = float(agent.state[1])
            cur_yaw = float(agent.state[4])
            cur_v = float(agent.state[3])
            cur_r = float(agent.state[5])

            t_solve_start = time.perf_counter()
            steer_final, speed_final, telem = ctrl.step(
                cur_x, cur_y, cur_yaw, cur_v, dt_actual=DT_CTRL, cur_r=cur_r
            )
            solve_times.append(time.perf_counter() - t_solve_start)

            raw = float(telem['opt_raw_steer'])
            raw_cmds.append(raw)
            lat_errs.append(abs(telem['lateral_error']))
            head_errs.append(abs(telem['heading_error']))

            # Reconstruct FIFO state [b0, b1]
            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) == 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) == 2 else 0.0
            cur_delta = float(agent.state[2])

            # Apply Policies
            if policy_name == 'BASELINE':
                cmd = steer_final

            elif policy_name == 'POLICY_A_RAW':
                # Direct raw MPC
                cmd = raw

            elif policy_name == 'POLICY_B_REACHABLE':
                # Actuator-State-Aware Reachability Policy:
                # 1. Predict actual steering after in-flight FIFO commands clear (20 ms = 2 steps)
                # Step 1: executes b1
                diff1 = b1 - cur_delta
                d_10 = cur_delta + (math.copysign(QUANTUM, diff1) if abs(diff1) > EPS_RELAY else 0.0)
                # Step 2: executes b0
                diff2 = b0 - d_10
                d_20 = d_10 + (math.copysign(QUANTUM, diff2) if abs(diff2) > EPS_RELAY else 0.0)

                # 2. Desired steering from MPC is raw
                # Error between desired steering and settled arrival state:
                err_arr = raw - d_20

                # 3. Decision logic:
                # If error is smaller than half a quantum (16 mrad), commanding a move will
                # force a 32 mrad kick and immediate reversal.
                # In that case, lock target to d_20 (stationary hold, zero chatter).
                # If error >= 16 mrad, move to the nearest reachable lattice point.
                if abs(err_arr) < QUANTUM * 0.50:
                    cmd = d_20
                else:
                    cmd = round(raw / QUANTUM) * QUANTUM

            elif policy_name == 'POLICY_C_SCHMITT':
                # True Schmitt Hysteresis on discrete lattice
                delta_m = (raw - schmitt_state) / QUANTUM
                if delta_m > 0.65:
                    schmitt_state = round(raw / QUANTUM) * QUANTUM
                elif delta_m < -0.65:
                    schmitt_state = round(raw / QUANTUM) * QUANTUM
                cmd = schmitt_state

            else:
                cmd = raw

            if len(published_cmds) > 0:
                cmd_increments.append(abs(cmd - prev_published_cmd))
            prev_published_cmd = cmd
            published_cmds.append(cmd)
            current_steer_cmd = cmd
            current_speed_cmd = speed_final

        # Record at 100 Hz
        act_deltas.append(agent.state[2])
        sv_val = (agent.state[2] - (act_deltas[-2] if len(act_deltas) > 1 else agent.state[2])) / DT_SIM
        steer_rates.append(sv_val)
        yaw_rates.append(agent.state[5])

        if 'scans' in obs and len(obs['scans']) > 0 and len(obs['scans'][0]) > 0:
            min_lidar_scans.append(float(np.min(obs['scans'][0])))

        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        if agent.in_collision:
            collisions += 1

    # Analysis
    raw_cmds = np.array(raw_cmds)
    published_cmds = np.array(published_cmds)
    cmd_increments = np.array(cmd_increments)
    act_deltas = np.array(act_deltas)
    steer_rates = np.array(steer_rates)
    yaw_rates = np.array(yaw_rates)
    lat_errs = np.array(lat_errs)
    head_errs = np.array(head_errs)
    solve_times = np.array(solve_times) * 1000.0 # ms

    # Straightaway FFT (steps 100 to 1000)
    straight_r = yaw_rates[100:1000]
    fft_r = np.abs(np.fft.rfft(straight_r - np.mean(straight_r)))
    freqs = np.fft.rfftfreq(len(straight_r), d=DT_SIM)
    idx_5_7 = np.where((freqs >= 5.0) & (freqs <= 7.0))[0]
    idx_2_4 = np.where((freqs >= 2.0) & (freqs <= 4.0))[0]
    energy_5_7 = np.sqrt(np.mean(fft_r[idx_5_7]**2)) / len(freqs) * 1000
    energy_2_4 = np.sqrt(np.mean(fft_r[idx_2_4]**2)) / len(freqs) * 1000

    s_progress = float(telem['cur_s'])
    lap_completed = s_progress > 50.0

    return {
        'policy': policy_name,
        'cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(published_cmds**2)))),
        'cmd_inc_rms_mrad': float(np.sqrt(np.mean(cmd_increments**2)) * 1000),
        'rate_rms_rad_s': float(np.sqrt(np.mean(steer_rates**2))),
        'rate_max_rad_s': float(np.max(np.abs(steer_rates))),
        'energy_5_7': float(energy_5_7),
        'energy_2_4': float(energy_2_4),
        'lat_rmse_cm': float(np.sqrt(np.mean(lat_errs**2)) * 100.0),
        'max_lat_err_cm': float(np.max(lat_errs) * 100.0),
        'head_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(head_errs**2)))),
        'min_lidar_m': float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0,
        'collisions': int(collisions),
        'lap_completed': lap_completed,
        's_final_m': float(s_progress),
        'solve_time_ms': float(np.mean(solve_times))
    }

if __name__ == '__main__':
    policies = ['BASELINE', 'POLICY_A_RAW', 'POLICY_B_REACHABLE', 'POLICY_C_SCHMITT']
    results = {}
    for p in policies:
        print(f"Executing closed-loop simulation for {p}...")
        results[p] = run_simulation(p, duration_s=26.0)

    print("\n" + "="*135)
    print("EXPERIMENT 20: CLOSED-LOOP VALIDATION & METRIC AUDIT (SPIELBERG BENCHMARK)")
    print("="*135)
    header = f"{'Policy':<22} | {'Cmd RMS':<9} | {'ΔCmd RMS':<10} | {'Rate RMS':<10} | {'Rate Max':<9} | {'5-7Hz (r)':<10} | {'2-4Hz (r)':<10} | {'Lat RMSE':<9} | {'Max Lat':<9} | {'Head RMSE':<10} | {'Min LiDAR':<10} | {'Solve':<7}"
    print(header)
    print("-" * 135)
    for p, r in results.items():
        print(f"{r['policy']:<22} | {r['cmd_rms_deg']:6.2f}°   | {r['cmd_inc_rms_mrad']:6.2f} mm  | {r['rate_rms_rad_s']:6.3f}    | {r['rate_max_rad_s']:6.2f}    | {r['energy_5_7']:8.4f}   | {r['energy_2_4']:8.4f}   | {r['lat_rmse_cm']:6.2f} cm | {r['max_lat_err_cm']:6.2f} cm | {r['head_rmse_deg']:6.2f}°    | {r['min_lidar_m']:6.2f} m   | {r['solve_time_ms']:5.2f} ms")

    with open('/home/yeswanth/.gemini/antigravity-ide/brain/0fb11780-fef7-4429-b020-0e74787650bf/scratch/exp20_closed_loop_results.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("\nSaved closed-loop validation results to exp20_closed_loop_results.json")
