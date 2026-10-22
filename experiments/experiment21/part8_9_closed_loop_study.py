#!/usr/bin/env python3
"""
Experiment 21: Parts 8 & 9 - Genuine Closed-Loop Validation and Repeatability Study
Runs on Spielberg benchmark track for 26.0 seconds per policy.
Measures all 15 required metrics plus repeatability.
"""

import os
import sys
import time
import math
import json
import numpy as np

for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/controllers/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws']:
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

def predict_20ms_arrival(cur_d, b0, b1):
    diff1 = b1 - cur_d
    sv1 = math.copysign(SV_MAX, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    diff2 = b0 - d_10
    sv2 = math.copysign(SV_MAX, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20, sv2

def continuous_actuator_aware_policy(raw_k, cur_d, b0, b1, prev_u, w_target=1.0, w_motion=0.005, w_reverse=0.05):
    """
    Policy C: Continuous Actuator-Aware Candidate.
    Uses exact lifted prediction to evaluate continuous command candidates around raw_k.
    Does NOT force output onto a 32 mrad lattice.
    """
    d_20, sv_20 = predict_20ms_arrival(cur_d, b0, b1)
    err_20 = raw_k - d_20
    
    # 1. Candidate 1: Hold arrival state (sv=0 for remaining 20ms, zero chatter, zero reversals)
    u_hold = d_20
    cost_hold = w_target * (err_20)**2 # d_40 = d_20, motion=0, reversals=0
    
    # 2. Candidate 2: Pass raw command continuously
    # If raw command causes sub-quantum crossing (|err_20| < 32 mrad), it causes a 2-step chatter (+32, -32)
    # with d_40 = d_20, but motion cost = 2*3.2*0.01 = 0.064 and 1-2 reversals.
    if abs(err_20) < QUANTUM:
        cost_raw = w_target * (err_20)**2 + w_motion * 0.064 + w_reverse * 1.0
    else:
        # Full slew step: moves 64 mrad in direction of error
        d_40_slew = d_20 + math.copysign(2 * QUANTUM, err_20)
        cost_raw = w_target * (d_40_slew - raw_k)**2 + w_motion * 0.064
        
    # 3. Decision:
    # If holding arrival state has lower cost than chattering with raw command, hold!
    # Otherwise, pass raw command continuously!
    if cost_hold <= cost_raw:
        return u_hold
    else:
        return raw_k

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

    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    act_deltas = []
    steer_rates = []
    yaw_rates = []
    lat_errs = []
    head_errs = []
    mpc_solve_times = []
    policy_calc_times = []
    min_lidar_scans = []
    collisions = 0
    
    two_step_lc_count = 0
    relay_reversal_count = 0

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
            mpc_solve_times.append(time.perf_counter() - t_solve_start)

            raw = float(telem['opt_raw_steer'])
            raw_cmds.append(raw)
            lat_errs.append(abs(telem['lateral_error']))
            head_errs.append(abs(telem['heading_error']))

            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) == 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) == 2 else 0.0
            cur_delta = float(agent.state[2])
            d_20, _ = predict_20ms_arrival(cur_delta, b0, b1)

            t_pol_start = time.perf_counter()
            # Policy selection
            if policy_name in ('RAW', 'REPEAT_RAW'):
                cmd = raw
            elif policy_name == 'HOLD_16':
                cmd = d_20 if abs(raw - d_20) < 0.016 else raw
            elif policy_name == 'HOLD_LATTICE_16':
                cmd = d_20 if abs(raw - d_20) < 0.016 else round(raw / QUANTUM) * QUANTUM
            elif policy_name == 'CONTINUOUS_C':
                cmd = continuous_actuator_aware_policy(raw, cur_delta, b0, b1, prev_published_cmd)
            else:
                cmd = raw
            policy_calc_times.append(time.perf_counter() - t_pol_start)

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
        
        if len(steer_rates) > 1 and steer_rates[-1] * steer_rates[-2] < 0:
            relay_reversal_count += 1
        if len(steer_rates) >= 3 and (steer_rates[-1] * steer_rates[-2] < 0) and (steer_rates[-2] * steer_rates[-3] < 0):
            two_step_lc_count += 1

        if 'scans' in obs and len(obs['scans']) > 0 and len(obs['scans'][0]) > 0:
            min_lidar_scans.append(float(np.min(obs['scans'][0])))

        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        if agent.in_collision:
            collisions += 1

    # Convert to arrays
    raw_cmds = np.array(raw_cmds)
    published_cmds = np.array(published_cmds)
    cmd_increments = np.array(cmd_increments)
    act_deltas = np.array(act_deltas)
    steer_rates = np.array(steer_rates)
    yaw_rates = np.array(yaw_rates)
    lat_errs = np.array(lat_errs)
    head_errs = np.array(head_errs)
    mpc_solve_times = np.array(mpc_solve_times) * 1000.0 # ms
    policy_calc_times = np.array(policy_calc_times) * 1000.0 # ms

    # Spectrum on straightaway window (steps 100 to 1000 = 1.0s to 10.0s, consistent window)
    straight_r = yaw_rates[100:1000]
    fft_r = np.abs(np.fft.rfft(straight_r - np.mean(straight_r)))
    freqs = np.fft.rfftfreq(len(straight_r), d=DT_SIM)
    
    idx_2_4 = np.where((freqs >= 2.0) & (freqs <= 4.0))[0]
    idx_5_7 = np.where((freqs >= 5.0) & (freqs <= 7.0))[0]
    idx_4_8 = np.where((freqs >= 4.0) & (freqs <= 8.0))[0]
    
    energy_2_4 = float(np.sqrt(np.mean(fft_r[idx_2_4]**2)) / len(freqs) * 1000)
    energy_5_7 = float(np.sqrt(np.mean(fft_r[idx_5_7]**2)) / len(freqs) * 1000)
    energy_4_8 = float(np.sqrt(np.mean(fft_r[idx_4_8]**2)) / len(freqs) * 1000)
    
    dom_idx = np.argmax(fft_r[1:]) + 1
    dom_freq = float(freqs[dom_idx])
    dom_amp = float(fft_r[dom_idx] / len(straight_r))

    s_progress = float(telem['cur_s'])
    lap_completed = s_progress > 50.0

    # Tracking error
    act_tracking_error = float(np.sqrt(np.mean((published_cmds - act_deltas[::4])**2)) * 1000)
    phase_aligned_err = float(np.sqrt(np.mean((raw_cmds[:-1] - act_deltas[4::4])**2)) * 1000)

    return {
        'policy': policy_name,
        'cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(published_cmds**2)))),
        'cmd_inc_rms_mrad': float(np.sqrt(np.mean(cmd_increments**2)) * 1000),
        'rate_rms_rad_s': float(np.sqrt(np.mean(steer_rates**2))),
        'rate_max_rad_s': float(np.max(np.abs(steer_rates))),
        'energy_2_4': energy_2_4,
        'energy_5_7': energy_5_7,
        'energy_4_8': energy_4_8,
        'dom_freq_hz': dom_freq,
        'dom_amp': dom_amp,
        'lat_rmse_cm': float(np.sqrt(np.mean(lat_errs**2)) * 100.0),
        'max_lat_err_cm': float(np.max(lat_errs) * 100.0),
        'head_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(head_errs**2)))),
        'min_lidar_m': float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0,
        'collisions': int(collisions),
        'lap_completed': lap_completed,
        's_final_m': float(s_progress),
        'mean_solve_ms': float(np.mean(mpc_solve_times)),
        'max_solve_ms': float(np.max(mpc_solve_times)),
        'mean_policy_calc_us': float(np.mean(policy_calc_times) * 1000.0),
        'relay_reversals': int(relay_reversal_count),
        'two_step_lc_count': int(two_step_lc_count),
        'act_tracking_error_mrad': act_tracking_error,
        'phase_aligned_err_mrad': phase_aligned_err,
        # Raw timeseries for repeatability test
        '_timeseries_sample': list(np.round(act_deltas[100:150], 5))
    }

if __name__ == '__main__':
    candidate_policies = ['RAW', 'HOLD_16', 'HOLD_LATTICE_16', 'CONTINUOUS_C', 'REPEAT_RAW']
    closed_loop_results = {}
    print("Executing genuine closed-loop evaluations on Spielberg (26.0s each)...")
    for p in candidate_policies:
        print(f"Running simulation for {p}...")
        closed_loop_results[p] = run_simulation(p, duration_s=26.0)

    # Repeatability check
    raw1_ts = closed_loop_results['RAW']['_timeseries_sample']
    raw2_ts = closed_loop_results['REPEAT_RAW']['_timeseries_sample']
    is_deterministic = (raw1_ts == raw2_ts)
    print(f"\nREPEATABILITY AUDIT (RAW Run 1 vs Run 2):")
    print(f"  Timeseries sample identical: {is_deterministic}")
    max_ts_diff = max(abs(a - b) for a, b in zip(raw1_ts, raw2_ts))
    print(f"  Max timeseries deviation: {max_ts_diff:.2e} rad")

    # Clean out internal keys before saving
    for p in candidate_policies:
        del closed_loop_results[p]['_timeseries_sample']

    closed_loop_results['determinism_verified'] = bool(is_deterministic)

    print("\n" + "="*145)
    print("EXPERIMENT 21: CLOSED-LOOP VALIDATION SUMMARY TABLE")
    print("="*145)
    header = f"{'Policy':<18} | {'Cmd RMS':<9} | {'ΔCmd RMS':<10} | {'Rate RMS':<9} | {'5-7Hz (r)':<10} | {'2-4Hz (r)':<10} | {'Lat RMSE':<9} | {'Max Lat':<9} | {'Head RMSE':<10} | {'Reversals':<10} | {'2-Step LC':<10} | {'Policy us':<9}"
    print(header)
    print("-" * 145)
    for p in ['RAW', 'HOLD_16', 'HOLD_LATTICE_16', 'CONTINUOUS_C']:
        r = closed_loop_results[p]
        print(f"{r['policy']:<18} | {r['cmd_rms_deg']:6.2f}°   | {r['cmd_inc_rms_mrad']:6.2f} mm  | {r['rate_rms_rad_s']:6.3f}   | {r['energy_5_7']:8.4f}   | {r['energy_2_4']:8.4f}   | {r['lat_rmse_cm']:6.2f} cm | {r['max_lat_err_cm']:6.2f} cm | {r['head_rmse_deg']:6.2f}°    | {r['relay_reversals']:<10d} | {r['two_step_lc_count']:<10d} | {r['mean_policy_calc_us']:6.2f} us")

    with open('/home/yeswanth/roboracer_ws/experiments/experiment21/experiment21_results.json', 'w') as f:
        json.dump(closed_loop_results, f, indent=2)
    print("\nSaved closed-loop results to experiment21_results.json")
