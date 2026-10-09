#!/usr/bin/env python3
"""
EXPERIMENT 22: Multi-Track Actuator-Interface A/B Evaluation Suite

Evaluates three candidate policies across three diverse benchmark circuits:
Policies:
  - Policy A: RAW MPC baseline (unfiltered reference)
  - Policy B: HOLD16 (continuous raw command space with 16 mrad arrival hold region)
  - Policy C: HOLD_LATTICE16 (reachable steering lattice outside 16 mrad hold region)

Circuits:
  - Spielberg (339.1 m, medium-speed flowing, asymmetric elevation-like corners)
  - Austin (410.6 m, technical high-slew chicanes and tight hairpins)
  - Monza (441.5 m, ultra-high-speed straights and wide sweepers like Curva Grande)

Tracks 8 rigorous metric categories:
  1. Tracking (25-Hz controller & 100-Hz independent geometric centerline projection)
  2. Command distribution (RMS, Δu RMS, max Δu, range)
  3. Actuator dynamics (steering RMS, rate RMS, rate max, phase-aligned error stats)
  4. Relay chatter (target crossings, reversals, +- and -+ pairs, persistent cycles >=4 steps, streaks)
  5. Corrected frequency spectrum (Parseval Band RMS in 2-4, 4-8, 5-7 Hz; dominant >2 Hz; dominant 5-7 Hz)
  6. Safety (collisions, minimum LiDAR clearance)
  7. Compute efficiency (mean/p95 MPC solve time, mean/p95 policy overhead)
  8. Repeatability (full-trajectory determinism verification for every policy on every track)

Saves all structured results to experiment22_results.json.
NO production MPC or simulator modifications.
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
QUANTUM = SV_MAX * DT_SIM  # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

def predict_20ms_arrival(cur_d, b0, b1):
    diff1 = b1 - cur_d
    sv1 = math.copysign(SV_MAX, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    diff2 = b0 - d_10
    sv2 = math.copysign(SV_MAX, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20, sv2

def compute_geometric_tracking_error(x, y, yaw, waypoints, N_wp):
    """
    Independently projects vehicle pose (x, y) onto continuous line segments of track centerline.
    Computes orthogonal lateral error and heading error relative to continuous line segment tangent.
    """
    dx = waypoints[:, 0] - x
    dy = waypoints[:, 1] - y
    dist_sq = dx * dx + dy * dy
    closest_idx = int(np.argmin(dist_sq))
    
    cand_offsets = [-1, 0, 1]
    p = np.array([x, y], dtype=np.float64)
    best_dist_sq = float('inf')
    best_lat_err = 0.0
    best_head_err = 0.0
    
    for off in cand_offsets:
        ia = (closest_idx + off) % N_wp
        ib = (ia + 1) % N_wp
        pa = waypoints[ia]
        pb = waypoints[ib]
        v = pb - pa
        L2 = float(np.dot(v, v))
        if L2 < 1e-12:
            t = 0.0
        else:
            t = float(np.clip(np.dot(p - pa, v) / L2, 0.0, 1.0))
        p_proj = pa + t * v
        d_sq = float(np.dot(p - p_proj, p - p_proj))
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            L = math.sqrt(L2)
            if L > 1e-6:
                tx = v[0] / L
                ty = v[1] / L
            else:
                tx, ty = 1.0, 0.0
            nx = -ty
            ny = tx
            best_lat_err = float((p[0] - p_proj[0]) * nx + (p[1] - p_proj[1]) * ny)
            seg_psi = math.atan2(ty, tx)
            best_head_err = float((yaw - seg_psi + math.pi) % (2.0 * math.pi) - math.pi)
            
    return abs(best_lat_err), abs(best_head_err)

def run_single_simulation(track_name, policy_name, duration_s=26.0):
    track = TrackManager.load_track(track_name)
    start_pose = track.start_pose
    waypoints = track.waypoints
    N_wp = len(waypoints)
    track_length = track.track_length
    
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    obs = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    agent = env.sim.agents[0]
    
    ctrl = DiagnosticMPCController(
        track, speed_scale=1.0, straight_w_psi=2.0, actuator_delay_s=0.020, model_type="yaw_first_order"
    )
    
    max_steps = int(round(duration_s / DT_SIM))  # 2600 steps
    control_interval = 4
    
    current_steer_cmd = 0.0
    current_speed_cmd = 7.50
    prev_published_cmd = 0.0
    
    # 25-Hz Telemetry Arrays
    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    lat_errs_25hz = []
    head_errs_25hz = []
    mpc_solve_times = []
    policy_calc_times = []
    
    # 100-Hz Simulator Arrays
    act_deltas = []
    act_rates_measured = []
    act_rates_relay = []
    relay_targets = []
    yaw_rates = []
    vehicle_x = []
    vehicle_y = []
    vehicle_yaw = []
    vehicle_v = []
    lat_errs_100hz = []
    head_errs_100hz = []
    min_lidar_scans = []
    
    held_count = 0
    collisions = 0
    
    for step_i in range(max_steps):
        t_sim = step_i * DT_SIM
        
        # 25-Hz Control Interval
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
            mpc_solve_times.append((time.perf_counter() - t_solve_start) * 1000.0)  # ms
            
            raw = float(telem['opt_raw_steer'])
            raw_cmds.append(raw)
            lat_errs_25hz.append(abs(telem['lateral_error']))
            head_errs_25hz.append(abs(telem['heading_error']))
            
            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) == 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) == 2 else 0.0
            cur_delta = float(agent.state[2])
            d_20, _ = predict_20ms_arrival(cur_delta, b0, b1)
            
            t_pol_start = time.perf_counter()
            # Policy selection
            if policy_name in ('RAW', 'REPEAT_RAW'):
                cmd = raw
            elif policy_name in ('HOLD_16', 'REPEAT_HOLD_16'):
                # Policy H: continuous raw command space with 16 mrad arrival-state hold region
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    held_count += 1
                else:
                    cmd = raw
            elif policy_name in ('HOLD_LATTICE_16', 'REPEAT_HOLD_LATTICE_16'):
                # Policy HL: 16 mrad hold region, projected onto reachable 32 mrad lattice outside hold region
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    held_count += 1
                else:
                    cmd = round(raw / QUANTUM) * QUANTUM
            else:
                cmd = raw
                
            policy_calc_times.append((time.perf_counter() - t_pol_start) * 1e6)  # us
            
            if len(published_cmds) > 0:
                cmd_increments.append(abs(cmd - prev_published_cmd))
            prev_published_cmd = cmd
            published_cmds.append(cmd)
            current_steer_cmd = cmd
            current_speed_cmd = speed_final
            
        # 100-Hz Recording before env.step
        cur_delta_100hz = float(agent.state[2])
        cur_target = float(agent.steer_buffer[-1]) if len(agent.steer_buffer) == 2 else 0.0
        act_deltas.append(cur_delta_100hz)
        relay_targets.append(cur_target)
        
        diff_tgt = cur_target - cur_delta_100hz
        sv = math.copysign(SV_MAX, diff_tgt) if abs(diff_tgt) > EPS_RELAY else 0.0
        act_rates_relay.append(sv)
        
        if len(act_deltas) > 1:
            measured_sv = (cur_delta_100hz - act_deltas[-2]) / DT_SIM
        else:
            measured_sv = 0.0
        act_rates_measured.append(measured_sv)
        
        yaw_rates.append(float(agent.state[5]))
        vehicle_x.append(float(agent.state[0]))
        vehicle_y.append(float(agent.state[1]))
        vehicle_yaw.append(float(agent.state[4]))
        vehicle_v.append(float(agent.state[3]))
        
        # 100-Hz Geometric Tracking Error
        lat_100, head_100 = compute_geometric_tracking_error(
            float(agent.state[0]), float(agent.state[1]), float(agent.state[4]), waypoints, N_wp
        )
        lat_errs_100hz.append(lat_100)
        head_errs_100hz.append(head_100)
        
        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        if agent.in_collision:
            collisions += 1
        if 'scans' in obs and len(obs['scans']) > 0 and len(obs['scans'][0]) > 0:
            min_lidar_scans.append(float(np.min(obs['scans'][0])))
            
    # Record post-final step steering angle
    act_deltas.append(float(agent.state[2]))
    
    # --------------------------------------------------------------------------
    # 1. TRACKING METRICS
    # --------------------------------------------------------------------------
    lat_errs_25hz = np.array(lat_errs_25hz)
    head_errs_25hz = np.array(head_errs_25hz)
    lat_errs_100hz = np.array(lat_errs_100hz)
    head_errs_100hz = np.array(head_errs_100hz)
    
    lat_rmse_25hz_cm = float(np.sqrt(np.mean(lat_errs_25hz**2)) * 100.0)
    max_lat_err_25hz_cm = float(np.max(lat_errs_25hz) * 100.0)
    head_rmse_25hz_deg = float(np.rad2deg(np.sqrt(np.mean(head_errs_25hz**2))))
    
    lat_rmse_100hz_cm = float(np.sqrt(np.mean(lat_errs_100hz**2)) * 100.0)
    max_lat_err_100hz_cm = float(np.max(lat_errs_100hz) * 100.0)
    head_rmse_100hz_deg = float(np.rad2deg(np.sqrt(np.mean(head_errs_100hz**2))))
    
    s_progress = float(telem['cur_s'])
    lap_completed = bool(s_progress >= track_length)
    progress_pct = float(s_progress / track_length * 100.0)
    
    tracking_summary = {
        'lat_rmse_25hz_cm': lat_rmse_25hz_cm,
        'lat_rmse_100hz_cm': lat_rmse_100hz_cm,
        'max_lat_err_25hz_cm': max_lat_err_25hz_cm,
        'max_lat_err_100hz_cm': max_lat_err_100hz_cm,
        'head_rmse_25hz_deg': head_rmse_25hz_deg,
        'head_rmse_100hz_deg': head_rmse_100hz_deg,
        'progress_m': s_progress,
        'track_length_m': track_length,
        'progress_pct': progress_pct,
        'lap_completed': lap_completed
    }
    
    # --------------------------------------------------------------------------
    # 2. COMMAND METRICS
    # --------------------------------------------------------------------------
    published_cmds = np.array(published_cmds)
    cmd_increments = np.array(cmd_increments)
    is_lattice = np.abs(published_cmds - np.round(published_cmds / QUANTUM) * QUANTUM) < 1e-5
    
    command_summary = {
        'cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(published_cmds**2)))),
        'cmd_inc_rms_mrad': float(np.sqrt(np.mean(cmd_increments**2)) * 1000.0),
        'cmd_inc_max_mrad': float(np.max(cmd_increments) * 1000.0),
        'cmd_min_deg': float(np.rad2deg(np.min(published_cmds))),
        'cmd_max_deg': float(np.rad2deg(np.max(published_cmds))),
        'frac_held': float(held_count / len(published_cmds)),
        'frac_lattice': float(np.mean(is_lattice)),
        'frac_non_lattice': float(1.0 - np.mean(is_lattice))
    }
    
    # --------------------------------------------------------------------------
    # 3. ACTUATOR DYNAMICS & PHASE-ALIGNED ERROR
    # --------------------------------------------------------------------------
    act_deltas = np.array(act_deltas)
    act_rates_measured = np.array(act_rates_measured)
    
    # e[k] = u_pub[k] - delta_actual(t_k + 40ms) = published_cmds[k] - act_deltas[4*k + 4]
    K = len(published_cmds) - 1
    phase_errors = []
    for k in range(K):
        phase_errors.append(published_cmds[k] - act_deltas[4 * k + 4])
    phase_errors = np.array(phase_errors, dtype=np.float64)
    abs_errors = np.abs(phase_errors)
    
    actuator_summary = {
        'steering_rms_deg': float(np.rad2deg(np.sqrt(np.mean(act_deltas**2)))),
        'rate_rms_rad_s': float(np.sqrt(np.mean(act_rates_measured**2))),
        'rate_max_rad_s': float(np.max(np.abs(act_rates_measured))),
        'phase_err_rms_mrad': float(np.sqrt(np.mean(phase_errors**2)) * 1000.0),
        'phase_err_mean_abs_mrad': float(np.mean(abs_errors) * 1000.0),
        'phase_err_p95_mrad': float(np.percentile(abs_errors, 95) * 1000.0),
        'phase_err_p99_mrad': float(np.percentile(abs_errors, 99) * 1000.0),
        'phase_err_max_mrad': float(np.max(abs_errors) * 1000.0),
        'samples_gt_1e9': int(np.sum(abs_errors > 1e-9)),
        'samples_gt_1e6': int(np.sum(abs_errors > 1e-6)),
        'total_phase_samples': len(phase_errors)
    }
    
    # --------------------------------------------------------------------------
    # 4. RELAY CHATTER & PHENOMENA SEPARATION
    # --------------------------------------------------------------------------
    target_crossings = 0
    diffs = [relay_targets[i] - act_deltas[i] for i in range(len(relay_targets))]
    for i in range(1, len(diffs)):
        if abs(diffs[i]) > EPS_RELAY and abs(diffs[i-1]) > EPS_RELAY:
            if diffs[i] * diffs[i-1] < 0:
                target_crossings += 1
                
    relay_reversals = 0
    plus_minus_pairs = 0
    minus_plus_pairs = 0
    for i in range(1, len(act_rates_relay)):
        if act_rates_relay[i] * act_rates_relay[i-1] < 0:
            relay_reversals += 1
            if act_rates_relay[i-1] > 0 and act_rates_relay[i] < 0:
                plus_minus_pairs += 1
            elif act_rates_relay[i-1] < 0 and act_rates_relay[i] > 0:
                minus_plus_pairs += 1
                
    flips = (np.array(act_rates_relay[1:]) * np.array(act_rates_relay[:-1]) < 0).astype(int)
    streak = 0
    longest_streak = 0
    persistent_cycles_ge4 = 0
    total_alt_steps = 0
    for f in flips:
        if f == 1:
            streak += 1
            total_alt_steps += 1
            longest_streak = max(longest_streak, streak + 1)
            if streak == 3:
                persistent_cycles_ge4 += 1
        else:
            streak = 0
            
    chatter_summary = {
        'target_crossings': int(target_crossings),
        'relay_sign_reversals': int(relay_reversals),
        'plus_minus_pairs': int(plus_minus_pairs),
        'minus_plus_pairs': int(minus_plus_pairs),
        'persistent_cycles_ge4': int(persistent_cycles_ge4),
        'longest_alternating_streak': int(longest_streak),
        'total_alternating_steps': int(total_alt_steps),
        'nonzero_relay_steps': int(np.sum(np.abs(act_rates_relay) > EPS_RELAY))
    }
    
    # --------------------------------------------------------------------------
    # 5. FREQUENCY ANALYSIS (100-Hz FFT over steps 100 to 1000 = 900 samples)
    # --------------------------------------------------------------------------
    straight_r = np.array(yaw_rates[100:1000])
    N_fft = len(straight_r)
    df = 1.0 / (N_fft * DT_SIM)  # 0.111111 Hz
    freqs = np.fft.rfftfreq(N_fft, d=DT_SIM)
    
    dc_component = float(np.mean(straight_r))
    fft_raw = np.fft.rfft(straight_r - dc_component)
    fft_amp = np.abs(fft_raw)
    
    def band_rms(f_low, f_high):
        idx = np.where((freqs >= f_low) & (freqs <= f_high))[0]
        if len(idx) == 0: return 0.0
        return float(np.sqrt(np.sum(fft_amp[idx]**2) * 2.0 / (N_fft**2)))
        
    energy_2_4 = band_rms(2.0, 4.0)
    energy_4_8 = band_rms(4.0, 8.0)
    energy_5_7 = band_rms(5.0, 7.0)
    
    idx_gt2 = np.where(freqs >= 2.0)[0]
    dom_gt2_idx = idx_gt2[np.argmax(fft_amp[idx_gt2])]
    dom_gt2_freq = float(freqs[dom_gt2_idx])
    
    idx_5_7 = np.where((freqs >= 5.0) & (freqs <= 7.0))[0]
    dom_5_7_idx = idx_5_7[np.argmax(fft_amp[idx_5_7])] if len(idx_5_7) > 0 else 0
    dom_5_7_freq = float(freqs[dom_5_7_idx])
    
    frequency_summary = {
        'df_hz': df,
        'dc_component_rad_s': dc_component,
        'energy_2_4_rad_s': energy_2_4,
        'energy_4_8_rad_s': energy_4_8,
        'energy_5_7_rad_s': energy_5_7,
        'dom_gt2_freq_hz': dom_gt2_freq,
        'dom_5_7_freq_hz': dom_5_7_freq
    }
    
    # --------------------------------------------------------------------------
    # 6. SAFETY & COMPUTE
    # --------------------------------------------------------------------------
    min_lidar = float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0
    safety_summary = {
        'collisions': int(collisions),
        'min_lidar_m': min_lidar,
        'safety_violations': int(collisions)
    }
    
    mpc_solve_times = np.array(mpc_solve_times)
    policy_calc_times = np.array(policy_calc_times)
    compute_summary = {
        'mean_solve_ms': float(np.mean(mpc_solve_times)),
        'p95_solve_ms': float(np.percentile(mpc_solve_times, 95)),
        'mean_policy_us': float(np.mean(policy_calc_times)),
        'p95_policy_us': float(np.percentile(policy_calc_times, 95))
    }
    
    timeseries = {
        'x': vehicle_x,
        'y': vehicle_y,
        'yaw': vehicle_yaw,
        'v': vehicle_v,
        'delta': act_deltas[:max_steps].tolist(),
        'u_pub': published_cmds.tolist(),
        'u_raw': raw_cmds
    }
    
    return {
        'track': track_name,
        'policy': policy_name,
        'tracking': tracking_summary,
        'command': command_summary,
        'actuator': actuator_summary,
        'chatter': chatter_summary,
        'frequency': frequency_summary,
        'safety': safety_summary,
        'compute': compute_summary,
        '_timeseries': timeseries
    }

def main():
    print("\n" + "="*90)
    print("STARTING EXPERIMENT 22: MULTI-TRACK ACTUATOR-INTERFACE A/B EVALUATION")
    print("="*90)
    
    tracks = ['Spielberg', 'Austin', 'Monza']
    policies = ['RAW', 'HOLD_16', 'HOLD_LATTICE_16']
    
    results = {
        'tracks': tracks,
        'policies': policies,
        'runs': {},
        'determinism': {}
    }
    
    for t_name in tracks:
        results['runs'][t_name] = {}
        results['determinism'][t_name] = {}
        print(f"\n--- EVALUATING CIRCUIT: {t_name.upper()} ---")
        
        for pol in policies:
            print(f"Executing {t_name} | {pol} (Run 1)...")
            res1 = run_single_simulation(t_name, pol, duration_s=26.0)
            
            print(f"Executing {t_name} | {pol} (Run 2 - Repeatability)...")
            res2 = run_single_simulation(t_name, f"REPEAT_{pol}", duration_s=26.0)
            
            # Determinism check
            ts1 = res1['_timeseries']
            ts2 = res2['_timeseries']
            pol_det = {}
            for var in ['x', 'y', 'yaw', 'v', 'delta', 'u_pub', 'u_raw']:
                arr1 = np.array(ts1[var])
                arr2 = np.array(ts2[var])
                max_diff = float(np.max(np.abs(arr1 - arr2)))
                rms_diff = float(np.sqrt(np.mean((arr1 - arr2)**2)))
                pol_det[var] = {
                    'max_diff': max_diff,
                    'rms_diff': rms_diff,
                    'is_identical': bool(max_diff == 0.0)
                }
            results['determinism'][t_name][pol] = pol_det
            
            det_pass = all(m['is_identical'] for m in pol_det.values())
            print(f"  -> Determinism verified: {det_pass}")
            
            del res1['_timeseries']
            del res2['_timeseries']
            results['runs'][t_name][pol] = res1
            
    out_path = '/home/yeswanth/roboracer_ws/experiments/experiment22/experiment22_results.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved comprehensive multi-track results to {out_path}")
    
    # --------------------------------------------------------------------------
    # PRINT PER-TRACK BENCHMARK TABLES
    # --------------------------------------------------------------------------
    for t_name in tracks:
        print("\n" + "="*160)
        print(f"CIRCUIT: {t_name.upper()} (Duration: 26.0s)")
        print("="*160)
        hdr = (
            f"{'Policy':<16} | {'CmdRMS':<7} | {'ΔCmdRMS':<8} | {'Held%':<6} | {'Lat%':<6} | "
            f"{'RateRMS':<7} | {'Reversals':<9} | {'Persist≥4':<9} | {'Streak':<6} | "
            f"{'5-7Hz (r)':<9} | {'Dom>2Hz':<7} | {'100Hz LatRMSE':<13} | {'MaxLat':<7} | "
            f"{'100Hz Head':<10} | {'PhaseErrRMS':<11} | {'LiDAR':<6} | {'Prog[m]':<8} | {'Solve[ms]'}"
        )
        print(hdr)
        print("-" * 160)
        for pol in policies:
            r = results['runs'][t_name][pol]
            tr = r['tracking']
            cmd = r['command']
            act = r['actuator']
            ch = r['chatter']
            fq = r['frequency']
            sf = r['safety']
            cp = r['compute']
            
            line = (
                f"{pol:<16} | "
                f"{cmd['cmd_rms_deg']:5.2f}° | "
                f"{cmd['cmd_inc_rms_mrad']:5.2f}mm | "
                f"{cmd['frac_held']*100:4.1f}% | "
                f"{cmd['frac_lattice']*100:4.1f}% | "
                f"{act['rate_rms_rad_s']:5.3f} | "
                f"{ch['relay_sign_reversals']:<9d} | "
                f"{ch['persistent_cycles_ge4']:<9d} | "
                f"{ch['longest_alternating_streak']:<6d} | "
                f"{fq['energy_5_7_rad_s']:7.4f} | "
                f"{fq['dom_gt2_freq_hz']:5.2f}Hz | "
                f"{tr['lat_rmse_100hz_cm']:7.4f} cm | "
                f"{tr['max_lat_err_100hz_cm']:5.2f}cm | "
                f"{tr['head_rmse_100hz_deg']:6.2f}°   | "
                f"{act['phase_err_rms_mrad']:7.3f} mm  | "
                f"{sf['min_lidar_m']:4.2f}m | "
                f"{tr['progress_m']:6.1f}m | "
                f"{cp['mean_solve_ms']:4.2f}ms"
            )
            print(line)
        print("="*160)

if __name__ == '__main__':
    main()
