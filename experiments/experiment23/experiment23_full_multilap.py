#!/usr/bin/env python3
"""
EXPERIMENT 23: Full 7-Track Multi-Lap Validation Suite

Evaluates the three actuator-interface policies over 3 COMPLETE LAPS across the full
7-track benchmark suite from the repository:
  Policies:
    - RAW (Policy A: unfiltered raw MPC baseline)
    - HOLD16 (Policy B: continuous raw command space with 16 mrad arrival hold region)
    - HOLD_LATTICE16 (Policy C: reachable 32 mrad steering lattice outside 16 mrad hold region)

  Tracks (Full 7-Track Benchmark Suite):
    1. Spielberg (339.1 m, medium-speed flowing, elevation-like chicanes)
    2. Austin (410.6 m, high-slew chicanes, hairpins)
    3. Monza (441.5 m, high-speed straights, Curva Grande)
    4. BrandsHatch (350.8 m, technical cambered corners)
    5. Silverstone (448.8 m, high-speed sweepers, complex transitions)
    6. Catalunya (403.9 m, long radius corners, heavy braking zones)
    7. Spa (544.8 m, endurance circuit, long straights, fast sweepers)

Runs 3 complete laps per track × policy (21 primary multi-lap runs) plus complete
multi-lap determinism verification (21 repetition runs).
Computes all 15 required metric categories and saves to experiment23_results.json.

NO production MPC code or simulator modifications.
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

BENCHMARK_TRACKS = [
    'Spielberg',
    'Austin',
    'Monza',
    'BrandsHatch',
    'Silverstone',
    'Catalunya',
    'Spa'
]

POLICIES = ['RAW', 'HOLD16', 'HOLD_LATTICE16']

def predict_20ms_arrival(cur_d, b0, b1):
    diff1 = b1 - cur_d
    sv1 = math.copysign(SV_MAX, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    diff2 = b0 - d_10
    sv2 = math.copysign(SV_MAX, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20, sv2

def compute_geometric_tracking_error(x, y, yaw, waypoints, N_wp, last_idx=0):
    """
    Independently projects vehicle pose (x, y) onto continuous line segments of track centerline.
    Returns:
      signed_lat_err: positive if vehicle is left of centerline, negative if right
      signed_head_err: vehicle yaw minus segment tangent angle (wrapped to [-pi, pi])
      closest_idx: index of closest waypoint
    """
    # Fast local window search around previous closest index
    w_size = 60
    indices = (np.arange(-20, w_size) + last_idx) % N_wp
    cand_pts = waypoints[indices]
    dist_sq = (cand_pts[:, 0] - x)**2 + (cand_pts[:, 1] - y)**2
    best_loc = int(np.argmin(dist_sq))
    closest_idx = int(indices[best_loc])
    
    # Global fallback if divergence occurs
    if dist_sq[best_loc] > 16.0:  # > 4m off
        all_dsq = (waypoints[:, 0] - x)**2 + (waypoints[:, 1] - y)**2
        closest_idx = int(np.argmin(all_dsq))
        
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
        
        L = math.sqrt(L2)
        if L > 1e-6:
            tx, ty = v[0] / L, v[1] / L
        else:
            tx, ty = 1.0, 0.0
        nx, ny = -ty, tx
        cand_lat_err = float((p[0] - p_proj[0]) * nx + (p[1] - p_proj[1]) * ny)
        seg_psi = math.atan2(ty, tx)
        cand_head_err = float((yaw - seg_psi + math.pi) % (2.0 * math.pi) - math.pi)
        
        # Prefer strictly closer projection; break ties by forward heading alignment
        if d_sq < best_dist_sq - 1e-9:
            best_dist_sq = d_sq
            best_lat_err = cand_lat_err
            best_head_err = cand_head_err
        elif abs(d_sq - best_dist_sq) <= 1e-9 and abs(cand_head_err) < abs(best_head_err):
            best_lat_err = cand_lat_err
            best_head_err = cand_head_err
            
    return best_lat_err, best_head_err, closest_idx

def compute_fft_metrics(signal_100hz):
    """
    Computes 100-Hz FFT spectral metrics according to validated methodology:
    - Removes DC component
    - Computes Parseval Band RMS for 2-4 Hz, 4-8 Hz, 5-7 Hz
    - Extracts dominant peak > 2 Hz and dominant peak in 5-7 Hz
    """
    sig = np.array(signal_100hz, dtype=np.float64)
    N_fft = len(sig)
    if N_fft < 100:
        return {
            'df_hz': 0.0, 'dc_component': 0.0,
            'energy_2_4_rad_s': 0.0, 'energy_4_8_rad_s': 0.0, 'energy_5_7_rad_s': 0.0,
            'dom_gt2_freq_hz': 0.0, 'dom_5_7_freq_hz': 0.0
        }
    df = 1.0 / (N_fft * DT_SIM)
    freqs = np.fft.rfftfreq(N_fft, d=DT_SIM)
    dc_comp = float(np.mean(sig))
    fft_raw = np.fft.rfft(sig - dc_comp)
    fft_amp = np.abs(fft_raw)
    
    def band_rms(f_low, f_high):
        idx = np.where((freqs >= f_low) & (freqs <= f_high))[0]
        if len(idx) == 0:
            return 0.0
        return float(np.sqrt(np.sum(fft_amp[idx]**2) * 2.0 / (N_fft**2)))
        
    energy_2_4 = band_rms(2.0, 4.0)
    energy_4_8 = band_rms(4.0, 8.0)
    energy_5_7 = band_rms(5.0, 7.0)
    
    idx_gt2 = np.where(freqs >= 2.0)[0]
    if len(idx_gt2) > 0:
        dom_gt2_idx = idx_gt2[np.argmax(fft_amp[idx_gt2])]
        dom_gt2_freq = float(freqs[dom_gt2_idx])
    else:
        dom_gt2_freq = 0.0
        
    idx_5_7 = np.where((freqs >= 5.0) & (freqs <= 7.0))[0]
    if len(idx_5_7) > 0:
        dom_5_7_idx = idx_5_7[np.argmax(fft_amp[idx_5_7])]
        dom_5_7_freq = float(freqs[dom_5_7_idx])
    else:
        dom_5_7_freq = 0.0
        
    return {
        'df_hz': float(df),
        'dc_component': float(dc_comp),
        'energy_2_4_rad_s': float(energy_2_4),
        'energy_4_8_rad_s': float(energy_4_8),
        'energy_5_7_rad_s': float(energy_5_7),
        'dom_gt2_freq_hz': float(dom_gt2_freq),
        'dom_5_7_freq_hz': float(dom_5_7_freq)
    }

def analyze_chatter(act_rates_relay, relay_targets, act_deltas):
    """
    Computes rigorous relay chatter metrics separating target crossings from limit cycles.
    """
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
            if streak == 3:  # 4 consecutive alternating steps
                persistent_cycles_ge4 += 1
        else:
            streak = 0
            
    return {
        'target_crossings': int(target_crossings),
        'relay_sign_reversals': int(relay_reversals),
        'plus_minus_pairs': int(plus_minus_pairs),
        'minus_plus_pairs': int(minus_plus_pairs),
        'persistent_cycles_ge4': int(persistent_cycles_ge4),
        'longest_alternating_streak': int(longest_streak),
        'total_alternating_steps': int(total_alt_steps),
        'nonzero_relay_steps': int(np.sum(np.abs(act_rates_relay) > EPS_RELAY))
    }

def run_multilap_simulation(track_name, policy_name, target_laps=3):
    """
    Executes a complete multi-lap simulation (default 3 laps) for a given track and policy.
    Warm-up / Start Procedure:
      Vehicle starts at track.start_pose (x0, y0, yaw0) with initial velocity v = 0 m/s.
      First control step accelerates from standstill towards target speed profile.
      Lap transitions are detected when cumulative arc-length s wraps around track_length.
    """
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
    
    control_interval = 4
    current_steer_cmd = 0.0
    current_speed_cmd = 7.50
    prev_published_cmd = 0.0
    
    # 25-Hz Telemetry Storage
    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    lat_errs_25hz = []
    head_errs_25hz = []
    mpc_solve_times = []
    policy_calc_times = []
    is_held_flags = []
    curvatures_25hz = []
    speeds_25hz = []
    s_cont_25hz = []
    
    # 100-Hz Simulator Storage
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
    
    # Lap Tracking State
    lap_boundaries_25hz = [0]
    lap_boundaries_100hz = [0]
    lap_times = []
    lap_distances = []
    lap_collisions = []
    lap_start_t = 0.0
    lap_start_step_100hz = 0
    prev_s = 0.0
    lap_count = 0
    collisions_total = 0
    current_lap_collisions = 0
    
    # Transition Audit for HOLD16 (and all policies)
    command_transition_events = []
    prev_held_status = False
    
    max_steps = 45000  # Up to 450 s safety limit (Spa 3 laps ~310 s)
    last_geom_idx = 0
    
    for step_i in range(max_steps):
        t_sim = step_i * DT_SIM
        
        # 25-Hz Control Cycle
        if step_i % control_interval == 0:
            k = step_i // control_interval
            cur_x = float(agent.state[0])
            cur_y = float(agent.state[1])
            cur_yaw = float(agent.state[4])
            cur_v = float(agent.state[3])
            cur_r = float(agent.state[5])
            
            t_solve_start = time.perf_counter()
            steer_final, speed_final, telem = ctrl.step(
                cur_x, cur_y, cur_yaw, cur_v, dt_actual=DT_CTRL, cur_r=cur_r
            )
            mpc_solve_times.append((time.perf_counter() - t_solve_start) * 1000.0)
            
            raw = float(telem['opt_raw_steer'])
            raw_cmds.append(raw)
            lat_errs_25hz.append(float(telem['lateral_error']))  # signed
            head_errs_25hz.append(float(telem['heading_error']))  # signed
            curvatures_25hz.append(float(telem['reference_kappas'][0]))
            speeds_25hz.append(cur_v)
            cur_s = float(telem['cur_s'])
            s_cont_25hz.append(cur_s)
            
            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) == 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) == 2 else 0.0
            cur_delta = float(agent.state[2])
            d_20, _ = predict_20ms_arrival(cur_delta, b0, b1)
            
            # Policy Application
            t_pol_start = time.perf_counter()
            is_held = False
            if 'RAW' in policy_name:
                cmd = raw
                is_held = False
            elif 'HOLD16' in policy_name and 'LATTICE' not in policy_name:
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    is_held = True
                else:
                    cmd = raw
                    is_held = False
            elif 'HOLD_LATTICE16' in policy_name:
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    is_held = True
                else:
                    cmd = round(raw / QUANTUM) * QUANTUM
                    is_held = False
            else:
                cmd = raw
                is_held = False
                
            policy_calc_times.append((time.perf_counter() - t_pol_start) * 1e6)
            is_held_flags.append(is_held)
            
            if len(published_cmds) > 0:
                inc = abs(cmd - prev_published_cmd)
                cmd_increments.append(inc)
                
                # Audit all command increments > 32 mrad (especially for HOLD16)
                if inc > (QUANTUM + 1e-5):
                    trans_type = f"{'held' if prev_held_status else 'raw'} -> {'held' if is_held else 'raw'}"
                    command_transition_events.append({
                        'step_25hz': k,
                        'time_s': t_sim,
                        'lap': lap_count + 1,
                        'inc_mrad': float(inc * 1000.0),
                        'trans_type': trans_type,
                        'u_pub_prev': float(prev_published_cmd),
                        'u_pub_cur': float(cmd),
                        'u_raw': float(raw),
                        'd_20': float(d_20),
                        'speed': float(cur_v),
                        'curvature': float(telem['reference_kappas'][0]),
                        'lat_err_cm': float(telem['lateral_error'] * 100.0)
                    })
            prev_held_status = is_held
            prev_published_cmd = cmd
            published_cmds.append(cmd)
            current_steer_cmd = cmd
            current_speed_cmd = speed_final
            
            # Check Lap Wrap-Around
            if prev_s > track_length * 0.80 and cur_s < track_length * 0.20:
                lap_count += 1
                lap_dur = t_sim - lap_start_t
                lap_times.append(lap_dur)
                
                # Integrated distance for this lap
                lap_pts_x = vehicle_x[lap_start_step_100hz:step_i]
                lap_pts_y = vehicle_y[lap_start_step_100hz:step_i]
                if len(lap_pts_x) > 1:
                    dx_arr = np.diff(lap_pts_x)
                    dy_arr = np.diff(lap_pts_y)
                    lap_dist = float(np.sum(np.sqrt(dx_arr**2 + dy_arr**2)))
                else:
                    lap_dist = track_length
                lap_distances.append(lap_dist)
                lap_collisions.append(current_lap_collisions)
                
                lap_boundaries_25hz.append(len(published_cmds))
                lap_boundaries_100hz.append(step_i)
                
                lap_start_t = t_sim
                lap_start_step_100hz = step_i
                current_lap_collisions = 0
                
                if lap_count >= target_laps:
                    break
            prev_s = cur_s
            
        # 100-Hz Recording
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
        lat_100, head_100, last_geom_idx = compute_geometric_tracking_error(
            float(agent.state[0]), float(agent.state[1]), float(agent.state[4]),
            waypoints, N_wp, last_idx=last_geom_idx
        )
        lat_errs_100hz.append(lat_100)
        head_errs_100hz.append(head_100)
        
        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        if agent.in_collision:
            collisions_total += 1
            current_lap_collisions += 1
        if 'scans' in obs and len(obs['scans']) > 0 and len(obs['scans'][0]) > 0:
            min_lidar_scans.append(float(np.min(obs['scans'][0])))
            
    # Record post-final step steering angle
    act_deltas.append(float(agent.state[2]))
    
    # Check if final lap boundary needed
    if lap_count < target_laps and len(lap_boundaries_25hz) <= target_laps:
        lap_boundaries_25hz.append(len(published_cmds))
        lap_boundaries_100hz.append(len(act_deltas) - 1)
        
    total_steps_25hz = len(published_cmds)
    total_steps_100hz = len(act_rates_measured)
    
    # --------------------------------------------------------------------------
    # METRICS EXTRACTION HELPER FOR SLICES
    # --------------------------------------------------------------------------
    def extract_slice_metrics(k_start, k_end, s_start, s_end):
        lat_25 = np.array(lat_errs_25hz[k_start:k_end])
        head_25 = np.array(head_errs_25hz[k_start:k_end])
        lat_100 = np.array(lat_errs_100hz[s_start:s_end])
        head_100 = np.array(head_errs_100hz[s_start:s_end])
        
        pub_cmd_sl = np.array(published_cmds[k_start:k_end])
        raw_cmd_sl = np.array(raw_cmds[k_start:k_end])
        
        if len(pub_cmd_sl) > 1:
            d_pub_sl = np.abs(np.diff(pub_cmd_sl))
            d_raw_sl = np.abs(np.diff(raw_cmd_sl))
        else:
            d_pub_sl = np.array([0.0])
            d_raw_sl = np.array([0.0])
            
        act_d_sl = np.array(act_deltas[s_start:s_end])
        act_r_sl = np.array(act_rates_measured[s_start:s_end])
        relay_r_sl = act_rates_relay[s_start:s_end]
        relay_t_sl = relay_targets[s_start:s_end]
        yaw_r_sl = yaw_rates[s_start:s_end]
        
        # Phase-aligned error: e[k] = u_pub[k] - delta(4k + 4)
        phase_errs = []
        for k_idx in range(k_start, min(k_end, (len(act_deltas) - 1) // 4)):
            phase_errs.append(published_cmds[k_idx] - act_deltas[4 * k_idx + 4])
        phase_errs = np.array(phase_errs, dtype=np.float64) if len(phase_errs) > 0 else np.array([0.0])
        abs_phase_errs = np.abs(phase_errs)
        
        chatter = analyze_chatter(relay_r_sl, relay_t_sl, act_d_sl)
        fft_res = compute_fft_metrics(yaw_r_sl)
        
        is_latt = np.abs(pub_cmd_sl - np.round(pub_cmd_sl / QUANTUM) * QUANTUM) < 1e-5
        held_sl = is_held_flags[k_start:k_end]
        
        # Command difference u_pub - u_raw
        cmd_diff = pub_cmd_sl - raw_cmd_sl
        abs_cmd_diff = np.abs(cmd_diff)
        
        return {
            'tracking': {
                'lat_rmse_25hz_cm': float(np.sqrt(np.mean(lat_25**2)) * 100.0),
                'lat_rmse_100hz_cm': float(np.sqrt(np.mean(lat_100**2)) * 100.0),
                'lat_mean_signed_cm': float(np.mean(lat_100) * 100.0),
                'lat_median_signed_cm': float(np.median(lat_100) * 100.0),
                'lat_std_cm': float(np.std(lat_100) * 100.0),
                'lat_max_abs_cm': float(np.max(np.abs(lat_100)) * 100.0),
                'head_rmse_25hz_deg': float(np.rad2deg(np.sqrt(np.mean(head_25**2)))),
                'head_rmse_100hz_deg': float(np.rad2deg(np.sqrt(np.mean(head_100**2)))),
                'head_mean_signed_deg': float(np.rad2deg(np.mean(head_100))),
                'head_std_deg': float(np.rad2deg(np.std(head_100)))
            },
            'steering': {
                'pub_cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(pub_cmd_sl**2)))),
                'raw_cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(raw_cmd_sl**2)))),
                'delta_pub_rms_mrad': float(np.sqrt(np.mean(d_pub_sl**2)) * 1000.0),
                'delta_raw_rms_mrad': float(np.sqrt(np.mean(d_raw_sl**2)) * 1000.0),
                'delta_pub_max_mrad': float(np.max(d_pub_sl) * 1000.0),
                'delta_raw_max_mrad': float(np.max(d_raw_sl) * 1000.0),
                'actual_steer_rms_deg': float(np.rad2deg(np.sqrt(np.mean(act_d_sl**2)))),
                'steer_rate_rms_rad_s': float(np.sqrt(np.mean(act_r_sl**2))),
                'steer_rate_max_rad_s': float(np.max(np.abs(act_r_sl))),
                'phase_err_rms_mrad': float(np.sqrt(np.mean(phase_errs**2)) * 1000.0),
                'phase_err_mean_abs_mrad': float(np.mean(abs_phase_errs) * 1000.0),
                'phase_err_p95_mrad': float(np.percentile(abs_phase_errs, 95) * 1000.0),
                'phase_err_p99_mrad': float(np.percentile(abs_phase_errs, 99) * 1000.0),
                'phase_err_max_mrad': float(np.max(abs_phase_errs) * 1000.0),
                'phase_gt_1e9': int(np.sum(abs_phase_errs > 1e-9)),
                'phase_gt_1e6': int(np.sum(abs_phase_errs > 1e-6)),
                'phase_samples': len(phase_errs)
            },
            'chatter': chatter,
            'frequency': fft_res,
            'command_mod': {
                'frac_lattice': float(np.mean(is_latt)),
                'frac_held': float(np.mean(held_sl)),
                'frac_changed': float(np.mean(abs_cmd_diff > 1e-6)),
                'diff_rms_mrad': float(np.sqrt(np.mean(cmd_diff**2)) * 1000.0),
                'diff_p95_mrad': float(np.percentile(abs_cmd_diff, 95) * 1000.0),
                'diff_max_mrad': float(np.max(abs_cmd_diff) * 1000.0),
                'diff_signed_mean_mrad': float(np.mean(cmd_diff) * 1000.0)
            }
        }
        
    # Overall 3-Lap Summary
    overall_metrics = extract_slice_metrics(0, total_steps_25hz, 0, total_steps_100hz)
    
    # Per-Lap Metrics
    per_lap_metrics = []
    for l_idx in range(len(lap_times)):
        k_s = lap_boundaries_25hz[l_idx]
        k_e = lap_boundaries_25hz[l_idx + 1]
        s_s = lap_boundaries_100hz[l_idx]
        s_e = lap_boundaries_100hz[l_idx + 1]
        lap_m = extract_slice_metrics(k_s, k_e, s_s, s_e)
        lap_m['lap_number'] = l_idx + 1
        lap_m['lap_time_s'] = float(lap_times[l_idx])
        lap_m['lap_dist_m'] = float(lap_distances[l_idx])
        lap_m['completed'] = True
        lap_m['collisions'] = int(lap_collisions[l_idx])
        per_lap_metrics.append(lap_m)
        
    # --------------------------------------------------------------------------
    # BUCKETED COMMAND MODIFICATION ANALYSIS (FOR HOLD_LATTICE16)
    # --------------------------------------------------------------------------
    pub_arr = np.array(published_cmds)
    raw_arr = np.array(raw_cmds)
    diff_arr = pub_arr - raw_arr
    abs_diff = np.abs(diff_arr)
    held_arr = np.array(is_held_flags)
    kappas = np.abs(np.array(curvatures_25hz))
    spds = np.array(speeds_25hz)
    lat_25_abs = np.abs(np.array(lat_errs_25hz))
    
    def calc_bucket_stats(mask):
        count = int(np.sum(mask))
        if count == 0:
            return {'count': 0, 'frac_changed': 0.0, 'frac_held': 0.0, 'rms_mod_mrad': 0.0, 'mean_signed_mrad': 0.0, 'max_mod_mrad': 0.0}
        return {
            'count': count,
            'frac_changed': float(np.mean(abs_diff[mask] > 1e-6)),
            'frac_held': float(np.mean(held_arr[mask])),
            'rms_mod_mrad': float(np.sqrt(np.mean(diff_arr[mask]**2)) * 1000.0),
            'mean_signed_mrad': float(np.mean(diff_arr[mask]) * 1000.0),
            'max_mod_mrad': float(np.max(abs_diff[mask]) * 1000.0)
        }
        
    bucketed_analysis = {
        'by_curvature': {
            'straight_gentle_lt_0_02': calc_bucket_stats(kappas < 0.02),
            'medium_curve_0_02_to_0_06': calc_bucket_stats((kappas >= 0.02) & (kappas < 0.06)),
            'tight_corner_ge_0_06': calc_bucket_stats(kappas >= 0.06)
        },
        'by_speed': {
            'low_speed_lt_3_5': calc_bucket_stats(spds < 3.5),
            'medium_speed_3_5_to_5_5': calc_bucket_stats((spds >= 3.5) & (spds < 5.5)),
            'high_speed_ge_5_5': calc_bucket_stats(spds >= 5.5)
        },
        'by_lateral_error': {
            'low_err_lt_2cm': calc_bucket_stats(lat_25_abs < 0.02),
            'medium_err_2cm_to_5cm': calc_bucket_stats((lat_25_abs >= 0.02) & (lat_25_abs < 0.05)),
            'high_err_ge_5cm': calc_bucket_stats(lat_25_abs >= 0.05)
        }
    }
    
    # --------------------------------------------------------------------------
    # HOLD16 TRANSITION AUDIT BREAKDOWN
    # --------------------------------------------------------------------------
    trans_audit_summary = {
        'total_gt_32mrad_events': len(command_transition_events),
        'by_class': {}
    }
    for ev in command_transition_events:
        c = ev['trans_type']
        if c not in trans_audit_summary['by_class']:
            trans_audit_summary['by_class'][c] = {'count': 0, 'max_inc_mrad': 0.0, 'events': []}
        trans_audit_summary['by_class'][c]['count'] += 1
        trans_audit_summary['by_class'][c]['max_inc_mrad'] = max(
            trans_audit_summary['by_class'][c]['max_inc_mrad'], ev['inc_mrad']
        )
        if len(trans_audit_summary['by_class'][c]['events']) < 5:  # Keep top 5 samples
            trans_audit_summary['by_class'][c]['events'].append(ev)
            
    # Compute summary
    mpc_solves = np.array(mpc_solve_times)
    pol_calcs = np.array(policy_calc_times)
    compute_summary = {
        'mean_solve_ms': float(np.mean(mpc_solves)),
        'p95_solve_ms': float(np.percentile(mpc_solves, 95)),
        'mean_policy_us': float(np.mean(pol_calcs)),
        'p95_policy_us': float(np.percentile(pol_calcs, 95))
    }
    
    # Safety summary
    min_lidar = float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0
    safety_summary = {
        'collisions': int(collisions_total),
        'min_lidar_m': min_lidar,
        'out_of_bounds_events': int(np.sum(np.abs(lat_errs_100hz) > 2.0))
    }
    
    timeseries = {
        'x': vehicle_x,
        'y': vehicle_y,
        'yaw': vehicle_yaw,
        'v': vehicle_v,
        'delta': act_deltas[:total_steps_100hz],
        'u_pub': published_cmds,
        'u_raw': raw_cmds
    }
    
    return {
        'track': track_name,
        'policy': policy_name,
        'laps_completed': lap_count,
        'total_time_s': float(total_steps_100hz * DT_SIM),
        'total_distance_m': float(np.sum(lap_distances)),
        'overall': overall_metrics,
        'per_lap': per_lap_metrics,
        'bucketed_command_mod': bucketed_analysis,
        'transition_audit': trans_audit_summary,
        'compute': compute_summary,
        'safety': safety_summary,
        '_timeseries': timeseries
    }

def main():
    print("\n" + "="*100)
    print("STARTING EXPERIMENT 23: FULL 7-TRACK MULTI-LAP VALIDATION SUITE (3 COMPLETE LAPS)")
    print("="*100)
    
    results = {
        'tracks': BENCHMARK_TRACKS,
        'policies': POLICIES,
        'target_laps': 3,
        'warmup_start_procedure': (
            "Vehicle placed at track.start_pose (x0, y0, yaw0) with initial velocity v = 0 m/s. "
            "First control step accelerates vehicle from standstill towards raceline velocity profile. "
            "Lap transitions detected when continuous arc-length s wraps around track_length."
        ),
        'runs': {},
        'determinism': {}
    }
    
    total_runs = len(BENCHMARK_TRACKS) * len(POLICIES)
    run_idx = 0
    t_suite_start = time.perf_counter()
    
    for t_name in BENCHMARK_TRACKS:
        results['runs'][t_name] = {}
        results['determinism'][t_name] = {}
        print(f"\n" + "#"*100)
        print(f"CIRCUIT [{BENCHMARK_TRACKS.index(t_name)+1}/7]: {t_name.upper()}")
        print("#"*100)
        
        for pol in POLICIES:
            run_idx += 1
            print(f"\n[{run_idx}/{total_runs}] Executing {t_name} | {pol} (3 Complete Laps - Primary Run)...")
            t_run_start = time.perf_counter()
            res1 = run_multilap_simulation(t_name, pol, target_laps=3)
            dur1 = time.perf_counter() - t_run_start
            print(f"  Completed 3 laps in {res1['total_time_s']:.2f} s sim time ({dur1:.2f} s wallclock).")
            for lp in res1['per_lap']:
                print(f"    Lap {lp['lap_number']}: time={lp['lap_time_s']:.2f}s, dist={lp['lap_dist_m']:.1f}m, "
                      f"100Hz LatRMSE={lp['tracking']['lat_rmse_100hz_cm']:.2f}cm, "
                      f"MeanSignedLat={lp['tracking']['lat_mean_signed_cm']:+.2f}cm, "
                      f"SteerRateRMS={lp['steering']['steer_rate_rms_rad_s']:.3f}rad/s, "
                      f"Reversals={lp['chatter']['relay_sign_reversals']}")
                      
            print(f"[{run_idx}/{total_runs}] Executing {t_name} | {pol} (3 Complete Laps - Repeatability Check)...")
            t_rep_start = time.perf_counter()
            res2 = run_multilap_simulation(t_name, pol, target_laps=3)
            dur2 = time.perf_counter() - t_rep_start
            
            # Determinism Evaluation
            ts1 = res1['_timeseries']
            ts2 = res2['_timeseries']
            pol_det = {}
            for var in ['x', 'y', 'yaw', 'v', 'delta', 'u_pub', 'u_raw']:
                arr1 = np.array(ts1[var])
                arr2 = np.array(ts2[var])
                min_l = min(len(arr1), len(arr2))
                diff = np.abs(arr1[:min_l] - arr2[:min_l])
                max_diff = float(np.max(diff))
                rms_diff = float(np.sqrt(np.mean(diff**2)))
                pol_det[var] = {
                    'max_diff': max_diff,
                    'rms_diff': rms_diff,
                    'is_identical': bool(max_diff == 0.0)
                }
            results['determinism'][t_name][pol] = pol_det
            det_pass = all(m['is_identical'] for m in pol_det.values())
            print(f"  Determinism verified across 7 state variables: {det_pass} (Max diff: {max(m['max_diff'] for m in pol_det.values()):.2e})")
            
            del res1['_timeseries']
            del res2['_timeseries']
            results['runs'][t_name][pol] = res1
            
    suite_dur = time.perf_counter() - t_suite_start
    print(f"\n" + "="*100)
    print(f"ALL 7-TRACK MULTI-LAP RUNS FINISHED IN {suite_dur/60.0:.2f} MINUTES WALLCLOCK TIME")
    print("="*100)
    
    out_path = '/home/yeswanth/roboracer_ws/experiments/experiment23/experiment23_results.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"Saved complete results to {out_path}")

if __name__ == '__main__':
    main()
