#!/usr/bin/env python3
"""
EXPERIMENT 24: Robustness Validation Under Perception, Timing, and Actuator Uncertainty
Harness for systematic, multi-lap evaluations across 7 circuits and 3 policies.

Phases:
  Phase 1: Nominal Regression vs Experiment 23 (All 7 tracks, 3 policies, 3 laps, deterministic)
  Phase 2: Position Noise Sweep (sigma = 0.005, 0.010, 0.020 m, 4 screening tracks, 5 seeds, 3 laps)
  Phase 3: Heading Noise Sweep (sigma = 0.25, 0.50, 1.00 deg, 4 screening tracks, 5 seeds, 3 laps)
  Phase 4: Delay Mismatch (tau_actual = 10, 20, 30, 40 ms, 4 screening tracks, 3 laps)
  Phase 5: Slew / Relay-Quantum Mismatch (sv_max = 2.8, 3.0, 3.2, 3.4, 3.6 rad/s => q = 28, 30, 32, 34, 36 mrad)
  Phase 6: Timing Jitter (tau_k in {20, 30} ms, 4 screening tracks, 5 seeds, 3 laps)
  Phase 7: Combined Stress Cases (STRESS-1, STRESS-2, STRESS-3, All 7 tracks, 5 seeds, 3 laps)

NO production MPC code or simulator modifications.
"""

import os
import sys
import time
import math
import json
import numpy as np
import multiprocessing as mp

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

SV_NOMINAL = 3.20
DT_SIM = 0.010
DT_CTRL = 0.040
QUANTUM_NOMINAL = SV_NOMINAL * DT_SIM  # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

ALL_TRACKS = ['Spielberg', 'Austin', 'Monza', 'BrandsHatch', 'Silverstone', 'Catalunya', 'Spa']
SCREENING_TRACKS = ['Spielberg', 'Austin', 'Monza', 'Spa']
POLICIES = ['RAW', 'HOLD16', 'HOLD_LATTICE16']

def predict_20ms_arrival(cur_d, b0, b1, sv_model=SV_NOMINAL):
    """Exact frozen nominal arrival predictor."""
    diff1 = b1 - cur_d
    sv1 = math.copysign(sv_model, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    diff2 = b0 - d_10
    sv2 = math.copysign(sv_model, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20, sv2

def compute_geometric_tracking_error(x, y, yaw, waypoints, N_wp, last_idx=0):
    w_size = 60
    indices = (np.arange(-20, w_size) + last_idx) % N_wp
    cand_pts = waypoints[indices]
    dist_sq = (cand_pts[:, 0] - x)**2 + (cand_pts[:, 1] - y)**2
    best_loc = int(np.argmin(dist_sq))
    closest_idx = int(indices[best_loc])
    
    if dist_sq[best_loc] > 16.0:
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
        
        if d_sq < best_dist_sq - 1e-9:
            best_dist_sq = d_sq
            best_lat_err = cand_lat_err
            best_head_err = cand_head_err
        elif abs(d_sq - best_dist_sq) <= 1e-9 and abs(cand_head_err) < abs(best_head_err):
            best_lat_err = cand_lat_err
            best_head_err = cand_head_err
            
    return best_lat_err, best_head_err, closest_idx

def analyze_chatter(act_rates_relay, relay_targets, act_deltas):
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

def run_simulation_run(args):
    """
    Executes a single multi-lap simulation run under specified perturbation parameters.
    args is a dictionary containing:
      - track_name
      - policy_name
      - target_laps (default 3)
      - seed (int or None)
      - sigma_pos (m)
      - sigma_heading (deg)
      - actual_delay_s (s, default 0.020)
      - actual_slew_rad_s (rad/s, default 3.20)
      - timing_jitter (bool, default False)
    """
    track_name = args['track_name']
    policy_name = args['policy_name']
    target_laps = args.get('target_laps', 3)
    seed = args.get('seed', None)
    sigma_pos = args.get('sigma_pos', 0.0)
    sigma_heading_deg = args.get('sigma_heading', 0.0)
    sigma_heading_rad = math.radians(sigma_heading_deg)
    actual_delay_s = args.get('actual_delay_s', 0.020)
    actual_slew = args.get('actual_slew_rad_s', SV_NOMINAL)
    timing_jitter = args.get('timing_jitter', False)
    
    # Initialize deterministic RNG if seed provided
    rng = np.random.default_rng(seed) if seed is not None else None
    
    track = TrackManager.load_track(track_name)
    start_pose = track.start_pose
    waypoints = track.waypoints
    N_wp = len(waypoints)
    track_length = track.track_length
    
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    obs = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    agent = env.sim.agents[0]
    
    # Apply actual actuator parameters to the physical simulator
    agent.params['sv_max'] = actual_slew
    agent.params['sv_min'] = -actual_slew
    
    # Set actual FIFO buffer size
    nominal_buffer_size = max(1, int(round(actual_delay_s / DT_SIM)))
    agent.steer_buffer_size = nominal_buffer_size
    
    # Controller remains frozen at nominal assumptions
    ctrl = DiagnosticMPCController(
        track, speed_scale=1.0, straight_w_psi=2.0, actuator_delay_s=0.020, model_type="yaw_first_order"
    )
    
    control_interval = 4
    current_steer_cmd = 0.0
    current_speed_cmd = 7.50
    prev_published_cmd = 0.0
    
    # Storage arrays
    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    lat_errs_25hz = []
    head_errs_25hz = []
    is_held_flags = []
    
    act_deltas = []
    act_rates_measured = []
    act_rates_relay = []
    relay_targets = []
    yaw_rates = []
    vehicle_x = []
    vehicle_y = []
    lat_errs_100hz = []
    head_errs_100hz = []
    min_lidar_scans = []
    
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
    
    max_steps = 45000
    last_geom_idx = 0
    
    for step_i in range(max_steps):
        t_sim = step_i * DT_SIM
        
        # 25-Hz Control Cycle
        if step_i % control_interval == 0:
            k = step_i // control_interval
            
            # Ground truth state
            true_x = float(agent.state[0])
            true_y = float(agent.state[1])
            true_yaw = float(agent.state[4])
            true_v = float(agent.state[3])
            true_r = float(agent.state[5])
            
            # Perturb observation if noise enabled
            if rng is not None and sigma_pos > 0.0:
                cur_x_noisy = true_x + rng.normal(0.0, sigma_pos)
                cur_y_noisy = true_y + rng.normal(0.0, sigma_pos)
            else:
                cur_x_noisy = true_x
                cur_y_noisy = true_y
                
            if rng is not None and sigma_heading_rad > 0.0:
                cur_yaw_noisy = wrap_angle(true_yaw + rng.normal(0.0, sigma_heading_rad))
            else:
                cur_yaw_noisy = true_yaw
                
            # If timing jitter enabled: randomly alternate actual buffer delay between 20ms and 30ms
            if timing_jitter and rng is not None:
                jittered_delay_steps = rng.choice([2, 3])  # 20 ms or 30 ms
                agent.steer_buffer_size = jittered_delay_steps
                
            # Controller step consumes noisy state
            steer_final, speed_final, telem = ctrl.step(
                cur_x_noisy, cur_y_noisy, cur_yaw_noisy, true_v, dt_actual=DT_CTRL, cur_r=true_r
            )
            
            raw = float(telem['opt_raw_steer'])
            raw_cmds.append(raw)
            lat_errs_25hz.append(float(telem['lateral_error']))
            head_errs_25hz.append(float(telem['heading_error']))
            cur_s = float(telem['cur_s'])
            
            # Predict arrival state (controller assumes nominal 20 ms / 3.2 rad/s)
            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) >= 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) >= 2 else 0.0
            cur_delta = float(agent.state[2])
            d_20, _ = predict_20ms_arrival(cur_delta, b0, b1, sv_model=SV_NOMINAL)
            
            # Candidate policy
            is_held = False
            if policy_name == 'RAW':
                cmd = raw
            elif policy_name == 'HOLD16':
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    is_held = True
                else:
                    cmd = raw
            elif policy_name == 'HOLD_LATTICE16':
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    is_held = True
                else:
                    cmd = round(raw / QUANTUM_NOMINAL) * QUANTUM_NOMINAL
            else:
                cmd = raw
                
            is_held_flags.append(is_held)
            if len(published_cmds) > 0:
                cmd_increments.append(abs(cmd - prev_published_cmd))
            prev_published_cmd = cmd
            published_cmds.append(cmd)
            current_steer_cmd = cmd
            current_speed_cmd = speed_final
            
            # Lap completion check
            if prev_s > track_length * 0.80 and cur_s < track_length * 0.20:
                lap_count += 1
                lap_dur = t_sim - lap_start_t
                lap_times.append(lap_dur)
                
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
        cur_target = float(agent.steer_buffer[-1]) if len(agent.steer_buffer) >= nominal_buffer_size else 0.0
        act_deltas.append(cur_delta_100hz)
        relay_targets.append(cur_target)
        
        diff_tgt = cur_target - cur_delta_100hz
        sv = math.copysign(actual_slew, diff_tgt) if abs(diff_tgt) > EPS_RELAY else 0.0
        act_rates_relay.append(sv)
        
        if len(act_deltas) > 1:
            measured_sv = (cur_delta_100hz - act_deltas[-2]) / DT_SIM
        else:
            measured_sv = 0.0
        act_rates_measured.append(measured_sv)
        
        yaw_rates.append(float(agent.state[5]))
        vehicle_x.append(float(agent.state[0]))
        vehicle_y.append(float(agent.state[1]))
        
        # 100-Hz Geometric Tracking Error against GROUND TRUTH state
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
            
    # Post-run append final state
    act_deltas.append(float(agent.state[2]))
            
    # Metrics extraction
    lat_100_arr = np.array(lat_errs_100hz)
    head_100_arr = np.array(head_errs_100hz)
    pub_cmd_arr = np.array(published_cmds)
    d_pub_arr = np.array(cmd_increments) if len(cmd_increments) > 0 else np.array([0.0])
    act_r_arr = np.array(act_rates_measured)
    
    # Phase-aligned error: e[k] = u_pub[k] - delta_actual(4k + 4)
    phase_errs = []
    for k_idx in range(len(published_cmds) - 1):
        target_idx = 4 * k_idx + 4
        if target_idx < len(act_deltas):
            phase_errs.append(published_cmds[k_idx] - act_deltas[target_idx])
    phase_errs = np.array(phase_errs, dtype=np.float64) if len(phase_errs) > 0 else np.array([0.0])
    abs_phase = np.abs(phase_errs)
    
    chatter = analyze_chatter(act_rates_relay, relay_targets, act_deltas[:-1])
    is_latt = np.abs(pub_cmd_arr - np.round(pub_cmd_arr / QUANTUM_NOMINAL) * QUANTUM_NOMINAL) < 1e-5
    
    min_lidar = float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0
    
    return {
        'track': track_name,
        'policy': policy_name,
        'seed': seed,
        'laps_completed': lap_count,
        'total_time_s': float(len(act_rates_measured) * DT_SIM),
        'total_distance_m': float(np.sum(lap_distances)),
        'lap_times': [float(x) for x in lap_times],
        'tracking': {
            'lat_rmse_100hz_cm': float(np.sqrt(np.mean(lat_100_arr**2)) * 100.0),
            'lat_mean_signed_cm': float(np.mean(lat_100_arr) * 100.0),
            'lat_median_signed_cm': float(np.median(lat_100_arr) * 100.0),
            'lat_std_cm': float(np.std(lat_100_arr) * 100.0),
            'lat_max_abs_cm': float(np.max(np.abs(lat_100_arr)) * 100.0),
            'head_rmse_100hz_deg': float(np.rad2deg(np.sqrt(np.mean(head_100_arr**2)))),
            'head_mean_signed_deg': float(np.rad2deg(np.mean(head_100_arr)))
        },
        'steering': {
            'pub_cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(pub_cmd_arr**2)))),
            'delta_pub_rms_mrad': float(np.sqrt(np.mean(d_pub_arr**2)) * 1000.0),
            'delta_pub_max_mrad': float(np.max(d_pub_arr) * 1000.0),
            'steer_rate_rms_rad_s': float(np.sqrt(np.mean(act_r_arr**2))),
            'steer_rate_max_rad_s': float(np.max(np.abs(act_r_arr))),
            'phase_err_rms_mrad': float(np.sqrt(np.mean(phase_errs**2)) * 1000.0),
            'phase_err_mean_abs_mrad': float(np.mean(abs_phase) * 1000.0),
            'phase_err_p95_mrad': float(np.percentile(abs_phase, 95) * 1000.0),
            'phase_err_max_mrad': float(np.max(abs_phase) * 1000.0),
            'phase_gt_1e9': int(np.sum(abs_phase > 1e-9)),
            'phase_gt_1e6': int(np.sum(abs_phase > 1e-6)),
            'phase_samples': len(phase_errs)
        },
        'chatter': chatter,
        'command_mod': {
            'frac_held': float(np.mean(is_held_flags)),
            'frac_lattice': float(np.mean(is_latt))
        },
        'safety': {
            'collisions': int(collisions_total),
            'min_lidar_m': min_lidar,
            'out_of_bounds': int(np.sum(np.abs(lat_100_arr) > 2.0))
        }
    }

def main():
    print("="*100)
    print("STARTING EXPERIMENT 24: PHASE 1 — NOMINAL REGRESSION VS EXPERIMENT 23")
    print("="*100)
    
    # Load Exp 23 reference for verification
    ref_path = '/home/yeswanth/roboracer_ws/experiments/experiment23/experiment23_results.json'
    with open(ref_path) as f:
        d23 = json.load(f)
        
    tasks = []
    for t_name in ALL_TRACKS:
        for pol in POLICIES:
            tasks.append({
                'track_name': t_name,
                'policy_name': pol,
                'target_laps': 3,
                'seed': None,
                'sigma_pos': 0.0,
                'sigma_heading': 0.0,
                'actual_delay_s': 0.020,
                'actual_slew_rad_s': 3.20,
                'timing_jitter': False
            })
            
    print(f"Launching Phase 1 regression: {len(tasks)} runs across 7 tracks using process pool...")
    t0 = time.time()
    with mp.Pool(processes=min(8, os.cpu_count())) as pool:
        phase1_results = pool.map(run_simulation_run, tasks)
    dur = time.time() - t0
    print(f"Phase 1 finished in {dur:.2f} s wallclock time.")
    
    # Compare with Experiment 23
    print("\n" + "="*120)
    print("PHASE 1 REGRESSION AUDIT: EXPERIMENT 24 vs EXPERIMENT 23 REFERENCE")
    print("="*120)
    hdr = f"{'Track':<12} | {'Policy':<15} | {'E24 LatRMSE':<12} | {'E23 LatRMSE':<12} | {'Delta Lat':<10} | {'E24 SlewRMS':<12} | {'E23 SlewRMS':<12} | {'E24 Reversals':<13} | {'E23 Reversals':<13} | {'Match?'}"
    print(hdr)
    print("-" * 140)
    
    all_matched = True
    for res in phase1_results:
        t = res['track']
        p = res['policy']
        e23 = d23['runs'][t][p]['overall']
        
        e24_lat = res['tracking']['lat_rmse_100hz_cm']
        e23_lat = e23['tracking']['lat_rmse_100hz_cm']
        d_lat = abs(e24_lat - e23_lat)
        
        e24_slew = res['steering']['steer_rate_rms_rad_s']
        e23_slew = e23['steering']['steer_rate_rms_rad_s']
        
        e24_rev = res['chatter']['relay_sign_reversals']
        e23_rev = e23['chatter']['relay_sign_reversals']
        
        # Strict match check (within numerical roundoff)
        matched = (d_lat < 0.005) and (abs(e24_slew - e23_slew) < 0.005) and (e24_rev == e23_rev)
        if not matched:
            all_matched = False
            
        print(f"{t:<12} | {p:<15} | {e24_lat:8.3f} cm | {e23_lat:8.3f} cm | {d_lat:8.4f}cm | {e24_slew:8.3f} rad | {e23_slew:8.3f} rad | {e24_rev:<13d} | {e23_rev:<13d} | {'PASS' if matched else 'FAIL'}")
        
    print("-" * 140)
    print(f"OVERALL PHASE 1 REGRESSION STATUS: {'PASSED (ALL MATCHED)' if all_matched else 'FAILED (DISCREPANCY DETECTED)'}")
    
    if not all_matched:
        print("[ERROR] Phase 1 regression failed to match Experiment 23! Halting per execution rules.")
        sys.exit(1)
        
    # Save Phase 1 results
    p1_out = '/home/yeswanth/roboracer_ws/experiments/experiment24_robustness/phase1_nominal_regression.json'
    with open(p1_out, 'w') as f:
        json.dump(phase1_results, f, indent=2)
    print(f"Saved Phase 1 regression data to {p1_out}")

if __name__ == '__main__':
    main()
