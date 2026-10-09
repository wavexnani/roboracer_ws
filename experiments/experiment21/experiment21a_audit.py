#!/usr/bin/env python3
"""
EXPERIMENT 21A: Rigorous Validation Audit and Correction Suite

Performs exhaustive verification of Experiment 21 methodology and results:
1. Actuator model verification against f110_gym (machine precision check).
2. Exact FIFO state reconstruction on Experiment 18 telemetry (steer_cmd vs raw MPC).
3. Critical audit of Policy H threshold sweep (0 to 32 mrad) measuring hold, raw, and lattice fractions.
4. Critical audit of Continuous Policy C using exact lifted actuator simulation (1 mrad grid).
5. Closed-loop determinism verification across full 2600-step trajectories for RAW Run 1 vs Run 2.
6. Corrected closed-loop benchmark on Spielberg (26.0s) for RAW, HOLD_16, HOLD_LATTICE_16, CONTINUOUS_C.
7. Corrected frequency analysis (FFT resolution, DC exclusion, >2 Hz dominant modes, normalized band powers).
8. Separation of relay reversals, two-step cycles, target crossings, and alternating streaks.
9. 100-Hz reconstructed geometric tracking errors vs 25-Hz controller-reported telemetry.
10. Precise phase-aligned actuator error timeline and metric analysis.

Saves comprehensive results to experiment21a_results.json.
NO production MPC or simulator modifications.
"""

import os
import sys
import time
import math
import json
import numpy as np

# Ensure path includes local workspace packages
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

# ==============================================================================
# EXACT LIFTED ACTUATOR SIMULATION
# ==============================================================================
def exact_lifted_actuator_step(X_k, u_k):
    """
    Exact 4-step propagation of the f110_gym steering relay over 40 ms.
    State X_k = [delta_actual, b0, b1].
    Returns:
      X_next: np.ndarray([delta_next, new_b0, new_b1])
      step_records: list of dicts detailing each 10 ms substep.
    """
    delta, b0, b1 = float(X_k[0]), float(X_k[1]), float(X_k[2])
    buf = [b0, b1]
    step_records = []
    
    for step_idx in range(4):
        target = buf[-1]
        buf = [u_k, buf[0]]
        diff = target - delta
        if abs(diff) > EPS_RELAY:
            sv = math.copysign(SV_MAX, diff)
        else:
            sv = 0.0
        delta_next = delta + sv * DT_SIM
        step_records.append({
            'substep': step_idx,
            'time_ms': (step_idx + 1) * 10,
            'target': target,
            'delta_start': delta,
            'sv': sv,
            'delta_end': delta_next,
            'diff': diff
        })
        delta = delta_next
        
    X_next = np.array([delta, buf[0], buf[1]], dtype=np.float64)
    return X_next, step_records

def predict_20ms_arrival(cur_d, b0, b1):
    """Predicts delta at t_k + 20ms using in-flight FIFO commands [b0, b1]."""
    diff1 = b1 - cur_d
    sv1 = math.copysign(SV_MAX, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    
    diff2 = b0 - d_10
    sv2 = math.copysign(SV_MAX, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20, sv2

# ==============================================================================
# AUDITED POLICY C (EXACT LIFTED CANDIDATE OPTIMIZATION OVER 1 mrad GRID)
# ==============================================================================
def evaluate_continuous_policy_c_exact(raw_k, X_k, prev_u, w_target=1.0, w_motion=0.005, w_reverse=0.05):
    """
    Evaluates exact lifted simulation for every candidate on a 1 mrad continuous grid.
    No analytical shortcuts.
    """
    u_min = max(-0.35, raw_k - 0.064)
    u_max = min(0.35, raw_k + 0.064)
    n_pts = int(round((u_max - u_min) / 0.001)) + 1
    candidates = np.linspace(u_min, u_max, n_pts)
    
    d_20, _ = predict_20ms_arrival(X_k[0], X_k[1], X_k[2])
    candidates = np.unique(np.append(candidates, [raw_k, d_20]))
    
    best_u = raw_k
    best_cost = float('inf')
    best_records = None
    
    for u in candidates:
        X_next, step_recs = exact_lifted_actuator_step(X_k, u)
        d_40 = X_next[0]
        motion = sum(abs(r['sv']) for r in step_recs) * DT_SIM
        reversals = sum(1 for j in range(1, 4) if step_recs[j]['sv'] * step_recs[j-1]['sv'] < 0)
        
        cost = w_target * (d_40 - raw_k)**2 + w_motion * motion + w_reverse * reversals
        
        if cost < best_cost - 1e-9:
            best_cost = cost
            best_u = u
            best_records = step_recs
        elif abs(cost - best_cost) <= 1e-9:
            # Tie breaker: choose candidate closest to raw_k to preserve MPC intent
            if abs(u - raw_k) < abs(best_u - raw_k):
                best_u = u
                best_records = step_recs
                
    return best_u, best_cost, best_records

# ==============================================================================
# 100-Hz GEOMETRIC TRACKING ERROR RECONSTRUCTION
# ==============================================================================
def compute_geometric_tracking_error(x, y, yaw, waypoints, N_wp):
    """
    Independently computes orthogonal lateral error and heading error against track centerline.
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
                tx = 1.0
                ty = 0.0
            nx = -ty
            ny = tx
            best_lat_err = float((p[0] - p_proj[0]) * nx + (p[1] - p_proj[1]) * ny)
            seg_psi = math.atan2(ty, tx)
            best_head_err = float((yaw - seg_psi + math.pi) % (2.0 * math.pi) - math.pi)
            
    return abs(best_lat_err), abs(best_head_err)

# ==============================================================================
# AUDIT PART 1: ACTUATOR VERIFICATION AGAINST F110_GYM
# ==============================================================================
def audit_part1_actuator_verification(n_trials=100):
    print("\n" + "="*80)
    print("AUDIT PART 1: ACTUATOR VERIFICATION AGAINST OFFICIAL F110_GYM")
    print("="*80)
    track = TrackManager.load_track('Spielberg')
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    agent = env.sim.agents[0]
    
    rng = np.random.default_rng(2026)
    max_err_delta = 0.0
    max_err_b0 = 0.0
    max_err_b1 = 0.0
    
    for _ in range(n_trials):
        env.reset(np.array([[track.start_pose[0], track.start_pose[1], track.start_pose[2]]]))
        rand_delta = float(rng.uniform(-0.35, 0.35))
        rand_b0 = float(rng.uniform(-0.35, 0.35))
        rand_b1 = float(rng.uniform(-0.35, 0.35))
        rand_u = float(rng.uniform(-0.35, 0.35))
        
        agent.state[2] = rand_delta
        agent.steer_buffer = np.array([rand_b0, rand_b1])
        
        for _ in range(4):
            env.step(np.array([[rand_u, 5.0]]))
            
        gym_delta = float(agent.state[2])
        gym_b0 = float(agent.steer_buffer[0])
        gym_b1 = float(agent.steer_buffer[1])
        
        X_k = [rand_delta, rand_b0, rand_b1]
        X_next, _ = exact_lifted_actuator_step(X_k, rand_u)
        
        max_err_delta = max(max_err_delta, abs(gym_delta - X_next[0]))
        max_err_b0 = max(max_err_b0, abs(gym_b0 - X_next[1]))
        max_err_b1 = max(max_err_b1, abs(gym_b1 - X_next[2]))
        
    passed = (max_err_delta < 1e-12) and (max_err_b0 < 1e-12) and (max_err_b1 < 1e-12)
    print(f"  Trials evaluated: {n_trials}")
    print(f"  Max error delta:  {max_err_delta:.2e} rad")
    print(f"  Max error b0:     {max_err_b0:.2e} rad")
    print(f"  Max error b1:     {max_err_b1:.2e} rad")
    print(f"  Actuator Verification: {'PASSED (Machine Precision)' if passed else 'FAILED'}")
    return {
        'passed': bool(passed),
        'max_err_delta': max_err_delta,
        'max_err_b0': max_err_b0,
        'max_err_b1': max_err_b1
    }

# ==============================================================================
# AUDIT PART 2: FIFO RECONSTRUCTION ON EXP 18 TELEMETRY
# ==============================================================================
def audit_part2_fifo_reconstruction():
    print("\n" + "="*80)
    print("AUDIT PART 2: FIFO STATE RECONSTRUCTION ON EXPERIMENT 18 TELEMETRY")
    print("="*80)
    data_path = '/home/yeswanth/roboracer_ws/experiments/experiment20/exp18_raw_commands.json'
    with open(data_path) as f:
        data = json.load(f)
    N = len(data)
    
    # 1. Reconstruct using published steer_cmd[k-1]
    max_err_tgt_published = 0.0
    for k in range(1, N):
        rec_b1 = data[k-1]['steer_cmd']
        rec_err = abs(rec_b1 - data[k]['relay_target'])
        if rec_err > max_err_tgt_published:
            max_err_tgt_published = rec_err
            
    # 2. Reconstruct using raw_mpc_delta[k-1]
    max_err_tgt_raw = 0.0
    mismatch_raw_count = 0
    for k in range(1, N):
        rec_b1_raw = data[k-1]['raw_mpc_delta']
        rec_err_raw = abs(rec_b1_raw - data[k]['relay_target'])
        if rec_err_raw > 1e-6:
            mismatch_raw_count += 1
        if rec_err_raw > max_err_tgt_raw:
            max_err_tgt_raw = rec_err_raw
            
    # 3. Open-loop simulation across all 800 cycles using exact lifted step
    cur_d = data[0]['actual_steering']
    b0 = data[0]['steer_cmd']
    b1 = data[0]['steer_cmd']
    max_err_delta_sim = 0.0
    for k in range(N):
        err_d = abs(cur_d - data[k]['actual_steering'])
        if err_d > max_err_delta_sim:
            max_err_delta_sim = err_d
        u = data[k]['steer_cmd']
        X_next, _ = exact_lifted_actuator_step([cur_d, b0, b1], u)
        cur_d, b0, b1 = X_next[0], X_next[1], X_next[2]
        
    print(f"  Total cycles: {N}")
    print(f"  Published steer_cmd[k-1] vs relay_target max error: {max_err_tgt_published:.2e} rad")
    print(f"  Raw MPC raw_mpc_delta[k-1] vs relay_target max error: {max_err_tgt_raw*1000:.2f} mrad")
    print(f"  Raw MPC mismatches (>1e-6 rad): {mismatch_raw_count}/{N-1} ({mismatch_raw_count/(N-1)*100:.1f}%)")
    print(f"  800-cycle open-loop steering reconstruction max error: {max_err_delta_sim:.2e} rad")
    
    return {
        'max_err_tgt_published': max_err_tgt_published,
        'max_err_tgt_raw_mrad': max_err_tgt_raw * 1000.0,
        'mismatch_raw_fraction': mismatch_raw_count / (N - 1),
        'max_err_delta_sim': max_err_delta_sim
    }

# ==============================================================================
# AUDIT PART 3: POLICY H THRESHOLD SWEEP AND LATTICE AUDIT
# ==============================================================================
def audit_part3_policy_h_sweep():
    print("\n" + "="*80)
    print("AUDIT PART 3: POLICY H THRESHOLD SWEEP & LATTICE PURITY AUDIT")
    print("="*80)
    data_path = '/home/yeswanth/roboracer_ws/experiments/experiment20/exp18_raw_commands.json'
    with open(data_path) as f:
        data = json.load(f)
    N = len(data)
    
    thresholds_mrad = [0, 4, 8, 12, 16, 20, 24, 28, 32]
    sweep_results = []
    
    for eps_mrad in thresholds_mrad:
        eps_rad = eps_mrad / 1000.0
        cur_d = data[0]['actual_steering']
        b0 = data[0]['steer_cmd']
        b1 = data[0]['steer_cmd']
        
        held_count = 0
        raw_count = 0
        lattice_count = 0
        cmds = []
        delta_us = []
        act_svs = []
        reversals = 0
        errors_40ms = []
        
        prev_u = 0.0
        for k in range(N):
            raw_k = data[k]['raw_mpc_delta']
            d_20, _ = predict_20ms_arrival(cur_d, b0, b1)
            
            # Policy H decision
            if abs(raw_k - d_20) < eps_rad:
                u_k = d_20
                held_count += 1
            else:
                u_k = raw_k
                raw_count += 1
                
            # Lattice check
            if abs(u_k - round(u_k / QUANTUM) * QUANTUM) < 1e-5:
                lattice_count += 1
                
            cmds.append(u_k)
            if k > 0:
                delta_us.append(abs(u_k - prev_u))
            prev_u = u_k
            
            X_next, recs = exact_lifted_actuator_step([cur_d, b0, b1], u_k)
            for j in range(4):
                act_svs.append(recs[j]['sv'])
                if len(act_svs) > 1 and recs[j]['sv'] * act_svs[-2] < 0:
                    reversals += 1
            errors_40ms.append(abs(raw_k - X_next[0]))
            
            cur_d, b0, b1 = X_next[0], X_next[1], X_next[2]
            
        cmds = np.array(cmds)
        delta_us = np.array(delta_us)
        act_svs = np.array(act_svs)
        errors_40ms = np.array(errors_40ms)
        
        sweep_results.append({
            'eps_mrad': eps_mrad,
            'frac_held': held_count / N,
            'frac_raw': raw_count / N,
            'frac_lattice': lattice_count / N,
            'frac_non_lattice': 1.0 - (lattice_count / N),
            'rms_delta_u_mrad': float(np.sqrt(np.mean(delta_us**2)) * 1000),
            'rms_sv_rad_s': float(np.sqrt(np.mean(act_svs**2))),
            'reversals': int(reversals),
            'zero_state_pct': float(np.mean(act_svs == 0.0) * 100.0),
            'err_40ms_rms_mrad': float(np.sqrt(np.mean(errors_40ms**2)) * 1000)
        })
        
    print(f"{'Eps [mrad]':<11} | {'Held %':<8} | {'Raw %':<8} | {'Lattice %':<10} | {'Non-Lat %':<10} | {'RMS(Δu)':<9} | {'RMS(sv)':<9} | {'Reversals':<10} | {'Err 40ms RMS'}")
    print("-" * 105)
    for r in sweep_results:
        print(f"{r['eps_mrad']:<11d} | {r['frac_held']*100:6.1f}% | {r['frac_raw']*100:6.1f}% | {r['frac_lattice']*100:8.1f}% | {r['frac_non_lattice']*100:8.1f}% | {r['rms_delta_u_mrad']:6.2f} mm | {r['rms_sv_rad_s']:6.3f}   | {r['reversals']:<10d} | {r['err_40ms_rms_mrad']:8.2f} mm")
        
    return sweep_results

# ==============================================================================
# AUDIT PART 4: RIGOROUS AUDIT OF CONTINUOUS POLICY C (OFFLINE)
# ==============================================================================
def audit_part4_policy_c_offline():
    print("\n" + "="*80)
    print("AUDIT PART 4: CONTINUOUS POLICY C CANDIDATE AUDIT (OFFLINE)")
    print("="*80)
    data_path = '/home/yeswanth/roboracer_ws/experiments/experiment20/exp18_raw_commands.json'
    with open(data_path) as f:
        data = json.load(f)
    N = len(data)
    
    cur_d = data[0]['actual_steering']
    b0 = data[0]['steer_cmd']
    b1 = data[0]['steer_cmd']
    prev_u = 0.0
    
    selected_cmds = []
    dist_from_lattice = []
    
    t0 = time.perf_counter()
    for k in range(N):
        raw_k = data[k]['raw_mpc_delta']
        X_k = [cur_d, b0, b1]
        best_u, _, _ = evaluate_continuous_policy_c_exact(raw_k, X_k, prev_u)
        selected_cmds.append(best_u)
        
        d_lat = abs(best_u - round(best_u / QUANTUM) * QUANTUM)
        dist_from_lattice.append(d_lat)
        
        prev_u = best_u
        X_next, _ = exact_lifted_actuator_step(X_k, best_u)
        cur_d, b0, b1 = X_next[0], X_next[1], X_next[2]
        
    t_calc = time.perf_counter() - t0
    selected_cmds = np.array(selected_cmds)
    dist_from_lattice = np.array(dist_from_lattice)
    
    n_unique = len(np.unique(selected_cmds))
    pct_lattice = float(np.mean(dist_from_lattice < 1e-5) * 100.0)
    min_dist_lattice = float(np.min(dist_from_lattice[dist_from_lattice > 1e-5])) if np.any(dist_from_lattice > 1e-5) else 0.0
    mean_dist_lattice = float(np.mean(dist_from_lattice))
    
    hist_counts, hist_bins = np.histogram(selected_cmds, bins=10)
    
    print(f"  Cycles evaluated: {N}")
    print(f"  Total evaluation time: {t_calc:.2f} s ({t_calc/N*1000:.2f} ms/cycle)")
    print(f"  Unique selected command values: {n_unique} / {N}")
    print(f"  Percentage exactly on lattice (n*q): {pct_lattice:.2f}%")
    print(f"  Minimum distance to nearest lattice (non-lattice points): {min_dist_lattice*1000:.3f} mrad")
    print(f"  Mean distance to nearest lattice: {mean_dist_lattice*1000:.3f} mrad")
    print(f"  Histogram bins (rad): {list(np.round(hist_bins, 3))}")
    print(f"  Histogram counts:     {list(hist_counts)}")
    
    return {
        'n_unique': n_unique,
        'pct_lattice': pct_lattice,
        'min_dist_lattice_mrad': min_dist_lattice * 1000.0,
        'mean_dist_lattice_mrad': mean_dist_lattice * 1000.0,
        'hist_counts': [int(c) for c in hist_counts],
        'hist_bins': [float(b) for b in hist_bins]
    }

# ==============================================================================
# AUDIT PART 5: CLOSED-LOOP SIMULATION RUNNER & FULL METRIC EVALUATOR
# ==============================================================================
def run_closed_loop_simulation(policy_name, duration_s=26.0):
    track = TrackManager.load_track('Spielberg')
    start_pose = track.start_pose
    map_path_no_ext = track.map_path_no_ext
    waypoints = track.waypoints
    N_wp = len(waypoints)
    
    env = gym.make('f110_gym:f110-v0', map=map_path_no_ext, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[start_pose[0], start_pose[1], start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret
    agent = env.sim.agents[0]
    
    ctrl = DiagnosticMPCController(
        track, speed_scale=1.0, straight_w_psi=2.0, actuator_delay_s=0.020, model_type="yaw_first_order"
    )
    
    max_steps = int(round(duration_s / DT_SIM))  # 2600 steps
    control_interval = 4
    
    current_steer_cmd = 0.0
    current_speed_cmd = 7.50
    prev_published_cmd = 0.0
    
    # 25-Hz arrays
    raw_cmds = []
    published_cmds = []
    cmd_increments = []
    lat_errs_25hz = []
    head_errs_25hz = []
    mpc_solve_times = []
    policy_calc_times = []
    arrival_d20_preds = []
    
    # 100-Hz arrays
    act_deltas = []
    act_rates_measured = []
    act_rates_relay = []
    yaw_rates = []
    vehicle_x = []
    vehicle_y = []
    vehicle_yaw = []
    vehicle_v = []
    lat_errs_100hz = []
    head_errs_100hz = []
    min_lidar_scans = []
    relay_targets = []
    
    collisions = 0
    held_cycles = 0
    
    for step_i in range(max_steps):
        t_sim = step_i * DT_SIM
        
        # 25-Hz Control Cycle
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
            lat_errs_25hz.append(abs(telem['lateral_error']))
            head_errs_25hz.append(abs(telem['heading_error']))
            
            b0 = float(agent.steer_buffer[0]) if len(agent.steer_buffer) == 2 else 0.0
            b1 = float(agent.steer_buffer[1]) if len(agent.steer_buffer) == 2 else 0.0
            cur_delta = float(agent.state[2])
            d_20, _ = predict_20ms_arrival(cur_delta, b0, b1)
            arrival_d20_preds.append(d_20)
            
            t_pol_start = time.perf_counter()
            if policy_name in ('RAW', 'REPEAT_RAW'):
                cmd = raw
            elif policy_name == 'HOLD_16':
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    held_cycles += 1
                else:
                    cmd = raw
            elif policy_name == 'HOLD_LATTICE_16':
                if abs(raw - d_20) < 0.016:
                    cmd = d_20
                    held_cycles += 1
                else:
                    cmd = round(raw / QUANTUM) * QUANTUM
            elif policy_name == 'CONTINUOUS_C':
                X_k = [cur_delta, b0, b1]
                cmd, _, _ = evaluate_continuous_policy_c_exact(raw, X_k, prev_published_cmd)
                if abs(cmd - d_20) < 1e-5:
                    held_cycles += 1
            else:
                cmd = raw
                
            policy_calc_times.append(time.perf_counter() - t_pol_start)
            
            if len(published_cmds) > 0:
                cmd_increments.append(abs(cmd - prev_published_cmd))
            prev_published_cmd = cmd
            published_cmds.append(cmd)
            current_steer_cmd = cmd
            current_speed_cmd = speed_final
            
        # 100-Hz Recording before env.step
        cur_delta_100hz = float(agent.state[2])
        act_deltas.append(cur_delta_100hz)
        cur_relay_target = float(agent.steer_buffer[-1]) if len(agent.steer_buffer) == 2 else 0.0
        relay_targets.append(cur_relay_target)
        
        # Exact relay velocity according to model
        diff_tgt = cur_relay_target - cur_delta_100hz
        expected_sv = math.copysign(SV_MAX, diff_tgt) if abs(diff_tgt) > EPS_RELAY else 0.0
        act_rates_relay.append(expected_sv)
        
        # Measured rate via backward difference
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
        
        if 'scans' in obs and len(obs['scans']) > 0 and len(obs['scans'][0]) > 0:
            min_lidar_scans.append(float(np.min(obs['scans'][0])))
            
        obs, _, _, _ = env.step(np.array([[current_steer_cmd, current_speed_cmd]]))
        if agent.in_collision:
            collisions += 1
            
    # Final step steering angle
    act_deltas.append(float(agent.state[2]))
    
    # Conversions
    raw_cmds = np.array(raw_cmds)
    published_cmds = np.array(published_cmds)
    cmd_increments = np.array(cmd_increments)
    act_deltas = np.array(act_deltas)
    act_rates_measured = np.array(act_rates_measured)
    act_rates_relay = np.array(act_rates_relay)
    yaw_rates = np.array(yaw_rates)
    vehicle_x = np.array(vehicle_x)
    vehicle_y = np.array(vehicle_y)
    vehicle_yaw = np.array(vehicle_yaw)
    vehicle_v = np.array(vehicle_v)
    lat_errs_25hz = np.array(lat_errs_25hz)
    head_errs_25hz = np.array(head_errs_25hz)
    lat_errs_100hz = np.array(lat_errs_100hz)
    head_errs_100hz = np.array(head_errs_100hz)
    mpc_solve_times = np.array(mpc_solve_times) * 1000.0  # ms
    policy_calc_times = np.array(policy_calc_times) * 1000.0  # ms
    
    # --------------------------------------------------------------------------
    # RELAY DYNAMICS & REVERSAL AUDIT
    # --------------------------------------------------------------------------
    nonzero_relay = int(np.sum(np.abs(act_rates_relay) > EPS_RELAY))
    reversals = 0
    plus_minus_pairs = 0
    minus_plus_pairs = 0
    two_step_cycles = 0
    
    for i in range(1, len(act_rates_relay)):
        if act_rates_relay[i] * act_rates_relay[i-1] < 0:
            reversals += 1
            if act_rates_relay[i-1] > 0 and act_rates_relay[i] < 0:
                plus_minus_pairs += 1
            elif act_rates_relay[i-1] < 0 and act_rates_relay[i] > 0:
                minus_plus_pairs += 1
        if i >= 2 and (act_rates_relay[i] * act_rates_relay[i-1] < 0) and (act_rates_relay[i-1] * act_rates_relay[i-2] < 0):
            two_step_cycles += 1
            
    # Alternating sequences
    flips = (act_rates_relay[1:] * act_rates_relay[:-1] < 0).astype(int)
    cur_streak = 0
    longest_streak = 0
    total_alt_steps = 0
    for f in flips:
        if f == 1:
            cur_streak += 1
            longest_streak = max(longest_streak, cur_streak + 1)
            total_alt_steps += 1
        else:
            cur_streak = 0
            
    # --------------------------------------------------------------------------
    # FREQUENCY ANALYSIS (WINDOW: 1.0s to 10.0s = 900 samples at 100 Hz)
    # --------------------------------------------------------------------------
    straight_r = yaw_rates[100:1000]
    N_fft = len(straight_r)  # 900
    df = 1.0 / (N_fft * DT_SIM)  # 0.111111 Hz
    freqs = np.fft.rfftfreq(N_fft, d=DT_SIM)
    
    dc_component = float(np.mean(straight_r))
    fft_raw = np.fft.rfft(straight_r - dc_component)
    fft_amp = np.abs(fft_raw)
    
    # Normalized spectral power by Parseval's theorem: RMS = sqrt(2/N^2 * sum(|X|^2))
    def band_rms(f_low, f_high):
        idx = np.where((freqs >= f_low) & (freqs <= f_high))[0]
        if len(idx) == 0: return 0.0
        return float(np.sqrt(np.sum(fft_amp[idx]**2) * 2.0 / (N_fft**2)))
        
    energy_2_4 = band_rms(2.0, 4.0)
    energy_4_8 = band_rms(4.0, 8.0)
    energy_5_7 = band_rms(5.0, 7.0)
    
    # Original Exp 21 metric formula for direct cross-audit
    idx_2_4_orig = np.where((freqs >= 2.0) & (freqs <= 4.0))[0]
    idx_4_8_orig = np.where((freqs >= 4.0) & (freqs <= 8.0))[0]
    idx_5_7_orig = np.where((freqs >= 5.0) & (freqs <= 7.0))[0]
    orig_energy_2_4 = float(np.sqrt(np.mean(fft_amp[idx_2_4_orig]**2)) / len(freqs) * 1000)
    orig_energy_4_8 = float(np.sqrt(np.mean(fft_amp[idx_4_8_orig]**2)) / len(freqs) * 1000)
    orig_energy_5_7 = float(np.sqrt(np.mean(fft_amp[idx_5_7_orig]**2)) / len(freqs) * 1000)
    
    dom_overall_idx = np.argmax(fft_amp[1:]) + 1
    dom_overall_freq = float(freqs[dom_overall_idx])
    
    idx_gt2 = np.where(freqs >= 2.0)[0]
    dom_gt2_idx = idx_gt2[np.argmax(fft_amp[idx_gt2])]
    dom_gt2_freq = float(freqs[dom_gt2_idx])
    
    dom_4_8_idx = idx_4_8_orig[np.argmax(fft_amp[idx_4_8_orig])] if len(idx_4_8_orig) > 0 else 0
    dom_4_8_freq = float(freqs[dom_4_8_idx])
    
    dom_5_7_idx = idx_5_7_orig[np.argmax(fft_amp[idx_5_7_orig])] if len(idx_5_7_orig) > 0 else 0
    dom_5_7_freq = float(freqs[dom_5_7_idx])
    
    # --------------------------------------------------------------------------
    # PHASE-ALIGNED ACTUATOR TRACKING ERROR
    # --------------------------------------------------------------------------
    # Cycle k issued at t_k (step 4*k).
    # t_k + 20ms is step 4*k + 2.
    # t_k + 40ms is step 4*k + 4.
    K = len(published_cmds) - 1
    errs_40ms_published = []
    errs_40ms_raw = []
    errs_20ms_published = []
    errs_00ms_misaligned = []
    
    for k in range(K):
        u_pub = published_cmds[k]
        u_raw = raw_cmds[k]
        d_00 = act_deltas[4 * k]
        d_20 = act_deltas[4 * k + 2]
        d_40 = act_deltas[4 * k + 4]
        
        errs_40ms_published.append(u_pub - d_40)
        errs_40ms_raw.append(u_raw - d_40)
        errs_20ms_published.append(u_pub - d_20)
        errs_00ms_misaligned.append(u_pub - d_00)
        
    errs_40ms_published = np.array(errs_40ms_published)
    errs_40ms_raw = np.array(errs_40ms_raw)
    errs_20ms_published = np.array(errs_20ms_published)
    errs_00ms_misaligned = np.array(errs_00ms_misaligned)
    
    # --------------------------------------------------------------------------
    # LATTICE PURITY CHECK ON CLOSED-LOOP COMMANDS
    # --------------------------------------------------------------------------
    is_lattice = np.abs(published_cmds - np.round(published_cmds / QUANTUM) * QUANTUM) < 1e-5
    frac_lattice = float(np.mean(is_lattice))
    frac_held = float(held_cycles / len(published_cmds))
    
    s_progress = float(telem['cur_s'])
    
    return {
        'policy': policy_name,
        # Commands
        'cmd_rms_deg': float(np.rad2deg(np.sqrt(np.mean(published_cmds**2)))),
        'cmd_inc_rms_mrad': float(np.sqrt(np.mean(cmd_increments**2)) * 1000),
        'frac_held': frac_held,
        'frac_lattice': frac_lattice,
        'frac_non_lattice': 1.0 - frac_lattice,
        # Actuator Rates
        'rate_rms_rad_s': float(np.sqrt(np.mean(act_rates_measured**2))),
        'rate_max_rad_s': float(np.max(np.abs(act_rates_measured))),
        # Relay Reversals
        'nonzero_relay_samples': nonzero_relay,
        'relay_reversals': reversals,
        'two_step_lc_count': two_step_cycles,
        'plus_minus_pairs': plus_minus_pairs,
        'minus_plus_pairs': minus_plus_pairs,
        'longest_alternating_seq': longest_streak,
        'total_alternating_steps': total_alt_steps,
        # Frequency Metrics
        'frequency_resolution_hz': df,
        'dc_component_rad_s': dc_component,
        'dom_overall_freq_hz': dom_overall_freq,
        'dom_gt2_freq_hz': dom_gt2_freq,
        'dom_4_8_freq_hz': dom_4_8_freq,
        'dom_5_7_freq_hz': dom_5_7_freq,
        'energy_2_4_rad_s': energy_2_4,
        'energy_4_8_rad_s': energy_4_8,
        'energy_5_7_rad_s': energy_5_7,
        'orig_energy_2_4': orig_energy_2_4,
        'orig_energy_4_8': orig_energy_4_8,
        'orig_energy_5_7': orig_energy_5_7,
        # Tracking: 25-Hz Controller
        'lat_rmse_25hz_cm': float(np.sqrt(np.mean(lat_errs_25hz**2)) * 100.0),
        'max_lat_err_25hz_cm': float(np.max(lat_errs_25hz) * 100.0),
        'head_rmse_25hz_deg': float(np.rad2deg(np.sqrt(np.mean(head_errs_25hz**2)))),
        # Tracking: 100-Hz Reconstructed Geometric
        'lat_rmse_100hz_cm': float(np.sqrt(np.mean(lat_errs_100hz**2)) * 100.0),
        'max_lat_err_100hz_cm': float(np.max(lat_errs_100hz) * 100.0),
        'head_rmse_100hz_deg': float(np.rad2deg(np.sqrt(np.mean(head_errs_100hz**2)))),
        # Phase-Aligned Actuator Error
        'err_40ms_published_mrad': float(np.sqrt(np.mean(errs_40ms_published**2)) * 1000),
        'err_40ms_raw_mrad': float(np.sqrt(np.mean(errs_40ms_raw**2)) * 1000),
        'err_20ms_published_mrad': float(np.sqrt(np.mean(errs_20ms_published**2)) * 1000),
        'err_00ms_misaligned_mrad': float(np.sqrt(np.mean(errs_00ms_misaligned**2)) * 1000),
        # Safety & Lap Progress
        'min_lidar_m': float(np.min(min_lidar_scans)) if len(min_lidar_scans) > 0 else 0.0,
        'collisions': int(collisions),
        's_final_m': float(s_progress),
        'lap_completed': bool(s_progress >= track.track_length),
        'lap_progress_pct': float(s_progress / track.track_length * 100.0),
        # Computation
        'mean_solve_ms': float(np.mean(mpc_solve_times)),
        'max_solve_ms': float(np.max(mpc_solve_times)),
        'mean_policy_calc_us': float(np.mean(policy_calc_times) * 1000.0),
        # Full timeseries for determinism
        '_timeseries': {
            'steering': act_deltas[:2600].tolist(),
            'x': vehicle_x.tolist(),
            'y': vehicle_y.tolist(),
            'yaw': vehicle_yaw.tolist(),
            'v': vehicle_v.tolist(),
            'raw_cmds': raw_cmds.tolist()
        }
    }

# ==============================================================================
# MAIN EXECUTION PIPELINE
# ==============================================================================
def main():
    print("\n" + "="*80)
    print("STARTING EXPERIMENT 21A: RIGOROUS AUDIT AND CORRECTION SUITE")
    print("="*80)
    
    results = {}
    
    # 1. Actuator Verification
    results['part1_actuator_verification'] = audit_part1_actuator_verification(100)
    
    # 2. FIFO Reconstruction
    results['part2_fifo_reconstruction'] = audit_part2_fifo_reconstruction()
    
    # 3. Policy H Sweep
    results['part3_policy_h_sweep'] = audit_part3_policy_h_sweep()
    
    # 4. Continuous Policy C Offline Audit
    results['part4_policy_c_offline'] = audit_part4_policy_c_offline()
    
    # 5. Closed-Loop Simulations
    policies = ['RAW', 'HOLD_16', 'HOLD_LATTICE_16', 'CONTINUOUS_C', 'REPEAT_RAW']
    cl_results = {}
    print("\n" + "="*80)
    print("EXECUTING CLOSED-LOOP BENCHMARKS ON SPIELBERG (26.0s EACH)")
    print("="*80)
    for p in policies:
        print(f"Running simulation for {p}...")
        cl_results[p] = run_closed_loop_simulation(p, duration_s=26.0)
        
    # Determinism check across full trajectories
    raw1_ts = cl_results['RAW']['_timeseries']
    raw2_ts = cl_results['REPEAT_RAW']['_timeseries']
    
    determinism_metrics = {}
    for k in ['steering', 'x', 'y', 'yaw', 'v', 'raw_cmds']:
        arr1 = np.array(raw1_ts[k])
        arr2 = np.array(raw2_ts[k])
        max_diff = float(np.max(np.abs(arr1 - arr2)))
        rms_diff = float(np.sqrt(np.mean((arr1 - arr2)**2)))
        determinism_metrics[k] = {
            'max_absolute_difference': max_diff,
            'rms_difference': rms_diff,
            'is_identical': bool(max_diff == 0.0)
        }
        
    print("\n" + "="*80)
    print("CLOSED-LOOP DETERMINISM AUDIT (RAW Run 1 vs Run 2 across all 2600 steps):")
    print("="*80)
    for k, m in determinism_metrics.items():
        print(f"  {k:<12}: max diff = {m['max_absolute_difference']:.2e}, RMS diff = {m['rms_difference']:.2e}, identical = {m['is_identical']}")
        
    results['determinism_audit'] = determinism_metrics
    
    # Clean out large raw timeseries before saving
    for p in policies:
        del cl_results[p]['_timeseries']
    results['closed_loop_results'] = cl_results
    
    # Save results to experiments/experiment21/experiment21a_results.json
    out_path = '/home/yeswanth/roboracer_ws/experiments/experiment21/experiment21a_results.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved comprehensive audit results to {out_path}")
    
    # --------------------------------------------------------------------------
    # PRINT REQUIRED FINAL COMPARISON TABLE (SECTION 12)
    # --------------------------------------------------------------------------
    print("\n" + "="*160)
    print("EXPERIMENT 21A: FINAL CORRECTED BENCHMARK TABLE")
    print("="*160)
    hdr1 = f"{'Policy':<16} | {'CmdRMS':<7} | {'ΔCmdRMS':<8} | {'Held%':<6} | {'Lat%':<6} | {'RateRMS':<7} | {'RateMax':<7} | {'Reversals':<9} | {'2-Step':<7} | {'AltMax':<6} | {'2-4Hz':<7} | {'4-8Hz':<7} | {'5-7Hz':<7} | {'Dom>2Hz':<7} | {'LatRMSE':<8} | {'MaxLat':<7} | {'HeadRMSE':<8} | {'LiDAR':<6} | {'Col':<3} | {'Lap%':<6} | {'MPC_ms':<6} | {'Pol_us':<7} | {'Err40ms':<7}"
    print(hdr1)
    print("-" * 160)
    for p in ['RAW', 'HOLD_16', 'HOLD_LATTICE_16', 'CONTINUOUS_C']:
        r = cl_results[p]
        line = (
            f"{r['policy']:<16} | "
            f"{r['cmd_rms_deg']:5.2f}° | "
            f"{r['cmd_inc_rms_mrad']:5.2f}mm | "
            f"{r['frac_held']*100:4.1f}% | "
            f"{r['frac_lattice']*100:4.1f}% | "
            f"{r['rate_rms_rad_s']:5.3f} | "
            f"{r['rate_max_rad_s']:5.2f} | "
            f"{r['relay_reversals']:<9d} | "
            f"{r['two_step_lc_count']:<7d} | "
            f"{r['longest_alternating_seq']:<6d} | "
            f"{r['energy_2_4_rad_s']:6.4f} | "
            f"{r['energy_4_8_rad_s']:6.4f} | "
            f"{r['energy_5_7_rad_s']:6.4f} | "
            f"{r['dom_gt2_freq_hz']:5.2f}Hz | "
            f"{r['lat_rmse_100hz_cm']:5.2f}cm | "
            f"{r['max_lat_err_100hz_cm']:5.2f}cm | "
            f"{r['head_rmse_100hz_deg']:5.2f}° | "
            f"{r['min_lidar_m']:4.2f}m | "
            f"{r['collisions']:<3d} | "
            f"{r['lap_progress_pct']:4.1f}% | "
            f"{r['mean_solve_ms']:4.2f} | "
            f"{r['mean_policy_calc_us']:6.1f} | "
            f"{r['err_40ms_published_mrad']:5.2f}mm"
        )
        print(line)
    print("="*160)

if __name__ == '__main__':
    main()
