#!/usr/bin/env python3
"""
Experiment 21: Parts 3, 4, 5, 6, 7 - Comprehensive Offline Policy and Threshold Sensitivity Study
"""

import json
import math
import numpy as np

SV_MAX = 3.2
DT_SIM = 0.010
DT_CTRL = 0.040
QUANTUM = SV_MAX * DT_SIM # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

data_path = '/home/yeswanth/roboracer_ws/experiments/experiment20/exp18_raw_commands.json'
with open(data_path) as f:
    data = json.load(f)

N = len(data)

def lifted_step_custom(delta, b0, b1, u_cmd):
    """
    Step 4 sub-steps of 10ms.
    Returns:
      delta_end,
      new_b0,
      new_b1,
      delta_20ms,
      sub_deltas (4),
      sub_svs (4),
      sub_targets (4)
    """
    buf = [b0, b1]
    cur_d = delta
    sub_deltas = []
    sub_svs = []
    sub_targets = []
    
    for step_i in range(4):
        tgt = buf[-1]
        buf = [u_cmd, buf[0]]
        diff = tgt - cur_d
        if abs(diff) > EPS_RELAY:
            sv = math.copysign(SV_MAX, diff)
        else:
            sv = 0.0
        cur_d += sv * DT_SIM
        sub_deltas.append(cur_d)
        sub_svs.append(sv)
        sub_targets.append(tgt)
        if step_i == 1:
            d_20ms = cur_d
            
    return cur_d, buf[0], buf[1], d_20ms, sub_deltas, sub_svs, sub_targets

def predict_20ms_arrival(cur_d, b0, b1):
    """Predicts delta at t_k + 20ms using in-flight FIFO commands [b0, b1]"""
    # Step 1: b1
    diff1 = b1 - cur_d
    sv1 = math.copysign(SV_MAX, diff1) if abs(diff1) > EPS_RELAY else 0.0
    d_10 = cur_d + sv1 * DT_SIM
    # Step 2: b0
    diff2 = b0 - d_10
    sv2 = math.copysign(SV_MAX, diff2) if abs(diff2) > EPS_RELAY else 0.0
    d_20 = d_10 + sv2 * DT_SIM
    return d_20

# ------------------------------------------------------------------------------
# Continuous Actuator-Aware Candidate Formulation (Policy C)
# ------------------------------------------------------------------------------
def evaluate_policy_c(raw_k, cur_d, b0, b1, w_target=1.0, w_motion=0.01, w_reverse=0.1):
    """
    1-D continuous search over candidate u in [raw_k - 0.064, raw_k + 0.064].
    Evaluates exact lifted response over 40 ms.
    Objective:
      J(u) = (delta_40ms - raw_k)^2 + w_motion * sum(|sv|) * dt + w_reverse * relay_reversals
    """
    u_min = max(-0.35, raw_k - 0.064)
    u_max = min(0.35, raw_k + 0.064)
    # Continuous candidate grid (fine resolution: 1.0 mrad)
    candidates = np.linspace(u_min, u_max, 129)
    # Also explicitly test exact d_20 (hold arrival) and raw_k
    d_20 = predict_20ms_arrival(cur_d, b0, b1)
    candidates = np.append(candidates, [raw_k, d_20])
    
    best_u = raw_k
    best_cost = float('inf')
    
    for u_cand in candidates:
        d_end, _, _, _, _, sub_svs, sub_tgts = lifted_step_custom(cur_d, b0, b1, u_cand)
        # Cost components
        tracking_term = (d_end - raw_k)**2
        motion_term = sum(abs(s) for s in sub_svs) * DT_SIM
        # Direction changes in the 4 steps
        reversals = sum(1 for j in range(1, 4) if sub_svs[j] * sub_svs[j-1] < 0)
        
        cost = w_target * tracking_term + w_motion * motion_term + w_reverse * reversals
        if cost < best_cost:
            best_cost = cost
            best_u = u_cand
            
    return best_u

# ------------------------------------------------------------------------------
# Metric Evaluator for an Offline Policy Trajectory
# ------------------------------------------------------------------------------
def evaluate_offline_policy(policy_fn, policy_name):
    # Initialize from step 0 of exp18
    cur_d = data[0]['actual_steering']
    b0 = data[0]['steer_cmd']
    b1 = data[0]['steer_cmd']
    
    u_cmds = []
    delta_us = []
    cmd_sign_changes = 0
    
    act_deltas_100hz = []
    act_svs_100hz = []
    act_errors_40ms = []
    act_errors_20ms = []
    overshoots = []
    
    relay_reversals = 0
    zero_state_count = 0
    two_step_cycles = 0
    persistent_limit_cycle_steps = 0
    
    prev_u = 0.0
    
    for k in range(N):
        raw_k = data[k]['raw_mpc_delta']
        d_20 = predict_20ms_arrival(cur_d, b0, b1)
        
        # Decide command u_k
        u_k = policy_fn(raw_k, d_20, cur_d, b0, b1, prev_u)
        u_cmds.append(u_k)
        if k > 0:
            delta_us.append(abs(u_k - prev_u))
            if u_k * prev_u < 0 and abs(u_k) > 1e-4 and abs(prev_u) > 1e-4:
                cmd_sign_changes += 1
        prev_u = u_k
        
        # Step exact lifted model 40 ms
        d_end, new_b0, new_b1, d_20ms_actual, s_deltas, s_svs, s_tgts = lifted_step_custom(cur_d, b0, b1, u_k)
        
        # Record 100 Hz
        for j in range(4):
            act_deltas_100hz.append(s_deltas[j])
            act_svs_100hz.append(s_svs[j])
            if s_svs[j] == 0.0:
                zero_state_count += 1
            if len(act_svs_100hz) > 1 and s_svs[j] * act_svs_100hz[-2] < 0:
                relay_reversals += 1
            # Check overshoot
            ov = max(0.0, abs(s_deltas[j] - s_tgts[j])) if abs(s_deltas[j] - s_tgts[j]) < abs(s_deltas[j-1 if j>0 else 0] - s_tgts[j]) else 0.0
            overshoots.append(ov)
            
        # Detect two-step cycles in the 4 steps: (+q, -q) or (-q, +q)
        if (s_svs[0] * s_svs[1] < 0) or (s_svs[1] * s_svs[2] < 0) or (s_svs[2] * s_svs[3] < 0):
            two_step_cycles += 1
            
        # Phase-aligned tracking errors
        act_errors_40ms.append(abs(raw_k - d_end))
        act_errors_20ms.append(abs(raw_k - d_20ms_actual))
        
        # Advance state
        cur_d = d_end
        b0 = new_b0
        b1 = new_b1

    u_cmds = np.array(u_cmds)
    delta_us = np.array(delta_us)
    act_deltas_100hz = np.array(act_deltas_100hz)
    act_svs_100hz = np.array(act_svs_100hz)
    act_errors_40ms = np.array(act_errors_40ms)
    act_errors_20ms = np.array(act_errors_20ms)

    # Persistent limit cycle check: count consecutive alternating signs >= 4 steps
    sign_flips = (act_svs_100hz[1:] * act_svs_100hz[:-1] < 0).astype(int)
    alt_streak = 0
    total_alt_streaks = 0
    for sf in sign_flips:
        if sf == 1:
            alt_streak += 1
            if alt_streak == 3: # 4 consecutive alternating steps
                total_alt_streaks += 1
        else:
            alt_streak = 0

    return {
        'name': policy_name,
        # 25-Hz command metrics
        'rms_u_mrad': float(np.sqrt(np.mean(u_cmds**2)) * 1000),
        'rms_delta_u_mrad': float(np.sqrt(np.mean(delta_us**2)) * 1000),
        'max_delta_u_mrad': float(np.max(delta_us) * 1000),
        'cmd_sign_changes': cmd_sign_changes,
        # 100-Hz actuator metrics
        'rms_delta_act_mrad': float(np.sqrt(np.mean(act_deltas_100hz**2)) * 1000),
        'rms_d_delta_act_mrad': float(np.sqrt(np.mean((act_deltas_100hz[1:] - act_deltas_100hz[:-1])**2)) * 1000),
        'rms_sv_rad_s': float(np.sqrt(np.mean(act_svs_100hz**2))),
        'max_sv_rad_s': float(np.max(np.abs(act_svs_100hz))),
        'relay_reversals': relay_reversals,
        'zero_state_pct': float(zero_state_count / len(act_svs_100hz) * 100.0),
        'two_step_cycles': two_step_cycles,
        'persistent_lc_streaks': total_alt_streaks,
        # Phase-aligned error metrics
        'err_40ms_rms_mrad': float(np.sqrt(np.mean(act_errors_40ms**2)) * 1000),
        'err_20ms_rms_mrad': float(np.sqrt(np.mean(act_errors_20ms**2)) * 1000),
        'err_40ms_max_mrad': float(np.max(act_errors_40ms) * 1000)
    }

# ==============================================================================
# PART 3: THRESHOLD SENSITIVITY SWEEP (Policy H: Hold)
# ==============================================================================
print("\n" + "=" * 80)
print("PART 3: THRESHOLD SENSITIVITY SWEEP (POLICY H: HOLD)")
print("=" * 80)

thresholds_mrad = [0, 4, 8, 12, 16, 20, 24, 28, 32]
hold_sweep_results = []

for eps_mrad in thresholds_mrad:
    eps_rad = eps_mrad / 1000.0
    def make_fn(eps):
        return lambda raw, d20, cur_d, b0, b1, prev_u: d20 if abs(raw - d20) < eps else raw
    res = evaluate_offline_policy(make_fn(eps_rad), f"Policy H (eps={eps_mrad}mrad)")
    hold_sweep_results.append(res)

print(f"{'Threshold':<14} | {'RMS(Δu)':<10} | {'RMS(sv)':<10} | {'Reversals':<10} | {'Zero-State %':<12} | {'2-Step Cyc':<11} | {'Err 40ms RMS':<12}")
print("-" * 90)
for r in hold_sweep_results:
    print(f"{r['name']:<14} | {r['rms_delta_u_mrad']:7.2f} mm | {r['rms_sv_rad_s']:7.3f}    | {r['relay_reversals']:<10d} | {r['zero_state_pct']:8.2f} %   | {r['two_step_cycles']:<11d} | {r['err_40ms_rms_mrad']:8.2f} mm")

# ==============================================================================
# PART 4: COMPARISON OF DISTINCT POLICIES
# ==============================================================================
print("\n" + "=" * 80)
print("PART 4: COMPARISON OF MULTIPLE INTERFACE POLICIES")
print("=" * 80)

# Policy R: Raw
fn_raw = lambda raw, d20, cur_d, b0, b1, prev_u: raw

# Policy H (best threshold, say 16 mrad)
fn_h16 = lambda raw, d20, cur_d, b0, b1, prev_u: d20 if abs(raw - d20) < 0.016 else raw

# Policy L: Static Lattice
fn_lattice = lambda raw, d20, cur_d, b0, b1, prev_u: round(raw / QUANTUM) * QUANTUM

# Policy HL (Hold 16 mrad + Lattice)
fn_hl16 = lambda raw, d20, cur_d, b0, b1, prev_u: d20 if abs(raw - d20) < 0.016 else round(raw / QUANTUM) * QUANTUM

# Policy C: Continuous Actuator-Aware Candidate
fn_cont = lambda raw, d20, cur_d, b0, b1, prev_u: evaluate_policy_c(raw, cur_d, b0, b1, w_target=1.0, w_motion=0.005, w_reverse=0.05)

policy_set = [
    (fn_raw, "Policy R (Raw MPC)"),
    (fn_h16, "Policy H (Hold 16mrad)"),
    (fn_lattice, "Policy L (Static Lattice)"),
    (fn_hl16, "Policy HL (Hold+Lattice 16mrad)"),
    (fn_cont, "Policy C (Continuous Actuator-Aware)")
]

policy_comparison_results = []
for fn, name in policy_set:
    print(f"Evaluating {name}...")
    res = evaluate_offline_policy(fn, name)
    policy_comparison_results.append(res)

print("\n" + "=" * 115)
print(f"{'Policy Name':<34} | {'RMS(Δu)':<9} | {'RMS(sv)':<9} | {'Reversals':<10} | {'Zero %':<8} | {'Alt Streaks':<11} | {'Err 40ms RMS':<12} | {'Err 40ms Max'}")
print("-" * 115)
for r in policy_comparison_results:
    print(f"{r['name']:<34} | {r['rms_delta_u_mrad']:6.2f} mm | {r['rms_sv_rad_s']:6.3f}   | {r['relay_reversals']:<10d} | {r['zero_state_pct']:6.2f}% | {r['persistent_lc_streaks']:<11d} | {r['err_40ms_rms_mrad']:8.2f} mm    | {r['err_40ms_max_mrad']:8.2f} mm")

with open('/home/yeswanth/roboracer_ws/experiments/experiment21/offline_study_results.json', 'w') as f:
    json.dump({
        'hold_sweep': hold_sweep_results,
        'policies': policy_comparison_results
    }, f, indent=2)
print("\nSaved offline study results to offline_study_results.json")
