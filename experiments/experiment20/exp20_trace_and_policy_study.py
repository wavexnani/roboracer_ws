#!/usr/bin/env python3
"""
Experiment 20: Parts 3, 5, 6, 7, 8 - Trace Audit, Reachability, Policy Comparison, Critical Test
"""

import json
import math
import numpy as np

SV_MAX = 3.2
DT_SIM = 0.010
DT_CTRL = 0.040
QUANTUM = SV_MAX * DT_SIM # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

def lifted_actuator_step(X_k, u_k):
    delta, b0, b1 = X_k
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
            'delta_end': delta_next
        })
        delta = delta_next
    X_next = [delta, buf[0], buf[1]]
    return X_next, step_records

import os
# Load 800 cycles of exp18 data
script_dir = os.path.dirname(os.path.abspath(__file__))
json_path = os.path.join(script_dir, 'exp18_raw_commands.json')
if not os.path.exists(json_path):
    json_path = '/home/yeswanth/.gemini/antigravity-ide/brain/0fb11780-fef7-4429-b020-0e74787650bf/scratch/exp18_raw_commands.json'
with open(json_path) as f:
    data = json.load(f)

N = len(data)

# ==============================================================================
# 3. DESIRED VS REACHABLE STEERING AUDIT
# ==============================================================================
print("=" * 80)
print("3. DESIRED VS REACHABLE STEERING AUDIT (800 CYCLES)")
print("=" * 80)

# Track how many times desired steering delta_desired is in the exact reachable set at k+1 and k+2
reachable_k1_count = 0
reachable_k2_count = 0
exact_match_k1 = 0

for k in range(N - 2):
    raw_k = data[k]['raw_mpc_delta']
    raw_prev = data[k]['prev_mpc_delta']
    d_act = data[k]['actual_steering']
    # Reconstruct FIFO state:
    # At t_k, FIFO has [raw_prev, raw_prev]
    X_k = [d_act, raw_prev, raw_prev]
    
    # Step 1 cycle with current raw command
    X_k1, recs1 = lifted_actuator_step(X_k, raw_k)
    d_k1 = X_k1[0]
    
    # Step 2nd cycle holding raw command
    X_k2, recs2 = lifted_actuator_step(X_k1, raw_k)
    d_k2 = X_k2[0]

    # Reachable states under ANY candidate future command u in [-0.35, 0.35]:
    # At step 2 and 3 of cycle k, command u_k acts.
    # From d(t_k + 20ms) = recs1[1]['delta_end'],
    # the achievable delta_end at 40ms depends on u_k.
    # Since each step moves by +/- 32 mrad or 0, achievable states at 40ms are:
    d_20ms = recs1[1]['delta_end']
    # If u_k > d_20ms + 1e-4: step 2 moves +32. Then at step 3, if u_k > d_30ms: moves +32.
    # Possible outcomes at 40ms:
    # 1. u_k > d_20ms + 32mrad: d_40ms = d_20ms + 64mrad
    # 2. d_20ms < u_k < d_20ms + 32mrad: d_40ms = d_20ms (chatter: +32 then -32)
    # 3. u_k == d_20ms: d_40ms = d_20ms (0 then 0)
    # 4. d_20ms - 32mrad < u_k < d_20ms: d_40ms = d_20ms (chatter: -32 then +32)
    # 5. u_k < d_20ms - 32mrad: d_40ms = d_20ms - 64mrad
    # NOTICE: d_40ms CAN ONLY EVER BE d_20ms - 64mrad, d_20ms, or d_20ms + 64mrad!
    achievable_k1 = {round(d_20ms - 2*QUANTUM, 6), round(d_20ms, 6), round(d_20ms + 2*QUANTUM, 6)}
    if round(raw_k, 6) in achievable_k1:
        reachable_k1_count += 1
    if abs(d_k1 - raw_k) <= EPS_RELAY:
        exact_match_k1 += 1

print(f"Total cycles audited: {N - 2}")
print(f"Cycles where raw MPC command is in exact 40ms reachable set: {reachable_k1_count} / {N - 2} ({reachable_k1_count/(N-2)*100:.2f}%)")
print(f"Cycles where actual steering matches raw command at 40ms:    {exact_match_k1} / {N - 2} ({exact_match_k1/(N-2)*100:.2f}%)")

# ==============================================================================
# 5 & 6. CRITICAL TEST & CANDIDATE INTERFACE OPTIMIZATION
# ==============================================================================
print("\n" + "=" * 80)
print("6. CRITICAL TEST: 2–10 mrad COMMANDS VS 32 mrad RELAY KICKS")
print("=" * 80)

# Find examples where raw MPC command changes by 2 to 10 mrad:
small_changes = []
for k in range(N - 1):
    raw = data[k]['raw_mpc_delta']
    prev = data[k]['prev_mpc_delta']
    d_act = data[k]['actual_steering']
    delta_change = raw - prev
    if 0.002 <= abs(delta_change) <= 0.010:
        small_changes.append((k, data[k]['t_sim'], raw, prev, delta_change, d_act))

print(f"Found {len(small_changes)} cycles where raw MPC command changed by 2–10 mrad.")
print("\nRepresentative Examples of Small Command Adjustments:")
print(f"{'Index':<6} | {'Time':<7} | {'Prev Cmd':<12} | {'Raw Cmd':<12} | {'Change (mrad)':<14} | {'Actual Steer':<14}")
print("-" * 75)
for item in small_changes[:5]:
    k, t, raw, prev, chg, d_act = item
    print(f"{k:<6d} | {t:5.2f} s | {prev*1000:+8.2f} mrad | {raw*1000:+8.2f} mrad | {chg*1000:+10.2f} mrad | {d_act*1000:+10.2f} mrad")

# For the first example, let's test 4 candidate strategies:
k_ex, t_ex, raw_ex, prev_ex, chg_ex, d_act_ex = small_changes[0]
print(f"\nDetailed Candidate Evaluation for Cycle k={k_ex} (t={t_ex:.2f} s):")
print(f"  Desired change: {chg_ex*1000:+.2f} mrad (from {prev_ex*1000:+.2f} to {raw_ex*1000:+.2f} mrad)")
print(f"  Current actual steering: {d_act_ex*1000:+.2f} mrad")

# Candidates:
# 1. Raw command: u = raw_ex
# 2. Hold previous: u = prev_ex
# 3. Lattice projection: u = round(raw_ex / q) * q
# 4. Zero-chatter setpoint: u matching settled lattice
candidates_test = {
    '1. Raw MPC (Naïve)': raw_ex,
    '2. Hold Previous Cmd': prev_ex,
    '3. Static Lattice': round(raw_ex / QUANTUM) * QUANTUM,
    '4. Actuator-Settled State': d_act_ex
}

X_init = [d_act_ex, prev_ex, prev_ex]
print(f"\n{'Candidate Policy':<25} | {'Cmd Sent':<12} | {'Delta @ 20ms':<13} | {'Delta @ 40ms':<13} | {'Relay Sw (40ms)':<15} | {'Error to Desired'}")
print("-" * 105)
for name, u_val in candidates_test.items():
    X_end, recs = lifted_actuator_step(X_init, u_val)
    d_20 = recs[1]['delta_end'] * 1000
    d_40 = X_end[0] * 1000
    sw = sum(1 for step_i in range(1, 4) if recs[step_i]['sv'] != recs[step_i-1]['sv'] and recs[step_i]['sv'] != 0 and recs[step_i-1]['sv'] != 0)
    err = abs(X_end[0] - raw_ex) * 1000
    print(f"{name:<25} | {u_val*1000:+8.2f} mrad | {d_20:+9.2f} mrad  | {d_40:+9.2f} mrad  | {sw:<15d} | {err:8.2f} mrad")

# ==============================================================================
# 8. TELEMETRY ALIGNMENT AUDIT
# ==============================================================================
print("\n" + "=" * 80)
print("8. TELEMETRY ALIGNMENT & PHASE AUDIT")
print("=" * 80)

# Check the exact time offsets in telemetry:
# MPC cycle k executes at t_k.
# MPC command u_k is published at t_k.
# The measurement passed to MPC at t_k is actual_steering[k] = delta(t_k).
# But u_k only affects the relay at t_k + 20ms and t_k + 30ms!
# And the yaw rate response r(t) lags steering by approximately 30-50 ms (yaw time constant tau_r = 0.052 s).
print("Phase Alignment of Signals:")
print("  t = t_k +  0 ms: MPC receives measurement delta(t_k), yaw_rate r(t_k); solves QP; publishes u[k]")
print("  t = t_k + 10 ms: Actuator relay is executing b1 = u[k-1] (old command)")
print("  t = t_k + 20 ms: Actuator relay is executing b0 = u[k-1] (old command)")
print("  t = t_k + 20 ms: u[k] FINALLY emerges from FIFO into relay")
print("  t = t_k + 30 ms: u[k] continues driving relay")
print("  t = t_k + 40 ms: Actuator state delta(t_k + 40ms) is reached; MPC cycle k+1 begins")
print("  t = t_k + 70-90 ms: Chassis yaw rate r reaches peak response to u[k]")
print("\nAlignment Conclusion:")
print("  Comparing u[k] against delta_actual[k] introduces an artificial 20 ms phase lead error.")
print("  The proper phase-aligned actuator tracking error for command u[k] is |u[k] - delta_actual[k+1]|")
print("  evaluated at t_k + 40 ms (or t_k + 20 ms in the relay frame).")

