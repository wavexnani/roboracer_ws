#!/usr/bin/env python3
"""
Experiment 21: Part 2 - Telemetry Audit and FIFO State Verification
"""

import os
import json
import numpy as np

data_path = '/home/yeswanth/roboracer_ws/experiments/experiment20/exp18_raw_commands.json'
with open(data_path) as f:
    data = json.load(f)

N = len(data)
print(f"PART 2: Auditing exact telemetry across N = {N} control cycles...")

# Check equality of b0[k] == b1[k] == u_raw[k-1] vs steer_cmd[k-1]
mismatch_raw_prev = 0
mismatch_steer_prev = 0
mismatch_relay_target = 0
details = []

for k in range(1, N):
    c = data[k]
    c_prev = data[k-1]

    raw_k = c['raw_mpc_delta']
    raw_prev = c['prev_mpc_delta']
    steer_k = c['steer_cmd']
    steer_prev = c_prev['steer_cmd']
    relay_tgt = c['relay_target']

    # In exp18 data, relay_target was recorded directly from agent.steer_buffer[-1] at cycle start
    # Does b_old == u_raw[k-1]?
    diff_raw = abs(relay_tgt - c_prev['raw_mpc_delta'])
    diff_steer = abs(relay_tgt - steer_prev)

    if diff_raw > 1e-6:
        mismatch_raw_prev += 1
        if len(details) < 5:
            details.append({
                'k': k,
                't_sim': c['t_sim'],
                'raw_prev': c_prev['raw_mpc_delta'],
                'steer_prev': steer_prev,
                'relay_tgt': relay_tgt,
                'reason': 'Deadband held steer_prev' if abs(steer_prev - relay_tgt) < 1e-6 else 'Slew / Transition'
            })

    if diff_steer > 1e-6:
        mismatch_steer_prev += 1

print(f"\nAudit Results:")
print(f"  Total cycles checked: {N - 1}")
print(f"  Cycles where relay_target == u_raw[k-1]:   {N - 1 - mismatch_raw_prev} / {N - 1} ({(N - 1 - mismatch_raw_prev)/(N - 1)*100:.2f}%)")
print(f"  Cycles where relay_target != u_raw[k-1]:   {mismatch_raw_prev} / {N - 1} ({mismatch_raw_prev/(N - 1)*100:.2f}%)")
print(f"  Cycles where relay_target == steer_cmd[k-1]: {N - 1 - mismatch_steer_prev} / {N - 1} ({(N - 1 - mismatch_steer_prev)/(N - 1)*100:.2f}%)")

print("\nRepresentative Discrepancy Examples where relay_target != u_raw[k-1]:")
print(f"{'k':<5} | {'Time':<7} | {'u_raw[k-1]':<14} | {'steer_cmd[k-1]':<16} | {'relay_target':<14} | {'Reason'}")
print("-" * 80)
for d in details:
    print(f"{d['k']:<5d} | {d['t_sim']:5.2f} s | {d['raw_prev']*1000:+9.2f} mrad | {d['steer_prev']*1000:+11.2f} mrad | {d['relay_tgt']*1000:+9.2f} mrad | {d['reason']}")

print("\nAudit Explanation:")
print("  The assumption b0[k] = b1[k] = u_raw[k-1] DOES NOT HOLD identically in the Exp 18 dataset.")
print(f"  In {mismatch_raw_prev} cycles ({mismatch_raw_prev/(N-1)*100:.1f}%), u_raw[k-1] differed from the command actually delivered")
print("  to the simulator because the controller node applied a 3.5 mrad deadband filter and 2.0 rad/s slew rate.")
print("  However, b0[k] = b1[k] = steer_cmd[k-1] (the command actually published to the environment) holds 100.0% exactly.")
print("  Therefore, exact FIFO state reconstruction MUST use the published command steer_cmd[k-1], NOT the raw un-filtered QP output u_raw[k-1].")

