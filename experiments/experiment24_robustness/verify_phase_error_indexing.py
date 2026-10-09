#!/usr/bin/env python3
"""
CORRECTION 2 AUDIT: Phase-Error Indexing & Timing Verification
Deterministically traces a known command sequence through:
1. Exact Experiment 20 lifted actuator state propagation [delta, b0, b1]
2. Official native f110_gym simulator environment with steer_buffer

Verifies exactly when u_pub[k] arrives at the actuator and which discrete
simulator sample corresponds to delta_actual at t_k + 40 ms.
"""

import math
import numpy as np
import gym

_orig_make = gym.make
def _patched_make(id, **kwargs):
    kwargs['disable_env_checker'] = True
    env = _orig_make(id, **kwargs)
    return env.unwrapped
gym.make = _patched_make

import f110_gym

DT_SIM = 0.010
DT_CTRL = 0.040
SV_MAX = 3.20
EPS_RELAY = 1e-4

def lifted_substep(delta, target, sv_max=SV_MAX, dt_sim=DT_SIM):
    diff = target - delta
    if abs(diff) > EPS_RELAY:
        sv = math.copysign(sv_max, diff)
        d_next = delta + sv * dt_sim
    else:
        d_next = delta
    return d_next

def run_synthetic_timing_audit():
    # Test command sequence
    u_seq = [0.0, 0.10, -0.10, 0.05, -0.05, 0.08, 0.08, 0.0]
    K = len(u_seq)
    
    # Initialize gym
    # Map doesn't matter for pure steering integration test
    map_path = '/home/yeswanth/roboracer_ws/src/f1tenth_gym_ros/maps/Spielberg_map'
    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    obs = env.reset(np.array([[0.0, 0.0, 0.0]]))
    agent = env.sim.agents[0]
    agent.params['sv_max'] = SV_MAX
    agent.params['sv_min'] = -SV_MAX
    agent.steer_buffer_size = 2 # 20 ms nominal delay
    
    # Lifted model state: [delta, b0, b1]
    lifted_delta = float(agent.state[2])
    # Note: in f110_gym, right after reset(poses), steer_buffer has shape (1,) with [0.0]
    # steer_buffer_size = 2 means target arrives after 2 steps (20 ms).
    print("="*120)
    print("PHASE-ERROR SYNTHETIC TIMING AUDIT (Correction 2)")
    print(f"Simulator: DT_SIM={DT_SIM}s, DT_CTRL={DT_CTRL}s, SV_MAX={SV_MAX}rad/s, steer_buffer_size={agent.steer_buffer_size}")
    print("="*120)
    
    hdr = f"{'Step':<5} | {'Cycle':<6} | {'t_sim':<7} | {'u_pub[k]':<9} | {'FIFO contents':<15} | {'Relay Tgt':<10} | {'Gym Delta':<10} | {'Lifted Delta':<12} | {'Err (rad)':<10}"
    print(hdr)
    print("-" * 120)
    
    act_deltas = []
    relay_targets = []
    
    step_global = 0
    lifted_buf = [0.0]  # matches gym initial buffer
    
    for k in range(K):
        u_pub_k = u_seq[k]
        t_k = k * DT_CTRL
        
        for sub in range(4):
            t_now = step_global * DT_SIM
            cur_gym_delta = float(agent.state[2])
            act_deltas.append(cur_gym_delta)
            
            buf_str = str([round(float(x), 3) for x in agent.steer_buffer])
            tgt_gym = float(agent.steer_buffer[-1]) if len(agent.steer_buffer) >= agent.steer_buffer_size else 0.0
            relay_targets.append(tgt_gym)
            
            lifted_err = abs(cur_gym_delta - lifted_delta)
            
            cycle_str = f"k={k},s={sub}"
            pub_str = f"{u_pub_k:+.3f}" if sub == 0 else "   -- "
            print(f"{step_global:<5d} | {cycle_str:<6} | {t_now:5.3f}s | {pub_str:<9} | {buf_str:<15} | {tgt_gym:+.4f}     | {cur_gym_delta:+.6f}   | {lifted_delta:+.6f}     | {lifted_err:.2e}")
            
            # Step Gym
            obs, _, _, _ = env.step(np.array([[u_pub_k, 5.0]]))
            
            # Step Lifted Model:
            # Gym's exact buffer logic from base_classes.py:
            if len(lifted_buf) < agent.steer_buffer_size:
                tgt_lifted = 0.0
                lifted_buf.insert(0, u_pub_k)
            else:
                tgt_lifted = lifted_buf[-1]
                lifted_buf = lifted_buf[:-1]
                lifted_buf.insert(0, u_pub_k)
                
            lifted_delta = lifted_substep(lifted_delta, tgt_lifted)
            step_global += 1
            
    # Post-run append final state
    act_deltas.append(float(agent.state[2]))
    
    print("-" * 120)
    print("\nEMPIRICAL TIMING & SAMPLE MAPPING VERIFICATION:")
    print("="*120)
    print("Evaluating candidate alignments for command u_pub[k] emitted at t_k:")
    print("  Mapping A: delta at t_k + 20 ms (index 4k + 2) -> Moment u_pub[k] first arrives at relay target")
    print("  Mapping B: delta at t_k + 40 ms (index 4k + 4) -> End of control cycle k (halfway through actuation window)")
    print("  Mapping C: delta at t_k + 60 ms (index 4k + 6) -> End of u_pub[k] actuation window (tau + T_ctrl = 20 + 40 ms)")
    print("-" * 120)
    
    for k in range(K - 2):
        u = u_seq[k]
        t_emit = k * DT_CTRL
        idx_emit = 4 * k
        idx_20ms = 4 * k + 2
        idx_40ms = 4 * k + 4
        idx_60ms = 4 * k + 6
        
        d_20 = act_deltas[idx_20ms]
        d_40 = act_deltas[idx_40ms]
        d_60 = act_deltas[idx_60ms]
        
        err_A = (u - d_20) * 1000.0
        err_B = (u - d_40) * 1000.0
        err_C = (u - d_60) * 1000.0
        
        print(f"Cycle k={k} (t={t_emit:5.3f}s, u_pub={u:+.3f}):")
        print(f"  Mapping A [4k+2, t={t_emit+0.020:5.3f}s]: delta={d_20:+.5f}, err={err_A:+8.2f} mrad")
        print(f"  Mapping B [4k+4, t={t_emit+0.040:5.3f}s]: delta={d_40:+.5f}, err={err_B:+8.2f} mrad")
        print(f"  Mapping C [4k+6, t={t_emit+0.060:5.3f}s]: delta={d_60:+.5f}, err={err_C:+8.2f} mrad")

if __name__ == '__main__':
    run_synthetic_timing_audit()
