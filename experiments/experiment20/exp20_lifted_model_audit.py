#!/usr/bin/env python3
"""
Experiment 20: Part 1 - Exact 25 Hz Lifted Actuator Map & Verification
"""

import os
import sys
import math
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

SV_MAX = 3.2
DT_SIM = 0.010
DT_CTRL = 0.040
QUANTUM = SV_MAX * DT_SIM # 0.032 rad = 32 mrad
EPS_RELAY = 1e-4

def lifted_actuator_step(X_k, u_k):
    """
    Exact 25 Hz Lifted Actuator Map:
    X_k = [delta_act, b0, b1]
    u_k = published scalar command held for 40 ms (4 physics steps).
    Returns X_{k+1} = [delta_next, b0_next, b1_next]
    and step-by-step telemetry for the 4 steps.
    """
    delta, b0, b1 = X_k
    buf = [b0, b1] # buf[0] is newest (10ms ago), buf[1] is oldest (20ms ago)
    
    step_records = []
    
    # Propagate 4 steps (each 10 ms)
    # At each step, the 25 Hz MPC command u_k is held constant as input to env.step
    for step_idx in range(4):
        # 1. Target emerging from buffer
        target = buf[-1] # b1
        # 2. Buffer update: u_k enters, oldest drops
        buf = [u_k, buf[0]]
        
        # 3. Relay evaluation
        diff = target - delta
        if abs(diff) > EPS_RELAY:
            sv = math.copysign(SV_MAX, diff)
        else:
            sv = 0.0
            
        # 4. RK4 integration on steer angle:
        # In vehicle_dynamics_st, d(delta)/dt = sv (constant over the 10 ms step).
        # Therefore RK4 integration is EXACTLY delta + sv * DT_SIM.
        delta_next = delta + sv * DT_SIM
        
        step_records.append({
            'substep': step_idx,
            'time_offset_ms': (step_idx + 1) * 10,
            'active_target': target,
            'active_target_source': 'b1 (u[k-1])' if step_idx == 0 else ('b0 (u[k-1])' if step_idx == 1 else 'u[k]'),
            'delta_start': delta,
            'sv': sv,
            'delta_end': delta_next,
            'buf_after': list(buf)
        })
        delta = delta_next
        
    X_next = np.array([delta, buf[0], buf[1]])
    return X_next, step_records

def verify_against_f110_gym(n_trials=100):
    print(f"Verifying lifted actuator map against official f110_gym for {n_trials} random scenarios...")
    track = TrackManager.load_track('Spielberg')
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    agent = env.sim.agents[0]
    
    rng = np.random.default_rng(42)
    max_err_delta = 0.0
    max_err_b0 = 0.0
    max_err_b1 = 0.0
    
    for trial in range(n_trials):
        env.reset(np.array([[track.start_pose[0], track.start_pose[1], track.start_pose[2]]]))
        
        # Random initial state
        rand_delta = float(rng.uniform(-0.35, 0.35))
        rand_b0 = float(rng.uniform(-0.35, 0.35))
        rand_b1 = float(rng.uniform(-0.35, 0.35))
        rand_u = float(rng.uniform(-0.35, 0.35))
        
        # Inject into gym agent
        agent.state[2] = rand_delta
        agent.steer_buffer = np.array([rand_b0, rand_b1])
        
        # Step f110_gym 4 times with rand_u
        for _ in range(4):
            env.step(np.array([[rand_u, 5.0]]))
            
        gym_delta = float(agent.state[2])
        gym_b0 = float(agent.steer_buffer[0])
        gym_b1 = float(agent.steer_buffer[1])
        
        # Step our lifted map
        X_k = [rand_delta, rand_b0, rand_b1]
        X_next, _ = lifted_actuator_step(X_k, rand_u)
        
        err_d = abs(gym_delta - X_next[0])
        err_b0 = abs(gym_b0 - X_next[1])
        err_b1 = abs(gym_b1 - X_next[2])
        
        if err_d > max_err_delta: max_err_delta = err_d
        if err_b0 > max_err_b0: max_err_b0 = err_b0
        if err_b1 > max_err_b1: max_err_b1 = err_b1

    print(f"Max error in delta_actual: {max_err_delta:.2e} rad")
    print(f"Max error in buffer_0:     {max_err_b0:.2e} rad")
    print(f"Max error in buffer_1:     {max_err_b1:.2e} rad")
    assert max_err_delta < 1e-12, "Delta error exceeds machine tolerance!"
    assert max_err_b0 < 1e-12, "b0 error exceeds machine tolerance!"
    assert max_err_b1 < 1e-12, "b1 error exceeds machine tolerance!"
    print("Lifted actuator map mathematically matches official f110_gym to machine precision!")

if __name__ == '__main__':
    verify_against_f110_gym(100)
    
    # Show example step breakdown
    print("\n" + "="*80)
    print("EXAMPLE 4-SUBSTEP BREAKDOWN OF LIFTED 25 Hz CYCLE")
    print("="*80)
    X0 = [0.0, 0.010, 0.010] # delta=0, past command was 10 mrad
    u0 = 0.030 # new command is 30 mrad
    X1, recs = lifted_actuator_step(X0, u0)
    print(f"Initial State X[k]: delta={X0[0]*1000:.1f} mrad, b0={X0[1]*1000:.1f} mrad, b1={X0[2]*1000:.1f} mrad")
    print(f"Published Command u[k]: {u0*1000:.1f} mrad")
    print(f"{'Step':<5} | {'Time':<7} | {'Active Target':<14} | {'Target Source':<15} | {'Delta Start':<12} | {'sv (rad/s)':<11} | {'Delta End':<10}")
    print("-" * 88)
    for r in recs:
        print(f"{r['substep']:<5d} | {r['time_offset_ms']:2d} ms   | {r['active_target']*1000:+8.2f} mrad   | {r['active_target_source']:<15} | {r['delta_start']*1000:+8.2f} mrad   | {r['sv']:+8.2f}    | {r['delta_end']*1000:+8.2f} mrad")
    print(f"End State X[k+1]: delta={X1[0]*1000:.1f} mrad, b0={X1[1]*1000:.1f} mrad, b1={X1[2]*1000:.1f} mrad")
