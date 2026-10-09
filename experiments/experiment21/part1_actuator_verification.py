#!/usr/bin/env python3
"""
Experiment 21: Part 1 - Actuator Model Verification against official f110_gym
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
    X_next = np.array([delta, buf[0], buf[1]])
    return X_next, step_records

def run_verification(n_trials=100):
    print("PART 1: Verifying Exact Lifted Actuator Map against f110_gym...")
    track = TrackManager.load_track('Spielberg')
    env = gym.make('f110_gym:f110-v0', map=track.map_path_no_ext, map_ext='.png', num_agents=1)
    agent = env.sim.agents[0]
    
    rng = np.random.default_rng(2026)
    max_err_delta = 0.0
    max_err_b0 = 0.0
    max_err_b1 = 0.0

    for trial in range(n_trials):
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
        X_next, _ = lifted_actuator_step(X_k, rand_u)

        err_d = abs(gym_delta - X_next[0])
        err_b0 = abs(gym_b0 - X_next[1])
        err_b1 = abs(gym_b1 - X_next[2])

        if err_d > max_err_delta: max_err_delta = err_d
        if err_b0 > max_err_b0: max_err_b0 = err_b0
        if err_b1 > max_err_b1: max_err_b1 = err_b1

    print(f"Results over {n_trials} trials:")
    print(f"  Max error in delta_actual: {max_err_delta:.2e} rad")
    print(f"  Max error in buffer_0:     {max_err_b0:.2e} rad")
    print(f"  Max error in buffer_1:     {max_err_b1:.2e} rad")
    
    passed = (max_err_delta < 1e-12) and (max_err_b0 < 1e-12) and (max_err_b1 < 1e-12)
    print(f"Machine-precision verification PASSED: {passed}")
    return passed, max_err_delta, max_err_b0, max_err_b1

if __name__ == '__main__':
    passed, ed, eb0, eb1 = run_verification(100)
    assert passed, "Verification failed!"
