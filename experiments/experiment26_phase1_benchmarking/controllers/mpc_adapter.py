"""Model Predictive Control (MPC) Controller Adapter."""

import math
import os
import sys
from typing import Tuple, Optional
import numpy as np

# Ensure mpc_controller package is importable
for p in [
    '/home/yeswanth/roboracer_ws/src/controllers/mpc_controller',
    '/sim_ws/src/controllers/mpc_controller',
    '/home/yeswanth/roboracer_ws'
]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig
from .base_controller import BaseBenchmarkController, compute_cross_track_error, wrap_angle


class MPCAdapter(BaseBenchmarkController):
    """
    Linear Time-Varying Model Predictive Controller with OSQP Optimization.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        headings: np.ndarray,
        target_speeds: np.ndarray,
        s_arr: Optional[np.ndarray] = None,
        track_length: float = 100.0,
        N: int = 10,
        dt_mpc: float = 0.08,
        w_x: float = 2.50,
        w_y: float = 2.50,
        w_psi: float = 1.80,
        w_v: float = 0.50,
        w_delta: float = 0.80,
        w_ddelta: float = 2.50,
        speed_scale: float = 0.70,
        max_speed: float = 7.00
    ):
        super().__init__("MPC_Controller")
        self.waypoints = waypoints
        self.headings = headings
        self.target_speeds = target_speeds * speed_scale
        self.s_arr = s_arr if s_arr is not None else np.linspace(0, track_length, len(waypoints))
        self.track_length = track_length
        self.num_pts = len(waypoints)
        self.last_idx = 0
        self.max_speed = max_speed

        self.cfg = MPCConfig(
            wheelbase=self.wheelbase,
            dt=dt_mpc,
            N=N,
            w_x=w_x,
            w_y=w_y,
            w_psi=w_psi,
            w_v=w_v,
            w_delta=w_delta,
            w_ddelta=w_ddelta,
            max_steer=self.max_steer,
            max_speed=max_speed
        )
        self.optimizer = MPCOptimizer(self.cfg)
        self.last_control = (0.0, 0.0)

    def reset(self):
        self.last_idx = 0
        self.last_control = (0.0, 0.0)

    def compute_control(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        dt: float,
        scans: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        # Ground truth error
        e_ct, e_psi, closest_idx = compute_cross_track_error(x, y, yaw, self.waypoints, self.headings, self.last_idx)
        self.last_idx = closest_idx

        # Construct reference trajectory over horizon N
        cur_s = self.s_arr[closest_idx]
        speed_est = max(1.50, v)
        ref_horizon = np.zeros((self.cfg.N + 1, 4))

        for k in range(self.cfg.N + 1):
            s_target = (cur_s + k * speed_est * self.cfg.dt) % self.track_length
            idx_k = int(np.argmin(np.abs(self.s_arr - s_target)))
            ref_horizon[k] = [
                self.waypoints[idx_k, 0],
                self.waypoints[idx_k, 1],
                self.headings[idx_k],
                min(self.target_speeds[idx_k], self.max_speed)
            ]

        # Solve MPC QP
        res = self.optimizer.solve(
            current_state=np.array([x, y, yaw, v]),
            ref_trajectory=ref_horizon,
            prev_control=self.last_control
        )

        steer_cmd = float(np.clip(res.steering, -self.max_steer, self.max_steer))
        self.last_control = (res.accel, res.steering)

        # Longitudinal speed target from raceline reference
        target_v = float(self.target_speeds[closest_idx])
        steer_ratio = abs(steer_cmd) / self.max_steer
        if steer_ratio > 0.50:
            target_v *= (1.0 - 0.20 * (steer_ratio - 0.50) / 0.50)

        speed_cmd = float(np.clip(target_v, 1.20, self.max_speed))

        return steer_cmd, speed_cmd, e_ct, e_psi
