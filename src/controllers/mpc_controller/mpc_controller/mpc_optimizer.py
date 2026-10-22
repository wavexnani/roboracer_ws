import time
import math
import numpy as np
from typing import Optional, Tuple

from .types import MPCConfig, MPCResult
from .models import compute_yaw_model_matrices, linearize_vehicle_model
from .qp_solver import solve_osqp, solve_cvxpy, solve_kinematic_fallback


class MPCOptimizer:
    """Linear Time-Varying Model Predictive Controller for path tracking."""

    def __init__(self, config: Optional[MPCConfig] = None):
        self.cfg = config or MPCConfig()
        self.nu = 2
        self.N = self.cfg.N
        self.dt = self.cfg.dt
        self.L = self.cfg.wheelbase

        if self.cfg.model_type == "yaw_first_order":
            self.nx = 5
            self.yaw_params = compute_yaw_model_matrices(self.dt, self.cfg.yaw_tau, self.cfg.yaw_K)
            self.yaw_c_r, self.yaw_d_r, self.yaw_c_psi, self.yaw_d_psi = self.yaw_params
        else:
            self.nx = 4
            self.yaw_params = (0.0, 0.0, 0.0, 0.0)
            self.yaw_c_r = self.yaw_d_r = self.yaw_c_psi = self.yaw_d_psi = 0.0

        self.last_steer = 0.0
        self.last_accel = 0.0
        self._last_sol_x = None

    def update_weights(
        self,
        w_x: Optional[float] = None,
        w_y: Optional[float] = None,
        w_psi: Optional[float] = None,
        w_v: Optional[float] = None,
        w_delta: Optional[float] = None,
        w_ddelta: Optional[float] = None
    ) -> None:
        """Update dynamic cost weights for gain-scheduled regimes."""
        if w_x is not None:
            self.cfg.w_x = float(w_x)
        if w_y is not None:
            self.cfg.w_y = float(w_y)
        if w_psi is not None:
            self.cfg.w_psi = float(w_psi)
        if w_v is not None:
            self.cfg.w_v = float(w_v)
        if w_delta is not None:
            self.cfg.w_delta = float(w_delta)
        if w_ddelta is not None:
            self.cfg.w_ddelta = float(w_ddelta)

    def linearize_kinematics(
        self,
        x_bar: np.ndarray,
        u_bar: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Linearize vehicle dynamics around operating point (x_bar, u_bar)."""
        return linearize_vehicle_model(
            x_bar, u_bar, self.nx, self.nu, self.dt, self.L, self.yaw_params
        )

    def solve(
        self,
        current_state: np.ndarray,
        ref_trajectory: np.ndarray,
        prev_control: Optional[Tuple[float, float]] = None
    ) -> MPCResult:
        """Solve MPC optimization given current state and reference horizon."""
        t_start = time.perf_counter()

        if prev_control is not None:
            self.last_accel, self.last_steer = prev_control

        # Unwind yaw continuously across reference horizon
        cur_psi = current_state[2]
        unwrapped_ref = np.copy(ref_trajectory)
        for i in range(len(unwrapped_ref)):
            dpsi = (unwrapped_ref[i, 2] - cur_psi + math.pi) % (2.0 * math.pi) - math.pi
            unwrapped_ref[i, 2] = cur_psi + dpsi
            cur_psi = unwrapped_ref[i, 2]

        # Primary OSQP solver
        res, self._last_sol_x = solve_osqp(
            current_state, unwrapped_ref, self.cfg, self.nx, self.nu,
            self.last_accel, self.last_steer, self.linearize_kinematics,
            self._last_sol_x
        )
        if res.success:
            res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
            return res

        # Secondary CVXPY fallback
        res = solve_cvxpy(
            current_state, unwrapped_ref, self.cfg, self.nx, self.nu,
            self.last_accel, self.last_steer, self.linearize_kinematics
        )
        if res.success:
            res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
            return res

        # Kinematic pursuit fallback
        res = solve_kinematic_fallback(current_state, unwrapped_ref, self.cfg, self.nx)
        res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
        return res
