import math
from typing import Optional, Tuple
import numpy as np

from .types import MPCConfig, MPCResult
from .qp_solver import (
    HAS_OSQP,
    linearize_kinematics,
    solve_osqp,
    kinematic_fallback
)

# Re-export for public API and benchmark scripts
__all__ = ['MPCOptimizer', 'MPCConfig', 'MPCResult']


class MPCOptimizer:
    """LTV-MPC optimizer with dynamic gain scheduling."""

    def __init__(self, config: Optional[MPCConfig] = None):
        self.cfg = config or MPCConfig()
        self.nx = 4
        self.nu = 2
        self.N = self.cfg.N
        self.dt = self.cfg.dt
        self.L = self.cfg.wheelbase

        self.last_steer: float = 0.0
        self.last_accel: float = 0.0
        self._last_sol_x = None

    def update_weights(
        self,
        w_x: Optional[float] = None,
        w_y: Optional[float] = None,
        w_psi: Optional[float] = None,
        w_v: Optional[float] = None,
        w_delta: Optional[float] = None,
        w_ddelta: Optional[float] = None
    ):
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
        return linearize_kinematics(x_bar, u_bar, self.dt, self.L, self.nx, self.nu)

    def solve(
        self,
        current_state: np.ndarray,
        ref_trajectory: np.ndarray,
        prev_control: Optional[Tuple[float, float]] = None
    ) -> MPCResult:
        if prev_control is not None:
            self.last_accel, self.last_steer = prev_control

        # Ensure continuous yaw across reference horizon relative to current vehicle yaw
        cur_psi = float(current_state[2])
        unwrapped_ref = np.copy(ref_trajectory)
        for i in range(len(unwrapped_ref)):
            dpsi = (unwrapped_ref[i, 2] - cur_psi + math.pi) % (2.0 * math.pi) - math.pi
            unwrapped_ref[i, 2] = cur_psi + dpsi
            cur_psi = unwrapped_ref[i, 2]

        if HAS_OSQP:
            res, sol_x = solve_osqp(
                current_state,
                unwrapped_ref,
                self.cfg,
                self.last_accel,
                self.last_steer,
                self._last_sol_x
            )
            if res is not None and res.success:
                self._last_sol_x = sol_x
                return res

        return kinematic_fallback(current_state, unwrapped_ref, self.cfg)
