#!/usr/bin/env python3
"""
Model Predictive Control (MPC) Optimizer for F1TENTH / ROBORACER.
Solves a Linear Time-Varying (LTV) Quadratic Program (QP) via OSQP over
a 4-state kinematic bicycle model [x, y, psi, v].
Supports dynamic gain-scheduling (Q, R, R_delta) across multi-regime hybrid operation.
Ensures continuous heading unwrapping across prediction horizon relative to current vehicle yaw.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import scipy.sparse as sp

try:
    import osqp
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False


@dataclass
class MPCConfig:
    """Tunable hyperparameters for MPC Optimizer."""
    wheelbase: float = 0.33        # Wheelbase L [m]
    dt: float = 0.08               # Discretization time step [s]
    N: int = 10                    # Prediction horizon steps (0.8s horizon)

    # State tracking weights [x, y, psi, v]
    w_x: float = 8.0
    w_y: float = 8.0
    w_psi: float = 3.0
    w_v: float = 0.8
    w_terminal_scale: float = 2.0

    # Control input weights [a, delta]
    w_a: float = 0.1
    w_delta: float = 0.25

    # Control rate (slew) weights [da, ddelta]
    w_da: float = 0.2
    w_ddelta: float = 1.5

    # Kinematic limits
    max_steer: float = 0.4189      # Max steering angle [rad] (~24 deg)
    max_steer_rate: float = 2.8    # Max steering angular velocity [rad/s]
    min_accel: float = -7.0        # Max braking deceleration [m/s^2]
    max_accel: float = 4.0         # Max forward acceleration [m/s^2]
    min_speed: float = 0.0         # Min velocity [m/s]
    max_speed: float = 12.0        # Max velocity [m/s]


@dataclass
class MPCResult:
    """Output solution from the MPC Optimizer."""
    success: bool
    steering: float                 # Optimal steering command delta_0 [rad]
    accel: float                    # Optimal acceleration command a_0 [m/s^2]
    target_speed: float             # Optimal target speed v_1 [m/s]
    predicted_trajectory: np.ndarray  # (N+1, 4) predicted [x, y, psi, v]
    solve_time_ms: float = 0.0


class MPCOptimizer:
    """High-performance LTV-MPC optimizer with dynamic gain scheduling."""

    def __init__(self, config: Optional[MPCConfig] = None):
        self.cfg = config or MPCConfig()
        self.nx = 4  # [x, y, psi, v]
        self.nu = 2  # [a, delta]
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
        """Dynamically adjusts cost weights for gain-scheduled hybrid regimes."""
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
        """
        Linearizes the kinematic bicycle model around operating point (x_bar, u_bar):
        x_{k+1} = A * x_k + B * u_k + c
        """
        v = float(x_bar[3])
        psi = float(x_bar[2])
        delta = float(u_bar[1])
        dt = self.dt
        L = self.L

        delta_clamped = float(np.clip(delta, -0.41, 0.41))
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        cos_delta = math.cos(delta_clamped)
        cos_delta_sq = max(cos_delta * cos_delta, 1e-4)
        tan_delta = math.tan(delta_clamped)

        # State transition matrix A
        A = np.eye(self.nx)
        A[0, 2] = -v * sin_psi * dt
        A[0, 3] = cos_psi * dt
        A[1, 2] = v * cos_psi * dt
        A[1, 3] = sin_psi * dt
        A[2, 3] = (tan_delta / L) * dt

        # Control input matrix B
        B = np.zeros((self.nx, self.nu))
        B[2, 1] = (v / (L * cos_delta_sq)) * dt
        B[3, 0] = dt

        # Affine offset term: c = f(x_bar, u_bar)*dt - A*x_bar - B*u_bar + x_bar
        f = np.array([
            v * cos_psi,
            v * sin_psi,
            (v / L) * tan_delta,
            float(u_bar[0])
        ])
        c = f * dt - A @ x_bar - B @ u_bar + x_bar

        return A, B, c

    def solve(
        self,
        current_state: np.ndarray,
        ref_trajectory: np.ndarray,
        prev_control: Optional[Tuple[float, float]] = None
    ) -> MPCResult:
        """
        Solves QP over prediction horizon N.
        Unwraps reference headings relative to current vehicle yaw to prevent 2*pi wrap jumps.
        """
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
            res = self._solve_osqp(current_state, unwrapped_ref)
            if res.success:
                return res

        return self._kinematic_fallback(current_state, unwrapped_ref)

    def _solve_osqp(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray
    ) -> MPCResult:
        """Solves sparse QP formulation via OSQP."""
        N = self.N
        nx = self.nx
        nu = self.nu
        n_vars = (N + 1) * nx + N * nu

        # State cost weights
        q_diag = np.array([self.cfg.w_x, self.cfg.w_y, self.cfg.w_psi, self.cfg.w_v])
        q_term_diag = q_diag * self.cfg.w_terminal_scale
        r_diag = np.array([self.cfg.w_a, self.cfg.w_delta])
        rd_diag = np.array([self.cfg.w_da, self.cfg.w_ddelta])

        # Quadratic cost diagonal P and linear vector q
        P_diag = np.zeros(n_vars)
        q_vec = np.zeros(n_vars)

        # State tracking cost
        for k in range(N):
            idx = k * nx
            P_diag[idx:idx + nx] = q_diag * 2.0
            q_vec[idx:idx + nx] = -2.0 * q_diag * x_ref[k]

        term_idx = N * nx
        P_diag[term_idx:term_idx + nx] = q_term_diag * 2.0
        q_vec[term_idx:term_idx + nx] = -2.0 * q_term_diag * x_ref[N]

        # Control effort and slew costs
        u_offset = (N + 1) * nx
        for k in range(N):
            uidx = u_offset + k * nu
            P_diag[uidx:uidx + nu] = (r_diag + rd_diag) * 2.0
            if k == 0:
                q_vec[uidx:uidx + nu] = -2.0 * rd_diag * np.array([self.last_accel, self.last_steer])

        # Linear dynamics and slew rate constraints
        n_eq = (N + 1) * nx
        n_slew = 1 + (N - 1)  # step 0 + subsequent steps
        n_constr = n_eq + n_slew

        row_ind = []
        col_ind = []
        data = []
        l_bounds = np.zeros(n_constr)
        u_bounds = np.zeros(n_constr)

        # 1. Initial condition: x_0 = x0
        for i in range(nx):
            row_ind.append(i)
            col_ind.append(i)
            data.append(1.0)
            l_bounds[i] = x0[i]
            u_bounds[i] = x0[i]

        constr_row = nx

        # 2. Dynamics constraints: x_{k+1} - A_k x_k - B_k u_k = c_k
        for k in range(N):
            x_bar = x_ref[k]
            u_bar = np.array([0.0, 0.0])
            A_k, B_k, c_k = self.linearize_kinematics(x_bar, u_bar)

            # -A_k * x_k
            for r in range(nx):
                for c in range(nx):
                    if abs(A_k[r, c]) > 1e-6:
                        row_ind.append(constr_row + r)
                        col_ind.append(k * nx + c)
                        data.append(-A_k[r, c])

            # + x_{k+1}
            for r in range(nx):
                row_ind.append(constr_row + r)
                col_ind.append((k + 1) * nx + r)
                data.append(1.0)

            # -B_k * u_k
            for r in range(nx):
                for c in range(nu):
                    if abs(B_k[r, c]) > 1e-6:
                        row_ind.append(constr_row + r)
                        col_ind.append(u_offset + k * nu + c)
                        data.append(-B_k[r, c])

            for r in range(nx):
                l_bounds[constr_row + r] = c_k[r]
                u_bounds[constr_row + r] = c_k[r]

            constr_row += nx

        # 3. Slew rate constraints on steering
        max_delta_steer = self.cfg.max_steer_rate * self.cfg.dt

        # Step 0 against last applied steer
        row_ind.append(constr_row)
        col_ind.append(u_offset + 1)
        data.append(1.0)
        l_bounds[constr_row] = self.last_steer - max_delta_steer
        u_bounds[constr_row] = self.last_steer + max_delta_steer
        constr_row += 1

        # Steps 1 to N-1
        for k in range(N - 1):
            u_curr = u_offset + k * nu
            u_next = u_offset + (k + 1) * nu

            row_ind.append(constr_row)
            col_ind.append(u_next + 1)
            data.append(1.0)
            row_ind.append(constr_row)
            col_ind.append(u_curr + 1)
            data.append(-1.0)
            l_bounds[constr_row] = -max_delta_steer
            u_bounds[constr_row] = max_delta_steer
            constr_row += 1

        P_sparse = sp.diags(P_diag, format='csc')
        A_sparse = sp.csc_matrix((data, (row_ind, col_ind)), shape=(constr_row, n_vars))

        # Box constraints
        lower_box = np.full(n_vars, -np.inf)
        upper_box = np.full(n_vars, np.inf)

        for k in range(N):
            uidx = u_offset + k * nu
            lower_box[uidx] = self.cfg.min_accel
            upper_box[uidx] = self.cfg.max_accel
            lower_box[uidx + 1] = -self.cfg.max_steer
            upper_box[uidx + 1] = self.cfg.max_steer

        for k in range(N + 1):
            lower_box[k * nx + 3] = self.cfg.min_speed
            upper_box[k * nx + 3] = self.cfg.max_speed

        A_full = sp.vstack([A_sparse, sp.eye(n_vars, format='csc')], format='csc')
        l_full = np.concatenate([l_bounds[:constr_row], lower_box])
        u_full = np.concatenate([u_bounds[:constr_row], upper_box])

        try:
            prob = osqp.OSQP()
            prob.setup(
                P=P_sparse,
                q=q_vec,
                A=A_full,
                l=l_full,
                u=u_full,
                verbose=False,
                eps_abs=1e-4,
                eps_rel=1e-4,
                max_iter=500,
                polish=True
            )
            if self._last_sol_x is not None and len(self._last_sol_x) == n_vars:
                prob.warm_start(x=self._last_sol_x)

            res = prob.solve()

            if res.info.status_val in (1, 2):
                x_sol = res.x
                self._last_sol_x = x_sol

                opt_a = float(x_sol[u_offset])
                opt_steer = float(x_sol[u_offset + 1])

                pred_traj = np.zeros((N + 1, nx))
                for k in range(N + 1):
                    pred_traj[k] = x_sol[k * nx:(k + 1) * nx]

                target_speed = float(pred_traj[1, 3])

                return MPCResult(
                    success=True,
                    steering=float(np.clip(opt_steer, -self.cfg.max_steer, self.cfg.max_steer)),
                    accel=opt_a,
                    target_speed=max(0.5, target_speed),
                    predicted_trajectory=pred_traj,
                    solve_time_ms=0.0
                )
        except Exception:
            pass

        return self._kinematic_fallback(x0, x_ref)

    def _kinematic_fallback(
        self,
        current_state: np.ndarray,
        ref_trajectory: np.ndarray
    ) -> MPCResult:
        """Fail-safe kinematic bicycle pursuit fallback if QP solver fails."""
        cur_x, cur_y, cur_yaw, cur_v = current_state
        target_pt = ref_trajectory[min(3, len(ref_trajectory) - 1)]

        dx = target_pt[0] - cur_x
        dy = target_pt[1] - cur_y
        target_angle = math.atan2(dy, dx)
        alpha = (target_angle - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        dist = max(0.5, math.hypot(dx, dy))
        steer = math.atan2(2.0 * self.L * math.sin(alpha), dist)
        steer = float(np.clip(steer, -self.cfg.max_steer, self.cfg.max_steer))

        target_speed = float(target_pt[3])
        accel = float(np.clip((target_speed - cur_v) * 2.0, self.cfg.min_accel, self.cfg.max_accel))

        pred_traj = np.zeros((self.N + 1, self.nx))
        pred_traj[0] = current_state
        for k in range(self.N):
            px, py, ppsi, pv = pred_traj[k]
            nx = px + pv * math.cos(ppsi) * self.dt
            ny = py + pv * math.sin(ppsi) * self.dt
            npsi = ppsi + (pv / self.L) * math.tan(steer) * self.dt
            nv = pv + accel * self.dt
            pred_traj[k + 1] = [nx, ny, npsi, nv]

        return MPCResult(
            success=True,
            steering=steer,
            accel=accel,
            target_speed=max(0.5, target_speed),
            predicted_trajectory=pred_traj,
            solve_time_ms=0.0
        )
