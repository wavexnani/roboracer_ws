#!/usr/bin/env python3
"""
High-Performance Model Predictive Control (MPC) Optimizer for F1TENTH.

Formulates a Linear Time-Varying (LTV) Model Predictive Control problem using
a 4-state kinematic bicycle model. Solves a Quadratic Program (QP) via OSQP
with warm-starting and CVXPY/SciPy fallbacks.
"""

import math
import numpy as np
import scipy.sparse as sp
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict

# Optional fast QP solver imports
try:
    import osqp
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False

try:
    import cvxpy as cp
    HAS_CVXPY = True
except ImportError:
    HAS_CVXPY = False


@dataclass
class MPCConfig:
    """Configuration and tuning weights for MPC Optimizer."""
    # Model parameters
    wheelbase: float = 0.33        # Wheelbase L [m]
    dt: float = 0.08               # Discretization time step [s]
    N: int = 10                    # Prediction horizon steps (0.8s horizon)

    # State tracking weights [x, y, psi, v]
    w_x: float = 8.0
    w_y: float = 8.0
    w_psi: float = 3.0
    w_v: float = 0.8
    w_terminal_scale: float = 2.0  # Terminal cost multiplier

    # Control input weights [a, delta]
    w_a: float = 0.1
    w_delta: float = 0.25

    # Control rate (smoothness / slew) weights [da, ddelta]
    w_da: float = 0.2
    w_ddelta: float = 1.5

    # Input constraints
    max_steer: float = 0.4189      # Max steering angle [rad] (~24 deg)
    max_steer_rate: float = 2.8    # Max steering angular velocity [rad/s] (yields 0.224 rad/step at dt=0.08)
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
    predicted_x: np.ndarray         # (N+1,) predicted X trajectory [m]
    predicted_y: np.ndarray         # (N+1,) predicted Y trajectory [m]
    predicted_psi: np.ndarray       # (N+1,) predicted yaw [rad]
    predicted_v: np.ndarray         # (N+1,) predicted speed [m/s]
    solve_time_ms: float            # QP solver execution time in ms


class MPCOptimizer:
    """
    Linear Time-Varying Model Predictive Controller for F1TENTH path tracking.
    """

    def __init__(self, config: Optional[MPCConfig] = None):
        self.cfg = config or MPCConfig()
        self.nx = 4  # [x, y, psi, v]
        self.nu = 2  # [a, delta]
        self.N = self.cfg.N
        self.dt = self.cfg.dt
        self.L = self.cfg.wheelbase

        # Last applied control for rate penalty
        self.last_steer: float = 0.0
        self.last_accel: float = 0.0

        # OSQP persistent solver instance for warm starting
        self._osqp_solver = None
        self._last_sol_x = None

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

        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        cos_delta = math.cos(delta)
        cos_delta_sq = max(cos_delta * cos_delta, 1e-4)
        tan_delta = math.tan(delta)

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
        Solves the MPC optimization problem.

        Args:
            current_state: (4,) array [x, y, psi, v]
            ref_trajectory: (N+1, 4) array [[x_ref, y_ref, psi_ref, v_ref], ...]
            prev_control: Optional (accel, steer) applied at previous step

        Returns:
            MPCResult with optimal control commands and predicted horizon.
        """
        import time
        t_start = time.perf_counter()

        if prev_control is not None:
            self.last_accel, self.last_steer = prev_control

        # Ensure continuous yaw across reference horizon relative to current yaw
        cur_psi = current_state[2]
        unwrapped_ref = np.copy(ref_trajectory)
        for i in range(len(unwrapped_ref)):
            dpsi = (unwrapped_ref[i, 2] - cur_psi + math.pi) % (2.0 * math.pi) - math.pi
            unwrapped_ref[i, 2] = cur_psi + dpsi
            cur_psi = unwrapped_ref[i, 2]

        # Attempt solving via OSQP
        if HAS_OSQP:
            res = self._solve_osqp(current_state, unwrapped_ref)
            if res.success:
                res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
                return res

        # Fallback to CVXPY
        if HAS_CVXPY:
            res = self._solve_cvxpy(current_state, unwrapped_ref)
            if res.success:
                res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
                return res

        # Ultimate kinematic fallback
        res = self._kinematic_fallback(current_state, unwrapped_ref)
        res.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
        return res

    def _solve_osqp(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray
    ) -> MPCResult:
        """Formulates and solves the sparse QP directly via OSQP."""
        N = self.N
        nx = self.nx
        nu = self.nu
        n_vars = (N + 1) * nx + N * nu

        # State cost weights
        q_diag = np.array([self.cfg.w_x, self.cfg.w_y, self.cfg.w_psi, self.cfg.w_v])
        q_term_diag = q_diag * self.cfg.w_terminal_scale
        r_diag = np.array([self.cfg.w_a, self.cfg.w_delta])
        rd_diag = np.array([self.cfg.w_da, self.cfg.w_ddelta])

        # Quadratic cost matrix P (sparse diagonal)
        P_diag = np.zeros(n_vars)
        q_vec = np.zeros(n_vars)

        # Populate state costs (k = 0 to N)
        for k in range(N):
            idx = k * nx
            P_diag[idx:idx + nx] = q_diag * 2.0
            q_vec[idx:idx + nx] = -2.0 * q_diag * x_ref[k]

        # Terminal state cost
        term_idx = N * nx
        P_diag[term_idx:term_idx + nx] = q_term_diag * 2.0
        q_vec[term_idx:term_idx + nx] = -2.0 * q_term_diag * x_ref[N]

        # Control effort costs (k = 0 to N-1)
        u_offset = (N + 1) * nx
        for k in range(N):
            uidx = u_offset + k * nu
            P_diag[uidx:uidx + nu] = (r_diag + rd_diag) * 2.0
            if k == 0:
                q_vec[uidx:uidx + nu] = -2.0 * rd_diag * np.array([self.last_accel, self.last_steer])

        # Build linear dynamics around reference trajectory operating points
        # Dynamics constraints: x_{k+1} - A_k x_k - B_k u_k = c_k (N * nx constraints)
        # Initial condition: x_0 = x0 (nx constraints)
        # Slew rate constraints: u_k - u_{k-1} between [-max_rate, max_rate] ((N-1) * nu constraints)
        # Variable bounds: handled via l and u

        n_eq = (N + 1) * nx
        n_slew = (N - 1) * nu
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

        # 2. Dynamics constraints
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

            # Affine equality bound: = c_k
            for r in range(nx):
                l_bounds[constr_row + r] = c_k[r]
                u_bounds[constr_row + r] = c_k[r]

            constr_row += nx

        # 3. Slew rate constraints on controls
        max_delta_steer = self.cfg.max_steer_rate * self.cfg.dt
        # Step 0 constraint against previously applied steering:
        # -max_delta_steer <= u_0[1] - self.last_steer <= max_delta_steer
        row_ind.append(constr_row)
        col_ind.append(u_offset + 1)
        data.append(1.0)
        l_bounds[constr_row] = self.last_steer - max_delta_steer
        u_bounds[constr_row] = self.last_steer + max_delta_steer
        constr_row += 1

        # Steps 1 to N-1: -max_delta_steer <= u_{k+1} - u_k <= max_delta_steer
        for k in range(N - 1):
            u_curr = u_offset + k * nu
            u_next = u_offset + (k + 1) * nu

            # Steering rate
            row_ind.append(constr_row)
            col_ind.append(u_next + 1)
            data.append(1.0)
            row_ind.append(constr_row)
            col_ind.append(u_curr + 1)
            data.append(-1.0)
            l_bounds[constr_row] = -max_delta_steer
            u_bounds[constr_row] = max_delta_steer
            constr_row += 1

        # Build sparse matrices
        P_sparse = sp.diags(P_diag, format='csc')
        A_sparse = sp.csc_matrix((data, (row_ind, col_ind)), shape=(constr_row, n_vars))

        # Decision variable box bounds
        lower_box = np.full(n_vars, -np.inf)
        upper_box = np.full(n_vars, np.inf)

        # Control box limits
        for k in range(N):
            uidx = u_offset + k * nu
            # accel
            lower_box[uidx] = self.cfg.min_accel
            upper_box[uidx] = self.cfg.max_accel
            # steer
            lower_box[uidx + 1] = -self.cfg.max_steer
            upper_box[uidx + 1] = self.cfg.max_steer

        # Velocity box limits
        for k in range(N + 1):
            lower_box[k * nx + 3] = self.cfg.min_speed
            upper_box[k * nx + 3] = self.cfg.max_speed

        # Stack general constraints with box constraints
        A_full = sp.vstack([A_sparse, sp.eye(n_vars, format='csc')], format='csc')
        l_full = np.concatenate([l_bounds[:constr_row], lower_box])
        u_full = np.concatenate([u_bounds[:constr_row], upper_box])

        # Solve with OSQP
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

            if res.info.status_val in (1, 2):  # OSQP_SOLVED, OSQP_SOLVED_INACCURATE
                x_sol = res.x
                self._last_sol_x = x_sol

                # Extract optimal commands
                opt_a = float(x_sol[u_offset])
                opt_steer = float(x_sol[u_offset + 1])

                # Extract predicted trajectory
                pred_x = np.array([x_sol[k * nx] for k in range(N + 1)])
                pred_y = np.array([x_sol[k * nx + 1] for k in range(N + 1)])
                pred_psi = np.array([x_sol[k * nx + 2] for k in range(N + 1)])
                pred_v = np.array([x_sol[k * nx + 3] for k in range(N + 1)])

                target_speed = float(pred_v[1])

                return MPCResult(
                    success=True,
                    steering=np.clip(opt_steer, -self.cfg.max_steer, self.cfg.max_steer),
                    accel=opt_a,
                    target_speed=max(0.5, target_speed),
                    predicted_x=pred_x,
                    predicted_y=pred_y,
                    predicted_psi=pred_psi,
                    predicted_v=pred_v,
                    solve_time_ms=0.0
                )
        except Exception:
            pass

        return MPCResult(
            success=False,
            steering=0.0,
            accel=0.0,
            target_speed=0.0,
            predicted_x=np.array([]),
            predicted_y=np.array([]),
            predicted_psi=np.array([]),
            predicted_v=np.array([]),
            solve_time_ms=0.0
        )

    def _solve_cvxpy(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray
    ) -> MPCResult:
        """Secondary fallback QP solver via CVXPY."""
        try:
            N = self.N
            nx = self.nx
            nu = self.nu

            x = cp.Variable((nx, N + 1))
            u = cp.Variable((nu, N))

            cost = 0
            constr = [x[:, 0] == x0]

            Q = np.diag([self.cfg.w_x, self.cfg.w_y, self.cfg.w_psi, self.cfg.w_v])
            Q_term = Q * self.cfg.w_terminal_scale
            R = np.diag([self.cfg.w_a, self.cfg.w_delta])
            R_d = np.diag([self.cfg.w_da, self.cfg.w_ddelta])

            for k in range(N):
                cost += cp.quad_form(x[:, k] - x_ref[k], Q)
                cost += cp.quad_form(u[:, k], R)
                if k == 0:
                    cost += cp.quad_form(u[:, 0] - np.array([self.last_accel, self.last_steer]), R_d)
                else:
                    cost += cp.quad_form(u[:, k] - u[:, k - 1], R_d)

                A_k, B_k, c_k = self.linearize_kinematics(x_ref[k], np.zeros(2))
                constr += [x[:, k + 1] == A_k @ x[:, k] + B_k @ u[:, k] + c_k]

                # Bounds
                constr += [
                    u[0, k] >= self.cfg.min_accel,
                    u[0, k] <= self.cfg.max_accel,
                    u[1, k] >= -self.cfg.max_steer,
                    u[1, k] <= self.cfg.max_steer,
                    x[3, k] >= self.cfg.min_speed,
                    x[3, k] <= self.cfg.max_speed
                ]

                # Slew rate constraints on steering
                max_delta_steer = self.cfg.max_steer_rate * self.cfg.dt
                if k == 0:
                    constr += [
                        u[1, 0] - self.last_steer >= -max_delta_steer,
                        u[1, 0] - self.last_steer <= max_delta_steer
                    ]
                else:
                    constr += [
                        u[1, k] - u[1, k - 1] >= -max_delta_steer,
                        u[1, k] - u[1, k - 1] <= max_delta_steer
                    ]

            cost += cp.quad_form(x[:, N] - x_ref[N], Q_term)
            prob = cp.Problem(cp.Minimize(cost), constr)
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)

            if prob.status in ('optimal', 'optimal_inaccurate') and u.value is not None:
                opt_a = float(u.value[0, 0])
                opt_steer = float(u.value[1, 0])
                pred_x = np.array(x.value[0, :])
                pred_y = np.array(x.value[1, :])
                pred_psi = np.array(x.value[2, :])
                pred_v = np.array(x.value[3, :])

                return MPCResult(
                    success=True,
                    steering=np.clip(opt_steer, -self.cfg.max_steer, self.cfg.max_steer),
                    accel=opt_a,
                    target_speed=max(0.5, float(pred_v[1])),
                    predicted_x=pred_x,
                    predicted_y=pred_y,
                    predicted_psi=pred_psi,
                    predicted_v=pred_v,
                    solve_time_ms=0.0
                )
        except Exception:
            pass

        return MPCResult(
            success=False,
            steering=0.0,
            accel=0.0,
            target_speed=0.0,
            predicted_x=np.array([]),
            predicted_y=np.array([]),
            predicted_psi=np.array([]),
            predicted_v=np.array([]),
            solve_time_ms=0.0
        )

    def _kinematic_fallback(
        self,
        current_state: np.ndarray,
        ref_trajectory: np.ndarray
    ) -> MPCResult:
        """Fail-safe kinematic proportional pursuit fallback if QP solvers fail."""
        cur_x, cur_y, cur_yaw, cur_v = current_state
        target_pt = ref_trajectory[min(3, len(ref_trajectory) - 1)]

        dx = target_pt[0] - cur_x
        dy = target_pt[1] - cur_y
        target_angle = math.atan2(dy, dx)
        alpha = (target_angle - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        dist = max(0.5, math.hypot(dx, dy))
        steer = math.atan2(2.0 * self.L * math.sin(alpha), dist)
        steer = np.clip(steer, -self.cfg.max_steer, self.cfg.max_steer)

        target_speed = float(target_pt[3])
        accel = np.clip((target_speed - cur_v) * 2.0, self.cfg.min_accel, self.cfg.max_accel)

        # Forward simulate simple kinematic horizon for visualization
        px, py, ppsi, pv = [cur_x], [cur_y], [cur_yaw], [cur_v]
        for _ in range(self.N):
            nx = px[-1] + pv[-1] * math.cos(ppsi[-1]) * self.dt
            ny = py[-1] + pv[-1] * math.sin(ppsi[-1]) * self.dt
            npsi = ppsi[-1] + (pv[-1] / self.L) * math.tan(steer) * self.dt
            nv = pv[-1] + accel * self.dt
            px.append(nx)
            py.append(ny)
            ppsi.append(npsi)
            pv.append(nv)

        return MPCResult(
            success=True,
            steering=steer,
            accel=accel,
            target_speed=max(0.5, target_speed),
            predicted_x=np.array(px),
            predicted_y=np.array(py),
            predicted_psi=np.array(ppsi),
            predicted_v=np.array(pv),
            solve_time_ms=0.0
        )
