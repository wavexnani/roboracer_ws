import math
import numpy as np
import scipy.sparse as sp
from typing import Optional, Tuple, Callable

from .types import MPCConfig, MPCResult

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


def solve_osqp(
    x0: np.ndarray,
    x_ref: np.ndarray,
    cfg: MPCConfig,
    nx: int,
    nu: int,
    last_accel: float,
    last_steer: float,
    linearize_fn: Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]],
    last_sol_x: Optional[np.ndarray] = None
) -> Tuple[MPCResult, Optional[np.ndarray]]:
    """Assemble and solve the sparse QP via OSQP."""
    if not HAS_OSQP:
        return MPCResult(False, 0.0, 0.0, 0.0, np.array([]), np.array([]), np.array([]), np.array([])), last_sol_x

    N = cfg.N
    n_vars = (N + 1) * nx + N * nu
    if len(x0) < nx:
        x0 = np.append(x0, 0.0)

    q_diag = np.array([cfg.w_x, cfg.w_y, cfg.w_psi, cfg.w_v, cfg.w_r]) if nx == 5 else np.array([cfg.w_x, cfg.w_y, cfg.w_psi, cfg.w_v])
    q_term_diag = q_diag * cfg.w_terminal_scale
    r_diag = np.array([cfg.w_a, cfg.w_delta])
    rd_diag = np.array([cfg.w_da, cfg.w_ddelta])

    P_rows, P_cols, P_data = [], [], []
    q_vec = np.zeros(n_vars)

    for k in range(N):
        idx = k * nx
        for i in range(nx):
            P_rows.append(idx + i)
            P_cols.append(idx + i)
            P_data.append(q_diag[i] * 2.0)
        q_vec[idx:idx + 4] = -2.0 * q_diag[:4] * x_ref[k][:4]

    term_idx = N * nx
    for i in range(nx):
        P_rows.append(term_idx + i)
        P_cols.append(term_idx + i)
        P_data.append(q_term_diag[i] * 2.0)
    q_vec[term_idx:term_idx + 4] = -2.0 * q_term_diag[:4] * x_ref[N][:4]

    u_offset = (N + 1) * nx
    for k in range(N):
        uidx = u_offset + k * nu
        diag_val = (r_diag + rd_diag) * 2.0 if (N == 1 or k == N - 1) else (r_diag + 2.0 * rd_diag) * 2.0
        for j in range(nu):
            P_rows.append(uidx + j)
            P_cols.append(uidx + j)
            P_data.append(diag_val[j])

        if k > 0:
            u_prev = u_offset + (k - 1) * nu
            for j in range(nu):
                P_rows.append(uidx + j)
                P_cols.append(u_prev + j)
                P_data.append(-2.0 * rd_diag[j])
                P_rows.append(u_prev + j)
                P_cols.append(uidx + j)
                P_data.append(-2.0 * rd_diag[j])

    q_vec[u_offset:u_offset + nu] = -2.0 * rd_diag * np.array([last_accel, last_steer])

    n_constr = (N + 1) * nx + (N - 1) * nu
    row_ind, col_ind, data = [], [], []
    l_bounds = np.zeros(n_constr)
    u_bounds = np.zeros(n_constr)

    for i in range(nx):
        row_ind.append(i)
        col_ind.append(i)
        data.append(1.0)
        l_bounds[i] = x0[i]
        u_bounds[i] = x0[i]

    constr_row = nx
    alpha = float(np.clip(cfg.actuator_delay_s / 0.040, 0.0, 1.0))

    for k in range(N):
        A_k, B_k, c_k = linearize_fn(x_ref[k], np.array([0.0, 0.0]))

        for r in range(nx):
            for c in range(nx):
                if abs(A_k[r, c]) > 1e-6:
                    row_ind.append(constr_row + r)
                    col_ind.append(k * nx + c)
                    data.append(-A_k[r, c])
            row_ind.append(constr_row + r)
            col_ind.append((k + 1) * nx + r)
            data.append(1.0)

        if abs(B_k[3, 0]) > 1e-6:
            row_ind.append(constr_row + 3)
            col_ind.append(u_offset + k * nu + 0)
            data.append(-B_k[3, 0])

        for r_state in range(nx):
            b_steer = B_k[r_state, 1]
            if abs(b_steer) > 1e-6:
                if alpha > 1e-5:
                    row_ind.append(constr_row + r_state)
                    col_ind.append(u_offset + k * nu + 1)
                    data.append(-b_steer * (1.0 - alpha))
                    if k == 0:
                        c_k = np.copy(c_k)
                        c_k[r_state] += b_steer * alpha * last_steer
                    else:
                        row_ind.append(constr_row + r_state)
                        col_ind.append(u_offset + (k - 1) * nu + 1)
                        data.append(-b_steer * alpha)
                else:
                    row_ind.append(constr_row + r_state)
                    col_ind.append(u_offset + k * nu + 1)
                    data.append(-b_steer)

        for r in range(nx):
            l_bounds[constr_row + r] = c_k[r]
            u_bounds[constr_row + r] = c_k[r]
        constr_row += nx

    max_delta_steer = cfg.max_steer_rate * cfg.dt
    row_ind.append(constr_row)
    col_ind.append(u_offset + 1)
    data.append(1.0)
    l_bounds[constr_row] = last_steer - max_delta_steer
    u_bounds[constr_row] = last_steer + max_delta_steer
    constr_row += 1

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

    P_sparse = sp.csc_matrix((P_data, (P_rows, P_cols)), shape=(n_vars, n_vars))
    A_sparse = sp.csc_matrix((data, (row_ind, col_ind)), shape=(constr_row, n_vars))

    lower_box = np.full(n_vars, -np.inf)
    upper_box = np.full(n_vars, np.inf)
    for k in range(N):
        uidx = u_offset + k * nu
        lower_box[uidx] = cfg.min_accel
        upper_box[uidx] = cfg.max_accel
        lower_box[uidx + 1] = -cfg.max_steer
        upper_box[uidx + 1] = cfg.max_steer

    for k in range(N + 1):
        lower_box[k * nx + 3] = cfg.min_speed
        upper_box[k * nx + 3] = cfg.max_speed

    A_full = sp.vstack([A_sparse, sp.eye(n_vars, format='csc')], format='csc')
    l_full = np.concatenate([l_bounds[:constr_row], lower_box])
    u_full = np.concatenate([u_bounds[:constr_row], upper_box])

    try:
        prob = osqp.OSQP()
        prob.setup(
            P=P_sparse, q=q_vec, A=A_full, l=l_full, u=u_full,
            verbose=False, eps_abs=1e-4, eps_rel=1e-4, max_iter=500, polish=True
        )
        if last_sol_x is not None and len(last_sol_x) == n_vars:
            prob.warm_start(x=last_sol_x)

        res = prob.solve()
        if res.info.status_val in (1, 2):
            x_sol = res.x
            opt_a = float(x_sol[u_offset])
            opt_steer = float(x_sol[u_offset + 1])
            pred_x = np.array([x_sol[k * nx] for k in range(N + 1)])
            pred_y = np.array([x_sol[k * nx + 1] for k in range(N + 1)])
            pred_psi = np.array([x_sol[k * nx + 2] for k in range(N + 1)])
            pred_v = np.array([x_sol[k * nx + 3] for k in range(N + 1)])
            pred_r = np.array([x_sol[k * nx + 4] for k in range(N + 1)]) if nx == 5 else None

            result = MPCResult(
                success=True,
                steering=np.clip(opt_steer, -cfg.max_steer, cfg.max_steer),
                accel=opt_a,
                target_speed=max(0.5, float(pred_v[1])),
                predicted_x=pred_x,
                predicted_y=pred_y,
                predicted_psi=pred_psi,
                predicted_v=pred_v,
                predicted_r=pred_r
            )
            return result, x_sol
    except Exception:
        pass

    return MPCResult(False, 0.0, 0.0, 0.0, np.array([]), np.array([]), np.array([]), np.array([])), last_sol_x


def solve_cvxpy(
    x0: np.ndarray,
    x_ref: np.ndarray,
    cfg: MPCConfig,
    nx: int,
    nu: int,
    last_accel: float,
    last_steer: float,
    linearize_fn: Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]
) -> MPCResult:
    """Solve the MPC program using CVXPY as secondary fallback."""
    if not HAS_CVXPY:
        return MPCResult(False, 0.0, 0.0, 0.0, np.array([]), np.array([]), np.array([]), np.array([]))

    try:
        N = cfg.N
        if len(x0) < nx:
            x0 = np.append(x0, 0.0)

        x = cp.Variable((nx, N + 1))
        u = cp.Variable((nu, N))
        cost = 0
        constr = [x[:, 0] == x0]

        q_vec = [cfg.w_x, cfg.w_y, cfg.w_psi, cfg.w_v, cfg.w_r] if nx == 5 else [cfg.w_x, cfg.w_y, cfg.w_psi, cfg.w_v]
        Q = np.diag(q_vec)
        Q_term = Q * cfg.w_terminal_scale
        R = np.diag([cfg.w_a, cfg.w_delta])
        R_d = np.diag([cfg.w_da, cfg.w_ddelta])

        alpha = float(np.clip(cfg.actuator_delay_s / 0.040, 0.0, 1.0))
        max_delta_steer = cfg.max_steer_rate * cfg.dt

        for k in range(N):
            x_ref_k = np.append(x_ref[k][:4], 0.0) if nx == 5 else x_ref[k][:4]
            cost += cp.quad_form(x[:, k] - x_ref_k, Q)
            cost += cp.quad_form(u[:, k], R)

            if k == 0:
                cost += cp.quad_form(u[:, 0] - np.array([last_accel, last_steer]), R_d)
            else:
                cost += cp.quad_form(u[:, k] - u[:, k - 1], R_d)

            A_k, B_k, c_k = linearize_fn(x_ref[k], np.zeros(2))
            if alpha > 1e-5:
                u_eff_k = cp.vstack([u[0, k], (1.0 - alpha) * u[1, k] + alpha * last_steer]) if k == 0 else cp.vstack([u[0, k], (1.0 - alpha) * u[1, k] + alpha * u[1, k - 1]])
                constr += [x[:, k + 1] == A_k @ x[:, k] + B_k @ u_eff_k + c_k]
            else:
                constr += [x[:, k + 1] == A_k @ x[:, k] + B_k @ u[:, k] + c_k]

            constr += [
                u[0, k] >= cfg.min_accel, u[0, k] <= cfg.max_accel,
                u[1, k] >= -cfg.max_steer, u[1, k] <= cfg.max_steer,
                x[3, k] >= cfg.min_speed, x[3, k] <= cfg.max_speed
            ]

            if k == 0:
                constr += [u[1, 0] - last_steer >= -max_delta_steer, u[1, 0] - last_steer <= max_delta_steer]
            else:
                constr += [u[1, k] - u[1, k - 1] >= -max_delta_steer, u[1, k] - u[1, k - 1] <= max_delta_steer]

        x_ref_N = np.append(x_ref[N][:4], 0.0) if nx == 5 else x_ref[N][:4]
        cost += cp.quad_form(x[:, N] - x_ref_N, Q_term)
        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        if prob.status in ('optimal', 'optimal_inaccurate') and u.value is not None:
            return MPCResult(
                success=True,
                steering=np.clip(float(u.value[1, 0]), -cfg.max_steer, cfg.max_steer),
                accel=float(u.value[0, 0]),
                target_speed=max(0.5, float(x.value[3, 1])),
                predicted_x=np.array(x.value[0, :]),
                predicted_y=np.array(x.value[1, :]),
                predicted_psi=np.array(x.value[2, :]),
                predicted_v=np.array(x.value[3, :]),
                predicted_r=np.array(x.value[4, :]) if nx == 5 else None
            )
    except Exception:
        pass

    return MPCResult(False, 0.0, 0.0, 0.0, np.array([]), np.array([]), np.array([]), np.array([]))


def solve_kinematic_fallback(
    current_state: np.ndarray,
    ref_trajectory: np.ndarray,
    cfg: MPCConfig,
    nx: int
) -> MPCResult:
    """Fail-safe pure pursuit geometric fallback when QP solver fails."""
    cur_x, cur_y, cur_yaw, cur_v = current_state[:4]
    target_pt = ref_trajectory[min(3, len(ref_trajectory) - 1)]

    dx = target_pt[0] - cur_x
    dy = target_pt[1] - cur_y
    alpha = (math.atan2(dy, dx) - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
    dist = max(0.5, math.hypot(dx, dy))

    steer = float(np.clip(math.atan2(2.0 * cfg.wheelbase * math.sin(alpha), dist), -cfg.max_steer, cfg.max_steer))
    target_speed = float(target_pt[3])
    accel = float(np.clip((target_speed - cur_v) * 2.0, cfg.min_accel, cfg.max_accel))

    px, py, ppsi, pv = [cur_x], [cur_y], [cur_yaw], [cur_v]
    for _ in range(cfg.N):
        px.append(px[-1] + pv[-1] * math.cos(ppsi[-1]) * cfg.dt)
        py.append(py[-1] + pv[-1] * math.sin(ppsi[-1]) * cfg.dt)
        ppsi.append(ppsi[-1] + (pv[-1] / cfg.wheelbase) * math.tan(steer) * cfg.dt)
        pv.append(pv[-1] + accel * cfg.dt)

    return MPCResult(
        success=True,
        steering=steer,
        accel=accel,
        target_speed=max(0.5, target_speed),
        predicted_x=np.array(px),
        predicted_y=np.array(py),
        predicted_psi=np.array(ppsi),
        predicted_v=np.array(pv),
        predicted_r=np.zeros(len(px)) if nx == 5 else None
    )
