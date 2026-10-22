import math
from typing import Optional, Tuple
import numpy as np
import scipy.sparse as sp

from .types import MPCConfig, MPCResult

try:
    import osqp
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False


def linearize_kinematics(
    x_bar: np.ndarray,
    u_bar: np.ndarray,
    dt: float,
    L: float,
    nx: int = 4,
    nu: int = 2
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    v = float(x_bar[3])
    psi = float(x_bar[2])
    delta = float(u_bar[1])

    delta_clamped = float(np.clip(delta, -0.41, 0.41))
    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    cos_delta = math.cos(delta_clamped)
    cos_delta_sq = max(cos_delta * cos_delta, 1e-4)
    tan_delta = math.tan(delta_clamped)

    A = np.eye(nx)
    A[0, 2] = -v * sin_psi * dt
    A[0, 3] = cos_psi * dt
    A[1, 2] = v * cos_psi * dt
    A[1, 3] = sin_psi * dt
    A[2, 3] = (tan_delta / L) * dt

    B = np.zeros((nx, nu))
    B[2, 1] = (v / (L * cos_delta_sq)) * dt
    B[3, 0] = dt

    f = np.array([
        v * cos_psi,
        v * sin_psi,
        (v / L) * tan_delta,
        float(u_bar[0])
    ])
    c = f * dt - A @ x_bar - B @ u_bar + x_bar

    return A, B, c


def solve_osqp(
    x0: np.ndarray,
    x_ref: np.ndarray,
    cfg: MPCConfig,
    last_accel: float,
    last_steer: float,
    last_sol_x: Optional[np.ndarray] = None
) -> Tuple[Optional[MPCResult], Optional[np.ndarray]]:
    if not HAS_OSQP:
        return None, None

    N = cfg.N
    nx = 4
    nu = 2
    dt = cfg.dt
    L = cfg.wheelbase
    n_vars = (N + 1) * nx + N * nu

    q_diag = np.array([cfg.w_x, cfg.w_y, cfg.w_psi, cfg.w_v])
    q_term_diag = q_diag * cfg.w_terminal_scale
    r_diag = np.array([cfg.w_a, cfg.w_delta])
    rd_diag = np.array([cfg.w_da, cfg.w_ddelta])

    P_diag = np.zeros(n_vars)
    q_vec = np.zeros(n_vars)

    for k in range(N):
        idx = k * nx
        P_diag[idx:idx + nx] = q_diag * 2.0
        q_vec[idx:idx + nx] = -2.0 * q_diag * x_ref[k]

    term_idx = N * nx
    P_diag[term_idx:term_idx + nx] = q_term_diag * 2.0
    q_vec[term_idx:term_idx + nx] = -2.0 * q_term_diag * x_ref[N]

    u_offset = (N + 1) * nx
    for k in range(N):
        uidx = u_offset + k * nu
        P_diag[uidx:uidx + nu] = (r_diag + rd_diag) * 2.0
        if k == 0:
            q_vec[uidx:uidx + nu] = -2.0 * rd_diag * np.array([last_accel, last_steer])

    n_eq = (N + 1) * nx
    n_slew = 1 + (N - 1)
    n_constr = n_eq + n_slew

    row_ind = []
    col_ind = []
    data = []
    l_bounds = np.zeros(n_constr)
    u_bounds = np.zeros(n_constr)

    # Initial condition: x_0 = x0
    for i in range(nx):
        row_ind.append(i)
        col_ind.append(i)
        data.append(1.0)
        l_bounds[i] = x0[i]
        u_bounds[i] = x0[i]

    constr_row = nx

    # Dynamics: x_{k+1} - A_k x_k - B_k u_k = c_k
    for k in range(N):
        x_bar = x_ref[k]
        u_bar = np.array([0.0, 0.0])
        A_k, B_k, c_k = linearize_kinematics(x_bar, u_bar, dt, L, nx, nu)

        for r in range(nx):
            for c in range(nx):
                if abs(A_k[r, c]) > 1e-6:
                    row_ind.append(constr_row + r)
                    col_ind.append(k * nx + c)
                    data.append(-A_k[r, c])

        for r in range(nx):
            row_ind.append(constr_row + r)
            col_ind.append((k + 1) * nx + r)
            data.append(1.0)

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

    # Steering slew rate constraints
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

    P_sparse = sp.diags(P_diag, format='csc')
    A_sparse = sp.csc_matrix((data, (row_ind, col_ind)), shape=(constr_row, n_vars))

    # Box bounds on inputs and speed
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
        if last_sol_x is not None and len(last_sol_x) == n_vars:
            prob.warm_start(x=last_sol_x)

        res = prob.solve()

        if res.info.status_val in (1, 2):
            x_sol = res.x
            opt_a = float(x_sol[u_offset])
            opt_steer = float(x_sol[u_offset + 1])

            pred_traj = np.zeros((N + 1, nx))
            for k in range(N + 1):
                pred_traj[k] = x_sol[k * nx:(k + 1) * nx]

            target_speed = float(pred_traj[1, 3])

            return MPCResult(
                success=True,
                steering=float(np.clip(opt_steer, -cfg.max_steer, cfg.max_steer)),
                accel=opt_a,
                target_speed=max(0.5, target_speed),
                predicted_trajectory=pred_traj,
                solve_time_ms=0.0
            ), x_sol
    except Exception:
        pass

    return None, None


def kinematic_fallback(
    current_state: np.ndarray,
    ref_trajectory: np.ndarray,
    cfg: MPCConfig
) -> MPCResult:
    cur_x, cur_y, cur_yaw, cur_v = current_state
    target_pt = ref_trajectory[min(3, len(ref_trajectory) - 1)]

    dx = target_pt[0] - cur_x
    dy = target_pt[1] - cur_y
    target_angle = math.atan2(dy, dx)
    alpha = (target_angle - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

    dist = max(0.5, math.hypot(dx, dy))
    steer = math.atan2(2.0 * cfg.wheelbase * math.sin(alpha), dist)
    steer = float(np.clip(steer, -cfg.max_steer, cfg.max_steer))

    target_speed = float(target_pt[3])
    accel = float(np.clip((target_speed - cur_v) * 2.0, cfg.min_accel, cfg.max_accel))

    pred_traj = np.zeros((cfg.N + 1, 4))
    pred_traj[0] = current_state
    for k in range(cfg.N):
        px, py, ppsi, pv = pred_traj[k]
        nx = px + pv * math.cos(ppsi) * cfg.dt
        ny = py + pv * math.sin(ppsi) * cfg.dt
        npsi = ppsi + (pv / cfg.wheelbase) * math.tan(steer) * cfg.dt
        nv = pv + accel * cfg.dt
        pred_traj[k + 1] = [nx, ny, npsi, nv]

    return MPCResult(
        success=True,
        steering=steer,
        accel=accel,
        target_speed=max(0.5, target_speed),
        predicted_trajectory=pred_traj,
        solve_time_ms=0.0
    )
