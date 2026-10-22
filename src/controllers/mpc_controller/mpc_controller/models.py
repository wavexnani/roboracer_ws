import math
import numpy as np
from typing import Tuple


def compute_yaw_model_matrices(dt: float, tau: float, k_gain: float) -> Tuple[float, float, float, float]:
    """Exact zero-order hold discretization parameters for first-order yaw dynamics."""
    a = math.exp(-dt / tau)
    c_r = a
    d_r = k_gain * (1.0 - a)
    c_psi = tau * (1.0 - a)
    d_psi = k_gain * (dt - tau * (1.0 - a))
    return c_r, d_r, c_psi, d_psi


def linearize_vehicle_model(
    x_bar: np.ndarray,
    u_bar: np.ndarray,
    nx: int,
    nu: int,
    dt: float,
    wheelbase: float,
    yaw_params: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Linearize vehicle dynamics around operating point:
    x_{k+1} = A * x_k + B * u_k + c
    """
    v = float(x_bar[3])
    psi = float(x_bar[2])
    delta = float(u_bar[1])

    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    cos_delta = math.cos(delta)
    cos_delta_sq = max(cos_delta * cos_delta, 1e-4)
    tan_delta = math.tan(delta)

    A = np.eye(nx)
    A[0, 2] = -v * sin_psi * dt
    A[0, 3] = cos_psi * dt
    A[1, 2] = v * cos_psi * dt
    A[1, 3] = sin_psi * dt

    B = np.zeros((nx, nu))
    B[3, 0] = dt

    if nx == 5:
        c_r, d_r, c_psi, d_psi = yaw_params
        A[2, 4] = c_psi
        B[2, 1] = d_psi
        A[4, 4] = c_r
        B[4, 1] = d_r

        c = np.zeros(nx)
        c[0] = v * cos_psi * dt - (A[0, :4] @ x_bar[:4]) + x_bar[0]
        c[1] = v * sin_psi * dt - (A[1, :4] @ x_bar[:4]) + x_bar[1]
    else:
        A[2, 3] = (tan_delta / wheelbase) * dt
        B[2, 1] = (v / (wheelbase * cos_delta_sq)) * dt

        f = np.array([
            v * cos_psi,
            v * sin_psi,
            (v / wheelbase) * tan_delta,
            float(u_bar[0])
        ])
        c = f * dt - A @ x_bar[:nx] - B @ u_bar + x_bar[:nx]

    return A, B, c
