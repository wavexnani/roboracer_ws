from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class MPCConfig:
    """Configuration and tuning weights for MPC Optimizer."""
    wheelbase: float = 0.33
    dt: float = 0.08
    N: int = 10

    # Tracking weights [x, y, psi, v]
    w_x: float = 6.5
    w_y: float = 6.5
    w_psi: float = 2.8
    w_v: float = 0.8
    w_terminal_scale: float = 2.0

    # Control weights [a, delta] and rate weights [da, ddelta]
    w_a: float = 0.1
    w_delta: float = 0.35
    w_da: float = 0.2
    w_ddelta: float = 2.2

    # Physical limits
    max_steer: float = 0.4189
    max_steer_rate: float = 2.0
    min_accel: float = -7.0
    max_accel: float = 4.0
    min_speed: float = 0.0
    max_speed: float = 12.0

    # Actuator transport delay
    actuator_delay_s: float = 0.0

    # Predictive model configuration ('kinematic' or 'yaw_first_order')
    model_type: str = "kinematic"
    yaw_K: float = 15.4016
    yaw_tau: float = 0.0363
    w_r: float = 0.0


@dataclass
class MPCResult:
    """Output solution from the MPC Optimizer."""
    success: bool
    steering: float
    accel: float
    target_speed: float
    predicted_x: np.ndarray
    predicted_y: np.ndarray
    predicted_psi: np.ndarray
    predicted_v: np.ndarray
    predicted_r: Optional[np.ndarray] = None
    solve_time_ms: float = 0.0
