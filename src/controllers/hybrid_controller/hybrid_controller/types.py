from dataclasses import dataclass
import numpy as np


@dataclass
class MPCConfig:
    wheelbase: float = 0.33
    dt: float = 0.08
    N: int = 10

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
    max_steer: float = 0.4189
    max_steer_rate: float = 2.8
    min_accel: float = -7.0
    max_accel: float = 4.0
    min_speed: float = 0.0
    max_speed: float = 12.0


@dataclass
class MPCResult:
    success: bool
    steering: float
    accel: float
    target_speed: float
    predicted_trajectory: np.ndarray
    solve_time_ms: float = 0.0
