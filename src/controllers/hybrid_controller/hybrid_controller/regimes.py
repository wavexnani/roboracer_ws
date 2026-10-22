from enum import Enum
from dataclasses import dataclass
from typing import Optional
import numpy as np


class DrivingRegime(Enum):
    WALL_DAMPED = 'WALL_DAMPED'
    OBSTACLE_BYPASS = 'OBSTACLE_BYPASS'
    HIGH_SPEED_STRAIGHT = 'HIGH_SPEED_STRAIGHT'
    CORNER_APEX = 'CORNER_APEX'
    STANLEY_RECOVERY = 'STANLEY_RECOVERY'


@dataclass
class HybridControlOutput:
    steering: float
    speed: float
    active_regime: DrivingRegime
    cross_track_err: float
    heading_err: float
    bypass_active: bool
    predicted_horizon: Optional[np.ndarray] = None
