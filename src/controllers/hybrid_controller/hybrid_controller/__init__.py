"""
Adaptive Multi-Regime Hybrid Controller for F1TENTH / ROBORACER (IFAC 2026).
Integrates:
- Pure Pursuit Path Tracker
- Non-linear Stanley Controller with Curvature Feedforward
- Gain-Scheduled Linear Time-Varying Model Predictive Controller (LTV-MPC)
- Real-Time 2D LiDAR Frenet Local Obstacle Avoidance Lattice Planner
- Anti-Deadlock Autonomous Watchdog (<5s rule)
- Urethane Concrete Low-Friction Physics Adaptation
"""

from .track_manager import TrackManager, TrackInfo
from .pure_pursuit import PurePursuitController
from .stanley_controller import StanleyController
from .mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
from .obstacle_planner import ObstaclePlanner, ObstacleCluster, BypassPath
from .hybrid_supervisor import HybridSupervisor, DrivingRegime, HybridControlOutput

__all__ = [
    'TrackManager',
    'TrackInfo',
    'PurePursuitController',
    'StanleyController',
    'MPCOptimizer',
    'MPCConfig',
    'MPCResult',
    'ObstaclePlanner',
    'ObstacleCluster',
    'BypassPath',
    'HybridSupervisor',
    'DrivingRegime',
    'HybridControlOutput'
]
