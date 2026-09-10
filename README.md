# RoboRacer Workspace (`roboracer_ws`)

Autonomous racing software stack for F1TENTH / RoboRacer in ROS 2.

## Included Packages
- **`mpc_controller`**: State-of-the-art Linear Time-Varying Model Predictive Controller (**LTV-MPC**) using OSQP:
  - Quadratic Program optimization over kinematic bicycle model ($N=10$, $dt=0.08\text{s}$)
  - Automated 2D Signed Distance Field (SDF) obstacle clearance buffer ($\ge 0.42\text{m}$)
  - Curvature-adaptive dynamic speed profiling ($a_{\text{lat}} \le 2.8\text{ m/s}^2$)
  - Dynamic map discovery across all 24 `f1tenth_racetracks` circuits
  - Real-time RViz predicted horizon visualizer
  - **100% Completion Rate across all 22 Raceline Maps (0 Crashes)** — see [`src/mpc_controller/README.md`](file:///home/yeswanth/roboracer_ws/src/mpc_controller/README.md) for full benchmark logs
- **`stanley_controller`**: High-performance Stanley path-tracking controller featuring:
  - Orthogonal cross-track error computation
  - Curvature feedforward steering ($\delta_{\text{ff}} = k_{\text{ff}} \arctan(L \kappa)$)
  - Speed-adaptive preview lookahead distance
  - Curvature-based braking horizon speed adaptation
  - RViz target and raceline marker visualization
  - Spielberg raceline & centerline waypoint profiles

## Requirements
- ROS 2 (Humble / Iron / Jazzy / Rolling)
- Python 3 with `numpy`, `scipy`, `osqp`, `Pillow`, `yaml`
- `f1tenth_gym` & `f1tenth_gym_ros`

## Build
```bash
cd ~/roboracer_ws
colcon build --symlink-install --packages-select stanley_controller mpc_controller
source install/setup.bash
```

## Running the Controllers

### Model Predictive Controller (MPC) - One-Command Launch:
```bash
ros2 launch mpc_controller mpc.launch.py map:=Spielberg launch_sim:=true
```

### Stanley Controller:
With the F1TENTH simulator running:
```bash
ros2 run stanley_controller stanley_controller_node.py \
  --ros-args -p waypoints_path:=/home/yeswanth/roboracer_ws/src/stanley_controller/waypoints/Spielberg_raceline.csv
```

