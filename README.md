# RoboRacer Workspace (`roboracer_ws`)

Autonomous racing software stack for F1TENTH / RoboRacer in ROS 2.

## Included Packages
- **`stanley_controller`**: High-performance Stanley path-tracking controller featuring:
  - Orthogonal cross-track error computation
  - Curvature feedforward steering ($\delta_{\text{ff}} = k_{\text{ff}} \arctan(L \kappa)$)
  - Speed-adaptive preview lookahead distance
  - Curvature-based braking horizon speed adaptation
  - RViz target and raceline marker visualization
  - Spielberg raceline & centerline waypoint profiles

## Requirements
- ROS 2 (Humble / Iron / Rolling)
- Python 3 with `numpy`, `scipy`
- `f1tenth_gym` & `f1tenth_gym_ros`

## Build
```bash
cd ~/roboracer_ws
colcon build --packages-select stanley_controller
source install/setup.bash
```

## Running the Stanley Controller
With the F1TENTH simulator running:
```bash
ros2 run stanley_controller stanley_controller_node.py \
  --ros-args -p waypoints_path:=/home/yeswanth/roboracer_ws/src/stanley_controller/waypoints/Spielberg_raceline.csv
```
