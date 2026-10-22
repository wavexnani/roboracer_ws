# RoboRacer Workspace (`roboracer_ws`)

High-performance autonomous racing software stack for F1TENTH / RoboRacer vehicles in ROS 2.

---

## Repository Structure

```
roboracer_ws/
├── benchmarks/                           # Benchmark harnesses, datasets, plots & reports
│   ├── scripts/                          # Evaluation and analysis harnesses
│   │   ├── benchmark_harness.py
│   │   ├── benchmark_hybrid_all_maps.py
│   │   ├── build_unified_benchmark.py
│   │   ├── generate_benchmark_plots.py
│   │   ├── eval_hybrid_roboracer.py
│   │   └── diagnose_mpc_jitter.py
│   ├── data/                             # Benchmark summary metrics & results JSONs
│   │   └── telemetry/                    # High-frequency run telemetry datasets
│   ├── plots/                            # Comparative performance dashboards and PNGs
│   └── reports/                          # Published controller benchmark reports
│       ├── CONTROLLER_BENCHMARK_REPORT.md
│       └── FIVE_CONTROLLER_BENCHMARK_REPORT.md
│
├── tests/                                # Consolidated unit & integration test suite
│   ├── test_continuous_projection.py
│   ├── test_delay_aware_mpc_unit.py
│   ├── test_exp11_yaw_mpc_unit.py
│   ├── test_mpc_rate_cost.py
│   └── test_steering_policy_unit.py
│
├── experiments/                          # Systematic experimental validation phases (20–25)
│
├── src/                                  # Modular ROS 2 packages
│   ├── controllers/                      # Autonomous vehicle control algorithms
│   │   ├── mpc_controller/               # Kinematic LTV-MPC with steering lattice policy
│   │   ├── stanley_controller/           # Non-linear Stanley tracking & speed profiler
│   │   ├── hybrid_controller/            # IFAC 2026 adaptive multi-regime supervisor
│   │   ├── pure_pursuit/                 # Geometric pure pursuit path tracker
│   │   ├── gap_follow/                   # Reactive Follow-the-Gap obstacle avoidance
│   │   ├── wall_follow/                  # Reactive PID wall tracker
│   │   └── safety_node/                  # Time-to-Collision (TTC) emergency braking
│   ├── simulation/                       # Simulation engines and track assets
│   │   ├── f1tenth_gym/                  # 2D vehicle dynamics simulation engine
│   │   ├── f1tenth_gym_ros/              # ROS 2 simulation bridge & RViz integration
│   │   └── f1tenth_racetracks/           # 24 official circuit maps and racelines
│   ├── hardware/                         # Embedded vehicle bridges and SITL
│   │   └── stm32_vcu_bridge/             # STM32 serial protocol, odometry & SITL
│   └── tutorials/                        # Coursework & introductory labs
│       └── lab1_pkg/                     # ROS 2 basics lab
│
├── pytest.ini                            # Configured test runner setup
├── pyrefly.toml                          # Type checker configuration
└── README.md
```

---

## Included Controllers

- **`mpc_controller`**: State-of-the-art Linear Time-Varying Model Predictive Controller (**LTV-MPC**) using OSQP:
  - Quadratic Program optimization over kinematic bicycle model ($N = 10$, $dt = 0.08\text{ s}$)
  - Integrated `HOLD_LATTICE16` discrete steering policy filter for physical actuator wear reduction
  - Automated 2D Signed Distance Field (SDF) obstacle clearance buffer ($\ge 0.42\text{ m}$)
  - Curvature-adaptive dynamic speed profiling ($a_{\text{lat}} \le 2.8\text{ m/s}^2$)
  - Dynamic map discovery across all 24 `f1tenth_racetracks` circuits
  - Real-time RViz predicted horizon visualizer
  - **100% Completion Rate across all 22 Raceline Maps (0 Crashes)** — see [`src/controllers/mpc_controller/README.md`](src/controllers/mpc_controller/README.md)
- **`stanley_controller`**: High-performance Stanley path-tracking controller:
  - Orthogonal cross-track error computation
  - Curvature feedforward steering ($\delta_{\text{ff}} = k_{\text{ff}} \cdot \arctan(L \cdot \kappa)$)
  - Speed-adaptive preview lookahead distance
  - Curvature-based braking horizon speed adaptation
  - RViz target and raceline marker visualization
  - Spielberg raceline & centerline waypoint profiles — see [`src/controllers/stanley_controller/README.md`](src/controllers/stanley_controller/README.md)
- **`hybrid_controller`**: Adaptive Multi-Regime Supervisor Controller combining geometric and predictive control.

---

## Requirements

- ROS 2 (Humble / Iron / Jazzy / Rolling)
- Python 3 with `numpy`, `scipy`, `osqp`, `Pillow`, `yaml`
- `f1tenth_gym` & `f1tenth_gym_ros`

---

## Build & Test

### Build ROS 2 Packages:
```bash
cd ~/roboracer_ws
colcon build --symlink-install
source install/setup.bash
```

To build only specific packages:
```bash
colcon build --symlink-install --packages-select mpc_controller stanley_controller hybrid_controller
```

### Run Unit Tests:
```bash
pytest
```
Or run individual verification tests:
```bash
python3 tests/test_continuous_projection.py
```

---

## Running the Controllers

### Model Predictive Controller (MPC) - One-Command Launch:
```bash
ros2 launch mpc_controller mpc.launch.py map:=Spielberg launch_sim:=true
```

### Stanley Controller:
```bash
ros2 launch stanley_controller stanley.launch.py map:=Spielberg launch_sim:=true
```
Or run the standalone node:
```bash
ros2 run stanley_controller stanley_controller_node.py \
  --ros-args -p waypoints_path:=/home/yeswanth/roboracer_ws/src/controllers/stanley_controller/waypoints/Spielberg_raceline.csv
```

---

## Benchmarks & Diagnostics

To regenerate 5-way comparative benchmark dashboards and analysis plots:
```bash
python3 benchmarks/scripts/build_unified_benchmark.py
python3 benchmarks/scripts/generate_benchmark_plots.py
```
Outputs will be saved in `benchmarks/plots/`.
