# Model Predictive Controller (MPC) for F1TENTH / RoboRacer

A high-performance **Linear Time-Varying Model Predictive Controller (LTV-MPC)** for autonomous racing on the F1TENTH platform. The controller solves a real-time Quadratic Program (QP) via **OSQP** at each time step to calculate optimal steering and acceleration commands while respecting physical vehicle constraints, steering limits, and track boundaries.

---

## 1. Control Architecture & Mathematical Formulation

### 1.1 Kinematic Bicycle Model
The vehicle state vector is defined as $\mathbf{x} = [x, y, \psi, v]^T \in \mathbb{R}^4$:
- $x, y$: Vehicle coordinates in the world/map frame [m]
- $\psi$: Heading (yaw angle) [rad]
- $v$: Longitudinal velocity [m/s]

The control input vector is $\mathbf{u} = [a, \delta]^T \in \mathbb{R}^2$:
- $a$: Longitudinal acceleration command [m/s²]
- $\delta$: Front wheel steering angle [rad]

Continuous kinematics:
$$\dot{x} = v \cos\psi$$
$$\dot{y} = v \sin\psi$$
$$\dot{\psi} = \frac{v}{L} \tan\delta$$
$$\dot{v} = a$$
where $L = 0.33\text{ m}$ is the wheelbase.

### 1.2 Linear Time-Varying (LTV) Discretization
Around an operating reference point $(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)$ with sampling time $dt = 0.08\text{ s}$, the non-linear dynamics are linearized:
$$\mathbf{x}_{k+1} = A_k \mathbf{x}_k + B_k \mathbf{u}_k + \mathbf{c}_k$$

Where the Jacobian matrices are:
$$A_k = \begin{bmatrix}
1 & 0 & -\bar{v}_k \sin\bar{\psi}_k \cdot dt & \cos\bar{\psi}_k \cdot dt \\
0 & 1 & \bar{v}_k \cos\bar{\psi}_k \cdot dt & \sin\bar{\psi}_k \cdot dt \\
0 & 0 & 1 & \frac{\tan\bar{\delta}_k}{L} \cdot dt \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
B_k = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & \frac{\bar{v}_k}{L \cos^2\bar{\delta}_k} \cdot dt \\
dt & 0
\end{bmatrix}$$

$$\mathbf{c}_k = \mathbf{f}(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k) \cdot dt - A_k \bar{\mathbf{x}}_k - B_k \bar{\mathbf{u}}_k + \bar{\mathbf{x}}_k$$

### 1.3 Quadratic Cost Function
Over prediction horizon $N = 10$ ($0.8\text{ s}$ preview horizon), the QP minimizes:
$$J = \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_k^{\text{ref}}\|_Q^2 + \|\mathbf{u}_k\|_R^2 + \|\mathbf{u}_k - \mathbf{u}_{k-1}\|_{R_\Delta}^2 \right) + \|\mathbf{x}_N - \mathbf{x}_N^{\text{ref}}\|_{Q_N}^2$$

- **State Error Weight $Q = \text{diag}(q_x, q_y, q_\psi, q_v)$**:
  - $q_x = 2.5, q_y = 2.5$: Tight cross-track tracking onto optimal racelines.
  - $q_\psi = 1.8$: Alignment with upcoming track tangent heading.
  - $q_v = 0.5$: Adherence to optimal speed profiles.
- **Control Effort Weight $R = \text{diag}(r_a, r_\delta)$**:
  - $r_\delta = 0.8$: Steering effort penalty to avoid aggressive tire slip.
  - $r_a = 0.1$: Smooth acceleration demands.
- **Control Rate Weight $R_\Delta = \text{diag}(r_{\Delta a}, r_{\Delta \delta})$**:
  - $r_{\Delta \delta} = 2.5$: Slew-rate penalty preventing high-frequency steering chatter.
- **Terminal Weight $Q_N = 2.0 \cdot Q$**:
  - Stabilizes the open-loop horizon tail.

### 1.4 Physical Constraints
- **Steering angle**: $|\delta_k| \le 0.4189\text{ rad}$ ($24^\circ$).
- **Steering rate**: $|\delta_{k+1} - \delta_k| \le 0.18\text{ rad/step}$.
- **Acceleration**: $-7.0\text{ m/s}^2 \le a_k \le 4.0\text{ m/s}^2$.
- **Velocity**: $0.0 \le v_k \le 12.0\text{ m/s}$.

---

## 2. Real-Time Horizon Visualization in RViz

The node actively publishes visual markers to RViz:
1. **Cyan Line Strip (`/mpc_reference_path`)**: The full reference track waypoints.
2. **Red Sphere (`/mpc_target_point`)**: The immediate lookahead target waypoint.
3. **Bright Green Ribbon (`/mpc_predicted_horizon`)**: The **predicted MPC trajectory** over the next $N$ steps $[x_0, \dots, x_N]$. You can visually see the controller calculating and shaping corners ahead of the car!

---

## 3. How to Run

### 3.1 Build the Package
```bash
cd /sim_ws  # or ~/roboracer_ws
colcon build --symlink-install --packages-select mpc_controller
source install/setup.bash
```

### 3.2 Recommended: One-Command Launch (Simulator + Controller)
Launches the F1TENTH Gym simulator, RViz with auto-focus follow camera, and the MPC controller together:
```bash
# Launch on Levine
ros2 launch mpc_controller mpc.launch.py map:=Levine launch_sim:=true

# Launch on Austin
ros2 launch mpc_controller mpc.launch.py map:=Austin launch_sim:=true

# Launch on Spielberg
ros2 launch mpc_controller mpc.launch.py map:=Spielberg launch_sim:=true

# Launch on Monza with centerline trajectory
ros2 launch mpc_controller mpc.launch.py map:=Monza waypoint_type:=centerline launch_sim:=true
```

### 3.3 Two-Terminal Workflow

**Terminal 1 (Simulator):**
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=Levine
```

**Terminal 2 (MPC Controller Node):**
```bash
ros2 run mpc_controller mpc_controller_node.py Levine
# or with alias:
rr mpc_controller mpc_controller_node.py Levine
```

---

## 4. Parameter Reference

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `map_name` | `string` | `'Spielberg'` | Target racetrack name from `f1tenth_racetracks` |
| `waypoint_type` | `string` | `'raceline'` | Trajectory profile type (`raceline` or `centerline`) |
| `speed_scale` | `float` | `0.68` | Speed scaling factor ($0.1$ to $1.0$) |
| `horizon` | `int` | `10` | MPC prediction horizon steps $N$ |
| `dt` | `float` | `0.08` | MPC discretization time step [s] |
| `w_x`, `w_y` | `float` | `2.5` | Planar position tracking weights |
| `w_psi` | `float` | `1.8` | Heading orientation tracking weight |
| `w_v` | `float` | `0.5` | Longitudinal speed tracking weight |
| `w_delta` | `float` | `0.8` | Steering effort penalty |
| `w_ddelta` | `float` | `2.5` | Steering slew-rate smoothness penalty |
| `autofocus` | `bool` | `true` | Auto-focus follow camera in RViz |
