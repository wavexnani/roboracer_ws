# Stanley Controller (`stanley_controller`)

A high-performance, production-grade path tracking ROS 2 node for F1TENTH / RoboRacer autonomous vehicles. Implements the non-linear Stanley steering control law augmented with **curvature feedforward**, **speed-adaptive lookahead preview**, **braking-horizon corner velocity profiling**, and **universal multi-map support** across all 24 tracks in `f1tenth_racetracks`.

---

## Table of Contents
1. [Overview](#overview)
2. [Control Architecture & Mathematical Formulation](#control-architecture--mathematical-formulation)
   - [Vehicle Kinematic Model](#1-vehicle-kinematic-model)
   - [Speed-Adaptive Reference Point](#2-speed-adaptive-reference-point)
   - [Continuous Orthogonal Cross-Track Error](#3-continuous-orthogonal-cross-track-error)
   - [Heading Error Normalization](#4-heading-error-normalization)
   - [Curvature Feedforward Steering](#5-curvature-feedforward-steering)
   - [Complete Steering Control Law](#6-complete-steering-control-law)
3. [Behavior in Sharp Turns & U-Bends (Hairpins)](#behavior-in-sharp-turns--u-bends-hairpins)
   - [Braking Horizon Anticipation](#1-braking-horizon-anticipation)
   - [Dynamic Steering Lock Speed Attenuation](#2-dynamic-steering-lock-speed-attenuation)
   - [Heading Divergence Safety Guard](#3-heading-divergence-safety-guard)
   - [Adaptive Corner-Cutting Mitigation](#4-adaptive-corner-cutting-mitigation)
   - [Corner Exit Stability](#5-corner-exit-stability)
4. [Controller States & Transitions Across Curvatures](#controller-states--transitions-across-curvatures)
5. [Universal Multi-Map Dynamic Loading (`f1tenth_racetracks`)](#universal-multi-map-dynamic-loading-f1tenth_racetracks)
   - [How It Works](#how-it-works)
   - [Simulator Synchronization (`sim.yaml` & `/initialpose`)](#simulator-synchronization-simyaml--initialpose)
   - [Dual-Format CSV Ingestion](#dual-format-csv-ingestion)
   - [Supported Racetracks (24 Tracks)](#supported-racetracks-24-tracks)
6. [Scalable Architecture for Future Controllers](#scalable-architecture-for-future-controllers)
7. [ROS 2 Parameters Reference](#ros-2-parameters-reference)
8. [Usage Guide & CLI Commands](#usage-guide--cli-commands)

---

## Overview

The `stanley_controller` package computes real-time Ackermann steering commands (`/drive`) from vehicle odometry (`/ego_racecar/odom`) to track optimal racelines or centerlines at racing speeds.

Classic Stanley controllers suffer from steady-state understeer (outward drift) on high-speed curves and cannot anticipate corners before the front axle enters them. This package resolves these limitations by combining:
- **Exact orthogonal cross-track projection** along continuous path tangents.
- **Velocity-adaptive preview lookahead** that overcomes actuator lag without clipping inside apex curbs.
- **Curvature feedforward ($\delta_{\text{ff}}$)** to preemptively steer into turns.
- **Kinematic braking lookahead horizon** that decelerates before corner entry based on maximum allowable tire lateral acceleration.
- **Plug-and-play multi-map selector** allowing any track from `f1tenth_racetracks` to be selected directly from the command line.

---

## Control Architecture & Mathematical Formulation

```
+-------------------+      Odometry       +------------------------------------+
|  f1tenth_gym_ros  | ------------------> |       stanley_controller_node      |
|  (or real car)    | <------------------ |                                    |
+-------------------+     /drive cmd      +------------------------------------+
                                             |              |
                                             v              v
                                        TrackManager    RViz Visualizer
                                     (24 Racetracks)   (Target & Raceline)
```

### 1. Vehicle Kinematic Model
The controller assumes a single-track bicycle kinematic model with wheelbase $L = 0.33\text{ m}$:
$$\dot{x} = v \cos(\psi + \beta), \quad \dot{y} = v \sin(\psi + \beta), \quad \dot{\psi} = \frac{v}{L} \tan(\delta) \cos(\beta)$$
where $v$ is vehicle longitudinal speed, $\psi$ is vehicle yaw heading, $\delta$ is steering angle, and $\beta = \arctan\left(\frac{1}{2}\tan\delta\right)$ is the vehicle slip angle.

### 2. Speed-Adaptive Reference Point
In standard Stanley, the reference point is placed at the front axle ($d_{\text{ref}} = L$). At high speeds ($6–8\text{ m/s}$), steering actuator delay causes the car to turn too late. Conversely, a large fixed preview distance causes the car's body to cut inside corners during low-speed hairpins.

This node uses a **speed-adaptive preview distance**:
$$L_{\text{preview}} = \text{lookahead\_dist} + k_{\text{lookahead}} \cdot v$$
$$d_{\text{eff}} = L + L_{\text{preview}}$$
$$\mathbf{p}_{\text{ref}} = \begin{bmatrix} x + d_{\text{eff}} \cos\psi \\ y + d_{\text{eff}} \sin\psi \end{bmatrix}$$
- **At low speeds** ($v \approx 2\text{ m/s}$): $L_{\text{preview}} \approx 0.19\text{ m}$ (stays tight to front axle, preventing apex curb collisions).
- **At high speeds** ($v \approx 7\text{ m/s}$): $L_{\text{preview}} \approx 0.29\text{ m}$ (anticipates turn entry, overcoming actuator delay).

### 3. Continuous Orthogonal Cross-Track Error
The controller locates the nearest target waypoint $\mathbf{w}_i = [x_i, y_i]^T$ within a progression window around the previous index.

Let $\mathbf{t} = [\cos\psi_{\text{ref}}, \sin\psi_{\text{ref}}]^T$ be the unit tangent vector of the path at waypoint $i$, and $\mathbf{e} = \mathbf{p}_{\text{ref}} - \mathbf{w}_i$ be the error vector from the waypoint to the vehicle reference point.

The continuous signed orthogonal lateral error is computed via the 2D cross product:
$$e_{\text{lat}} = \mathbf{t} \times \mathbf{e} = t_x e_y - t_y e_x$$
By convention:
- $e_{\text{lat}} > 0$: Vehicle reference point is **to the left** of the path.
- $e_{\text{lat}} < 0$: Vehicle reference point is **to the right** of the path.

The Stanley cross-track error is defined as:
$$e_{\text{ct}} = -e_{\text{lat}}$$
The non-linear cross-track steering correction is:
$$\delta_{\text{ct}} = \arctan\left(\frac{k \cdot e_{\text{ct}}}{v + k_{\text{soft}}}\right)$$
where $k$ is the cross-track gain (`gain_k: 2.3`) and $k_{\text{soft}}$ is the softening velocity constant (`software_k: 0.5 m/s`) to prevent singularity at zero speed.

### 4. Heading Error Normalization
The heading error is the angular difference between path tangent and vehicle yaw:
$$\theta_e = \text{wrap}_{[-\pi, \pi]}(\psi_{\text{ref}} - \psi)$$
$$\text{wrap}_{[-\pi, \pi]}(\alpha) = (\alpha + \pi) \pmod{2\pi} - \pi$$

### 5. Curvature Feedforward Steering
Classic Stanley lacks feedforward control. To sustain a curve of radius $R = 1/\kappa$, the vehicle geometrically requires steering angle $\delta = \arctan(L \cdot \kappa)$. Without feedforward, Stanley must drift outward by $e_{\text{drift}} \approx \frac{v + k_{\text{soft}}}{k} L \kappa \approx 0.30\text{ m}$ before the cross-track error produces enough steering.

The controller injects an anticipatory **curvature feedforward term**:
$$\delta_{\text{ff}} = k_{\text{ff}} \cdot \arctan(L \cdot \kappa)$$
where $k_{\text{ff}}$ is the feedforward gain (`gain_ff: 0.12`).

### 6. Complete Steering Control Law
The total commanded steering angle is:
$$\delta = \text{clip}\left(\theta_e + \arctan\left(\frac{k \cdot e_{\text{ct}}}{v + k_{\text{soft}}}\right) + k_{\text{ff}} \cdot \arctan(L \cdot \kappa), \; -\delta_{\text{max}}, \; \delta_{\text{max}}\right)$$
with $\delta_{\text{max}} = 0.4189\text{ rad}$ ($\approx 24^\circ$).

---

## Behavior in Sharp Turns & U-Bends (Hairpins)

When negotiating sharp U-shaped turns (e.g., Turn 2 and the hairpins at Spielberg, Monaco, or Austin), the vehicle undergoes a synchronized 5-stage adaptation:

```
[Straightaway: 8 m/s]
        │
        ▼
[1. Horizon Curvature Detection] ---> Looks ahead: H = (v^2)/(2 a_brake) + 1.0 m
        │
        ▼
[2. Kinematic Pre-Braking] ---------> v_corner = sqrt(a_lat_max / kappa_max)
        │
        ▼
[3. Apex Steering Modulation] ------> Reduces speed up to 25% under lock (|delta| > 0.5 delta_max)
        │
        ▼
[4. Heading Divergence Guard] ------> Damps speed 15% if |theta_e| > 0.35 rad (20 deg)
        │
        ▼
[5. Stabilized Exit Acceleration] --> Smooth ramp-up back to straightaway speed
```

### 1. Braking Horizon Anticipation
The controller dynamically computes a braking distance horizon based on the vehicle's current velocity and braking deceleration capability $a_{\text{brake}} = 4.0\text{ m/s}^2$:
$$H = \frac{v^2}{2 \cdot a_{\text{brake}}} + d_{\text{min}}$$
It inspects all waypoints within distance $H$ ahead along the closed loop to find the peak upcoming curvature:
$$\kappa_{\text{max}} = \max_{s \in [s_{\text{cur}}, s_{\text{cur}} + H]} |\kappa(s)|$$
The maximum safe cornering speed is calculated from the lateral tire acceleration limit $a_{\text{lat,max}} = 4.0\text{ m/s}^2$:
$$v_{\text{corner}} = \sqrt{\frac{a_{\text{lat,max}}}{\kappa_{\text{max}}}}$$
The vehicle begins braking on the straightaway **before** entering the U-turn, arriving at the turn entry at the exact physics-limited cornering speed.

### 2. Dynamic Steering Lock Speed Attenuation
Under hard cornering where the steering angle exceeds 50% of maximum lock ($\frac{|\delta|}{\delta_{\text{max}}} > 0.5$):
$$v_{\text{target}} \leftarrow v_{\text{target}} \times \left(1.0 - 0.25 \cdot \frac{\frac{|\delta|}{\delta_{\text{max}}} - 0.5}{0.5}\right)$$
This smoothly tapers speed by up to **25%** under full lock, preventing front tire friction saturation and understeer push into outside barriers.

### 3. Heading Divergence Safety Guard
If the vehicle's heading diverges from the path tangent by more than $0.35\text{ rad}$ ($20^\circ$):
$$v_{\text{target}} \leftarrow v_{\text{target}} \times 0.85$$
This prevents power oversteer or spinning when re-aligning from aggressive hairpin transitions.

### 4. Adaptive Corner-Cutting Mitigation
In tight U-turns, off-tracking naturally causes the rear wheels and chassis to cut inside the curve by $\Delta r \approx \frac{L^2}{2 R}$.
Because lookahead is speed-adaptive ($L_{\text{preview}} = 0.15 + 0.02 v$), slowing down to $2.5–3.5\text{ m/s}$ in the hairpin shrinks the preview distance from $0.30\text{ m}$ down to $0.20\text{ m}$. This keeps the reference point close to the physical front axle, preventing the car from clipping inside apex walls.

### 5. Corner Exit Stability
The curvature search window encompasses both the entry and active body of the corner. The vehicle maintains its cornering speed until the turn curvature subsides, preventing premature full-throttle acceleration while still loaded in the turn.

---

## Controller States & Transitions Across Curvatures

The node dynamically transitions between 4 operational profiles as curvature changes:

| Track Segment | Curvature Range | Controller Behavior | Speed Profile |
| :--- | :--- | :--- | :--- |
| **Straightaways** | $|\kappa| < 0.02\text{ rad/m}$ | $\delta_{\text{ff}} \approx 0$; lookahead extends to $0.30+\text{m}$; cross-track term dampens high-speed jitter | Full speed ($v = v_{\text{raceline}} \cdot \text{scale} \le 8.0\text{ m/s}$) |
| **Corner Approach** | Upcoming $|\kappa| > 0.05\text{ rad/m}$ | Horizon scans $H\text{ meters}$ forward; triggers longitudinal deceleration | Decelerating towards $v = \sqrt{a_{\text{lat,max}} / \kappa_{\text{max}}}$ |
| **Sharp U-Turn / Apex** | $|\kappa| \ge 0.15\text{ rad/m}$ | Feedforward $\delta_{\text{ff}}$ commands geometric steering; preview shortens to $0.20\text{m}$; steering lock speed attenuation active | Physics-limited corner speed ($2.5–4.5\text{ m/s}$) |
| **Corner Exit** | $|\kappa| \to 0\text{ rad/m}$ | Feedforward decays; steering straightens; velocity smoothly ramps back to target straight speed | Accelerating back to top speed |

---

## Universal Multi-Map Dynamic Loading (`f1tenth_racetracks`)

### How It Works
The controller integrates directly with the `f1tenth_racetracks` repository. You can select any map by passing its name as a command-line argument:

```bash
ros2 run stanley_controller stanley_controller_node.py Austin
ros2 run stanley_controller stanley_controller_node.py Monza
ros2 run stanley_controller stanley_controller_node.py BrandsHatch
```

### Simulator Synchronization (`sim.yaml` & `/initialpose`)
When a map name is passed:
1. **Fuzzy Name Resolution**: Resolves case-insensitively (`austin` $\to$ `Austin`, `spielberg` $\to$ `Spielberg`, `mexico` $\to$ `Mexico City`).
2. **Auto-Map Discovery**: Discovers `<TrackName>_map.yaml`, `<TrackName>_map.png`, and waypoints in `src/f1tenth_racetracks/<TrackName>/`.
3. **`sim.yaml` Synchronization**: Updates `map_path`, `sx`, `sy`, and `stheta` in `f1tenth_gym_ros/config/sim.yaml` with the track's starting waypoint.
4. **`/initialpose` Teleportation**: If the simulator is already active, publishes `geometry_msgs/PoseWithCovarianceStamped` to `/initialpose` to instantly reposition the vehicle to the starting line on that track.

### Dual-Format CSV Ingestion
The `TrackManager` automatically ingests both standard formats found in autonomous racing:
- **7-Column Raceline** (`;` separated): `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`.
- **4-Column Centerline** (`,` separated): `x_m, y_m, w_tr_right_m, w_tr_left_m`.
  - Automatically calculates cumulative arc length $s$, numerical tangent gradient heading $\psi$, and curvature $\kappa = \Delta \psi / \Delta s$.
  - Generates safe cornering velocity profiles automatically.

### Supported Racetracks (24 Tracks)
All 24 racetracks in `src/f1tenth_racetracks` are supported:

| Track Name | Profile Available | Start Pose $(x_0, y_0, \psi_0)$ | Closed Length |
| :--- | :--- | :--- | :--- |
| **Austin** | Raceline + Centerline | $(-0.41, -0.69, 322.9^\circ)$ | $406.5\text{ m}$ |
| **BrandsHatch** | Raceline + Centerline | $(-0.52, 0.65, 24.1^\circ)$ | $350.9\text{ m}$ |
| **Budapest** | Raceline + Centerline | $(-0.71, 0.35, 153.9^\circ)$ | $364.7\text{ m}$ |
| **Catalunya** | Raceline + Centerline | $(-0.67, 0.08, 93.3^\circ)$ | $412.3\text{ m}$ |
| **Hockenheim** | Raceline + Centerline | $(-0.69, -0.06, 81.3^\circ)$ | $378.1\text{ m}$ |
| **IMS** | Raceline + Centerline | $(-0.65, -0.19, 73.7^\circ)$ | $420.2\text{ m}$ |
| **Levine** | Raceline + Centerline | $(-0.21, -0.42, 296.6^\circ)$ | $156.4\text{ m}$ |
| **Melbourne** | Raceline + Centerline | $(-0.66, 0.17, 104.9^\circ)$ | $428.6\text{ m}$ |
| **Mexico City** | Raceline + Centerline | $(-0.58, 0.14, 103.5^\circ)$ | $372.5\text{ m}$ |
| **Montreal** | Centerline (auto-fallback) | $(0.00, 0.00, -77.2^\circ)$ | $285.0\text{ m}$ |
| **Monza** | Raceline + Centerline | $(-0.66, 0.14, 86.1^\circ)$ | $439.2\text{ m}$ |
| **MoscowRaceway** | Raceline + Centerline | $(-0.44, 0.54, 30.7^\circ)$ | $385.1\text{ m}$ |
| **Nuerburgring** | Raceline + Centerline | $(-0.69, -0.07, 84.1^\circ)$ | $410.8\text{ m}$ |
| **Oschersleben** | Raceline + Centerline | $(-0.62, 0.34, 118.7^\circ)$ | $341.2\text{ m}$ |
| **Sakhir** | Raceline + Centerline | $(-0.68, 0.07, 95.8^\circ)$ | $441.7\text{ m}$ |
| **SaoPaulo** | Raceline + Centerline | $(-0.68, 0.06, 94.9^\circ)$ | $368.9\text{ m}$ |
| **Sepang** | Raceline + Centerline | $(-0.59, 0.44, 36.8^\circ)$ | $436.5\text{ m}$ |
| **Shanghai** | Centerline (auto-fallback) | $(0.00, 0.00, -78.4^\circ)$ | $362.8\text{ m}$ |
| **Silverstone** | Raceline + Centerline | $(-0.69, -0.04, 86.7^\circ)$ | $440.0\text{ m}$ |
| **Sochi** | Raceline + Centerline | $(-0.65, 0.22, 108.6^\circ)$ | $442.1\text{ m}$ |
| **Spa** | Raceline + Centerline | $(-0.67, 0.13, 101.0^\circ)$ | $542.4\text{ m}$ |
| **Spielberg** | Raceline + Centerline | $(-0.04, -0.85, 195.0^\circ)$ | $338.1\text{ m}$ |
| **YasMarina** | Raceline + Centerline | $(-0.67, 0.16, 103.4^\circ)$ | $437.0\text{ m}$ |
| **Zandvoort** | Raceline + Centerline | $(-0.68, 0.12, 99.8^\circ)$ | $382.6\text{ m}$ |

---

## Scalable Architecture for Future Controllers

The track management logic is encapsulated in the independent Python module `stanley_controller.track_manager.TrackManager`.

Any future controller package (e.g. `pure_pursuit`, `mpc_controller`, `lqr_controller`) can reuse this module directly:

```python
from stanley_controller.track_manager import TrackManager

# Load track with 1 line of code
track = TrackManager.load_track(track_name='Monza', waypoint_type='raceline')

# Access all arrays directly
wpts = track.waypoints       # (N, 2) [x, y]
headings = track.headings   # (N,) yaw angles
kappa = track.kappa         # (N,) curvatures
speeds = track.target_speeds # (N,) velocities

# Synchronize simulator config
TrackManager.sync_sim_yaml(track)
```

---

## ROS 2 Parameters Reference

| Parameter Name | Type | Default | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `map_name` | `string` | `'Spielberg'` | — | Target racetrack name from `f1tenth_racetracks` (case-insensitive) |
| `waypoint_type` | `string` | `'raceline'` | — | Profile type: `'raceline'` or `'centerline'` |
| `waypoints_path` | `string` | `''` | — | Optional direct path to override automated track discovery |
| `sync_sim_map` | `bool` | `true` | — | Automatically synchronizes `f1tenth_gym_ros/config/sim.yaml` |
| `publish_initial_pose` | `bool` | `true` | — | Publishes initial pose to `/initialpose` on startup |
| `initialpose_topic` | `string` | `'/initialpose'` | — | Topic for simulator pose reset |
| `odom_topic` | `string` | `'/ego_racecar/odom'` | — | Input vehicle odometry topic |
| `drive_topic` | `string` | `'/drive'` | — | Output Ackermann command topic |
| `gain_k` | `double` | `2.3` | $1/\text{s}$ | Stanley cross-track error gain |
| `software_k` | `double` | `0.5` | $\text{m/s}$ | Low-speed softening constant |
| `gain_ff` | `double` | `0.12` | — | Curvature feedforward gain ($k_{\text{ff}}$) |
| `wheelbase` | `double` | `0.33` | $\text{m}$ | Vehicle wheelbase distance ($L$) |
| `lookahead_dist` | `double` | `0.15` | $\text{m}$ | Base preview distance ahead of front axle ($d_0$) |
| `lookahead_gain` | `double` | `0.02` | $\text{s}$ | Velocity-proportional preview gain ($k_{\text{lh}}$) |
| `max_steering_angle` | `double` | `0.4189` | $\text{rad}$ | Maximum steering lock limit ($\sim 24^\circ$) |
| `speed_scale` | `double` | `0.68` | — | Global target speed scaling multiplier |
| `min_speed` | `double` | `1.0` | $\text{m/s}$ | Minimum commanded speed clamp |
| `max_speed` | `double` | `8.0` | $\text{m/s}$ | Maximum commanded speed clamp |
| `enable_curvature_speed` | `bool` | `true` | — | Enables lookahead curvature-based speed limiting |
| `lat_accel_max` | `double` | `4.0` | $\text{m/s}^2$ | Maximum allowable lateral tire acceleration |
| `brake_decel` | `double` | `4.0` | $\text{m/s}^2$ | Braking deceleration for sizing the lookahead horizon |
| `min_lookahead_horizon` | `double` | `1.0` | $\text{m}$ | Minimum forward distance for curvature scanning |
| `visualize` | `bool` | `true` | — | Enables RViz marker publication |
| `target_marker_topic` | `string` | `'/vis/stanley_target'` | — | RViz sphere marker topic for target waypoint |
| `path_marker_topic` | `string` | `'/vis/stanley_path'` | — | RViz line strip marker topic for active raceline |

---

## Usage Guide & CLI Commands

### 1. Build the Package
```bash
cd ~/roboracer_ws
colcon build --packages-select stanley_controller
source install/setup.bash
```

### 2. Run with Map Name (Positional CLI Argument)
Pass any track name directly after the node executable:
```bash
# Run on Austin
ros2 run stanley_controller stanley_controller_node.py Austin

# Run on Monza
ros2 run stanley_controller stanley_controller_node.py Monza

# Run on Brands Hatch
ros2 run stanley_controller stanley_controller_node.py BrandsHatch

# Run on Spielberg (default)
ros2 run stanley_controller stanley_controller_node.py Spielberg
```

### 3. Run with Parameter Overrides
```bash
# Select centerline profile on Austin
ros2 run stanley_controller stanley_controller_node.py Austin --ros-args -p waypoint_type:=centerline

# Run with custom speed scale
ros2 run stanley_controller stanley_controller_node.py Monza --ros-args -p speed_scale:=0.75
```

### 4. Launch via ROS 2 Launch File
Launch the controller with arguments:
```bash
# Launch on Austin
ros2 launch stanley_controller stanley.launch.py map:=Austin

# Launch both simulator and controller together on Spielberg
ros2 launch stanley_controller stanley.launch.py map:=Spielberg launch_sim:=true
```
