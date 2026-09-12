# Adaptive Multi-Regime Hybrid Controller (`hybrid_controller`)

A production-grade, regulation-compliant **Adaptive Multi-Regime Hybrid Controller (ARM-HC)** developed for the **ROBORACER International Autonomous Racing Grand Prix (IFAC 2026, BEXCO, Busan)** and general F1TENTH competition racing.

---

## 1. Overview & Motivation

In autonomous racing, no single static controller optimizes all track conditions:
- **Pure Pursuit** provides ultra-low latency geometric pursuit but understeers at racing speeds.
- **Classic Stanley** offers mathematically guaranteed global recovery from spins or large heading deviations, but suffers from high-frequency steering chatter and understeer in high-speed sweepers.
- **Baseline MPC** achieves rock-solid laser stability along straight walls ($\text{std}(\dot{\delta}) \approx 0.10\text{ rad/s}$), but fatally understeers in sharp hairpins due to excessive steering damping penalties.
- **Aggressive Curvature MPC** takes sharp corners at high speed, but can induce micro-jitter near straight walls due to high position gains responding to sensor discretization.

The **`hybrid_controller`** package unifies these architectures into a **Supervisory Multi-Regime Hybrid Engine** with **Continuous Gain Scheduling** and **Bumpless Transfer**:

```
+---------------------------------------------------------------------------------------------------------+
| REGIME                    | CONTROLLER ENGINE             | OBJECTIVE                                   |
+---------------------------+-------------------------------+---------------------------------------------+
| 1. WALL_DAMPED            | Damped MPC (w_ddelta = 3.2)   | Eradicate wall jiggling & steering chatter  |
| 2. OBSTACLE_BYPASS        | Reactive Frenet Lattice + MPC | Avoid Q2 & Finals obstacles (>=0.20m margin)|
| 3. HIGH_SPEED_STRAIGHT    | High-Speed MPC (v <= 7.5 m/s) | Maximize straight speed with acceleration   |
| 4. CORNER_APEX            | Pre-Braking MPC (w_delta=0.20)| Crisp 24° steering lock into hairpin apex   |
| 5. STANLEY_RECOVERY       | Non-linear Stanley Control    | Fail-safe return to raceline on spinout     |
+---------------------------------------------------------------------------------------------------------+
```

---

## 2. ROBORACER IFAC 2026 Regulations Compliance

| Regulation | Rule Specification | Hybrid Controller Implementation |
| :--- | :--- | :--- |
| **§ 3.2 Track Surface** | BEXCO Hall 5A: Hardened concrete with smooth urethane finish. **Lower friction ($\mu \approx 0.45–0.55$) than outdoor asphalt.** | **Urethane Friction Mode (`low_friction_mode:=true`)**: Caps lateral acceleration to $a_{\text{lat,max}} = 2.0\text{ m/s}^2$ ($v \le \sqrt{a_{\text{lat}}/\kappa}$), preventing snap oversteer / spinouts on urethane. |
| **§ 3.2 Track Dimensions** | Track area $\approx 8\text{m} \times 22\text{m}$, **minimum track width = $1.0\text{m}$**. | **Corridor Damping Mode**: Boosts steering rate penalty ($w_{\Delta \delta} = 3.2$) when walls $<0.65\text{m}$, reducing steering rate jitter to $\le 0.10\text{ rad/s}$. |
| **§ 5.2–5.4 Qualifying** | **Q1**: 3 clean laps (no obstacles).<br>**Q2**: 3 clean laps vs. 2 random static obstacles.<br>**Q3**: 1-minute Time Trial. | **Selectable Modes**: Launch with `mode:=Q1`, `mode:=Q2`, or `mode:=Q3` to activate optimal task behaviors. |
| **§ 5.5 Fully Autonomous** | **Zero human intervention** during qualifying. **Stopping $>5\text{s}$ fails FA.** | **Anti-Deadlock Autonomous Watchdog**: If forward speed $<0.15\text{ m/s}$ for $>1.5\text{s}$, automatically executes a reverse-and-realign maneuver in $<2.2\text{s}$, well before the 5.0s threshold. |
| **§ 5.7 & § 8 Obstacles** | In Q2, **any contact invalidates the lap**. Track clearance with obstacle $\ge 0.5\text{m}$. | **Real-Time Frenet Lattice Planner**: Ingests 2D LiDAR, clusters obstacle points, and modulates trajectory with smooth raised-cosine splines maintaining $\ge 0.22\text{m}$ vehicle safety margin. |
| **§ 7.1 Onboard Only** | All computation **100% onboard**. Code/path modification after obstacle placement is **prohibited**. | **Fully Online Reactive Avoidance**: Trajectory modification executes in real time ($25\text{Hz}$) onboard without manual path pre-generation. |

---

## 3. How to Build & Run

### 3.1 Build the Package
```bash
cd /home/yeswanth/roboracer_ws
colcon build --symlink-install --packages-select hybrid_controller
source install/setup.bash
```

### 3.2 Launch with Simulator (One-Command Launch)
```bash
# Launch on Spielberg in Q1 mode
ros2 launch hybrid_controller hybrid.launch.py map:=Spielberg mode:=Q1 launch_sim:=true

# Launch on BEXCO-style indoor corridors (Levine) with low friction urethane mode
ros2 launch hybrid_controller hybrid.launch.py map:=Levine mode:=Q1 low_friction_mode:=true launch_sim:=true

# Launch in Q2 mode (with real-time LiDAR obstacle avoidance enabled)
ros2 launch hybrid_controller hybrid.launch.py map:=Spielberg mode:=Q2 launch_sim:=true

# Launch on Austin, Monza, Spa, or Catalunya
ros2 launch hybrid_controller hybrid.launch.py map:=Austin mode:=Q3 launch_sim:=true
ros2 launch hybrid_controller hybrid.launch.py map:=Spa mode:=Q3 launch_sim:=true
```

---

## 4. Operational Regimes & Diagnostics

The controller actively broadcasts 3D diagnostic markers to RViz:
- **Floating Status Text (`/vis/active_regime`)**: Displays the active regime above the car:
  - `[WALL_DAMPED]` (Orange): High damping active, eliminating wall chatter.
  - `[OBSTACLE_BYPASS]` (Magenta): Local Frenet spline active around obstacle.
  - `[HIGH_SPEED_STRAIGHT]` (Bright Green): Full straightaway acceleration up to $7.50\text{ m/s}$.
  - `[CORNER_APEX]` (Blue): Pre-braking pass engaged, full $24^\circ$ steering lock allowed.
  - `[STANLEY_RECOVERY]` (Red): Global re-alignment fallback.
- **Predicted Horizon (`/vis/predicted_horizon`)**: High-resolution ribbon of the open-loop MPC trajectory.
