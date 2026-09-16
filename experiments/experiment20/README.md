# Experiment 20 — Exact 25 Hz Lifted Relay Model and Reachability-Based Interface Analysis

## Scientific Milestone Overview

This milestone freezes the analytical and experimental findings of **Experiment 20** as a formal reproducibility baseline.  
**Important Note:** This is a scientific research milestone. No production MPC, controller, or simulator code changes were introduced by this milestone.

---

## 1. System Model: Exact 25 Hz Lifted Actuator Map

The official `f110_gym` steering actuator operates at $100\text{ Hz}$ ($\Delta t_{\text{sim}} = 0.010\text{ s}$) with a $20\text{ ms}$ FIFO queue and an un-damped discontinuous relay with slew limit $sv_{\text{max}} = 3.2\text{ rad/s}$ and deadband threshold $10^{-4}\text{ rad}$.

The exact discrete-time state at control cycle $k$ ($t_k = k \cdot 0.040\text{ s}$) is:

$$X[k] = \begin{bmatrix} \delta_{\text{actual}}[k] \\ b_0[k] \\ b_1[k] \end{bmatrix} \in \mathbb{R}^3$$

where:
- $\delta_{\text{actual}}[k]$: physical front wheel steering angle.
- $b_0[k] = u_{\text{cmd}}(t_k - 0.010\text{ s})$: in-flight command queued $10\text{ ms}$ ago.
- $b_1[k] = u_{\text{cmd}}(t_k - 0.020\text{ s})$: in-flight command queued $20\text{ ms}$ ago.

Over each $40\text{ ms}$ MPC cycle, the published command $u[k]$ is held constant across four $10\text{ ms}$ integration steps:

$$\begin{aligned}
\text{Step } 0 \; (t_k \to t_k + 10\text{ ms}): & \quad \delta_{\text{tgt}}^{(0)} = b_1[k], & \delta^{(1)} &= \delta_{\text{actual}}[k] + q \cdot \operatorname{sgn}^*(b_1[k] - \delta_{\text{actual}}[k]) \\
\text{Step } 1 \; (t_k + 10 \to t_k + 20\text{ ms}): & \quad \delta_{\text{tgt}}^{(1)} = b_0[k], & \delta^{(2)} &= \delta^{(1)} + q \cdot \operatorname{sgn}^*(b_0[k] - \delta^{(1)}) \\
\text{Step } 2 \; (t_k + 20 \to t_k + 30\text{ ms}): & \quad \delta_{\text{tgt}}^{(2)} = u[k], & \delta^{(3)} &= \delta^{(2)} + q \cdot \operatorname{sgn}^*(u[k] - \delta^{(2)}) \\
\text{Step } 3 \; (t_k + 30 \to t_k + 40\text{ ms}): & \quad \delta_{\text{tgt}}^{(3)} = u[k], & \delta_{\text{actual}}[k+1] &= \delta^{(3)} + q \cdot \operatorname{sgn}^*(u[k] - \delta^{(3)})
\end{aligned}$$

where $q = sv_{\text{max}} \cdot \Delta t_{\text{sim}} = 0.032\text{ rad} = 32.0\text{ mrad}$ ($1.833^\circ$).

### Verification to Machine Precision:
Tested against official `f110_gym` over 100 arbitrary states:
- Max error in $\delta_{\text{actual}}$: **$1.11 \times 10^{-16}\text{ rad}$**
- Max error in $b_0, b_1$: **$0.00\text{ rad}$**

---

## 2. Multi-Rate Clock-Domain Timeline

Because transport delay ($20\text{ ms}$) is half the control period ($40\text{ ms}$), command $u[k]$ is active at the physical relay during:
$$t \in [t_k + 20\text{ ms}, \; t_k + 60\text{ ms}]$$
symmetrically straddling the cycle boundary with a $20\text{ ms}$ phase shift.

```
Timeline of Published Command u[k]:
t_k                   t_k + 10 ms         t_k + 20 ms         t_k + 30 ms         t_k + 40 ms (t_k+1)
 ├───────────────────┼───────────────────┼───────────────────┼───────────────────┤
  Executing u[k-1]    Executing u[k-1]    Active: u[k] (1/4)  Active: u[k] (2/4)  Cycle k+1 Begins
                                                                                  Active: u[k] (3/4 & 4/4 in k+1)
```

---

## 3. Reachable-State Analysis

Over a single $40\text{ ms}$ cycle, from the settled $20\text{ ms}$ state $\delta_{20\text{ms}}$, the actual steering at $40\text{ ms}$ can only land on one of three discrete outcomes:

$$\delta_{\text{actual}}[k+1] \in \left\{ \delta_{20\text{ms}} - 64\text{ mrad}, \quad \delta_{20\text{ms}}, \quad \delta_{20\text{ms}} + 64\text{ mrad} \right\}$$

Any command $u[k]$ differing from $\delta_{20\text{ms}}$ by less than $32\text{ mrad}$ produces **zero net displacement** at $t_{k+1}$, but injects two full opposing $\pm 32\text{ mrad}$ velocity impulses into the chassis.

---

## 4. Closed-Loop Validation (Spielberg Benchmark)

Four policies were evaluated in closed loop on the Spielberg circuit ($26.0\text{ s}$, complete lap):

| Metric | BASELINE | POLICY_A_RAW | POLICY_B_REACHABLE | POLICY_C_SCHMITT |
| :--- | :---: | :---: | :---: | :---: |
| **Command RMS** | $2.26^\circ$ | $2.30^\circ$ | $2.30^\circ$ | $2.32^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $5.71\text{ mrad}$ | $5.98\text{ mrad}$ | $12.05\text{ mrad}$ | $11.09\text{ mrad}$ |
| **Steering Rate RMS ($\dot{\delta}$)** | $3.128\text{ rad/s}$ | $3.173\text{ rad/s}$ | **$0.602\text{ rad/s}$ ($-80.8\%$)** | **$0.554\text{ rad/s}$ ($-82.3\%$)** |
| **Steering Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **$5\text{--}7\text{ Hz}$ Yaw Rate Energy** | $22.37$ | $17.14$ | **$8.07$ ($-63.9\%$)** | $11.02$ ($-50.7\%$) |
| **$2\text{--}4\text{ Hz}$ Yaw Rate Energy** | $20.45$ | $25.45$ | **$16.07$ ($-21.4\%$)** | $26.92$ ($+31.6\%$) |
| **Lateral Tracking RMSE** | **$2.95\text{ cm}$** | $3.03\text{ cm}$ | $3.35\text{ cm}$ | $3.20\text{ cm}$ |
| **Maximum Lateral Error** | $12.30\text{ cm}$ | $12.96\text{ cm}$ | **$11.57\text{ cm}$** | **$11.08\text{ cm}$** |
| **Heading Tracking RMSE** | $0.99^\circ$ | $0.95^\circ$ | **$0.95^\circ$** | $1.05^\circ$ |
| **Minimum LiDAR Clearance** | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ | $0$ |
| **Lap Completion** | Yes ($147.4\text{ m}$) | Yes ($147.4\text{ m}$) | Yes ($147.4\text{ m}$) | Yes ($147.4\text{ m}$) |
| **MPC Solve Time** | $2.81\text{ ms}$ | $2.86\text{ ms}$ | **$1.90\text{ ms}$** | $2.00\text{ ms}$ |

---

## 5. Scientific Notes & Scope

1. **Policy B Characterization:** Policy B is an actuator-state-aware reachability filter that combines predictive arrival-state holding with $32\text{ mrad}$ lattice projection; it is not a continuous non-quantized solution.
2. **Causal Attribution:** While the un-damped relay integration is proven to generate $50\text{ Hz}$ chattering that aliases into subharmonic modes under $25\text{ Hz}$ sampling, this milestone does not claim the actuator is the sole contributor to closed-loop vehicle behavior.
3. **Reproducibility:** All scripts in this directory are self-contained and run against the standard `f110_gym` environment.

---

## 6. How to Reproduce

```bash
# 1. Verify lifted model against official f110_gym (machine precision)
python3 experiments/experiment20/exp20_lifted_model_audit.py

# 2. Run reachability and telemetry phase-alignment audit
python3 experiments/experiment20/exp20_trace_and_policy_study.py

# 3. Run closed-loop benchmark evaluation on Spielberg
python3 experiments/experiment20/exp20_closed_loop_eval.py
```
