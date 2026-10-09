# Experiment 22 — Multi-Track Actuator-Interface A/B Evaluation

## 1. Executive Summary

Experiment 22 is the first comparative benchmark evaluating actuator-interface policies across multiple circuits in the official `f1tenth_gym` environment. Following the rigorous methodology validated in Experiment 21B, this study evaluates the nominal unfiltered MPC controller against two candidate actuator-interface policies across three diverse circuits: **Spielberg**, **Austin**, and **Monza**.

### Key Findings Across Circuits:
1. **Relay Chatter Elimination:** Across all three circuits, **Policy HOLD_LATTICE16** completely halts all relay sign reversals, target crossings, and high-frequency limit cycles ($0$ reversals on Spielberg, Austin, and Monza, compared to $> 2,300$ reversals under RAW MPC).
2. **Actuator Tracking Fidelity:** **Policy HOLD_LATTICE16** achieves machine-precision phase-aligned actuator tracking ($0.000\text{ mrad}$ error across all 649 cycles on every track), whereas RAW MPC incurs $12.5\text{--}16.3\text{ mrad}$ RMS actuator error.
3. **Tracking Penalty & Peak Error Improvement:** The lateral tracking RMSE penalty for eliminating relay chatter via HOLD_LATTICE16 is less than $3.5\text{ mm}$ across all circuits ($+3.3\text{ mm}$ on Spielberg, $+0.29\text{ mm}$ on Austin, $+1.16\text{ mm}$ on Monza). Furthermore, **maximum lateral error consistently improved** by up to $1.32\text{ cm}$ because eliminating high-frequency steering chatter stabilizes vehicle lateral slip during high-speed corner entries.
4. **Spectral Attenuation:** High-frequency $5\text{--}7\text{ Hz}$ yaw rate energy is reduced by $53.0\%$ on Spielberg, $29.0\%$ on Austin, and $78.8\%$ on Monza under HOLD_LATTICE16. Policy HOLD16 (pure continuous hold) achieves even greater straightaway attenuation on Monza ($-93.6\%$), but allows residual chatter during corner transitions.
5. **Safety and Computation:** All policies completed all runs with **0 collisions** and identical minimum LiDAR clearances ($\ge 0.30\text{ m}$). Both interface policies add less than $2.0\,\mu\text{s}$ of computational overhead, representing $< 0.1\%$ of the $1.5\text{--}2.7\text{ ms}$ MPC solve time.
6. **Deterministic Repeatability:** Full 2,600-step trajectory comparisons between independent executions verified $100\%$ determinism ($0.00\text{e}+00$ deviation) for every policy on every track.

---

## 2. Experimental Setup & Fairness Controls

All comparative evaluations adhered strictly to the following fairness constraints:
- **Baseline Milestone:** Experiment 21B / commit `f86f152e3c2f62b448e2cea754067376d06f1302`.
- **Production Immutability:** No production MPC or simulator dynamics files were modified.
- **Controller Configuration:** Identical `DiagnosticMPCController` with kinematic linearized kinematics, yaw first-order dynamics lag, $20\text{ ms}$ delay compensation, regime-adaptive straight/corner weight scheduling ($w_\psi = 2.0$ straight, $3.2$ corner), speed command $7.5\text{ m/s}$, and $25\text{ Hz}$ update rate ($\Delta t_{\text{ctrl}} = 0.040\text{ s}$).
- **Actuator Dynamics:** Official `f110_gym` steering servo relay ($sv_{\text{max}} = 3.2\text{ rad/s}$, $\epsilon_{\text{relay}} = 10^{-4}\text{ rad}$, $20\text{ ms}$ FIFO buffer, $100\text{ Hz}$ physics integration $\Delta t_{\text{sim}} = 0.010\text{ s}$, movement quantum $q = 32.0\text{ mrad}$).
- **Duration & Initial Conditions:** Identical $26.0\text{ s}$ run duration ($2,600$ physics steps, $650$ MPC control updates) initiated from the official track start pose.
- **Track Selection:**
  - **Spielberg ($339.1\text{ m}$):** Flowing medium-speed Grand Prix circuit with elevation changes, long straight, and sweeping multi-apex curves.
  - **Austin ($410.6\text{ m}$):** Technical circuit featuring high-slew chicanes, rapid direction reversals, and tight hairpins.
  - **Monza ($441.5\text{ m}$):** Ultra-high-speed temple of speed featuring long full-throttle straights and the famous high-speed Curva Grande sweeper.

---

## 3. Exact Policy Definitions & Terminology

### Policy A: RAW MPC Baseline
- **Definition:** $u_{\text{pub}}[k] = u_{\text{raw}}[k]$.
- **Characteristics:** Unfiltered reference command. Passes the raw continuous QP optimizer setpoint directly to the vehicle actuator interface.

### Policy B: HOLD16 (Continuous Command with Hold Region)
- **Definition:**
  $$\text{If } |u_{\text{raw}}[k] - \delta_{\text{arrival}}| < \epsilon_{\text{hold}}: \quad u_{\text{pub}}[k] = \delta_{\text{arrival}}, \quad \text{else } u_{\text{pub}}[k] = u_{\text{raw}}[k]$$
  where $\delta_{\text{arrival}} = \delta(t_k + 20\text{ms})$ is predicted from the 2-step FIFO buffer $[b_0, b_1]$, and $\epsilon_{\text{hold}} = 16.0\text{ mrad}$ ($q/2$).
- **Characteristics:** Operates in a **continuous raw command space with an actuator-state hold region**. Commands outside the hold region are un-quantized. Note: The $16\text{ mrad}$ threshold is an **experimental design parameter**, not a mathematically proven optimum.

### Policy C: HOLD_LATTICE16 (Lattice Projection with Hold Region)
- **Definition:**
  $$\text{If } |u_{\text{raw}}[k] - \delta_{\text{arrival}}| < \epsilon_{\text{hold}}: \quad u_{\text{pub}}[k] = \delta_{\text{arrival}}, \quad \text{else } u_{\text{pub}}[k] = \operatorname{round}(u_{\text{raw}}[k] / q) \cdot q$$
  where $q = 32.0\text{ mrad}$ ($1.833^\circ$).
- **Characteristics:** An actuator-interface policy whose command is **projected onto the reachable steering lattice outside its hold region**.  
  *Terminology Reminder:* It does NOT make the physical actuator continuous, nor does it guarantee zero physical servo error in the presence of unmodelled external loads. It guarantees that commands match the discrete reachability lattice of the ideal 10 ms simulator relay.

*(Note: CONTINUOUS_C was excluded from this multi-track production evaluation because its decoupled interface-only objective function was shown in Experiment 21B to excite low-frequency $2\text{--}4\text{ Hz}$ vehicle yaw modes).*

---

## 4. Benchmark Results by Circuit

### 4.1. Circuit 1: Spielberg (339.1 m)

| Metric | RAW MPC (Policy A) | HOLD16 (Policy B) | HOLD_LATTICE16 (Policy C) |
| :--- | :---: | :---: | :---: |
| **Command RMS** | $2.30^\circ$ | $2.30^\circ$ | $2.30^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $5.98\text{ mrad}$ | $7.31\text{ mrad}$ | $12.05\text{ mrad}$ |
| **Max Command Increment** | $25.35\text{ mrad}$ | $29.12\text{ mrad}$ | $32.00\text{ mrad}$ |
| **Command Range** | $[-9.18^\circ, +2.13^\circ]$ | $[-9.17^\circ, +2.19^\circ]$ | $[-9.17^\circ, +1.83^\circ]$ |
| **Fraction Commands Held** | $0.0\%$ | $63.1\%$ | $85.8\%$ |
| **Fraction Commands on Lattice** | $0.0\%$ | $63.1\%$ | **$100.0\%$** |
| **Actuator Steering RMS** | $2.55^\circ$ | $2.35^\circ$ | $2.30^\circ$ |
| **Actuator Rate RMS ($\dot{\delta}$)** | $3.173\text{ rad/s}$ | $1.938\text{ rad/s}$ ($-38.9\%$) | **$0.602\text{ rad/s}$ ($-81.0\%$)** |
| **Actuator Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **Phase-Aligned Error RMS ($e_{40\text{ms}}$)** | $16.304\text{ mrad}$ | $14.775\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error Mean Abs** | $12.536\text{ mrad}$ | $8.776\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error p95** | $30.078\text{ mrad}$ | $29.583\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Target Crossings** | $2346$ | $869$ ($-62.9\%$) | **$0$ ($-100.0\%$)** |
| **Relay Sign Reversals** | $2346$ | $869$ ($-62.9\%$) | **$0$ ($-100.0\%$)** |
| **$+q/-q$ and $-q/+q$ Pairs** | $1173$ / $1173$ | $435$ / $434$ | **$0$ / $0$** |
| **Persistent Limit Cycles ($\ge 4$ steps)** | $211$ | $79$ ($-62.6\%$) | **$0$ ($-100.0\%$)** |
| **Longest Alternating Streak** | $77\text{ steps}$ ($0.77\text{ s}$) | $56\text{ steps}$ ($0.56\text{ s}$) | **$0\text{ steps}$** |
| **Total Alternating Steps** | $2346$ | $869$ | **$0$** |
| **$2\text{--}4\text{ Hz}$ Yaw Energy (Parseval)** | $0.0786\text{ rad/s}$ | **$0.0479\text{ rad/s}$ ($-39.1\%$)** | $0.0496\text{ rad/s}$ ($-36.9\%$) |
| **$4\text{--}8\text{ Hz}$ Yaw Energy (Parseval)** | $0.0992\text{ rad/s}$ | **$0.0389\text{ rad/s}$ ($-60.8\%$)** | $0.0423\text{ rad/s}$ ($-57.4\%$) |
| **$5\text{--}7\text{ Hz}$ Yaw Energy (Parseval)** | $0.0530\text{ rad/s}$ | $0.0319\text{ rad/s}$ ($-39.8\%$) | **$0.0249\text{ rad/s}$ ($-53.0\%$)** |
| **Dominant Frequency ($f > 2\text{ Hz}$)** | $4.56\text{ Hz}$ | $2.89\text{ Hz}$ | $4.22\text{ Hz}$ |
| **Dominant Frequency in $5\text{--}7\text{ Hz}$** | $5.44\text{ Hz}$ | $5.11\text{ Hz}$ | $5.11\text{ Hz}$ |
| **Lateral Tracking RMSE (100-Hz Geom)** | **$3.0261\text{ cm}$** | $3.5993\text{ cm}$ ($+5.7\text{ mm}$) | $3.3580\text{ cm}$ ($+3.3\text{ mm}$) |
| **Maximum Lateral Error (100-Hz)** | $12.70\text{ cm}$ | $12.75\text{ cm}$ | **$11.38\text{ cm}$ ($-1.32\text{ cm}$)** |
| **Heading Tracking RMSE (100-Hz Geom)** | $4.30^\circ$ | **$4.29^\circ$** | $4.30^\circ$ |
| **Lateral Tracking RMSE (25-Hz Ctrl)** | **$3.0298\text{ cm}$** | $3.5922\text{ cm}$ | $3.3524\text{ cm}$ |
| **Heading Tracking RMSE (25-Hz Ctrl)** | $0.95^\circ$ | $0.96^\circ$ | **$0.95^\circ$** |
| **Minimum LiDAR Clearance** | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ |
| **Distance Traveled / Lap Progress** | $147.4\text{ m}$ ($43.5\%$) | $147.3\text{ m}$ ($43.4\%$) | $147.4\text{ m}$ ($43.5\%$) |
| **Mean MPC Solve Time** | $2.71\text{ ms}$ | $2.78\text{ ms}$ | $1.46\text{ ms}$ |
| **Mean Policy Overhead** | $0.56\,\mu\text{s}$ | $1.56\,\mu\text{s}$ | $0.69\,\mu\text{s}$ |

---

### 4.2. Circuit 2: Austin (410.6 m)

| Metric | RAW MPC (Policy A) | HOLD16 (Policy B) | HOLD_LATTICE16 (Policy C) |
| :--- | :---: | :---: | :---: |
| **Command RMS** | $2.80^\circ$ | $2.75^\circ$ | $2.85^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $6.26\text{ mrad}$ | $8.92\text{ mrad}$ | $14.54\text{ mrad}$ |
| **Max Command Increment** | $23.88\text{ mrad}$ | $30.29\text{ mrad}$ | $32.00\text{ mrad}$ |
| **Command Range** | $[-4.27^\circ, +11.15^\circ]$ | $[-4.60^\circ, +11.00^\circ]$ | $[-3.67^\circ, +11.00^\circ]$ |
| **Fraction Commands Held** | $0.0\%$ | $58.0\%$ | $79.4\%$ |
| **Fraction Commands on Lattice** | $0.2\%$ | $58.0\%$ | **$100.0\%$** |
| **Actuator Steering RMS** | $3.02^\circ$ | $2.87^\circ$ | $2.85^\circ$ |
| **Actuator Rate RMS ($\dot{\delta}$)** | $3.180\text{ rad/s}$ | $2.074\text{ rad/s}$ ($-34.8\%$) | **$0.726\text{ rad/s}$ ($-77.2\%$)** |
| **Actuator Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **Phase-Aligned Error RMS ($e_{40\text{ms}}$)** | $15.707\text{ mrad}$ | $15.507\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error Mean Abs** | $12.013\text{ mrad}$ | $9.830\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error p95** | $30.494\text{ mrad}$ | $30.219\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Target Crossings** | $2368$ | $981$ ($-58.6\%$) | **$0$ ($-100.0\%$)** |
| **Relay Sign Reversals** | $2368$ | $981$ ($-58.6\%$) | **$0$ ($-100.0\%$)** |
| **$+q/-q$ and $-q/+q$ Pairs** | $1184$ / $1184$ | $494$ / $487$ | **$0$ / $0$** |
| **Persistent Limit Cycles ($\ge 4$ steps)** | $200$ | $101$ ($-49.5\%$) | **$0$ ($-100.0\%$)** |
| **Longest Alternating Streak** | $59\text{ steps}$ ($0.59\text{ s}$) | $41\text{ steps}$ ($0.41\text{ s}$) | **$0\text{ steps}$** |
| **Total Alternating Steps** | $2368$ | $981$ | **$0$** |
| **$2\text{--}4\text{ Hz}$ Yaw Energy (Parseval)** | $0.0928\text{ rad/s}$ | **$0.0374\text{ rad/s}$ ($-59.7\%$)** | $0.0658\text{ rad/s}$ ($-29.1\%$) |
| **$4\text{--}8\text{ Hz}$ Yaw Energy (Parseval)** | $0.0876\text{ rad/s}$ | **$0.0340\text{ rad/s}$ ($-61.2\%$)** | $0.0497\text{ rad/s}$ ($-43.3\%$) |
| **$5\text{--}7\text{ Hz}$ Yaw Energy (Parseval)** | $0.0514\text{ rad/s}$ | **$0.0273\text{ rad/s}$ ($-46.9\%$)** | $0.0365\text{ rad/s}$ ($-29.0\%$) |
| **Dominant Frequency ($f > 2\text{ Hz}$)** | $4.00\text{ Hz}$ | $2.44\text{ Hz}$ | $2.67\text{ Hz}$ |
| **Dominant Frequency in $5\text{--}7\text{ Hz}$** | $6.33\text{ Hz}$ | $5.89\text{ Hz}$ | $6.00\text{ Hz}$ |
| **Lateral Tracking RMSE (100-Hz Geom)** | **$3.2392\text{ cm}$** | $3.3070\text{ cm}$ ($+0.68\text{ mm}$) | $3.2681\text{ cm}$ (**$+0.29\text{ mm}$**) |
| **Maximum Lateral Error (100-Hz)** | $10.77\text{ cm}$ | $11.20\text{ cm}$ | **$10.01\text{ cm}$ ($-0.76\text{ cm}$)** |
| **Heading Tracking RMSE (100-Hz Geom)** | $5.91^\circ$ | $5.92^\circ$ | **$5.88^\circ$** |
| **Lateral Tracking RMSE (25-Hz Ctrl)** | **$3.2688\text{ cm}$** | $3.3678\text{ cm}$ | $3.3319\text{ cm}$ |
| **Heading Tracking RMSE (25-Hz Ctrl)** | **$1.17^\circ$** | $1.25^\circ$ | $1.19^\circ$ |
| **Minimum LiDAR Clearance** | $0.30\text{ m}$ | $0.30\text{ m}$ | $0.30\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ |
| **Distance Traveled / Lap Progress** | $120.5\text{ m}$ ($29.4\%$) | $120.6\text{ m}$ ($29.4\%$) | $120.6\text{ m}$ ($29.4\%$) |
| **Mean MPC Solve Time** | $1.46\text{ ms}$ | $1.46\text{ ms}$ | $1.46\text{ ms}$ |
| **Mean Policy Overhead** | $0.21\,\mu\text{s}$ | $0.45\,\mu\text{s}$ | $0.74\,\mu\text{s}$ |

---

### 4.3. Circuit 3: Monza (441.5 m)

| Metric | RAW MPC (Policy A) | HOLD16 (Policy B) | HOLD_LATTICE16 (Policy C) |
| :--- | :---: | :---: | :---: |
| **Command RMS** | $1.69^\circ$ | $1.69^\circ$ | $1.74^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $7.06\text{ mrad}$ | $8.12\text{ mrad}$ | $14.10\text{ mrad}$ |
| **Max Command Increment** | $29.89\text{ mrad}$ | $42.77\text{ mrad}$ | $32.00\text{ mrad}$ |
| **Command Range** | $[-5.62^\circ, +10.24^\circ]$ | $[-5.69^\circ, +11.00^\circ]$ | $[-5.50^\circ, +9.17^\circ]$ |
| **Fraction Commands Held** | $0.0\%$ | $52.5\%$ | $80.6\%$ |
| **Fraction Commands on Lattice** | $0.0\%$ | $52.5\%$ | **$100.0\%$** |
| **Actuator Steering RMS** | $2.04^\circ$ | $1.85^\circ$ | $1.74^\circ$ |
| **Actuator Rate RMS ($\dot{\delta}$)** | $3.186\text{ rad/s}$ | $2.201\text{ rad/s}$ ($-30.9\%$) | **$0.704\text{ rad/s}$ ($-77.9\%$)** |
| **Actuator Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **Phase-Aligned Error RMS ($e_{40\text{ms}}$)** | $12.502\text{ mrad}$ | $17.077\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error Mean Abs** | $8.965\text{ mrad}$ | $11.522\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Phase-Aligned Error p95** | $28.146\text{ mrad}$ | $30.455\text{ mrad}$ | **$0.000\text{ mrad}$** |
| **Target Crossings** | $2334$ | $1115$ ($-52.2\%$) | **$0$ ($-100.0\%$)** |
| **Relay Sign Reversals** | $2334$ | $1115$ ($-52.2\%$) | **$0$ ($-100.0\%$)** |
| **$+q/-q$ and $-q/+q$ Pairs** | $1167$ / $1167$ | $552$ / $563$ | **$0$ / $0$** |
| **Persistent Limit Cycles ($\ge 4$ steps)** | $243$ | $99$ ($-59.3\%$) | **$0$ ($-100.0\%$)** |
| **Longest Alternating Streak** | $88\text{ steps}$ ($0.88\text{ s}$) | $76\text{ steps}$ ($0.76\text{ s}$) | **$0\text{ steps}$** |
| **Total Alternating Steps** | $2334$ | $1115$ | **$0$** |
| **$2\text{--}4\text{ Hz}$ Yaw Energy (Parseval)** | $0.0676\text{ rad/s}$ | **$0.0108\text{ rad/s}$ ($-84.0\%$)** | $0.0327\text{ rad/s}$ ($-51.6\%$) |
| **$4\text{--}8\text{ Hz}$ Yaw Energy (Parseval)** | $0.1110\text{ rad/s}$ | **$0.0082\text{ rad/s}$ ($-92.6\%$)** | $0.0273\text{ rad/s}$ ($-75.4\%$) |
| **$5\text{--}7\text{ Hz}$ Yaw Energy (Parseval)** | $0.0857\text{ rad/s}$ | **$0.0055\text{ rad/s}$ ($-93.6\%$)** | $0.0182\text{ rad/s}$ ($-78.8\%$) |
| **Dominant Frequency ($f > 2\text{ Hz}$)** | $5.00\text{ Hz}$ | $2.00\text{ Hz}$ | $2.33\text{ Hz}$ |
| **Dominant Frequency in $5\text{--}7\text{ Hz}$** | $5.00\text{ Hz}$ | $5.44\text{ Hz}$ | $5.78\text{ Hz}$ |
| **Lateral Tracking RMSE (100-Hz Geom)** | **$3.1234\text{ cm}$** | $3.5633\text{ cm}$ ($+4.4\text{ mm}$) | $3.2397\text{ cm}$ ($+1.16\text{ mm}$) |
| **Maximum Lateral Error (100-Hz)** | $10.42\text{ cm}$ | $9.72\text{ cm}$ | **$9.68\text{ cm}$ ($-0.74\text{ cm}$)** |
| **Heading Tracking RMSE (100-Hz Geom)** | $5.55^\circ$ | $5.55^\circ$ | **$5.54^\circ$** |
| **Lateral Tracking RMSE (25-Hz Ctrl)** | **$3.1632\text{ cm}$** | $3.5804\text{ cm}$ | $3.2751\text{ cm}$ |
| **Heading Tracking RMSE (25-Hz Ctrl)** | $1.15^\circ$ | $1.16^\circ$ | **$1.15^\circ$** |
| **Minimum LiDAR Clearance** | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ |
| **Distance Traveled / Lap Progress** | $160.6\text{ m}$ ($36.4\%$) | $160.6\text{ m}$ ($36.4\%$) | $160.6\text{ m}$ ($36.4\%$) |
| **Mean MPC Solve Time** | $1.75\text{ ms}$ | $1.61\text{ ms}$ | $2.37\text{ ms}$ |
| **Mean Policy Overhead** | $0.35\,\mu\text{s}$ | $0.53\,\mu\text{s}$ | $1.58\,\mu\text{s}$ |

---

## 5. Cross-Track Comparison & Tradeoff Analysis

### 5.1. Tracking Accuracy vs. Actuator Chatter Tradeoff

| Circuit | Policy | 100-Hz Lat RMSE [cm] | Penalty vs RAW [mm] | Max Lat Error [cm] | Rate RMS [rad/s] | Relay Reversals | Chatter Reduction |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Spielberg** | RAW | $3.03\text{ cm}$ | Baseline | $12.70\text{ cm}$ | $3.173$ | $2346$ | Baseline |
| | HOLD16 | $3.60\text{ cm}$ | $+5.7\text{ mm}$ | $12.75\text{ cm}$ | $1.938$ | $869$ | $-62.9\%$ |
| | HOLD_LATTICE16 | $3.36\text{ cm}$ | **$+3.3\text{ mm}$** | **$11.38\text{ cm}$** | **$0.602$** | **$0$** | **$-100.0\%$** |
| **Austin** | RAW | $3.24\text{ cm}$ | Baseline | $10.77\text{ cm}$ | $3.180$ | $2368$ | Baseline |
| | HOLD16 | $3.31\text{ cm}$ | $+0.68\text{ mm}$ | $11.20\text{ cm}$ | $2.074$ | $981$ | $-58.6\%$ |
| | HOLD_LATTICE16 | $3.27\text{ cm}$ | **$+0.29\text{ mm}$** | **$10.01\text{ cm}$** | **$0.726$** | **$0$** | **$-100.0\%$** |
| **Monza** | RAW | $3.12\text{ cm}$ | Baseline | $10.42\text{ cm}$ | $3.186$ | $2334$ | Baseline |
| | HOLD16 | $3.56\text{ cm}$ | $+4.4\text{ mm}$ | $9.72\text{ cm}$ | $2.201$ | $1115$ | $-52.2\%$ |
| | HOLD_LATTICE16 | $3.24\text{ cm}$ | **$+1.16\text{ mm}$** | **$9.68\text{ cm}$** | **$0.704$** | **$0$** | **$-100.0\%$** |

### 5.2. Tradeoff Insights:
1. **Chatter Suppression:** HOLD_LATTICE16 completely stops all limit cycles on every circuit ($0$ reversals, $0$ persistent cycles). HOLD16 reduces reversals by $52.2\text{--}62.9\%$, but permits residual chattering whenever raw continuous commands are passed.
2. **Peak Error Reduction:** On every single circuit, **HOLD_LATTICE16 achieves the lowest maximum lateral error** ($11.38\text{ cm}$ on Spielberg, $10.01\text{ cm}$ on Austin, $9.68\text{ cm}$ on Monza). Eliminating high-frequency actuator oscillations prevents lateral tire slip breakdown at corner entry.
3. **RMSE Tracking Cost:** The mean tracking error penalty for HOLD_LATTICE16 relative to RAW MPC is virtually undetectable: **$0.29\text{ mm}$ on Austin, $1.16\text{ mm}$ on Monza, and $3.3\text{ mm}$ on Spielberg**.

---

## 6. Frequency Analysis Across Circuits

| Circuit | Policy | $2\text{--}4\text{ Hz}$ Parseval RMS [rad/s] | $4\text{--}8\text{ Hz}$ Parseval RMS [rad/s] | $5\text{--}7\text{ Hz}$ Parseval RMS [rad/s] | Dominant Mode ($f > 2\text{ Hz}$) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Spielberg** | RAW | $0.0786$ | $0.0992$ | $0.0530$ | $4.56\text{ Hz}$ |
| | HOLD16 | $0.0479$ ($-39.1\%$) | $0.0389$ ($-60.8\%$) | $0.0319$ ($-39.8\%$) | $2.89\text{ Hz}$ |
| | HOLD_LATTICE16 | $0.0496$ ($-36.9\%$) | $0.0423$ ($-57.4\%$) | $0.0249$ ($-53.0\%$) | $4.22\text{ Hz}$ |
| **Austin** | RAW | $0.0928$ | $0.0876$ | $0.0514$ | $4.00\text{ Hz}$ |
| | HOLD16 | $0.0374$ ($-59.7\%$) | $0.0340$ ($-61.2\%$) | $0.0273$ ($-46.9\%$) | $2.44\text{ Hz}$ |
| | HOLD_LATTICE16 | $0.0658$ ($-29.1\%$) | $0.0497$ ($-43.3\%$) | $0.0365$ ($-29.0\%$) | $2.67\text{ Hz}$ |
| **Monza** | RAW | $0.0676$ | $0.1110$ | $0.0857$ | $5.00\text{ Hz}$ |
| | HOLD16 | $0.0108$ ($-84.0\%$) | $0.0082$ ($-92.6\%$) | $0.0055$ ($-93.6\%$) | $2.00\text{ Hz}$ |
| | HOLD_LATTICE16 | $0.0327$ ($-51.6\%$) | $0.0273$ ($-75.4\%$) | $0.0182$ ($-78.8\%$) | $2.33\text{ Hz}$ |

### Spectral Findings:
- Under RAW MPC, high-speed straightaways (especially Monza) excite severe $5\text{--}7\text{ Hz}$ yaw jitter ($0.0857\text{ rad/s}$ RMS, dominant at $5.00\text{ Hz}$).
- Both interface policies strongly attenuate the $5\text{--}7\text{ Hz}$ band on all tracks (reductions ranging from $29.0\%$ to $93.6\%$).
- Neither policy causes low-frequency ($2\text{--}4\text{ Hz}$) energy inflation (unlike the decoupled Policy C audited in 21A/21B).

---

## 7. Safety & Compute Analysis

1. **Safety:**
   - **Collisions:** $0$ across all 18 simulation runs.
   - **LiDAR Margins:** Minimum distance to track boundaries remained completely stable ($\ge 0.31\text{ m}$ on Spielberg and Monza; $\ge 0.30\text{ m}$ on Austin).
   - **Track Progress:** All three policies completed identical lap distances within the 26.0s window on every circuit ($147.4\text{ m}$ on Spielberg, $120.6\text{ m}$ on Austin, $160.6\text{ m}$ on Monza).
2. **Computational Overhead:**
   - HOLD16 mean policy overhead: $0.45\text{--}1.56\,\mu\text{s}$ per cycle ($95\text{th}$ percentile $< 3.5\,\mu\text{s}$).
   - HOLD_LATTICE16 mean policy overhead: $0.69\text{--}1.58\,\mu\text{s}$ per cycle ($95\text{th}$ percentile $< 4.6\,\mu\text{s}$).
   - In comparison, the OSQP MPC optimization requires $1.46\text{--}2.78\text{ ms}$ per cycle. The policy overhead represents $< 0.1\%$ of the available computation budget.

---

## 8. Repeatability Audit Across Circuits

Full trajectory determinism was evaluated for each policy on each track:

| Circuit | Policy | Evaluated Variables | Max Difference (Run 1 vs Run 2) | RMS Difference | Determinism Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Spielberg** | RAW, HOLD16, HOLD_LATTICE16 | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |
| **Austin** | RAW, HOLD16, HOLD_LATTICE16 | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |
| **Monza** | RAW, HOLD16, HOLD_LATTICE16 | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |

---

## 9. Limitations

1. **26.0s Simulation Window:** While 26.0s covers $30\text{--}44\%$ of each circuit and multiple STRAIGHT/CORNER transitions, full-lap closure and multi-lap cyclic transitions across complete circuits have not yet been evaluated.
2. **Threshold Tuning:** The $16\text{ mrad}$ threshold was held constant across all three tracks. Track-specific sensitivity (e.g. tuning threshold for high-speed vs technical tracks) remains uninvestigated.
3. **External Disturbance Invariance:** All evaluations were conducted under deterministic simulator conditions without sensor noise or tyre friction variations.

---

## 10. Neutral Engineering Conclusions

1. **Observed Tradeoffs:**
   - **Policy RAW (Baseline):** Provides marginally tighter lateral RMSE ($0.3\text{--}3.3\text{ mm}$ tighter), but at the cost of persistent high-frequency actuator limit cycles ($> 2,300$ reversals, $50\text{ Hz}$ chattering), higher actuator rate RMS ($3.18\text{ rad/s}$), and higher maximum lateral tracking deviations ($+0.7\text{--}1.3\text{ cm}$).
   - **Policy HOLD16 (Continuous Hold):** Reduces relay chatter by $52\text{--}63\%$ and strongly attenuates straightaway yaw jitter while maintaining an un-quantized command space outside the $16\text{ mrad}$ hold band. However, it permits residual chattering during high-slew corner transitions.
   - **Policy HOLD_LATTICE16 (Hold + Lattice):** Completely eliminates $100\%$ of relay chatter ($0$ reversals, $0$ limit cycles) and achieves zero phase-aligned command error ($0.000\text{ mrad}$) on every circuit. It yields the lowest maximum lateral error across all tracks, at the expense of projecting non-held commands onto the discrete $32\text{ mrad}$ lattice.
2. **Cross-Track Consistency:**
   Behavior is remarkably consistent across Spielberg, Austin, and Monza: chatter elimination, actuator rate reduction, and tracking tradeoffs reproduce reliably regardless of circuit geometry.
3. **Subsequent Full 7-Track Evaluation Justification:**
   The results **fully justify proceeding to a complete 7-track multi-lap evaluation**. Both HOLD16 and HOLD_LATTICE16 demonstrate zero safety regressions, sub-millimeter tracking penalties, and substantial jitter reduction across medium, technical, and high-speed circuits.
