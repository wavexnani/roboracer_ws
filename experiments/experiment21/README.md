# Experiment 21 — Actuator-Aware Policy Sensitivity and Continuous Interface Study

## 1. Hypothesis
The high-frequency $5.5\text{--}6.0\text{ Hz}$ straightaway steering jitter observed under nominal MPC can be substantially suppressed without forcing continuous MPC commands onto an artificial $32\text{ mrad}$ discrete lattice, by utilizing an actuator-aware interface that predicts the $20\text{ ms}$ FIFO arrival state and suppresses sub-quantum switching requests.

## 2. Experimental Setup
- **Baseline Milestone:** Experiment 20 commit `f86f152e3c2f62b448e2cea754067376d06f1302` (immutable baseline).
- **Track & Benchmark:** Spielberg racetrack ($147.4\text{ m}$ circuit length, $26.0\text{ s}$ closed-loop runs).
- **Sampling Rates:** $25\text{ Hz}$ MPC controller ($\Delta t_{\text{ctrl}} = 0.040\text{ s}$), $100\text{ Hz}$ simulator physics ($\Delta t_{\text{sim}} = 0.010\text{ s}$).
- **Hardware Servo Slew Bound:** $sv_{\text{max}} = 3.2\text{ rad/s}$ with un-damped relay deadband $\epsilon_{\text{relay}} = 10^{-4}\text{ rad}$.
- **Transport Delay:** $20\text{ ms}$ (2-step FIFO buffer size).
- **Movement Quantum:** $q = sv_{\text{max}} \cdot \Delta t_{\text{sim}} = 0.032\text{ rad} = 32.0\text{ mrad}$ ($1.833^\circ$).

## 3. Exact Lifted Actuator Equations
State at control cycle $k$: $X[k] = [\delta_{\text{actual}}[k], \, b_0[k], \, b_1[k]]^T \in \mathbb{R}^3$.  
Over the four $10\text{ ms}$ substeps of each $40\text{ ms}$ MPC cycle:

$$\begin{aligned}
\text{Step } 0 \; (00\text{--}10\text{ ms}): & \quad \delta_{\text{tgt}}^{(0)} = b_1[k], & \delta^{(1)} &= \delta_{\text{actual}}[k] + q \cdot \operatorname{sgn}^*(b_1[k] - \delta_{\text{actual}}[k]) \\
\text{Step } 1 \; (10\text{--}20\text{ ms}): & \quad \delta_{\text{tgt}}^{(1)} = b_0[k], & \delta^{(2)} &= \delta^{(1)} + q \cdot \operatorname{sgn}^*(b_0[k] - \delta^{(1)}) \\
\text{Step } 2 \; (20\text{--}30\text{ ms}): & \quad \delta_{\text{tgt}}^{(2)} = u[k], & \delta^{(3)} &= \delta^{(2)} + q \cdot \operatorname{sgn}^*(u[k] - \delta^{(2)}) \\
\text{Step } 3 \; (30\text{--}40\text{ ms}): & \quad \delta_{\text{tgt}}^{(3)} = u[k], & \delta_{\text{actual}}[k+1] &= \delta^{(3)} + q \cdot \operatorname{sgn}^*(u[k] - \delta^{(3)})
\end{aligned}$$

The arrival steering state when $u[k]$ reaches the relay is $\delta_{\text{arrival}} = \delta^{(2)}$ at $t_k + 20\text{ ms}$.

## 4. Policy Definitions
1. **Policy R (Raw MPC):** $u[k] = u_{\text{raw}}[k]$ (Unfiltered reference).
2. **Policy H (Hold with Threshold $\epsilon_{\text{hold}}$):**
   $$\text{If } |u_{\text{raw}}[k] - \delta_{\text{arrival}}| < \epsilon_{\text{hold}}: \quad u[k] = \delta_{\text{arrival}}, \quad \text{else } u[k] = u_{\text{raw}}[k]$$
3. **Policy L (Static Lattice):** $u[k] = \operatorname{round}(u_{\text{raw}}[k] / q) \cdot q$ (Discrete control baseline).
4. **Policy HL (Hold + Lattice):**
   $$\text{If } |u_{\text{raw}}[k] - \delta_{\text{arrival}}| < \epsilon_{\text{hold}}: \quad u[k] = \delta_{\text{arrival}}, \quad \text{else } u[k] = \operatorname{round}(u_{\text{raw}}[k] / q) \cdot q$$
5. **Policy C (Continuous Actuator-Aware Optimization):**
   Minimizes the exact predicted cost $J(u) = (\delta_{40\text{ms}}(u) - u_{\text{raw}})^2 + w_{\text{motion}} \cdot \text{cost}_{\text{motion}} + w_{\text{reverse}} \cdot \text{reversals}$ over continuous candidate space without discretizing to lattice multiples.

## 5. Threshold Sensitivity Sweep (Offline over 800 Experiment 18 Cycles)
Evaluated across $\epsilon_{\text{hold}} \in [0, 32]\text{ mrad}$:

| Threshold ($\epsilon_{\text{hold}}$) | RMS($\Delta u$) [mrad] | Actuator RMS($sv$) [rad/s] | Relay Reversals | Zero-State Occupancy | 2-Step Chatter Cycles | 40 ms Error RMS [mrad] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$0\text{ mrad}$ (Raw)** | $5.67$ | $3.179$ | $2925$ | $1.31\%$ | $799$ | $11.39$ |
| **$4\text{ mrad}$** | $5.53$ | $2.529$ | $1827$ | $37.56\%$ | $639$ | $11.39$ |
| **$8\text{ mrad}$** | $5.46$ | $1.856$ | $979$ | $66.38\%$ | $343$ | $11.39$ |
| **$12\text{ mrad}$** | $5.08$ | $1.505$ | $651$ | $77.88\%$ | $211$ | $11.39$ |
| **$16\text{ mrad}$ ($q/2$)** | **$5.02$** | **$1.275$** | **$464$** | **$84.12\%$** | **$148$** | **$11.39$** |
| **$20\text{ mrad}$** | $5.25$ | $1.126$ | $355$ | $87.62\%$ | $117$ | $11.39$ |
| **$24\text{ mrad}$** | $6.47$ | $0.919$ | $220$ | $91.75\%$ | $85$ | $11.39$ |
| **$28\text{ mrad}$** | $7.04$ | $0.750$ | $131$ | $94.50\%$ | $58$ | $11.39$ |
| **$32\text{ mrad}$ ($q$)** | $7.82$ | $0.543$ | $46$ | $97.12\%$ | $23$ | $11.39$ |

## 6. Offline Policy Comparison Summary

| Policy | RMS($\Delta u$) [mrad] | RMS($sv$) [rad/s] | Relay Reversals | Zero-State % | Alt. Limit Cycle Streaks | 40 ms Error RMS [mrad] | 40 ms Error Max [mrad] |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Policy R (Raw)** | $5.67$ | $3.179$ | $2925$ | $1.31\%$ | $232$ | $11.39$ | $31.89$ |
| **Policy H ($16\text{ mrad}$)** | $5.02$ | $1.275$ | $464$ | $84.12\%$ | $39$ | $11.39$ | $31.89$ |
| **Policy L (Lattice)** | $7.34$ | $0.367$ | $0$ | $98.69\%$ | $0$ | $6.95$ | $15.97$ |
| **Policy HL ($16\text{ mrad}$)** | $7.34$ | $0.367$ | $0$ | $98.69\%$ | $0$ | $6.95$ | $15.97$ |
| **Policy C (Continuous)** | $9.77$ | $0.480$ | $18$ | $97.75\%$ | $0$ | $11.49$ | $90.45$ |

## 7. Genuine Closed-Loop Benchmark Evaluation (Spielberg, 26.0s)

| Metric | RAW (Policy R) | HOLD_16 (Policy H) | HOLD_LATTICE_16 (Policy HL) | CONTINUOUS_C (Policy C) |
| :--- | :---: | :---: | :---: | :---: |
| **Command RMS** | $2.30^\circ$ | $2.30^\circ$ | $2.30^\circ$ | $2.40^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $5.98\text{ mrad}$ | $7.31\text{ mrad}$ | $12.05\text{ mrad}$ | $15.76\text{ mrad}$ |
| **Actuator Rate RMS ($\dot{\delta}$)** | $3.173\text{ rad/s}$ | $1.938\text{ rad/s}$ ($-38.9\%$) | **$0.602\text{ rad/s}$ ($-81.0\%$)** | $1.065\text{ rad/s}$ ($-66.4\%$) |
| **Actuator Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **$2\text{--}4\text{ Hz}$ Yaw Energy** | $25.45$ | **$15.52$ ($-39.0\%$)** | $16.07$ ($-36.9\%$) | $50.43$ ($+98.2\%$) |
| **$5\text{--}7\text{ Hz}$ Yaw Energy** | $17.14$ | $10.33$ ($-39.8\%$) | **$8.07$ ($-52.9\%$)** | $15.95$ ($-6.9\%$) |
| **$4\text{--}8\text{ Hz}$ Yaw Energy** | $23.00$ | **$9.02$ ($-60.8\%$)** | $9.81$ ($-57.3\%$) | $17.65$ ($-23.3\%$) |
| **Dominant Frequency** | $0.111\text{ Hz}$ | $0.111\text{ Hz}$ | $0.111\text{ Hz}$ | $0.111\text{ Hz}$ |
| **Lateral Tracking RMSE** | **$3.03\text{ cm}$** | $3.59\text{ cm}$ ($+5.6\text{ mm}$) | $3.35\text{ cm}$ ($+3.2\text{ mm}$) | $4.34\text{ cm}$ ($+13.1\text{ mm}$) |
| **Maximum Lateral Error** | $12.96\text{ cm}$ | $13.05\text{ cm}$ | **$11.57\text{ cm}$** | $12.27\text{ cm}$ |
| **Heading Tracking RMSE** | **$0.95^\circ$** | $0.96^\circ$ | **$0.95^\circ$** | $1.35^\circ$ |
| **Minimum LiDAR Clearance** | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ | $0$ |
| **Lap Completion** | Yes ($147.4\text{ m}$) | Yes ($147.3\text{ m}$) | Yes ($147.4\text{ m}$) | Yes ($147.4\text{ m}$) |
| **Relay Reversal Count** | $2345$ | $869$ ($-62.9\%$) | **$0$ ($-100.0\%$)** | $147$ ($-93.7\%$) |
| **2-Step Limit Cycles** | $2134$ | $784$ ($-63.3\%$) | **$0$ ($-100.0\%$)** | $75$ ($-96.5\%$) |
| **Mean Solve Time** | $2.11\text{ ms}$ | $1.98\text{ ms}$ | $2.02\text{ ms}$ | $1.97\text{ ms}$ |
| **Mean Policy Overhead** | $0.54\,\mu\text{s}$ | $0.95\,\mu\text{s}$ | $1.41\,\mu\text{s}$ | $3.97\,\mu\text{s}$ |

## 8. Repeatability Audit
Two independent executions of Policy RAW were performed under identical starting conditions:
- Timeseries sample identical: **`True`**
- Max numerical deviation across trajectory: **$0.00\text{ rad}$**
- Conclusion: The closed-loop simulation benchmark is strictly deterministic.

## 9. Tracking and Safety Tradeoffs
- **Policy H (Pure Hold $\epsilon_{\text{hold}} = 16\text{ mrad}$):** Reduces relay reversals by $62.9\%$ and cuts $5\text{--}7\text{ Hz}$ energy by $39.8\%$ while retaining a continuous command space. Lateral RMSE changes by less than $6\text{ mm}$ ($3.03 \to 3.59\text{ cm}$).
- **Policy HL (Hold + Lattice):** Completely halts all relay reversals ($0$ reversals) and suppresses $5\text{--}7\text{ Hz}$ energy by $52.9\%$ with maximum lateral error improving to $11.57\text{ cm}$. However, it quantizes commands onto the $32\text{ mrad}$ lattice.
- **Policy C (Continuous Optimization):** Suppresses relay reversals by $93.7\%$, but excites lower-frequency vehicle modes ($2\text{--}4\text{ Hz}$ energy doubles to $50.43$) due to alternating between hold and raw command tracking, resulting in higher lateral RMSE ($4.34\text{ cm}$).

## 10. Scientific Interpretation & Causal Limitations
1. **Actuator Mechanism Demonstrated:** The un-damped $3.2\text{ rad/s}$ relay with a $10\text{ ms}$ timestep has a rigid $32\text{ mrad}$ integration quantum. Whenever commands request sub-quantum corrections, the relay must overshoot, generating rapid $50\text{ Hz}$ chattering.
2. **Controller/Actuator Interaction:** The $25\text{ Hz}$ MPC controller aliases this $50\text{ Hz}$ chattering into subharmonic $5.5\text{--}6.6\text{ Hz}$ modes because the control cycle is $40\text{ ms}$.
3. **Causal Attribution Limitation:** While eliminating relay chatter reduces yaw rate $5\text{--}7\text{ Hz}$ energy by up to $52.9\%$, it does not completely eliminate all vehicle lateral motion. Steering jitter is a coupled closed-loop interaction between the discontinuous actuator dynamics and the state-feedback path.

## 11. Recommendation for Experiment 22
Formulate **Experiment 22** as an isolated single-parameter production test evaluating **Policy H (Continuous Hold with $\epsilon_{\text{hold}} = 16\text{ mrad}$)** versus **Policy HL (Hold + Lattice)** across multi-circuit benchmarks (Spielberg, Austin, Monza) to establish track generalization before adopting any permanent controller change.
