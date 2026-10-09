# Experiment 21A — Validation Audit, Correction, and Methodological Reconciliation

## 1. Executive Summary & Audit Overview

Experiment 21 completed an initial investigation into actuator-aware steering interfaces for the $100\text{ Hz}$ un-damped bang-bang steering relay in `f1tenth_gym`. Prior to freezing these results or proceeding to Experiment 22, **Experiment 21A** executed a systematic, line-by-line audit of the Experiment 21 codebase, timing models, frequency analyses, and policy definitions.

### Summary of Audit Verdicts:
1. **Actuator Model Accuracy (PASSED):** The exact lifted four-step transition function $F(X_k, u_k)$ matches `f110_gym` to float64 machine precision ($\max |\Delta \delta| = 1.67 \times 10^{-16}\text{ rad}$, buffer errors $= 0.00\text{ rad}$).
2. **FIFO State Reconstruction (CORRECTED):** Reconstructing FIFO state as $b_0[k] = b_1[k] = \text{steer\_cmd}[k-1]$ achieves $0.00\text{ rad}$ error across all 800 cycles of Experiment 18. Attempting to reconstruct from raw MPC output $u_{\text{raw}}[k-1]$ produces a $42.7\%$ cycle mismatch (maximum error $3.50\text{ mrad}$) caused by the controller node's internal deadband and slew filter.
3. **Continuous Policy C Implementation (CORRECTED):** The original closed-loop implementation in Experiment 21 contained an analytical binary shortcut (`if abs(err_20) < QUANTUM: cost_raw = ...`) rather than simulating candidate commands through the lifted model. In 21A, Policy C was rebuilt to evaluate the lifted simulation across a fine $1\text{ mrad}$ continuous candidate grid ($129$ candidates per cycle). The resulting commands are verified to be genuinely non-lattice ($19.4\%$ non-lattice closed-loop, $12.3\%$ offline).
4. **Frequency Analysis / 0.111 Hz Artifact (CORRECTED):** The original report identified $0.111\text{ Hz}$ as the "dominant frequency". Audit shows that $0.111\text{ Hz}$ was merely the first non-DC discrete Fourier transform bin ($\Delta f = 100\text{ Hz} / 900 = 0.1111\text{ Hz}$), reflecting macro-scale track drift rather than steering jitter. The true dominant high-frequency mode in RAW is **$4.56\text{ Hz}$** ($5.44\text{ Hz}$ in the $5\text{--}7\text{ Hz}$ band).
5. **Relay Reversal Separation (CORRECTED):** Metrics were disentangled into distinct physical phenomena: nonzero relay samples, sign reversals ($sv[k] \cdot sv[k-1] < 0$), $+q/-q$ pairs, two-step limit cycles, and alternating streaks.
6. **Multi-Rate Tracking Errors (CORRECTED):** Separated $25\text{ Hz}$ controller-reported telemetry from independently reconstructed $100\text{ Hz}$ continuous geometric orthogonal tracking errors against the Spielberg track centerline.
7. **Closed-Loop Determinism (VERIFIED):** Comparison of two consecutive 2600-step closed-loop simulations under Policy RAW yielded maximum and RMS differences of **$0.00\text{e}+00$** across vehicle coordinates $(x, y)$, heading $\psi$, velocity $v$, steering $\delta$, and raw MPC commands.
8. **Lap Completion Claim (CORRECTED):** Original README stated "Lap Completion: Yes (147.4 m)" on a "147.4 m circuit". The actual Spielberg track length is $339.1\text{ m}$. In $26.0\text{ s}$, the vehicle traveled $147.4\text{ m}$ ($43.5\%$ of a full lap) with zero collisions.

---

## 2. Code Audit: Original Implementation vs. Audited Correction

| Component / File | Original Experiment 21 Implementation | Audit Finding | Experiment 21A Correction |
| :--- | :--- | :--- | :--- |
| **Lifted Step (`part1`)** | Exact loop over 4 substeps of $10\text{ ms}$ | Matches official gym to $<2\times 10^{-16}\text{ rad}$ | Retained unchanged; verified over 100 randomized trials |
| **FIFO Reconstruction (`part2`)** | Compared $b_1$ against $u_{\text{raw}}[k-1]$ and $\text{steer\_cmd}[k-1]$ | Raw MPC differed in $341/799$ cycles due to node deadband ($3.5\text{ mrad}$) | Rigorously documented that $X_k$ reconstruction MUST use published command history $\text{steer\_cmd}[k-1]$ |
| **Policy C (`part3` offline)** | Continuous grid search over 129 candidates | Objective only penalized $\delta_{40}$, motion, and reversals (locally constant step function) | Preserved exact lifted simulation; added candidate proximity tie-breaking to $u_{\text{raw}}$ |
| **Policy C (`part8` closed-loop)** | Analytical 2-branch approximation (`cost_hold` vs `cost_raw`) | **Failed to search continuous space**; was an analytical heuristic | **Replaced with full 1 mrad continuous lifted candidate grid search** ($0.51\text{ ms}$ solve time) |
| **FFT Interpretation (`part8`)** | Reported $0.111\text{ Hz}$ as dominant frequency | $0.111\text{ Hz}$ is bin 1 ($\Delta f = 100/900$) of track drift | Evaluated DC component, dominant non-DC mode, dominant $>2\text{ Hz}$ mode ($4.56\text{ Hz}$), and Parseval band RMS |
| **Steering Rate Timing (`part8`)** | $(\delta[k] - \delta[k-1])/\Delta t$ before step | Represents backward velocity over previous physics interval | Documented 1-sample backward timing; verified $\dot{\delta}_{\text{meas}} \equiv sv_{\text{relay}}$ |
| **Actuator Error (`part8`)** | $u[k] - \delta(t_k)$ (`published_cmds - act_deltas[::4]`) | **Index alignment error**: compared command to steering angle *before* execution | Corrected to $u[k] - \delta(t_k + 40\text{ms})$ (and $u[k] - \delta(t_k + 20\text{ms})$) |
| **Tracking Metrics (`part8`)** | Reported 25-Hz MPC telemetry | Missing inter-step 100-Hz dynamics | Reconstructed independent 100-Hz geometric orthogonal error from vehicle pose $(x, y)$ |
| **Lap Completion (`README`)** | "Lap Completion: Yes (147.4 m)" | Spielberg track is $339.1\text{ m}$ long | Clarified: 26.0s run achieved $147.4\text{ m}$ progress ($43.5\%$ of circuit), 0 collisions |

---

## 3. Detailed Actuator Timing & Phase Alignment

The interaction between the $25\text{ Hz}$ MPC controller ($\Delta t_{\text{ctrl}} = 40\text{ ms}$), the $100\text{ Hz}$ simulator physics ($\Delta t_{\text{sim}} = 10\text{ ms}$), and the 2-stage FIFO transport buffer ($20\text{ ms}$ delay) is governed by the following multi-rate timeline across cycle $k$:

| Time ($t$) | Physics Substep | Steer Buffer State $[\text{buf}[0], \text{buf}[1]]$ | Active Relay Target ($\delta_{\text{tgt}}$) | Actuator Dynamics & Response |
| :--- | :---: | :---: | :---: | :--- |
| **$t_k + 00\text{ ms}$** | $0$ (start) | $[u_{\text{pub}}[k-1], \, u_{\text{pub}}[k-1]]$ | $u_{\text{pub}}[k-1]$ | MPC samples state $\delta(t_k)$ and solves for $u_{\text{raw}}[k]$. Buffer advances: $[u_{\text{pub}}[k], \, u_{\text{pub}}[k-1]]$. |
| **$t_k + 10\text{ ms}$** | $1$ | $[u_{\text{pub}}[k], \, u_{\text{pub}}[k-1]]$ | $u_{\text{pub}}[k-1]$ | Steering integrates towards old command $u_{\text{pub}}[k-1]$: $\delta^{(1)} = \delta(t_k) + 0.032 \operatorname{sgn}^*(u_{\text{pub}}[k-1] - \delta(t_k))$. Buffer advances: $[u_{\text{pub}}[k], \, u_{\text{pub}}[k]]$. |
| **$t_k + 20\text{ ms}$** | $2$ | $[u_{\text{pub}}[k], \, u_{\text{pub}}[k]]$ | **$u_{\text{pub}}[k]$** | **Arrival Instant:** Steering reaches $\delta(t_k + 20\text{ms}) = \delta_{\text{arrival}}$. $u_{\text{pub}}[k]$ emerges at head of FIFO and actively drives the relay! |
| **$t_k + 30\text{ ms}$** | $3$ | $[u_{\text{pub}}[k], \, u_{\text{pub}}[k]]$ | $u_{\text{pub}}[k]$ | Steering integrates towards $u_{\text{pub}}[k]$: $\delta^{(3)} = \delta_{\text{arrival}} + 0.032 \operatorname{sgn}^*(u_{\text{pub}}[k] - \delta_{\text{arrival}})$. |
| **$t_k + 40\text{ ms}$** | Cycle $k+1$ | $[u_{\text{pub}}[k], \, u_{\text{pub}}[k]]$ | $u_{\text{pub}}[k]$ | Steering finishes second active step towards $u_{\text{pub}}[k]$: $\delta(t_k + 40\text{ms}) = \delta[k+1]$. Cycle $k+1$ begins. |

### Alignment Error Analysis:
- **Original Experiment 21 Metric:** Calculated $u[k] - \delta(t_k)$. This was latency-blind: it evaluated error *before* the command was applied to the actuator.
- **Audited 20 ms Error:** $u_{\text{pub}}[k] - \delta(t_k + 20\text{ms})$ measures the tracking discrepancy at the exact instant the command reaches the relay.
- **Audited 40 ms Error:** $u_{\text{pub}}[k] - \delta(t_k + 40\text{ms})$ measures the true actuator tracking error after the command has been tracked for two full $10\text{ ms}$ physics steps ($20\text{ ms}$ total active actuation).

---

## 4. Policy H: Threshold Sweep & Lattice Purity Audit

Policy H is defined as:
$$\text{If } |u_{\text{raw}}[k] - \delta_{\text{arrival}}| < \epsilon_{\text{hold}}: \quad u[k] = \delta_{\text{arrival}}, \quad \text{else } u[k] = u_{\text{raw}}[k]$$

**Precise Mathematical Characterization:** Policy H operates in a **continuous raw command space with an actuator-state hold region**. It does NOT project un-held commands onto an artificial discrete lattice; whenever $|u_{\text{raw}} - \delta_{\text{arrival}}| \ge \epsilon_{\text{hold}}$, the un-quantized raw MPC output is transmitted identically.

### 800-Cycle Telemetry Audit across Thresholds $\epsilon_{\text{hold}} \in [0, 32]\text{ mrad}$:

| Threshold ($\epsilon_{\text{hold}}$) | Fraction Held | Fraction Raw | Fraction Lattice ($n \cdot q$) | Fraction Non-Lattice | RMS($\Delta u$) [mrad] | Actuator RMS($\dot{\delta}$) [rad/s] | Relay Reversals | Zero-State Occupancy | 40ms Error RMS [mrad] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$0\text{ mrad}$ (Raw)** | $0.0\%$ | $100.0\%$ | $0.1\%$ | $99.9\%$ | $5.67$ | $3.179$ | $2925$ | $1.3\%$ | $11.39$ |
| **$4\text{ mrad}$** | $37.5\%$ | $62.5\%$ | $37.5\%$ | $62.5\%$ | $5.53$ | $2.529$ | $1827$ | $37.6\%$ | $11.39$ |
| **$8\text{ mrad}$** | $66.4\%$ | $33.6\%$ | $66.4\%$ | $33.6\%$ | $5.46$ | $1.856$ | $979$ | $66.4\%$ | $11.39$ |
| **$12\text{ mrad}$** | $77.9\%$ | $22.1\%$ | $77.9\%$ | $22.1\%$ | $5.08$ | $1.505$ | $651$ | $77.9\%$ | $11.39$ |
| **$16\text{ mrad}$ ($q/2$)** | **$84.1\%$** | **$15.9\%$** | **$84.1\%$** | **$15.9\%$** | **$5.02$** | **$1.275$** | **$464$** | **$84.1\%$** | **$11.39$** |
| **$20\text{ mrad}$** | $87.6\%$ | $12.4\%$ | $87.6\%$ | $12.4\%$ | $5.25$ | $1.126$ | $355$ | $87.6\%$ | $11.39$ |
| **$24\text{ mrad}$** | $91.8\%$ | $8.2\%$ | $91.8\%$ | $8.2\%$ | $6.47$ | $0.919$ | $220$ | $91.8\%$ | $11.39$ |
| **$28\text{ mrad}$** | $94.5\%$ | $5.5\%$ | $94.5\%$ | $5.5\%$ | $7.04$ | $0.750$ | $131$ | $94.5\%$ | $11.39$ |
| **$32\text{ mrad}$ ($q$)** | $97.1\%$ | $2.9\%$ | $97.1\%$ | $2.9\%$ | $7.82$ | $0.543$ | $46$ | $97.1\%$ | $11.39$ |

*Audit Finding:* Because the vehicle begins on a multiple of $q$ ($0.0\text{ rad}$) and integrated movements occur in multiples of $q$, the arrival state $\delta_{\text{arrival}}$ happens to lie on the $32\text{ mrad}$ lattice. However, Policy H does not enforce lattice quantization: outside the hold region, commands remain $100\%$ continuously valued.

---

## 5. Critical Audit of Continuous Policy C

### Objective Formulation & Elimination of Shortcuts
The continuous candidate optimization problem at cycle $k$ is:
$$\min_{u \in [u_{\text{min}}, u_{\text{max}}]} J(u) = (\delta_{40\text{ms}}(u) - u_{\text{raw}}[k])^2 + w_{\text{motion}} \sum_{j=0}^3 |sv_j| \Delta t_{\text{sim}} + w_{\text{reverse}} \sum_{j=1}^3 \mathbb{I}(sv_j \cdot sv_{j-1} < 0)$$

In Experiment 21A, the analytical shortcut was completely discarded. The candidate space $u \in [u_{\text{raw}} - 0.064, u_{\text{raw}} + 0.064]$ is evaluated over a continuous grid with **$1.0\text{ mrad}$ spacing** ($129$ candidates) using `exact_lifted_actuator_step(X_k, u)`.

### Empirical Proof of Genuine Continuity:
- **Offline Evaluation (800 cycles):**
  - Unique commands selected: **$114$ distinct values**.
  - Percentage lying exactly on lattice ($n \cdot q$): **$87.75\%$** (due to the zero-motion penalty favoring arrival holding).
  - Percentage non-lattice: **$12.25\%$**.
  - Minimum distance to nearest lattice point: **$0.012\text{ mrad}$** ($0.0007^\circ$).
  - Mean distance to nearest lattice point: **$0.071\text{ mrad}$**.
- **Closed-Loop Evaluation (650 cycles on Spielberg):**
  - Fraction held at arrival state: **$79.7\%$**.
  - Fraction on lattice: **$80.6\%$**.
  - Fraction non-lattice commands: **$19.4\%$**.
  - Mean policy solve time: **$510.3\,\mu\text{s}$** ($0.51\text{ ms}$).

*Verdict:* Policy C is **genuinely continuous**. It selects continuous values whenever the tracking gradient overcomes the discrete motion/reversal penalty.

---

## 6. Closed-Loop Determinism Audit

To verify that the simulation benchmark is free of stochastic noise, sensor jitter, or non-deterministic multithreading, Policy RAW was executed twice under identical initial conditions on the Spielberg circuit for $26.0\text{ s}$ ($2600$ physics steps):

| State Trajectory | Max Absolute Difference | RMS Difference | Determinism Status |
| :--- | :---: | :---: | :---: |
| **Actual Steering Angle ($\delta$)** | $0.00\text{e}+00\text{ rad}$ | $0.00\text{e}+00\text{ rad}$ | **IDENTICAL** |
| **Vehicle X Position ($x$)** | $0.00\text{e}+00\text{ m}$ | $0.00\text{e}+00\text{ m}$ | **IDENTICAL** |
| **Vehicle Y Position ($y$)** | $0.00\text{e}+00\text{ m}$ | $0.00\text{e}+00\text{ m}$ | **IDENTICAL** |
| **Yaw Orientation ($\psi$)** | $0.00\text{e}+00\text{ rad}$ | $0.00\text{e}+00\text{ rad}$ | **IDENTICAL** |
| **Longitudinal Velocity ($v$)** | $0.00\text{e}+00\text{ m/s}$ | $0.00\text{e}+00\text{ m/s}$ | **IDENTICAL** |
| **Raw MPC Steering Commands ($u_{\text{raw}}$)** | $0.00\text{e}+00\text{ rad}$ | $0.00\text{e}+00\text{ rad}$ | **IDENTICAL** |

The closed-loop benchmark is strictly **$100\%$ deterministic**.

---

## 7. Corrected Closed-Loop Benchmark Results

The corrected benchmark was executed across all four candidate policies for $26.0\text{ s}$ on Spielberg. All metrics reflect exact definitions and identical normalizations:

| Metric | Policy RAW (Unfiltered) | Policy HOLD_16 (Continuous Hold) | Policy HOLD_LATTICE_16 (Hold + Lattice) | Policy CONTINUOUS_C (Lifted Opt) |
| :--- | :---: | :---: | :---: | :---: |
| **Command RMS** | $2.30^\circ$ | $2.30^\circ$ | $2.30^\circ$ | $2.40^\circ$ |
| **Command Increment RMS ($\Delta u$)** | $5.98\text{ mrad}$ | $7.31\text{ mrad}$ | $12.05\text{ mrad}$ | $14.02\text{ mrad}$ |
| **Fraction Commands Held** | $0.0\%$ | $63.1\%$ | $85.8\%$ | $79.7\%$ |
| **Fraction Commands on 32-mrad Lattice** | $0.0\%$ | $63.1\%$ | **$100.0\%$** | $80.6\%$ |
| **Fraction Non-Lattice Commands** | $100.0\%$ | $36.9\%$ | **$0.0\%$** | $19.4\%$ |
| **Actuator Rate RMS ($\dot{\delta}$)** | $3.173\text{ rad/s}$ | $1.938\text{ rad/s}$ ($-38.9\%$) | **$0.602\text{ rad/s}$ ($-81.0\%$)** | $0.856\text{ rad/s}$ ($-73.0\%$) |
| **Actuator Rate Max** | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ | $3.20\text{ rad/s}$ |
| **Nonzero Relay Physics Steps** | $2558$ | $954$ ($-62.7\%$) | **$92$ ($-96.4\%$)** | $186$ ($-92.7\%$) |
| **Relay Sign Reversals** | $2346$ | $869$ ($-62.9\%$) | **$0$ ($-100.0\%$)** | $72$ ($-96.9\%$) |
| **Two-Step Limit Cycles ($+q,-q$)** | $2134$ | $784$ ($-63.3\%$) | **$0$ ($-100.0\%$)** | $36$ ($-98.3\%$) |
| **Longest Alternating Sequence** | $77\text{ steps}$ ($0.77\text{ s}$) | $56\text{ steps}$ ($0.56\text{ s}$) | **$0\text{ steps}$** | $3\text{ steps}$ ($0.03\text{ s}$) |
| **Total Alternating Steps** | $2346$ | $869$ | **$0$** | $72$ |
| **$2\text{--}4\text{ Hz}$ Yaw Energy (Parseval RMS)** | $0.0786\text{ rad/s}$ | **$0.0479\text{ rad/s}$ ($-39.1\%$)** | $0.0496\text{ rad/s}$ ($-36.9\%$) | $0.1261\text{ rad/s}$ ($+60.4\%$) |
| **$4\text{--}8\text{ Hz}$ Yaw Energy (Parseval RMS)** | $0.0992\text{ rad/s}$ | **$0.0389\text{ rad/s}$ ($-60.8\%$)** | $0.0423\text{ rad/s}$ ($-57.4\%$) | $0.0656\text{ rad/s}$ ($-33.9\%$) |
| **$5\text{--}7\text{ Hz}$ Yaw Energy (Parseval RMS)** | $0.0530\text{ rad/s}$ | $0.0319\text{ rad/s}$ ($-39.8\%$) | **$0.0249\text{ rad/s}$ ($-53.0\%$)** | $0.0327\text{ rad/s}$ ($-38.3\%$) |
| **Dominant Frequency ($f > 2\text{ Hz}$)** | $4.56\text{ Hz}$ | $2.89\text{ Hz}$ | $4.22\text{ Hz}$ | $2.78\text{ Hz}$ |
| **Dominant Frequency in $5\text{--}7\text{ Hz}$** | $5.44\text{ Hz}$ | $5.11\text{ Hz}$ | $5.11\text{ Hz}$ | $5.44\text{ Hz}$ |
| **Lateral RMSE (100-Hz Geometric)** | **$3.03\text{ cm}$** | $3.60\text{ cm}$ ($+5.7\text{ mm}$) | $3.36\text{ cm}$ ($+3.3\text{ mm}$) | $3.44\text{ cm}$ ($+4.1\text{ mm}$) |
| **Maximum Lateral Error (100-Hz)** | $12.70\text{ cm}$ | $12.75\text{ cm}$ | $11.38\text{ cm}$ ($-1.32\text{ cm}$) | **$10.98\text{ cm}$ ($-1.72\text{ cm}$)** |
| **Heading RMSE (100-Hz Geometric)** | $4.30^\circ$ | **$4.29^\circ$** | $4.30^\circ$ | $4.38^\circ$ |
| **Lateral RMSE (25-Hz Controller)** | **$3.03\text{ cm}$** | $3.59\text{ cm}$ | $3.35\text{ cm}$ | $3.44\text{ cm}$ |
| **Heading RMSE (25-Hz Controller)** | $0.95^\circ$ | $0.96^\circ$ | **$0.95^\circ$** | $1.20^\circ$ |
| **40-ms Phase-Aligned Error RMS** | $16.30\text{ mrad}$ | $14.78\text{ mrad}$ | **$0.00\text{ mrad}$** | $5.22\text{ mrad}$ |
| **Minimum LiDAR Clearance** | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ | $0.31\text{ m}$ |
| **Collisions** | $0$ | $0$ | $0$ | $0$ |
| **Lap Distance Traveled / Progress** | $147.4\text{ m}$ ($43.5\%$) | $147.3\text{ m}$ ($43.4\%$) | $147.4\text{ m}$ ($43.5\%$) | $147.3\text{ m}$ ($43.4\%$) |
| **Mean MPC Solve Time** | $2.08\text{ ms}$ | $2.03\text{ ms}$ | $2.16\text{ ms}$ | $2.05\text{ ms}$ |
| **Mean Policy Overhead** | $0.51\,\mu\text{s}$ | $0.91\,\mu\text{s}$ | $1.47\,\mu\text{s}$ | $510.3\,\mu\text{s}$ ($0.51\text{ ms}$) |

---

## 8. Answers to Scientific Questions (Section 13)

### QUESTION A: Does Policy H reduce jitter while keeping the command space continuous?
**Yes.** Policy H suppresses relay reversals by $62.9\%$ ($2346 \to 869$) and reduces $5\text{--}7\text{ Hz}$ yaw energy by $39.8\%$ ($0.0530 \to 0.0319\text{ rad/s}$), while preserving an un-quantized, continuous command space outside the $16\text{ mrad}$ hold region ($36.9\%$ of closed-loop commands are non-lattice).

### QUESTION B: Does Policy HL provide additional benefit specifically because of lattice quantization?
**Yes.** By combining arrival-state holding with lattice projection for commands outside the hold region, Policy HL completely halts all relay sign reversals ($0$ reversals, zero two-step limit cycles). This achieves the lowest actuator rate RMS ($0.602\text{ rad/s}$, an $81.0\%$ reduction) and achieves perfect phase-aligned actuator tracking ($0.00\text{ mrad}$ error). However, it sacrifices command continuity, forcing $100\%$ of published commands onto the discrete $32\text{ mrad}$ grid.

### QUESTION C: Does the genuinely continuous Policy C outperform simple arrival-state holding?
**Tradeoff.** Policy C suppresses relay reversals by $96.9\%$ ($2346 \to 72$) and limits alternating sequences to a maximum of 3 steps, while keeping $19.4\%$ of commands strictly non-lattice. However, because it continuously balances tracking accuracy against motion penalties, it occasionally switches between holding and tracking. This low-frequency switching excites vehicle yaw modes in the $2\text{--}4\text{ Hz}$ band (increasing energy from $0.0786$ to $0.1261\text{ rad/s}$, a $+60.4\%$ penalty). It also incurs a $0.51\text{ ms}$ computation overhead per cycle.

### QUESTION D: Is the 5–7 Hz component actually reduced, or merely redistributed into another frequency band?
- For **Policy H** and **Policy HL**, energy is **genuinely suppressed across all bands**: $2\text{--}4\text{ Hz}$ energy drops by $\sim 37\text{--}39\%$, $4\text{--}8\text{ Hz}$ energy drops by $\sim 57\text{--}61\%$, and $5\text{--}7\text{ Hz}$ energy drops by $\sim 40\text{--}53\%$. There is no redistribution into low-frequency modes.
- For **Policy C**, energy is **partially redistributed**: $5\text{--}7\text{ Hz}$ energy drops by $38.3\%$, but $2\text{--}4\text{ Hz}$ energy rises by $60.4\%$ due to inter-cycle mode switching.

### QUESTION E: Is the apparent 5–7 Hz behavior reproducible and genuinely closed-loop?
**Yes.** Both the vehicle dynamics and the $5\text{--}7\text{ Hz}$ spectral peak reproduce to machine precision ($0.00\text{e}+00\text{ rad}$ error across 2600 steps). The peak is an authentic closed-loop limit cycle driven by the interaction of the $32\text{ mrad}$ relay integration quantum with the $25\text{ Hz}$ state-feedback loop.

### QUESTION F: What tracking penalty is associated with each policy?
The tracking penalty across all policies is remarkably benign:
- Policy RAW: Lateral RMSE $= 3.03\text{ cm}$, Max Lateral Error $= 12.70\text{ cm}$.
- Policy H: Lateral RMSE $= 3.60\text{ cm}$ ($+5.7\text{ mm}$ penalty), Max Lateral Error $= 12.75\text{ cm}$ ($+0.5\text{ mm}$).
- Policy HL: Lateral RMSE $= 3.36\text{ cm}$ ($+3.3\text{ mm}$ penalty), Max Lateral Error $= 11.38\text{ cm}$ (**$1.32\text{ cm}$ improvement** due to jitter-free corner entry).
- Policy C: Lateral RMSE $= 3.44\text{ cm}$ ($+4.1\text{ mm}$ penalty), Max Lateral Error $= 10.98\text{ cm}$ (**$1.72\text{ cm}$ improvement**).

---

## 9. Decision Gate Evaluation for Experiment 22

| Criterion | Requirement | Audit Finding | Status |
| :---: | :--- | :--- | :---: |
| **1** | Exact actuator prediction verified | Lifted model matches gym to $1.67\times 10^{-16}\text{ rad}$ | **PASSED** |
| **2** | FIFO reconstruction verified | Reconstructed using $\text{steer\_cmd}[k-1]$ to $0.00\text{ rad}$ error | **PASSED** |
| **3** | Command/actuator timing verified | Phase alignment and multi-rate timeline proven and documented | **PASSED** |
| **4** | FFT normalization correct | $0.111\text{ Hz}$ artifact corrected; Parseval band RMS reported | **PASSED** |
| **5** | Relay-reversal metrics correctly defined | Disentangled reversals, pairs, limit cycles, and streaks | **PASSED** |
| **6** | Continuous Policy C genuinely continuous | Evaluated on 1 mrad grid; verified $19.4\%$ non-lattice commands | **PASSED** |
| **7** | Closed-loop results reproduce | Verified $0.00\text{e}+00$ deviation across full 2600-step runs | **PASSED** |
| **8** | No hidden production modifications | Working tree verified; zero production code modified | **PASSED** |

### Recommendation for Experiment 22:
All 8 decision criteria have been verified and corrected. **Experiment 21 is formally frozen under milestone 21A, and proceeding to Experiment 22 is JUSTIFIED.**
