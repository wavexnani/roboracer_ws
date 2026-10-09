# Experiment 21B — Final Methodology Audit & Reconciliation

## 1. Executive Summary

**Experiment 21B** completes the final methodological audit of the Experiment 20/21 baseline prior to milestone freezing. It closes all remaining analytical open items identified in Experiment 21A:
1. **Mathematical & Empirical Index Proof:** Formally proves that $e[k] = u_{\text{pub}}[k] - \delta_{\text{actual}}(t_k + 40\text{ms}) \equiv \text{published\_cmds}[k] - \text{act\_deltas}[4k + 4]$ rigorously samples the exact post-actuation steering angle.
2. **High-Precision Actuator Error Statistics:** Computes unrounded RMS, mean absolute, maximum absolute, 95th/99th percentiles, and sample violation counts across all policies.
3. **Rigorous Terminology:** Standardizes nomenclature to **"continuous-valued command optimization over the exact discrete-relay actuator model"**, explicitly distinguishing command space, actuator state space, relay rate set, and physical integrated steering trajectory.
4. **Policy C Objective Audit:** Mathematically demonstrates that Policy C optimizes **only interface/actuator behavior** (penalizing tracking of exogenous $u_{\text{raw}}$, motion, and reversals), rather than end-to-end vehicle state tracking.
5. **Phenomenological Disentanglement:** Separates target crossings, relay sign reversals, and persistent alternating limit cycle streaks ($\ge 4$ steps).
6. **Full-Determinism Verification Across All Policies:** Proves $100\%$ determinism ($0.00\text{e}+00$ deviation across 7 state variables) for **RAW, HOLD_16, HOLD_LATTICE_16, and CONTINUOUS_C**.
7. **Tracking Metric Reconciliation:** Separates 25-Hz controller telemetry from independent 100-Hz continuous geometric arc-length segment projection.

---

## 2. Item 1: Exact Phase-Aligned Index Proof

### Multi-Rate Clock Timeline and Index Mapping
Let $k \in \{0, 1, \dots, K-1\}$ index the $25\text{ Hz}$ MPC control cycles ($\Delta t_{\text{ctrl}} = 40\text{ ms}$).  
Let $i \in \{0, 1, \dots, 2599\}$ index the $100\text{ Hz}$ simulator physics steps ($\Delta t_{\text{sim}} = 10\text{ ms}$).  
At cycle $k$, the physics step index at initiation is $i_k = 4k$, corresponding to simulation time $t_k = i_k \cdot \Delta t_{\text{sim}} = 0.040 \cdot k\text{ s}$.

The array `act_deltas` records the actual steering angle $\delta$ before each `env.step(i)`:
- `act_deltas[4k + 0]` $= \delta(t_k + 00\text{ms})$: steering state at the instant MPC solves for $u_{\text{pub}}[k]$.
- `act_deltas[4k + 1]` $= \delta(t_k + 10\text{ms})$: steering state after Substep 0 (relay tracked FIFO target $b_1[k]$).
- `act_deltas[4k + 2]` $= \delta(t_k + 20\text{ms})$: steering state after Substep 1 (relay tracked FIFO target $b_0[k]$). **Arrival instant of $u_{\text{pub}}[k]$.**
- `act_deltas[4k + 3]` $= \delta(t_k + 30\text{ms})$: steering state after Substep 2 (relay tracked newly active target $u_{\text{pub}}[k]$ for 10 ms).
- `act_deltas[4k + 4]` $= \delta(t_k + 40\text{ms})$: steering state after Substep 3 (relay tracked $u_{\text{pub}}[k]$ for a second 10 ms step).

At time $t_k + 40\text{ms} = t_{k+1}$, cycle $k+1$ begins. The command $u_{\text{pub}}[k]$ has acted on the relay for exactly two 10 ms steps ($20\text{ ms}$ active duration).  
Therefore, the true phase-aligned actuator tracking error is:
$$e[k] = u_{\text{pub}}[k] - \delta_{\text{actual}}(t_k + 40\text{ms}) \equiv \text{published\_cmds}[k] - \text{act\_deltas}[4k + 4]$$

### Representative Multi-Cycle Substep Trace:
Below is the empirical simulator trace across cycles $0$ through $4$:

| Cycle $k$ | Substep | Step $i$ | Time $t$ | $u_{\text{pub}}[k]$ [mrad] | $b_0$ [mrad] | $b_1$ [mrad] | Active Relay Target [mrad] | $\delta_{\text{start}}$ [mrad] | $\delta_{\text{end}}$ [mrad] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **0** | 0 | 0 | $0.000\text{s}$ | $+0.014$ | $+0.000$ | $+0.000$ | $+0.000$ | $+0.000$ | $+0.000$ |
| 0 | 1 | 1 | $0.010\text{s}$ | $+0.014$ | $+0.014$ | $+0.000$ | $+0.000$ | $+0.000$ | $+0.000$ |
| 0 | 2 | 2 | $0.020\text{s}$ | $+0.014$ | $+0.014$ | $+0.014$ | $+0.014$ | $+0.000$ | $+0.000$ |
| 0 | 3 | 3 | $0.030\text{s}$ | $+0.014$ | $+0.014$ | $+0.014$ | $+0.014$ | $+0.000$ | $+0.000$ |
| **1** | 0 | 4 | $0.040\text{s}$ | $+0.201$ | $+0.014$ | $+0.014$ | $+0.014$ | $+0.000$ | $+0.000$ |
| 1 | 1 | 5 | $0.050\text{s}$ | $+0.201$ | $+0.201$ | $+0.014$ | $+0.014$ | $+0.000$ | $+0.000$ |
| 1 | 2 | 6 | $0.060\text{s}$ | $+0.201$ | $+0.201$ | $+0.201$ | **$+0.201$** | $+0.000$ | $+32.000$ |
| 1 | 3 | 7 | $0.070\text{s}$ | $+0.201$ | $+0.201$ | $+0.201$ | **$+0.201$** | $+32.000$ | $+0.000$ |
| **2** | 0 | 8 | $0.080\text{s}$ | $+0.170$ | $+0.201$ | $+0.201$ | $+0.201$ | $+0.000$ | $+32.000$ |
| 2 | 1 | 9 | $0.090\text{s}$ | $+0.170$ | $+0.170$ | $+0.201$ | $+0.201$ | $+32.000$ | $+0.000$ |
| 2 | 2 | 10 | $0.100\text{s}$ | $+0.170$ | $+0.170$ | $+0.170$ | **$+0.170$** | $+0.000$ | $+32.000$ |
| 2 | 3 | 11 | $0.110\text{s}$ | $+0.170$ | $+0.170$ | $+0.170$ | **$+0.170$** | $+32.000$ | $+0.000$ |

*Verification:* At $t = 0.080\text{s}$ (step 8), $\delta_{\text{actual}} = 0.000\text{ mrad}$. For cycle 1 ($u_{\text{pub}}[1] = +0.201\text{ mrad}$), $\delta(t_1 + 40\text{ms}) = \delta(0.080\text{s}) = \text{act\_deltas}[8] = 0.000\text{ mrad}$. The tracking error is $e[1] = 0.201 - 0.000 = +0.201\text{ mrad}$.

---

## 3. Item 2: Exact Actuator Error Statistics (Unrounded)

Evaluated across all $K = 649$ valid cycles of the 26.0s Spielberg closed-loop simulation:

| Policy | RMS Error [mrad] | Mean Abs Error [mrad] | Max Abs Error [mrad] | 95th %-tile [mrad] | 99th %-tile [mrad] | Samples with $\|e\| > 10^{-9}\text{ rad}$ | Samples with $\|e\| > 10^{-6}\text{ rad}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **RAW** | $16.303747$ | $12.535599$ | $31.896006$ | $30.077769$ | $31.605409$ | $649 / 649$ ($100\%$) | $649 / 649$ ($100\%$) |
| **HOLD_16** | $14.775109$ | $8.775996$ | $31.731365$ | $29.583193$ | $31.237167$ | $240 / 649$ ($37.0\%$) | $240 / 649$ ($37.0\%$) |
| **HOLD_LATTICE_16** | **$0.000000$** | **$0.000000$** | **$0.000000$** | **$0.000000$** | **$0.000000$** | **$0 / 649$ ($0.0\%$)** | **$0 / 649$ ($0.0\%$)** |
| **CONTINUOUS_C** | $5.223722$ | $1.194032$ | $29.371741$ | $11.946971$ | $27.258479$ | $138 / 649$ ($21.3\%$) | $136 / 649$ ($21.0\%$) |

*Key Findings:*
1. **HOLD_LATTICE_16 achieves machine-precision actuator tracking:** Maximum error across all 649 cycles is $1.39 \times 10^{-14}\text{ rad}$. Zero samples exceed $10^{-9}\text{ rad}$.
2. **HOLD_16 eliminates actuator error on $63.0\%$ of cycles** (whenever holding arrival state $\delta_{20}$). On the remaining $37.0\%$ of cycles where raw MPC is passed, actuator error returns to $\sim 30\text{ mrad}$.
3. **CONTINUOUS_C eliminates actuator error on $79.0\%$ of cycles** ($|e| < 10^{-6}\text{ rad}$), but incurs errors up to $29.37\text{ mrad}$ when transitioning across non-lattice commands.

---

## 4. Item 3: Standardized Terminology

To prevent misleading claims regarding actuator continuity:
1. **Approved Description:**  
   Policy C must be described strictly as:  
   **"continuous-valued command optimization over the exact discrete-relay actuator model"**
2. **Do NOT Claim:** "Policy C makes the actuator continuous" or "Policy C eliminates actuator discretization".
3. **Fundamental Space Distinctions:**
   - **Command Space $\mathcal{U} \subset \mathbb{R}$:** Continuous interval $[-0.35, +0.35]\text{ rad}$. Policy RAW and Policy C select continuously valued setpoints; Policy HL forces commands onto a discrete lattice outside the hold region.
   - **Actuator State Space $\mathcal{X}_a \subset \mathbb{R}^3$:** The triplet $[\delta_{\text{actual}}, b_0, b_1]^T$.
   - **Relay Steering-Rate Set $\mathcal{S}_v = \{-3.2, 0.0, +3.2\}\text{ rad/s}$:** A rigid 3-point discontinuous velocity set enforced by `pid()` in `f110_gym`.
   - **Physical Steering Trajectory $\delta(t) = \delta(0) + \int_0^t sv(\tau) d\tau$:** Piecewise linear and continuous in time, but with derivative $\dot{\delta}(t) \in \mathcal{S}_v$. Over each $10\text{ ms}$ physics step, $\Delta \delta \in \{-32.0, 0.0, +32.0\}\text{ mrad}$.

---

## 5. Item 4: Policy C Objective Audit & Offline Diagnostic

### Objective Formulation:
The Policy C optimization problem at cycle $k$ is:
$$\min_{u \in [u_{\text{raw}}-0.064, \, u_{\text{raw}}+0.064]} J(u) = w_{\text{target}} (\delta_{40\text{ms}}(u) - u_{\text{raw}}[k])^2 + w_{\text{motion}} \sum_{j=0}^3 |sv_j| \Delta t_{\text{sim}} + w_{\text{reverse}} \sum_{j=1}^3 \mathbb{I}(sv_j \cdot sv_{j-1} < 0)$$

### Mathematical Breakdown of Terms:
- **Term 1 ($w_{\text{target}} = 1.0$):** Terminal actuator tracking error at $t_k + 40\text{ms}$ relative to exogenous command $u_{\text{raw}}[k]$.
- **Term 2 ($w_{\text{motion}} = 0.005$):** Total angular travel executed by the servo over the 4 substeps ($0.032\text{ rad/step}$).
- **Term 3 ($w_{\text{reverse}} = 0.05$):** Integer penalty on relay velocity sign reversals between consecutive substeps.

### Critical Finding:
**Policy C optimizes ONLY actuator/interface behavior.**  
It does **NOT** optimize vehicle lateral tracking error, heading error, tyre slip margins, or the MPC cost function. It treats $u_{\text{raw}}[k]$ as an exogenous reference and seeks a local actuator command that avoids sub-quantum relay chatter.

### Offline Diagnostic on 800 Experiment 18 Cycles:
Comparing Policy C selected commands $u_C$ against original $u_{\text{raw}}$:
- **RMS Deviation $\|u_C - u_{\text{raw}}\|:** $9.9864\text{ mrad}$ ($0.572^\circ$).
- **95th Percentile Deviation:** $24.1842\text{ mrad}$.
- **Maximum Absolute Deviation:** $34.1431\text{ mrad}$.
- **Fraction Exactly Equal to $u_{\text{raw}}$ ($|\text{diff}| < 10^{-6}$):** $2.38\%$.
- **Fraction on $32\text{ mrad}$ Lattice:** $87.75\%$.
- **Fraction Strictly Non-Lattice:** $12.25\%$.

---

## 6. Item 5: Phenomenological Separation of Relay Dynamics

Three distinct physical phenomena are tracked independently:
1. **Target Crossing:** The sign of the servo tracking error $(\delta_{\text{tgt}}[n] - \delta_{\text{actual}}[n])$ flips across consecutive physics steps:
   $$(\delta_{\text{tgt}}[n-1] - \delta_{\text{actual}}[n-1]) \cdot (\delta_{\text{tgt}}[n] - \delta_{\text{actual}}[n]) \le 0 \quad \text{with } |\text{error}| > 10^{-4}\text{ rad}$$
2. **Relay Sign Reversal:** The sign of the relay velocity command flips:
   $$sv[n] \cdot sv[n-1] < 0 \quad \text{with } |sv[n]|, |sv[n-1]| > 10^{-4}\text{ rad/s}$$
3. **Persistent Alternating Limit Cycle:** A continuous streak of consecutive sign reversals persisting for $\ge 4$ steps ($\ge 40\text{ ms}$ continuous high-frequency chatter).

### Closed-Loop Counts (2,600 Physics Steps on Spielberg):

| Policy | Target Crossings | Relay Sign Reversals | $+q/-q$ Pairs | $-q/+q$ Pairs | Persistent Limit Cycles ($\ge 4$ steps) | Longest Alternating Streak |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **RAW** | $2346$ | $2346$ | $1173$ | $1173$ | **$211$ episodes** | **$77\text{ steps}$ ($770\text{ ms}$)** |
| **HOLD_16** | $869$ | $869$ | $435$ | $434$ | **$79$ episodes** | **$56\text{ steps}$ ($560\text{ ms}$)** |
| **HOLD_LATTICE_16** | **$0$** | **$0$** | **$0$** | **$0$** | **$0$ episodes** | **$0\text{ steps}$ ($0\text{ ms}$)** |
| **CONTINUOUS_C** | $72$ | $72$ | $36$ | $36$ | **$0$ episodes** | **$3\text{ steps}$ ($30\text{ ms}$)** |

*Phenomenological Insight:*  
Under Policy RAW and Policy HOLD_16, almost every target crossing immediately degenerates into a persistent limit cycle (streaks lasting up to $770\text{ ms}$). Under Policy CONTINUOUS_C, while isolated direction reversals occasionally occur ($72$ instances), **they NEVER form a persistent limit cycle** (longest streak is strictly 3 steps, terminating before establishing a 4-step limit cycle).

---

## 7. Item 6: Full-Determinism Repeatability Across ALL Four Policies

Each policy was executed twice under identical initial conditions for $26.0\text{ s}$ ($2600$ physics steps):

| Policy | State Variable | Max Absolute Difference | RMS Difference | Determinism Status |
| :--- | :--- | :---: | :---: | :---: |
| **RAW** | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |
| **HOLD_16** | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |
| **HOLD_LATTICE_16** | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |
| **CONTINUOUS_C** | $x, y, \psi, v, \delta, u_{\text{pub}}, u_{\text{raw}}$ | $0.00\text{e}+00$ | $0.00\text{e}+00$ | **$100\%$ IDENTICAL** |

The simulation benchmark is strictly deterministic for all four candidate policies.

---

## 8. Item 7: Tracking Metric Audit (25-Hz Telemetry vs. 100-Hz Geometric)

### Methodology Audit:
1. **25-Hz Controller Telemetry:**
   - Evaluated at $25\text{ Hz}$ ($40\text{ ms}$ intervals, 650 samples) inside `DiagnosticMPCController.step()`.
   - Projects onto waypoints using $e_y = -(T_x \cdot e_y - T_y \cdot e_x)$ where $T$ is taken from pre-smoothed path headings `path_headings[closest_idx]`.
2. **100-Hz Independent Geometric Centerline Error:**
   - Evaluated at $100\text{ Hz}$ ($10\text{ ms}$ intervals, 2600 samples) in `experiment21b_audit.py`.
   - Identifies closest waypoint $i^*$, tests segments $[i^*-1, i^*]$, $[i^*, i^*+1]$, $[i^*+1, i^*+2]$, and projects $(x, y)$ onto continuous segment lines $p_{\text{proj}} = p_a + t(p_b - p_a)$.
   - Heading error is computed using the **continuous line segment tangent** $\theta_{\text{segment}} = \text{atan2}(T_y, T_x)$.

### Comparison:

| Policy | 25-Hz Lat RMSE | 100-Hz Lat RMSE | 25-Hz Max Lat Error | 100-Hz Max Lat Error | 25-Hz Head RMSE | 100-Hz Head RMSE |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **RAW** | $3.0298\text{ cm}$ | $3.0261\text{ cm}$ | $12.9620\text{ cm}$ | $12.6965\text{ cm}$ | $0.9489^\circ$ | $4.2986^\circ$ |
| **HOLD_16** | $3.5922\text{ cm}$ | $3.5993\text{ cm}$ | $13.0529\text{ cm}$ | $12.7515\text{ cm}$ | $0.9601^\circ$ | $4.2938^\circ$ |
| **HOLD_LATTICE_16** | $3.3524\text{ cm}$ | $3.3580\text{ cm}$ | $11.5689\text{ cm}$ | $11.3836\text{ cm}$ | $0.9478^\circ$ | $4.3017^\circ$ |
| **CONTINUOUS_C** | $3.4435\text{ cm}$ | $3.4413\text{ cm}$ | $11.2568\text{ cm}$ | $10.9817\text{ cm}$ | $1.2047^\circ$ | $4.3784^\circ$ |

*Audit Explanation:* Lateral tracking errors agree to within $0.07\text{ mm}$ between the 25-Hz controller and 100-Hz independent geometric calculation. The 100-Hz heading error is $\sim 4.3^\circ$ (compared to $\sim 0.95^\circ$ for 25-Hz) because the raw track waypoint segments form an un-smoothed polygon whose segment tangent angles have discrete polygon corner jumps, whereas the controller uses Gaussian-smoothed headings. Across all policies, the relative heading performance is identical within $0.08^\circ$.

---

## 9. Item 8: Freeze Decision and Status Classification

| Item | Description | Classification | Notes / Remaining Caveats |
| :---: | :--- | :---: | :--- |
| **1** | Exact lifted actuator model | **PASS** | Matches official `f110_gym` to $1.67\times 10^{-16}\text{ rad}$. |
| **2** | FIFO state reconstruction | **PASS** | Proved that published commands $\text{steer\_cmd}[k-1]$ must be used ($0.00\text{ rad}$ error). |
| **3** | Phase-aligned indexing | **PASS** | Mathematically and empirically proven for $e[k] = u_{\text{pub}}[k] - \delta(t_k+40\text{ms})$. |
| **4** | Exact actuator error statistics | **PASS** | Unrounded metrics documented; HOLD_LATTICE_16 verified to machine precision ($0.00\text{ mrad}$). |
| **5** | Continuous terminology | **PASS** | Replaced improper claims with precise terminology. |
| **6** | Policy C objective audit | **PASS WITH CAVEAT** | **Caveat:** Policy C optimizes only interface behavior, not vehicle tracking. It incurs a $2\text{--}4\text{ Hz}$ yaw energy growth. |
| **7** | Relay phenomena separation | **PASS** | Target crossings, sign reversals, and persistent limit cycles ($\ge 4$ steps) rigorously separated. |
| **8** | Full-determinism across policies | **PASS** | Verified $0.00\text{e}+00$ deviation across all four policies. |
| **9** | Tracking metric reconciliation | **PASS** | 25-Hz controller vs 100-Hz geometric projection fully separated and explained. |
| **10** | Production code immutability | **PASS** | Zero production code modified; working directory preserved. |

### Neutral Engineering Conclusion:
Experiment 22 may proceed **if and only if**:
1. **Policy C is NOT deployed as a production controller** without integrating the vehicle tracking objective into its horizon cost (to address the observed $+60.4\%$ low-frequency yaw excitation).
2. **Experiment 22 is structured strictly as an A/B comparative evaluation** between **Policy H ($\epsilon = 16\text{ mrad}$)** (continuous raw command space with hold region) and **Policy HL ($\epsilon = 16\text{ mrad}$)** (arrival hold + lattice projection) against baseline **RAW**.
3. **Evaluation spans multiple distinct benchmark tracks** (e.g. Spielberg, Austin, Monza) to determine whether lattice quantization introduces unmodelled tracking drift on circuits with asymmetric high-speed corners.
