# Experiment 25B: Production Node Software-in-the-Loop Validation Walkthrough

## Summary of Accomplishments
Experiment 25B validated the integration of `SteeringPolicyFilter` directly into the production ROS 2 node (`mpc_controller_node.py`). The production node was tested in nominal Software-in-the-Loop (SIL) simulation across all 7 validated circuits for 3 complete laps under both `RAW` and `HOLD_LATTICE16` steering policies.

## Validation Results Across 7 Tracks (3 Laps Each)

| Circuit | Policy | Max Command Difference | Relay Reversals | Lateral RMSE Difference | Actuator Rate RMS Difference | Command RMS Difference | Collisions / Laps | Acceptance Verdict |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Spielberg** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0000\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **Austin** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0000\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **Monza** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0001\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **BrandsHatch** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0000\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **Silverstone** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0003\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **Catalunya** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0000\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |
| **Spa** | HOLD_LATTICE16 | $2.78 \times 10^{-17}\text{ rad}$ | 0 | $0.0000\text{ cm}$ | $0.0000\text{ rad/s}$ | $0.0000^{\circ}$ | 0 / 3 | **PASS** |

### Acceptance Criteria Verdict
- **Command Equivalence:** $\max_k |u_{\text{exp}}[k] - u_{\text{prod}}[k]| = 2.78\times 10^{-17}\text{ rad} \le 1\times 10^{-6}\text{ rad}$ on all 7 tracks (**PASS**).
- **Chatter:** 0 relay sign reversals on all 7 tracks (**PASS**).
- **Tracking:** Lateral RMSE difference $\le 0.0003\text{ cm} \le 0.050\text{ cm}$ (**PASS**).
- **Steering Rate:** Rate RMS difference $= 0.0000\text{ rad/s} \le 0.010\text{ rad/s}$ (**PASS**).
- **Command RMS:** Command RMS difference $= 0.0000^{\circ} \le 0.020^{\circ}$ (**PASS**).
- **Safety:** 0 collisions, 0 out-of-bounds, 3 completed laps per track (**PASS**).
- **Unit Tests:** 25/25 unit tests passing, including 100k-cycle differential equivalence at $10^{-12}\text{ rad}$ (**PASS**).

## Generated Artifacts
1. [`run_experiment25_sil.py`](file:///home/yeswanth/roboracer_ws/experiments/experiment25_integration/run_experiment25_sil.py)
2. [`experiment25_results.json`](file:///home/yeswanth/roboracer_ws/experiments/experiment25_integration/experiment25_results.json)
3. [`experiment25_summary.csv`](file:///home/yeswanth/roboracer_ws/experiments/experiment25_integration/experiment25_summary.csv)
4. [`EXPERIMENT_25B_REPORT.md`](file:///home/yeswanth/roboracer_ws/experiments/experiment25_integration/EXPERIMENT_25B_REPORT.md)
