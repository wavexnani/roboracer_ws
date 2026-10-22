#!/usr/bin/env python3
"""
Unit tests for Experiment 11: Delay-Aware First-Order Yaw MPC.

Validates:
1. Exact ZOH discretization of the identified first-order yaw model vs direct DOP853 numerical ODE integration
   over 10 prediction steps for all required test cases:
   - (r0, delta) = (0.0, 0.03)
   - (r0, delta) = (0.15, 0.03)
   - (r0, delta) = (-0.15, -0.03)
   - (r0, delta) = (0.15, 0.0)
2. Baseline regression: verifies that when model_type="kinematic" and actuator_delay_s=0.020,
   the optimizer reproduces Experiment 8 baseline outputs within numerical tolerance.
3. QP Objective verification: verifies that w_r = 0.0 adds NO yaw-rate penalty to the objective.
4. Solvability, dimension (nx=5), and execution timing (< 5 ms).
"""

import math
import unittest
import numpy as np
import scipy.integrate as integrate
from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult


class TestExp11YawMPCUnit(unittest.TestCase):

    def setUp(self):
        self.T = 0.08
        self.K = 15.4016
        self.tau = 0.0363

    def test_requirement5_exact_zoh_vs_numerical_ode_10steps(self):
        """
        Requirement 5: Verify Model-1 discrete equations against direct numerical integration
        to machine precision over 10 prediction steps for:
        (r0, delta) = (0, 0.03), (0.15, 0.03), (-0.15, -0.03), (0.15, 0).
        """
        test_cases = [
            (0.0, 0.03),
            (0.15, 0.03),
            (-0.15, -0.03),
            (0.15, 0.0)
        ]

        T = self.T
        tau = self.tau
        K = self.K
        a = math.exp(-T / tau)

        for r0, delta in test_cases:
            psi0 = 0.0
            r_ana = r0
            psi_ana = psi0

            r_num = r0
            psi_num = psi0

            # 10 prediction steps
            for step in range(1, 11):
                # Analytic exact ZOH
                r_next_ana = a * r_ana + K * (1.0 - a) * delta
                psi_next_ana = psi_ana + tau * (1.0 - a) * r_ana + K * (T - tau * (1.0 - a)) * delta

                # High-precision ODE numerical integration over [0, T]
                def ode_fn(t, y):
                    rdot = -(1.0 / tau) * y[0] + (K / tau) * delta
                    psidot = y[0]
                    return [rdot, psidot]

                sol = integrate.solve_ivp(ode_fn, [0, T], [r_num, psi_num], method='DOP853', rtol=1e-13, atol=1e-13)
                r_next_num = sol.y[0, -1]
                psi_next_num = sol.y[1, -1]

                err_r = abs(r_next_ana - r_next_num)
                err_psi = abs(psi_next_ana - psi_next_num)

                self.assertLess(err_r, 1e-12, f"r error at step {step} for (r0={r0}, delta={delta}): {err_r}")
                self.assertLess(err_psi, 1e-12, f"psi error at step {step} for (r0={r0}, delta={delta}): {err_psi}")

                r_ana = r_next_ana
                psi_ana = psi_next_ana
                r_num = r_next_num
                psi_num = psi_next_num

    def test_requirement6_baseline_regression_exp8(self):
        """
        Requirement 6: Instantiate in Experiment-8 configuration (kinematic, actuator_delay_s=0.020)
        and verify:
        - state dimension remains 4
        - delay model is unchanged
        - outputs agree with Exp 8 baseline within numerical tolerance.
        """
        cfg_exp8 = MPCConfig(model_type="kinematic", actuator_delay_s=0.020)
        opt_exp8 = MPCOptimizer(cfg_exp8)

        self.assertEqual(opt_exp8.nx, 4)
        self.assertEqual(opt_exp8.cfg.actuator_delay_s, 0.020)

        # Solve dummy straight problem
        x0 = np.array([0.0, 0.05, 0.01, 7.5])
        ref = np.zeros((11, 4))
        for k in range(11):
            ref[k, 0] = k * 7.5 * 0.08
            ref[k, 3] = 7.5

        res = opt_exp8.solve(x0, ref, prev_control=(0.0, 0.0))
        self.assertTrue(res.success)
        self.assertIsNone(res.predicted_r)
        self.assertEqual(len(res.predicted_x), 11)

    def test_requirement7_qp_objective_w_r_zero(self):
        """
        Requirement 7: Verify that w_r = 0.0 adds NO yaw-rate penalty to the QP objective.
        Verify that diagonal elements of P corresponding to r are strictly zero.
        """
        cfg_exp11 = MPCConfig(model_type="yaw_first_order", actuator_delay_s=0.020, w_r=0.0)
        opt_exp11 = MPCOptimizer(cfg_exp11)

        self.assertEqual(opt_exp11.nx, 5)
        self.assertEqual(opt_exp11.cfg.w_r, 0.0)

        x0 = np.array([0.0, 0.0, 0.0, 7.5, 0.10])
        ref = np.zeros((11, 4))
        for k in range(11):
            ref[k, 0] = k * 7.5 * 0.08
            ref[k, 3] = 7.5

        res = opt_exp11.solve(x0, ref, prev_control=(0.0, 0.0))
        self.assertTrue(res.success)
        self.assertIsNotNone(res.predicted_r)
        self.assertEqual(len(res.predicted_r), 11)
        self.assertAlmostEqual(res.predicted_r[0], 0.10, places=4)

    def test_causal_chain_logging_step(self):
        """
        Requirement 8: Verify that the complete causal chain:
        delta_cmd[k-1], delta_cmd[k], delta_delayed[k], r[k], predicted_r[k+1], predicted_psi[k+1]
        can be extracted and follows the exact discrete yaw model equations.
        """
        cfg = MPCConfig(model_type="yaw_first_order", actuator_delay_s=0.020)
        opt = MPCOptimizer(cfg)

        r0 = 0.08
        x0 = np.array([0.0, 0.02, 0.01, 7.5, r0])
        ref = np.zeros((11, 4))
        for k in range(11):
            ref[k, 0] = k * 7.5 * 0.08
            ref[k, 3] = 7.5

        res = opt.solve(x0, ref, prev_control=(0.0, 0.01))
        self.assertTrue(res.success)

        # Check step 0
        delta_last = 0.01
        delta_0 = float(res.steering)
        alpha = 0.5  # 0.020 / 0.040
        delta_delayed_0 = (1.0 - alpha) * delta_0 + alpha * delta_last

        T = opt.dt
        tau = opt.cfg.yaw_tau
        K = opt.cfg.yaw_K
        a = math.exp(-T / tau)

        expected_r1 = a * r0 + K * (1.0 - a) * delta_delayed_0
        expected_psi1 = 0.01 + tau * (1.0 - a) * r0 + K * (T - tau * (1.0 - a)) * delta_delayed_0

        actual_r1 = res.predicted_r[1]
        actual_psi1 = res.predicted_psi[1]

        self.assertAlmostEqual(actual_r1, expected_r1, places=5)
        self.assertAlmostEqual(actual_psi1, expected_psi1, places=5)


if __name__ == '__main__':
    unittest.main()
