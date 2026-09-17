#!/usr/bin/env python3
"""
Experiment 25A Unit Equivalence Test Suite:
Validates that SteeringPolicyFilter produces bit-exact identical behavior
to the authoritative Experiment 24 reference implementation.
"""

import math
import sys
import os
import unittest
import numpy as np

# Ensure mpc_controller package is in sys.path
for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from mpc_controller.steering_policy import SteeringPolicyFilter


# ==============================================================================
# AUTHORITATIVE EXPERIMENT 24 REFERENCE IMPLEMENTATION
# Extracted verbatim from experiments/experiment24_robustness/run_experiment24_suite.py
# ==============================================================================
class Exp24ReferencePolicy:
    """
    Exact behavioral reference from run_experiment24_suite.py:L56-64 and L293-315.
    """
    def __init__(self, policy='HOLD_LATTICE16'):
        self.policy = policy
        self.SV_NOMINAL = 3.20
        self.DT_SIM = 0.010
        self.DT_CTRL = 0.040
        self.QUANTUM_NOMINAL = 0.032
        self.EPS_RELAY = 1e-4
        self.reset()

    def reset(self, init_val=0.0):
        self.cur_delta = float(init_val)
        self.steer_buffer = [float(init_val), float(init_val)]
        self.is_held = False
        self.last_pub = float(init_val)

    def predict_20ms_arrival(self, cur_d, b0, b1):
        # Exact copy of run_experiment24_suite.py:L56-64
        diff1 = b1 - cur_d
        sv1 = math.copysign(self.SV_NOMINAL, diff1) if abs(diff1) > self.EPS_RELAY else 0.0
        d_10 = cur_d + sv1 * self.DT_SIM
        diff2 = b0 - d_10
        sv2 = math.copysign(self.SV_NOMINAL, diff2) if abs(diff2) > self.EPS_RELAY else 0.0
        d_20 = d_10 + sv2 * self.DT_SIM
        return d_20, sv2

    def step(self, raw):
        # Exact copy of run_experiment24_suite.py:L293-315
        b0 = float(self.steer_buffer[0])
        b1 = float(self.steer_buffer[1])
        d_20, _ = self.predict_20ms_arrival(self.cur_delta, b0, b1)

        self.is_held = False
        if self.policy == 'RAW':
            cmd = raw
        elif self.policy == 'HOLD16':
            if abs(raw - d_20) < 0.016:
                cmd = d_20
                self.is_held = True
            else:
                cmd = raw
        elif self.policy == 'HOLD_LATTICE16':
            if abs(raw - d_20) < 0.016:
                cmd = d_20
                self.is_held = True
            else:
                cmd = round(raw / self.QUANTUM_NOMINAL) * self.QUANTUM_NOMINAL
        else:
            cmd = raw

        # Gym simulation of 4 sub-steps of 10 ms (40 ms control interval):
        for _ in range(4):
            tgt = self.steer_buffer[-1]
            self.steer_buffer = [cmd] + self.steer_buffer[:-1]
            diff = tgt - self.cur_delta
            sv = math.copysign(self.SV_NOMINAL, diff) if abs(diff) > self.EPS_RELAY else 0.0
            self.cur_delta += sv * self.DT_SIM

        self.last_pub = cmd
        return cmd


class TestSteeringPolicyUnit(unittest.TestCase):
    """Experiment 25A Unit Equivalence Test Suite."""

    TOLERANCE = 1e-12  # Strict 1 picoradian tolerance

    def setUp(self):
        self.ref = Exp24ReferencePolicy('HOLD_LATTICE16')
        self.prod = SteeringPolicyFilter(
            policy='HOLD_LATTICE16',
            hold_threshold_rad=0.016,
            lattice_quantum_rad=0.032,
            nominal_delay_s=0.020,
            nominal_slew_rate_rad_s=3.20,
            dt_sim_step_s=0.010,
            ctrl_period_s=0.040,
            eps_relay_rad=1e-4
        )

    def _assert_step_equivalence(self, raw_input, step_idx=0, desc=""):
        u_ref = self.ref.step(raw_input)
        u_prod = self.prod.step(raw_input)

        diff_cmd = abs(u_ref - u_prod)
        self.assertLessEqual(
            diff_cmd, self.TOLERANCE,
            f"Step {step_idx} [{desc}]: Output divergence |u_ref - u_prod| = {diff_cmd:.2e} rad "
            f"(u_ref={u_ref:.15f}, u_prod={u_prod:.15f}, raw={raw_input})"
        )

        # Internal state comparison
        self.assertEqual(
            self.ref.is_held, self.prod.is_held,
            f"Step {step_idx} [{desc}]: is_held flag mismatch (ref={self.ref.is_held}, prod={self.prod.is_held})"
        )
        diff_delta = abs(self.ref.cur_delta - self.prod.estimated_delta)
        self.assertLessEqual(
            diff_delta, self.TOLERANCE,
            f"Step {step_idx} [{desc}]: Observer delta mismatch |ref - prod| = {diff_delta:.2e} rad"
        )
        for b_idx in range(2):
            diff_b = abs(self.ref.steer_buffer[b_idx] - self.prod.buffer_contents[b_idx])
            self.assertLessEqual(
                diff_b, self.TOLERANCE,
                f"Step {step_idx} [{desc}]: Buffer[{b_idx}] mismatch |ref - prod| = {diff_b:.2e} rad"
            )

    # --------------------------------------------------------------------------
    # 8. MANDATORY TEST VECTORS (A through N)
    # --------------------------------------------------------------------------

    def test_vector_a_zero_input(self):
        """Vector A: Zero input sequences."""
        for k in range(20):
            self._assert_step_equivalence(0.0, k, "Vector A: Zero input")

    def test_vector_b_constant_positive(self):
        """Vector B: Constant positive steering: +0.032, +0.064, +0.096, +0.160 rad."""
        for target in [0.032, 0.064, 0.096, 0.160]:
            self.ref.reset()
            self.prod.reset()
            for k in range(15):
                self._assert_step_equivalence(target, k, f"Vector B: Const Pos {target}")

    def test_vector_c_constant_negative(self):
        """Vector C: Constant negative steering: -0.032, -0.064, -0.096, -0.160 rad."""
        for target in [-0.032, -0.064, -0.096, -0.160]:
            self.ref.reset()
            self.prod.reset()
            for k in range(15):
                self._assert_step_equivalence(target, k, f"Vector C: Const Neg {target}")

    def test_vector_d_perturbations_around_arrival(self):
        """Vector D: Small perturbations around arrival state: +/-1e-6, +/-0.0035, +/-0.015999, +/-0.016000, +/-0.016001."""
        deltas = [
            +1e-6, -1e-6,
            +0.0035, -0.0035,
            +0.015999, -0.015999,
            +0.016000, -0.016000,
            +0.016001, -0.016001
        ]
        # Start from settled +0.064 rad position
        for _ in range(5):
            self.ref.step(0.064)
            self.prod.step(0.064)

        d_arrival_ref = self.ref.predict_20ms_arrival(self.ref.cur_delta, self.ref.steer_buffer[0], self.ref.steer_buffer[1])[0]
        d_arrival_prod, _ = self.prod.predict_arrival_state()
        self.assertAlmostEqual(d_arrival_ref, d_arrival_prod, delta=self.TOLERANCE)

        for idx, delta_val in enumerate(deltas):
            test_input = d_arrival_ref + delta_val
            self._assert_step_equivalence(test_input, idx, f"Vector D: delta={delta_val:+.6f}")

    def test_vector_e_lattice_boundaries(self):
        """Vector E: Lattice boundaries n*q +/- epsilon for multiple positive and negative n."""
        q = 0.032
        epsilons = [1e-7, 1e-5, 1e-3]
        for n in range(-6, 7):
            center = n * q
            for eps in epsilons:
                for sign in [+1, -1]:
                    test_val = center + sign * eps
                    self.ref.reset()
                    self.prod.reset()
                    self._assert_step_equivalence(test_val, 0, f"Vector E: n={n}, eps={sign*eps:+.1e}")

    def test_vector_f_half_lattice_tiebreaking(self):
        """Vector F: Exact half-lattice points (n + 0.5)*q for positive and negative n."""
        q = 0.032
        for n in range(-6, 7):
            half_point = (n + 0.5) * q
            self.ref.reset()
            self.prod.reset()
            self._assert_step_equivalence(half_point, 0, f"Vector F: n+0.5={n+0.5}, val={half_point:+.6f}")

    def test_vector_g_zero_crossings(self):
        """Vector G: Zero crossings: positive -> zero -> negative."""
        sequence = [
            +0.128, +0.096, +0.064, +0.032, +0.010, +0.002, 0.0,
            -0.002, -0.010, -0.032, -0.064, -0.096, -0.128,
            -0.050, 0.0, +0.050, 0.0
        ]
        for k, u in enumerate(sequence):
            self._assert_step_equivalence(u, k, f"Vector G: zero crossing u={u:+.4f}")

    def test_vector_h_rapid_alternating(self):
        """Vector H: Rapid alternating chatter stimulus."""
        alternating = [+0.004, -0.004, +0.006, -0.006, +0.015, -0.015, +0.017, -0.017] * 5
        for k, u in enumerate(alternating):
            self._assert_step_equivalence(u, k, f"Vector H: alternating u={u:+.4f}")

    def test_vector_i_large_transitions(self):
        """Vector I: Large full-lock steering transitions (+/-0.35 rad)."""
        lock_sequence = [+0.35, -0.35, +0.35, -0.35, 0.0, +0.35, 0.0]
        for k, u in enumerate(lock_sequence):
            self._assert_step_equivalence(u, k, f"Vector I: lock transition u={u:+.2f}")

    def test_vector_j_long_constant_holds(self):
        """Vector J: Long constant command sequence (500 steps)."""
        for k in range(500):
            self._assert_step_equivalence(0.096, k, "Vector J: long hold")

    def test_vector_k_reset_after_arbitrary_state(self):
        """Vector K: Reset after arbitrary internal state."""
        # Drive into complex non-zero state
        for u in [+0.15, -0.22, +0.07, +0.31]:
            self.ref.step(u)
            self.prod.step(u)

        # Reset both
        self.ref.reset(0.0)
        self.prod.reset(0.0)

        self.assertEqual(self.ref.cur_delta, 0.0)
        self.assertEqual(self.prod.estimated_delta, 0.0)
        self.assertEqual(self.ref.steer_buffer, [0.0, 0.0])
        self.assertEqual(self.prod.buffer_contents, [0.0, 0.0])

        # Step zero and verify identical state
        self._assert_step_equivalence(0.0, 0, "Vector K: post-reset zero")

    def test_vector_l_repeated_reset_determinism(self):
        """Vector L: Repeated reset determinism."""
        for r_idx in range(5):
            self.ref.reset()
            self.prod.reset()
            for k in range(10):
                self._assert_step_equivalence(0.05 * math.sin(k), k, f"Vector L: run {r_idx}")

    def test_vector_m_negative_zero(self):
        """Vector M: Negative-zero behavior (-0.0 == +0.0)."""
        self._assert_step_equivalence(-0.0, 0, "Vector M: -0.0")
        self._assert_step_equivalence(+0.0, 1, "Vector M: +0.0")

    def test_vector_n_floating_point_boundaries(self):
        """Vector N: Floating point boundary cases near max physical steering."""
        for u in [+0.349999999999, -0.349999999999, +0.350000000001, -0.350000000001]:
            self._assert_step_equivalence(u, 0, f"Vector N: float boundary u={u}")

    # --------------------------------------------------------------------------
    # 9. RANDOMIZED DIFFERENTIAL TEST (100,000 CYCLES)
    # --------------------------------------------------------------------------

    def test_randomized_differential_100k(self):
        """
        Runs 100,000 deterministic pseudo-random inputs through reference and production.
        Verifies max absolute divergence <= 1e-12 rad across the entire sequence.
        """
        N_SAMPLES = 100_000
        rng = np.random.default_rng(123456789)
        test_inputs = rng.uniform(-0.35, 0.35, size=N_SAMPLES)

        self.ref.reset()
        self.prod.reset()

        max_err_cmd = 0.0
        max_err_delta = 0.0
        max_err_buf = 0.0

        for k in range(N_SAMPLES):
            u_raw = float(test_inputs[k])
            u_ref = self.ref.step(u_raw)
            u_prod = self.prod.step(u_raw)

            err_c = abs(u_ref - u_prod)
            if err_c > max_err_cmd:
                max_err_cmd = err_c

            err_d = abs(self.ref.cur_delta - self.prod.estimated_delta)
            if err_d > max_err_delta:
                max_err_delta = err_d

            err_b = max(abs(self.ref.steer_buffer[i] - self.prod.buffer_contents[i]) for i in range(2))
            if err_b > max_err_buf:
                max_err_buf = err_b

            if k % 20000 == 0:
                self.assertLessEqual(max_err_cmd, self.TOLERANCE, f"Failed at cycle {k}")

        self.assertLessEqual(max_err_cmd, self.TOLERANCE, f"Max cmd err {max_err_cmd:.2e} exceeded 1e-12")
        self.assertLessEqual(max_err_delta, self.TOLERANCE, f"Max delta err {max_err_delta:.2e} exceeded 1e-12")
        self.assertLessEqual(max_err_buf, self.TOLERANCE, f"Max buffer err {max_err_buf:.2e} exceeded 1e-12")
        print(f"\n[100k Differential Test Passed] Max Output Error: {max_err_cmd:.2e} rad, Max Observer Delta Error: {max_err_delta:.2e} rad")

    # --------------------------------------------------------------------------
    # 10. MATHEMATICAL INVARIANT TESTS
    # --------------------------------------------------------------------------

    def test_hold_lattice_16_modification_bound(self):
        """
        Verifies mathematical invariant: |u_pub - u_raw| <= 0.016000000001 rad
        across 10,000 continuous inputs.
        """
        rng = np.random.default_rng(98765)
        samples = rng.uniform(-0.35, 0.35, size=10_000)
        self.prod.reset()

        for k, u_raw in enumerate(samples):
            u_pub = self.prod.step(float(u_raw))
            mod = abs(u_pub - u_raw)
            self.assertLessEqual(
                mod, 0.016000000001,
                f"Step {k}: Modification invariant violated: |{u_pub:.6f} - {u_raw:.6f}| = {mod:.6f} rad > 16 mrad"
            )

    def test_no_hidden_state_independence(self):
        """Verifies two separate instances do not share state or globals."""
        f1 = SteeringPolicyFilter('HOLD_LATTICE16')
        f2 = SteeringPolicyFilter('HOLD_LATTICE16')

        f1.step(0.15)
        self.assertEqual(f2.estimated_delta, 0.0)
        self.assertEqual(f2.buffer_contents, [0.0, 0.0])

    # --------------------------------------------------------------------------
    # 11. PROPERTY TEST FOR RAW MODE
    # --------------------------------------------------------------------------

    def test_raw_mode_bypass(self):
        """Verifies policy='RAW' bypasses the policy completely: u_pub == u_raw."""
        raw_filter = SteeringPolicyFilter('RAW')
        rng = np.random.default_rng(55555)
        samples = rng.uniform(-0.35, 0.35, size=10_000)

        for u in samples:
            u_pub = raw_filter.step(float(u))
            self.assertEqual(u_pub, float(u))
            self.assertFalse(raw_filter.is_held)

    # --------------------------------------------------------------------------
    # 12. HOLD16 ONLY MODE
    # --------------------------------------------------------------------------

    def test_hold16_mode_differential(self):
        """Verifies HOLD16 without lattice snapping matches Exp 24 HOLD16 reference."""
        ref_hold16 = Exp24ReferencePolicy('HOLD16')
        prod_hold16 = SteeringPolicyFilter('HOLD16')
        rng = np.random.default_rng(112233)
        samples = rng.uniform(-0.35, 0.35, size=20_000)

        for k, u in enumerate(samples):
            r = ref_hold16.step(float(u))
            p = prod_hold16.step(float(u))
            self.assertLessEqual(abs(r - p), self.TOLERANCE, f"HOLD16 mismatch at step {k}")


class TestSteeringPolicyIntegration(unittest.TestCase):
    """
    Experiment 25B Integration Unit Tests:
    Validates ROS 2 parameter parsing, state persistence, reset semantics,
    and fallback path behaviors as specified in Sections 0, 8, and 17.
    """

    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

    # --------------------------------------------------------------------------
    # SECTION 0: PRE-INTEGRATION EQUIVALENCE CORRECTION TESTS
    # --------------------------------------------------------------------------

    def test_section0_nominal_zero_reset_equivalence(self):
        """
        Section 0: Verifies that reset(0.0) or reset() deterministically produces
        cur_delta_est == 0.0 and steer_buffer == [0.0, 0.0], matching the frozen
        Experiment 24 reference bit-identically.
        """
        filt = SteeringPolicyFilter('HOLD_LATTICE16')
        filt.reset()
        self.assertEqual(filt.estimated_delta, 0.0)
        self.assertEqual(filt.buffer_contents, [0.0, 0.0])
        self.assertEqual(filt.last_published_cmd, 0.0)
        self.assertFalse(filt.is_held)

        filt.step(0.12)
        filt.reset(0.0)
        self.assertEqual(filt.estimated_delta, 0.0)
        self.assertEqual(filt.buffer_contents, [0.0, 0.0])

    def test_section0_nonzero_warm_reset_semantics(self):
        """
        Section 0: Verifies warm-reset production semantics for initial_steer != 0.0.
        Demonstrates that warm reset initializes cur_delta_est and the full buffer
        to initial_steer, which diverges from the frozen zero-initialized Exp 24 reference.
        """
        init_val = 0.064
        filt = SteeringPolicyFilter('HOLD_LATTICE16')
        filt.reset(initial_steer=init_val)

        self.assertEqual(filt.estimated_delta, init_val)
        self.assertEqual(filt.buffer_contents, [init_val, init_val])
        self.assertEqual(filt.last_published_cmd, init_val)

        # Contrast with frozen Exp 24 reference which strictly initializes at 0.0:
        ref_frozen = Exp24ReferencePolicy('HOLD_LATTICE16')
        ref_frozen.reset(0.0)  # Frozen Exp 24 reference
        self.assertNotEqual(filt.estimated_delta, ref_frozen.cur_delta)
        self.assertNotEqual(filt.buffer_contents, ref_frozen.steer_buffer)

    # --------------------------------------------------------------------------
    # SECTION 8 & 17: OBSERVER STATE PERSISTENCE ACROSS CYCLES
    # --------------------------------------------------------------------------

    def test_state_persistence_across_cycles(self):
        """
        Section 8: Proves that the internal command-side observer state persists
        and evolves across 25-Hz control cycles, and is NOT re-initialized on each step.
        """
        filt = SteeringPolicyFilter('HOLD_LATTICE16')
        filt.reset(0.0)

        # Step 1 with a nonzero target: +0.096 rad
        u1 = filt.step(0.096)
        delta_after_step1 = filt.estimated_delta
        buf_after_step1 = filt.buffer_contents
        # Observer must have integrated 4 substeps (40 ms * 3.2 rad/s = slew up towards target)
        self.assertGreater(delta_after_step1, 0.0, "Observer delta should have advanced")
        self.assertEqual(buf_after_step1[0], u1, "Buffer head should contain last command")

        # Step 2 with same target: state should continue from step 1 state, not from zero
        u2 = filt.step(0.096)
        delta_after_step2 = filt.estimated_delta
        self.assertGreater(delta_after_step2, delta_after_step1, "Observer delta should persist and continue advancing")

    # --------------------------------------------------------------------------
    # SECTION 17: PARAMETER DEFAULTS, PARSING & INVALID POLICY REJECTION
    # --------------------------------------------------------------------------

    def test_invalid_policy_rejection(self):
        """Section 17: Verifies invalid policy strings are rejected with ValueError."""
        with self.assertRaises(ValueError):
            SteeringPolicyFilter('INVALID_POLICY')

        filt = SteeringPolicyFilter('RAW')
        with self.assertRaises(ValueError):
            filt.policy = 'UNKNOWN'

    def test_ros_parameter_parsing_and_defaults(self):
        """
        Section 6 & 17: Verifies that MPCControllerNode declares and retrieves
        all parameters with recommended production defaults.
        """
        from scripts.mpc_controller_node import MPCControllerNode
        node = MPCControllerNode(map_name_override='Spielberg')

        # Production defaults
        self.assertEqual(node.steering_policy_name, 'RAW', "Default steering_policy must be RAW")
        self.assertEqual(node.policy_hold_threshold_rad, 0.016)
        self.assertEqual(node.policy_lattice_quantum_rad, 0.032)
        self.assertEqual(node.nominal_actuator_delay_s, 0.020)
        self.assertEqual(node.nominal_actuator_slew_rad_s, 3.20)
        self.assertEqual(node.model_type, 'yaw_first_order')
        self.assertEqual(node.actuator_delay_s, 0.020)

        # Verify SteeringPolicyFilter is initialized with RAW policy
        self.assertEqual(node.steering_policy_filter.policy, 'RAW')
        self.assertEqual(node.steering_policy_filter.estimated_delta, 0.0)

        node.destroy_node()

    def test_ros_node_reset_determinism(self):
        """
        Section 8 & 17: Verifies node.reset(0.0) cleanly resets both controller
        tracking state and steering_policy_filter observer state.
        """
        from scripts.mpc_controller_node import MPCControllerNode
        node = MPCControllerNode(map_name_override='Spielberg')

        # Simulate state evolution
        node._last_idx = 42
        node._last_steer_cmd = 0.15
        node.steering_policy_filter.step(0.15)

        # Call reset
        node.reset(0.0)
        self.assertEqual(node._last_idx, 0)
        self.assertEqual(node._last_steer_cmd, 0.0)
        self.assertEqual(node.steering_policy_filter.estimated_delta, 0.0)
        self.assertEqual(node.steering_policy_filter.buffer_contents, [0.0, 0.0])

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
