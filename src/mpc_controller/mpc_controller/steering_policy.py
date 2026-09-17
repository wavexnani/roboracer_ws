#!/usr/bin/env python3
"""
Stateful Steering Command Policy Filter for F1TENTH Model Predictive Control.

This module implements the discrete actuator-interface policies evaluated in
Experiments 20 through 24:
  1. RAW: Pure passthrough of raw optimizer steering setpoints.
  2. HOLD16: Deadband hold relative to predicted arrival steering angle (16 mrad).
  3. HOLD_LATTICE16: Deadband hold (16 mrad) combined with discrete relay-lattice
     quantization (32 mrad grid matching nominal physical simulator displacement).

CRITICAL ARCHITECTURAL DISTINCTIONS:
  - MPC Prediction Time Step: dt_pred = 0.080 s (prediction step inside OSQP)
  - Controller Execution Period: T_ctrl = 0.040 s (25 Hz control node callback)
  - Physics Discretization Step: dt_sim = 0.010 s (100 Hz simulator physics step)
  - Actuator Transport Delay: tau_nom = 0.020 s (2 discrete 10 ms steps in flight)
  - Actuator Slew Rate: sv_nom = 3.20 rad/s (nominal servo velocity limit)

OBSERVER ASSUMPTION STATEMENT:
  The internal observer maintained by this filter is a COMMAND-SIDE MODEL-BASED
  ACTUATOR OBSERVER. In the current production ROS computational graph, front-wheel
  steering feedback is NOT measured or published on /odom or any sensor topic.
  Therefore:
      estimated_actuator_state != measured_physical_actuator_state
  The observer assumes the physical actuator tracks published commands following
  nominal transport delay and slew-rate dynamics without unmodelled external loads.
"""

import math
from typing import List, Tuple, Optional


class SteeringPolicyFilter:
    """
    Stateful discrete-time steering command policy filter.

    Wraps the raw continuous steering output of the MPC optimizer before publication
    to eliminate high-frequency relay limit-cycle chattering.
    """

    SUPPORTED_POLICIES = ('RAW', 'HOLD16', 'HOLD_LATTICE16')

    def __init__(
        self,
        policy: str = 'HOLD_LATTICE16',
        hold_threshold_rad: float = 0.016,
        lattice_quantum_rad: float = 0.032,
        nominal_delay_s: float = 0.020,
        nominal_slew_rate_rad_s: float = 3.20,
        dt_sim_step_s: float = 0.010,
        ctrl_period_s: float = 0.040,
        eps_relay_rad: float = 1e-4
    ) -> None:
        """
        Initializes the steering policy filter.

        Args:
            policy: Operating mode ('RAW', 'HOLD16', or 'HOLD_LATTICE16').
            hold_threshold_rad: Deadband radius around arrival angle [rad]. Default: 0.016 rad (16 mrad).
            lattice_quantum_rad: Lattice grid quantum [rad]. Default: 0.032 rad (32 mrad).
            nominal_delay_s: Nominal transport delay assumption [s]. Default: 0.020 s (20 ms).
            nominal_slew_rate_rad_s: Nominal actuator slew rate [rad/s]. Default: 3.20 rad/s.
            dt_sim_step_s: Sub-step discretization time for observer [s]. Default: 0.010 s (10 ms).
            ctrl_period_s: Controller execution period [s]. Default: 0.040 s (25 Hz).
            eps_relay_rad: Sub-threshold deadband to prevent numerical relay chattering [rad]. Default: 1e-4 rad.
        """
        policy_upper = policy.upper()
        if policy_upper not in self.SUPPORTED_POLICIES:
            raise ValueError(
                f"Unsupported policy '{policy}'. Must be one of {self.SUPPORTED_POLICIES}"
            )
        self._policy = policy_upper
        self._hold_threshold = float(hold_threshold_rad)
        self._lattice_quantum = float(lattice_quantum_rad)
        self._nominal_delay = float(nominal_delay_s)
        self._nominal_slew = float(nominal_slew_rate_rad_s)
        self._dt_sim = float(dt_sim_step_s)
        self._ctrl_period = float(ctrl_period_s)
        self._eps_relay = float(eps_relay_rad)

        # Buffer sizing based on nominal transport delay
        self._buffer_size = max(1, int(round(self._nominal_delay / self._dt_sim)))
        self._substeps_per_cycle = max(1, int(round(self._ctrl_period / self._dt_sim)))

        # State initialization
        self._cur_delta_est: float = 0.0
        self._steer_buffer: List[float] = [0.0] * self._buffer_size
        self._last_published_cmd: float = 0.0
        self._is_held: bool = False
        self._step_count: int = 0

        self.reset()

    def reset(self, initial_steer: float = 0.0) -> None:
        """
        Resets the command-side model-based observer to a deterministic state.

        EQUIVALENCE & RESET SEMANTICS (Experiment 25B Section 0):
          - Nominal SIL Equivalence: For nominal controller initialization and validation
            against the frozen Experiment 24 reference, initial steering is explicitly
            defined as 0.0 rad:
                cur_delta_est = 0.0
                steer_buffer = [0.0] * buffer_size  (i.e. [0.0, 0.0] for 20 ms delay)
            This guarantees exact bit-identical state initialization with Experiment 24.
          - Warm-Reset Production Semantics: If a nonzero initial_steer is provided, the
            observer assumes steady-state actuator alignment at that angle:
                cur_delta_est = float(initial_steer)
                steer_buffer = [float(initial_steer)] * buffer_size
            Note: Nonzero initial_steer diverges from the zero-initialized Experiment 24
            benchmark and is intended solely for dynamic production re-initializations.

        Args:
            initial_steer: Initial estimated front-wheel steering angle [rad]. Default: 0.0.
        """
        self._cur_delta_est = float(initial_steer)
        self._steer_buffer = [float(initial_steer)] * self._buffer_size
        self._last_published_cmd = float(initial_steer)
        self._is_held = False
        self._step_count = 0

    @property
    def policy(self) -> str:
        """Returns active policy name ('RAW', 'HOLD16', 'HOLD_LATTICE16')."""
        return self._policy

    @policy.setter
    def policy(self, value: str) -> None:
        val_upper = value.upper()
        if val_upper not in self.SUPPORTED_POLICIES:
            raise ValueError(f"Unsupported policy '{value}'. Must be one of {self.SUPPORTED_POLICIES}")
        self._policy = val_upper

    @property
    def is_held(self) -> bool:
        """Returns True if the last step activated the deadband hold condition."""
        return self._is_held

    @property
    def estimated_delta(self) -> float:
        """Returns the current command-side estimated wheel angle [rad]."""
        return self._cur_delta_est

    @property
    def buffer_contents(self) -> List[float]:
        """Returns a copy of the command-side FIFO buffer of commands in flight."""
        return list(self._steer_buffer)

    @property
    def last_published_cmd(self) -> float:
        """Returns the last published steering command [rad]."""
        return self._last_published_cmd

    def predict_arrival_state(self) -> Tuple[float, float]:
        """
        Calculates the predicted steering angle at arrival after nominal transport delay.

        Uses the exact 2-step lifted relay kinematics:
          Step 1: delta_10 = cur_delta + sgn(b1 - cur_delta) * sv_nom * dt_sim
          Step 2: delta_20 = delta_10  + sgn(b0 - delta_10)  * sv_nom * dt_sim

        Returns:
            Tuple of (d_arrival [rad], sv_arrival [rad/s]).
        """
        b0 = self._steer_buffer[0] if len(self._steer_buffer) >= 1 else 0.0
        b1 = self._steer_buffer[1] if len(self._steer_buffer) >= 2 else b0
        cur_d = self._cur_delta_est

        # First physics sub-step (arrival from buffer tail)
        diff1 = b1 - cur_d
        sv1 = math.copysign(self._nominal_slew, diff1) if abs(diff1) > self._eps_relay else 0.0
        d_10 = cur_d + sv1 * self._dt_sim

        # Second physics sub-step (arrival to vehicle actuator target)
        diff2 = b0 - d_10
        sv2 = math.copysign(self._nominal_slew, diff2) if abs(diff2) > self._eps_relay else 0.0
        d_20 = d_10 + sv2 * self._dt_sim

        return d_20, sv2

    def step(self, raw_steer_cmd: float) -> float:
        """
        Filters a single raw steering setpoint emitted by the MPC optimizer.

        Args:
            raw_steer_cmd: Raw unconstrained/unquantized steering angle [rad] from MPC.

        Returns:
            Filtered steering command [rad] to be published to /drive.
        """
        raw = float(raw_steer_cmd)

        if self._policy == 'RAW':
            cmd = raw
            self._is_held = False
        else:
            d_arrival, _ = self.predict_arrival_state()

            # Hold condition check
            if abs(raw - d_arrival) < self._hold_threshold:
                cmd = d_arrival
                self._is_held = True
            elif self._policy == 'HOLD_LATTICE16':
                # Exact lattice quantization with Python round-half-to-even tie-breaking
                cmd = round(raw / self._lattice_quantum) * self._lattice_quantum
                self._is_held = False
            else:  # HOLD16 without lattice snapping
                cmd = raw
                self._is_held = False

        # Advance command-side model-based observer across one controller period (4 substeps of 10ms)
        self._advance_observer(cmd)

        self._last_published_cmd = cmd
        self._step_count += 1
        return cmd

    def _advance_observer(self, published_cmd: float) -> None:
        """
        Advances the internal command-side observer by T_ctrl (40 ms = 4 sub-steps of 10 ms).

        Matches the exact discrete sub-step FIFO and relay update of the simulator model.
        """
        for _ in range(self._substeps_per_cycle):
            tgt = self._steer_buffer[-1]
            self._steer_buffer = [published_cmd] + self._steer_buffer[:-1]
            diff = tgt - self._cur_delta_est
            if abs(diff) > self._eps_relay:
                sv = math.copysign(self._nominal_slew, diff)
            else:
                sv = 0.0
            self._cur_delta_est += sv * self._dt_sim
