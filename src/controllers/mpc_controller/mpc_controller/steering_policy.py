import math
from typing import List, Tuple


class SteeringPolicyFilter:
    """Stateful steering command filter supporting RAW, HOLD16, and HOLD_LATTICE16."""

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
        policy_upper = policy.upper()
        if policy_upper not in self.SUPPORTED_POLICIES:
            raise ValueError(f"Unsupported policy '{policy}'. Must be one of {self.SUPPORTED_POLICIES}")

        self._policy = policy_upper
        self._hold_threshold = float(hold_threshold_rad)
        self._lattice_quantum = float(lattice_quantum_rad)
        self._nominal_delay = float(nominal_delay_s)
        self._nominal_slew = float(nominal_slew_rate_rad_s)
        self._dt_sim = float(dt_sim_step_s)
        self._ctrl_period = float(ctrl_period_s)
        self._eps_relay = float(eps_relay_rad)

        self._buffer_size = max(1, int(round(self._nominal_delay / self._dt_sim)))
        self._substeps_per_cycle = max(1, int(round(self._ctrl_period / self._dt_sim)))

        self._cur_delta_est: float = 0.0
        self._steer_buffer: List[float] = [0.0] * self._buffer_size
        self._last_published_cmd: float = 0.0
        self._is_held: bool = False
        self._step_count: int = 0

        self.reset()

    def reset(self, initial_steer: float = 0.0) -> None:
        """Reset internal command-side observer state."""
        self._cur_delta_est = float(initial_steer)
        self._steer_buffer = [float(initial_steer)] * self._buffer_size
        self._last_published_cmd = float(initial_steer)
        self._is_held = False
        self._step_count = 0

    @property
    def policy(self) -> str:
        return self._policy

    @policy.setter
    def policy(self, value: str) -> None:
        val_upper = value.upper()
        if val_upper not in self.SUPPORTED_POLICIES:
            raise ValueError(f"Unsupported policy '{value}'. Must be one of {self.SUPPORTED_POLICIES}")
        self._policy = val_upper

    @property
    def is_held(self) -> bool:
        return self._is_held

    @property
    def estimated_delta(self) -> float:
        return self._cur_delta_est

    @property
    def buffer_contents(self) -> List[float]:
        return list(self._steer_buffer)

    @property
    def last_published_cmd(self) -> float:
        return self._last_published_cmd

    def predict_arrival_state(self) -> Tuple[float, float]:
        """Predict steering angle after nominal transport delay via 2-step lifted relay kinematics."""
        b0 = self._steer_buffer[0] if len(self._steer_buffer) >= 1 else 0.0
        b1 = self._steer_buffer[1] if len(self._steer_buffer) >= 2 else b0
        cur_d = self._cur_delta_est

        diff1 = b1 - cur_d
        sv1 = math.copysign(self._nominal_slew, diff1) if abs(diff1) > self._eps_relay else 0.0
        d_10 = cur_d + sv1 * self._dt_sim

        diff2 = b0 - d_10
        sv2 = math.copysign(self._nominal_slew, diff2) if abs(diff2) > self._eps_relay else 0.0
        d_20 = d_10 + sv2 * self._dt_sim

        return d_20, sv2

    def step(self, raw_steer_cmd: float) -> float:
        """Filter raw MPC steering command according to configured policy."""
        raw = float(raw_steer_cmd)

        if self._policy == 'RAW':
            cmd = raw
            self._is_held = False
        else:
            d_arrival, _ = self.predict_arrival_state()
            if abs(raw - d_arrival) < self._hold_threshold:
                cmd = d_arrival
                self._is_held = True
            elif self._policy == 'HOLD_LATTICE16':
                cmd = round(raw / self._lattice_quantum) * self._lattice_quantum
                self._is_held = False
            else:
                cmd = raw
                self._is_held = False

        self._advance_observer(cmd)
        self._last_published_cmd = cmd
        self._step_count += 1
        return cmd

    def _advance_observer(self, published_cmd: float) -> None:
        """Advance command-side observer by one control period (4 substeps of 10ms)."""
        for _ in range(self._substeps_per_cycle):
            tgt = self._steer_buffer[-1]
            self._steer_buffer = [published_cmd] + self._steer_buffer[:-1]
            diff = tgt - self._cur_delta_est
            sv = math.copysign(self._nominal_slew, diff) if abs(diff) > self._eps_relay else 0.0
            self._cur_delta_est += sv * self._dt_sim
