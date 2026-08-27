"""Closed-loop eVTOL altitude simulation (REQ-F-007, REQ-F-010)."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

try:
    import evtol_core
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "evtol_core が見つかりません。先に CMake でビルドしてください。"
    ) from exc

CONTROL_PERIOD_S = 0.01  # 10 ms


@dataclass
class LogSample:
    time_s: float
    target_altitude_m: float
    altitude_m: float
    altitude_error_m: float
    velocity_mps: float
    thrust_n: float
    disturbance_n: float
    flight_state: str


@dataclass
class SimulationResult:
    samples: List[LogSample] = field(default_factory=list)

    def column(self, name: str) -> List[float]:
        return [getattr(s, name) for s in self.samples]

    def states(self) -> List[str]:
        return [s.flight_state for s in self.samples]

    def final_altitude(self) -> float:
        return self.samples[-1].altitude_m if self.samples else 0.0

    def steady_state_error(self, window_s: float = 2.0) -> float:
        if not self.samples:
            return 0.0
        t_end = self.samples[-1].time_s
        window = [s for s in self.samples if s.time_s >= t_end - window_s]
        if not window:
            window = self.samples[-1:]
        errors = [abs(s.altitude_error_m) for s in window]
        return sum(errors) / len(errors)

    def overshoot(self, target_m: float) -> float:
        if not self.samples:
            return 0.0
        peak = max(s.altitude_m for s in self.samples)
        return max(0.0, peak - target_m)

    def settling_time(self, band_m: float = 2.0) -> Optional[float]:
        if not self.samples:
            return None
        for i, sample in enumerate(self.samples):
            rest = self.samples[i:]
            if all(abs(s.altitude_error_m) <= band_m for s in rest):
                return sample.time_s
        return None

    def max_deviation_after(self, t_start_s: float) -> float:
        after = [s for s in self.samples if s.time_s >= t_start_s]
        if not after:
            return 0.0
        return max(abs(s.altitude_error_m) for s in after)

    def write_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "time_s",
                    "target_altitude_m",
                    "altitude_m",
                    "altitude_error_m",
                    "velocity_mps",
                    "thrust_n",
                    "disturbance_n",
                    "flight_state",
                ]
            )
            for s in self.samples:
                writer.writerow(
                    [
                        f"{s.time_s:.4f}",
                        f"{s.target_altitude_m:.6f}",
                        f"{s.altitude_m:.6f}",
                        f"{s.altitude_error_m:.6f}",
                        f"{s.velocity_mps:.6f}",
                        f"{s.thrust_n:.6f}",
                        f"{s.disturbance_n:.6f}",
                        s.flight_state,
                    ]
                )


class AltitudeController:
    """PID on altitude error plus hover thrust feedforward, then limiter."""

    def __init__(
        self,
        hover_thrust_n: float,
        max_thrust_n: float,
        kp: float = 4.0,
        ki: float = 0.45,
        kd: float = 16.0,
    ) -> None:
        self.hover_thrust_n = hover_thrust_n
        self.max_thrust_n = max_thrust_n
        self.pid = evtol_core.PidController(
            kp, ki, kd, -hover_thrust_n, max_thrust_n - hover_thrust_n
        )

    def update(self, target_m: float, measured_m: float, dt_s: float) -> float:
        error = self.pid.calculate_error(target_m, measured_m)
        delta = self.pid.update(error, dt_s)
        thrust = self.hover_thrust_n + delta
        return min(self.max_thrust_n, max(0.0, thrust))

    def reset(self) -> None:
        self.pid.reset()


class Simulator:
    def __init__(
        self,
        dt_s: float = CONTROL_PERIOD_S,
        kp: float = 4.0,
        ki: float = 0.45,
        kd: float = 16.0,
        noise_std_m: float = 0.0,
        seed: int = 1,
    ) -> None:
        self.dt_s = dt_s
        self.model = evtol_core.EvtolModel()
        self.sensor = evtol_core.AltitudeSensor(noise_std_m, seed)
        self.state = evtol_core.FlightStateEstimator()
        self.controller = AltitudeController(
            self.model.hover_thrust(), self.model.max_thrust(), kp, ki, kd
        )

    def reset(self, altitude_m: float = 0.0, velocity_mps: float = 0.0) -> None:
        self.model.reset(altitude_m, velocity_mps)
        self.state.reset()
        self.controller.reset()

    def run(
        self,
        duration_s: float,
        target_fn: Callable[[float], float],
        disturbance_fn: Optional[Callable[[float], float]] = None,
    ) -> SimulationResult:
        if disturbance_fn is None:
            disturbance_fn = lambda _t: 0.0  # noqa: E731

        result = SimulationResult()
        steps = int(round(duration_s / self.dt_s))
        for i in range(steps + 1):
            t = i * self.dt_s
            target = target_fn(t)
            measured = self.sensor.measure(self.model.altitude())
            error = self.controller.pid.calculate_error(target, measured)
            thrust = self.controller.update(target, measured, self.dt_s)
            disturbance = disturbance_fn(t)
            flight_state = evtol_core.flight_state_name(
                self.state.update(
                    self.model.altitude(),
                    self.model.velocity(),
                    target,
                    error,
                )
            )
            result.samples.append(
                LogSample(
                    time_s=t,
                    target_altitude_m=target,
                    altitude_m=self.model.altitude(),
                    altitude_error_m=error,
                    velocity_mps=self.model.velocity(),
                    thrust_n=thrust,
                    disturbance_n=disturbance,
                    flight_state=flight_state,
                )
            )
            if i < steps:
                self.model.step(thrust, disturbance, self.dt_s)
        return result
