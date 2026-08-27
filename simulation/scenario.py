"""Flight test scenarios (TEST-SCENARIO-001 .. 004)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Tuple

from simulation.simulator import Simulator, SimulationResult


@dataclass
class Scenario:
    name: str
    duration_s: float
    initial_altitude_m: float
    initial_velocity_mps: float
    target_fn: Callable[[float], float]
    disturbance_fn: Callable[[float], float]


def takeoff() -> Scenario:
    return Scenario(
        name="TEST-SCENARIO-001-takeoff",
        duration_s=80.0,
        initial_altitude_m=0.0,
        initial_velocity_mps=0.0,
        target_fn=lambda _t: 100.0,
        disturbance_fn=lambda _t: 0.0,
    )


def hover() -> Scenario:
    return Scenario(
        name="TEST-SCENARIO-002-hover",
        duration_s=25.0,
        initial_altitude_m=100.0,
        initial_velocity_mps=0.0,
        target_fn=lambda _t: 100.0,
        disturbance_fn=lambda _t: 0.0,
    )


def descend() -> Scenario:
    def target(t: float) -> float:
        return 100.0 if t < 8.0 else 50.0

    return Scenario(
        name="TEST-SCENARIO-003-descend",
        duration_s=70.0,
        initial_altitude_m=100.0,
        initial_velocity_mps=0.0,
        target_fn=target,
        disturbance_fn=lambda _t: 0.0,
    )


def disturbance() -> Scenario:
    def force(t: float) -> float:
        return -100.0 if 8.0 <= t <= 12.0 else 0.0

    return Scenario(
        name="TEST-SCENARIO-004-disturbance",
        duration_s=60.0,
        initial_altitude_m=100.0,
        initial_velocity_mps=0.0,
        target_fn=lambda _t: 100.0,
        disturbance_fn=force,
    )


def all_scenarios() -> Tuple[Scenario, ...]:
    return (takeoff(), hover(), descend(), disturbance())


def run_scenario(scenario: Scenario, **sim_kwargs) -> SimulationResult:
    sim = Simulator(**sim_kwargs)
    sim.reset(scenario.initial_altitude_m, scenario.initial_velocity_mps)
    return sim.run(scenario.duration_s, scenario.target_fn, scenario.disturbance_fn)
