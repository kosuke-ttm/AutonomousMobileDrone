import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build"))

from simulation.scenario import descend, disturbance, hover, run_scenario, takeoff
from simulation.simulator import AltitudeController, Simulator
import evtol_core


def test_altitude_hold_to_100m():
    # TEST-001 / TEST-SCENARIO-001
    result = run_scenario(takeoff())
    assert result.final_altitude() == pytest.approx(100.0, abs=2.0)
    assert result.steady_state_error() < 2.0
    states = result.states()
    assert "GROUND" in states or "TAKEOFF" in states
    assert "CLIMB" in states
    assert "HOVER" in states


def test_hover_stability():
    # TEST-SCENARIO-002
    result = run_scenario(hover())
    altitudes = result.column("altitude_m")
    assert max(abs(a - 100.0) for a in altitudes) < 1.0
    assert all(s == "HOVER" for s in result.states())


def test_target_change_100_to_50():
    # TEST-002 / TEST-SCENARIO-003
    result = run_scenario(descend())
    assert result.final_altitude() == pytest.approx(50.0, abs=2.0)
    assert "DESCEND" in result.states()
    assert result.samples[-1].flight_state == "HOVER"


def test_disturbance_rejection():
    # TEST-003 / TEST-SCENARIO-004
    result = run_scenario(disturbance())
    max_dev = result.max_deviation_after(8.0)
    assert max_dev > 0.2
    assert result.final_altitude() == pytest.approx(100.0, abs=2.0)
    assert result.steady_state_error() < 2.0


def test_thrust_upper_limit():
    # TEST-004
    pid = evtol_core.PidController(1000.0, 0.0, 0.0, 0.0, 250.0)
    assert pid.update(100.0, 0.01) == 250.0


def test_thrust_lower_limit():
    # TEST-005
    pid = evtol_core.PidController(1000.0, 0.0, 0.0, 0.0, 250.0)
    assert pid.update(-100.0, 0.01) == 0.0


def test_controller_respects_vehicle_limits():
    model = evtol_core.EvtolModel()
    ctrl = AltitudeController(model.hover_thrust(), model.max_thrust(), kp=1.0e6, ki=0.0, kd=0.0)
    assert ctrl.update(1000.0, 0.0, 0.01) == pytest.approx(model.max_thrust())
    ctrl.reset()
    ctrl = AltitudeController(model.hover_thrust(), model.max_thrust(), kp=1.0e6, ki=0.0, kd=0.0)
    assert ctrl.update(0.0, 1000.0, 0.01) == pytest.approx(0.0)


def test_logging_fields_present():
    sim = Simulator()
    sim.reset(0.0, 0.0)
    result = sim.run(0.05, lambda _t: 10.0)
    sample = result.samples[0]
    assert sample.time_s == 0.0
    assert hasattr(sample, "target_altitude_m")
    assert hasattr(sample, "altitude_m")
    assert hasattr(sample, "altitude_error_m")
    assert hasattr(sample, "velocity_mps")
    assert hasattr(sample, "thrust_n")
    assert hasattr(sample, "disturbance_n")
    assert hasattr(sample, "flight_state")
