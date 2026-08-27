#include <pybind11/pybind11.h>

#include <string>

#include "controller/pid_controller.hpp"
#include "dynamics/evtol_model.hpp"
#include "flight_state/flight_state.hpp"
#include "sensor/altitude_sensor.hpp"

namespace py = pybind11;

PYBIND11_MODULE(evtol_core, m) {
    m.doc() = "eVTOL flight control core (C++)";

    py::class_<evtol::PidController>(m, "PidController")
        .def(py::init<double, double, double, double, double>(), py::arg("kp"),
             py::arg("ki"), py::arg("kd"), py::arg("min_output"),
             py::arg("max_output"))
        .def("calculate_error", &evtol::PidController::calculateError,
             py::arg("target"), py::arg("current"))
        .def("update", &evtol::PidController::update, py::arg("error"),
             py::arg("dt_s"))
        .def("reset", &evtol::PidController::reset)
        .def("last_output", &evtol::PidController::lastOutput)
        .def("integral", &evtol::PidController::integral)
        .def_property_readonly("kp", &evtol::PidController::kp)
        .def_property_readonly("ki", &evtol::PidController::ki)
        .def_property_readonly("kd", &evtol::PidController::kd)
        .def_property_readonly("min_output", &evtol::PidController::minOutput)
        .def_property_readonly("max_output", &evtol::PidController::maxOutput);

    py::class_<evtol::VehicleParams>(m, "VehicleParams")
        .def(py::init<>())
        .def_readwrite("mass_kg", &evtol::VehicleParams::mass_kg)
        .def_readwrite("gravity_mps2", &evtol::VehicleParams::gravity_mps2)
        .def_readwrite("max_thrust_n", &evtol::VehicleParams::max_thrust_n);

    py::class_<evtol::EvtolModel>(m, "EvtolModel")
        .def(py::init<evtol::VehicleParams>(),
             py::arg("params") = evtol::VehicleParams{})
        .def("reset", &evtol::EvtolModel::reset, py::arg("altitude_m") = 0.0,
             py::arg("velocity_mps") = 0.0)
        .def("step", &evtol::EvtolModel::step, py::arg("thrust_n"),
             py::arg("disturbance_n"), py::arg("dt_s"))
        .def("altitude", &evtol::EvtolModel::altitude)
        .def("velocity", &evtol::EvtolModel::velocity)
        .def("acceleration", &evtol::EvtolModel::acceleration)
        .def("mass", &evtol::EvtolModel::mass)
        .def("gravity", &evtol::EvtolModel::gravity)
        .def("hover_thrust", &evtol::EvtolModel::hoverThrust)
        .def("max_thrust", &evtol::EvtolModel::maxThrust);

    py::class_<evtol::AltitudeSensor>(m, "AltitudeSensor")
        .def(py::init<double, unsigned int>(), py::arg("noise_std_m") = 0.0,
             py::arg("seed") = 1)
        .def("measure", &evtol::AltitudeSensor::measure,
             py::arg("true_altitude_m"))
        .def("set_noise_std", &evtol::AltitudeSensor::setNoiseStd)
        .def("noise_std", &evtol::AltitudeSensor::noiseStd);

    py::enum_<evtol::FlightState>(m, "FlightState")
        .value("GROUND", evtol::FlightState::Ground)
        .value("TAKEOFF", evtol::FlightState::Takeoff)
        .value("CLIMB", evtol::FlightState::Climb)
        .value("HOVER", evtol::FlightState::Hover)
        .value("DESCEND", evtol::FlightState::Descend)
        .value("LANDING", evtol::FlightState::Landing);

    m.def("flight_state_name", [](evtol::FlightState state) {
        return std::string(evtol::flightStateName(state));
    });

    py::class_<evtol::FlightStateEstimator>(m, "FlightStateEstimator")
        .def(py::init<>())
        .def("update", &evtol::FlightStateEstimator::update,
             py::arg("altitude_m"), py::arg("velocity_mps"),
             py::arg("target_altitude_m"), py::arg("altitude_error_m"))
        .def("current", &evtol::FlightStateEstimator::current)
        .def("reset", &evtol::FlightStateEstimator::reset);
}
