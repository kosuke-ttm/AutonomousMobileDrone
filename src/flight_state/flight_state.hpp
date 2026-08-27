#pragma once

#include <string>

namespace evtol {

enum class FlightState {
    Ground,
    Takeoff,
    Climb,
    Hover,
    Descend,
    Landing,
};

const char* flightStateName(FlightState state);

// REQ-F-009
class FlightStateEstimator {
public:
    FlightStateEstimator();

    FlightState update(double altitude_m, double velocity_mps,
                       double target_altitude_m, double altitude_error_m);
    FlightState current() const;
    void reset();

private:
    FlightState state_;
};

}  // namespace evtol
