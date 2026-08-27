#pragma once

namespace evtol {

struct VehicleParams {
    double mass_kg = 10.0;
    double gravity_mps2 = 9.81;
    double max_thrust_n = 250.0;
};

// Vertical-only point-mass model: m * a = T - m g - Fd  (REQ-F-006)
class EvtolModel {
public:
    explicit EvtolModel(VehicleParams params = {});

    void reset(double altitude_m = 0.0, double velocity_mps = 0.0);
    void step(double thrust_n, double disturbance_n, double dt_s);

    double altitude() const;
    double velocity() const;
    double acceleration() const;
    double mass() const;
    double gravity() const;
    double hoverThrust() const;
    double maxThrust() const;
    const VehicleParams& params() const;

private:
    VehicleParams params_;
    double altitude_m_;
    double velocity_mps_;
    double acceleration_mps2_;
};

}  // namespace evtol
