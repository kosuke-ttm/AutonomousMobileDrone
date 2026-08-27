#include "evtol_model.hpp"

#include <algorithm>

namespace evtol {

EvtolModel::EvtolModel(VehicleParams params)
    : params_(params),
      altitude_m_(0.0),
      velocity_mps_(0.0),
      acceleration_mps2_(0.0) {}

void EvtolModel::reset(double altitude_m, double velocity_mps) {
    altitude_m_ = std::max(0.0, altitude_m);
    velocity_mps_ = velocity_mps;
    acceleration_mps2_ = 0.0;
}

void EvtolModel::step(double thrust_n, double disturbance_n, double dt_s) {
    if (dt_s <= 0.0) {
        return;
    }

    const double thrust = std::clamp(thrust_n, 0.0, params_.max_thrust_n);
    const double net_force =
        thrust - params_.mass_kg * params_.gravity_mps2 - disturbance_n;
    acceleration_mps2_ = net_force / params_.mass_kg;

    velocity_mps_ += acceleration_mps2_ * dt_s;
    altitude_m_ += velocity_mps_ * dt_s;

    if (altitude_m_ <= 0.0) {
        altitude_m_ = 0.0;
        if (velocity_mps_ < 0.0) {
            velocity_mps_ = 0.0;
        }
        if (acceleration_mps2_ < 0.0) {
            acceleration_mps2_ = 0.0;
        }
    }
}

double EvtolModel::altitude() const { return altitude_m_; }
double EvtolModel::velocity() const { return velocity_mps_; }
double EvtolModel::acceleration() const { return acceleration_mps2_; }
double EvtolModel::mass() const { return params_.mass_kg; }
double EvtolModel::gravity() const { return params_.gravity_mps2; }
double EvtolModel::hoverThrust() const {
    return params_.mass_kg * params_.gravity_mps2;
}
double EvtolModel::maxThrust() const { return params_.max_thrust_n; }
const VehicleParams& EvtolModel::params() const { return params_; }

}  // namespace evtol
