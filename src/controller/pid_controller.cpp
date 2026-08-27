#include "pid_controller.hpp"

#include <algorithm>

namespace evtol {

PidController::PidController(double kp, double ki, double kd, double min_output,
                             double max_output)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      min_output_(min_output),
      max_output_(max_output),
      integral_(0.0),
      previous_error_(0.0),
      last_output_(0.0),
      has_previous_error_(false) {}

double PidController::calculateError(double target, double current) const {
    return target - current;
}

double PidController::update(double error, double dt_s) {
    if (dt_s <= 0.0) {
        return last_output_;
    }

    const double p_term = kp_ * error;

    double d_term = 0.0;
    if (has_previous_error_) {
        d_term = kd_ * (error - previous_error_) / dt_s;
    }

    const double i_candidate = integral_ + error * dt_s;
    double output = p_term + ki_ * i_candidate + d_term;
    output = std::clamp(output, min_output_, max_output_);

    // Integrator anti-windup: freeze I when saturated in the same direction.
    const bool saturate_high = output >= max_output_ && error > 0.0;
    const bool saturate_low = output <= min_output_ && error < 0.0;
    if (!saturate_high && !saturate_low) {
        integral_ = i_candidate;
        output = std::clamp(p_term + ki_ * integral_ + d_term, min_output_,
                            max_output_);
    }

    previous_error_ = error;
    has_previous_error_ = true;
    last_output_ = output;
    return last_output_;
}

void PidController::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    last_output_ = 0.0;
    has_previous_error_ = false;
}

double PidController::lastOutput() const { return last_output_; }
double PidController::integral() const { return integral_; }
double PidController::kp() const { return kp_; }
double PidController::ki() const { return ki_; }
double PidController::kd() const { return kd_; }
double PidController::minOutput() const { return min_output_; }
double PidController::maxOutput() const { return max_output_; }

}  // namespace evtol
