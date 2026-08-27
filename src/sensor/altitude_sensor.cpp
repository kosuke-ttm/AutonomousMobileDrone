#include "altitude_sensor.hpp"

#include <cmath>

namespace evtol {

AltitudeSensor::AltitudeSensor(double noise_std_m, unsigned int seed)
    : noise_std_m_(noise_std_m), seed_(seed), rng_state_(seed) {}

double AltitudeSensor::measure(double true_altitude_m) {
    return true_altitude_m + nextNoise();
}

void AltitudeSensor::setNoiseStd(double noise_std_m) {
    noise_std_m_ = noise_std_m;
}

double AltitudeSensor::noiseStd() const { return noise_std_m_; }

double AltitudeSensor::nextNoise() {
    if (noise_std_m_ == 0.0) {
        return 0.0;
    }

    // Deterministic Box-Muller using a portable LCG (REQ-NF-001).
    rng_state_ = 1664525u * rng_state_ + 1013904223u;
    const double u1 =
        (static_cast<double>(rng_state_ & 0x7fffffffu) + 1.0) / 2147483648.0;
    rng_state_ = 1664525u * rng_state_ + 1013904223u;
    const double u2 =
        (static_cast<double>(rng_state_ & 0x7fffffffu) + 1.0) / 2147483648.0;
    const double n = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
    return n * noise_std_m_;
}

}  // namespace evtol
