#pragma once

namespace evtol {

// Virtual altitude sensor. V1.0 defaults to zero noise (REQ-F-002).
class AltitudeSensor {
public:
    explicit AltitudeSensor(double noise_std_m = 0.0, unsigned int seed = 1);

    double measure(double true_altitude_m);
    void setNoiseStd(double noise_std_m);
    double noiseStd() const;

private:
    double noise_std_m_;
    unsigned int seed_;
    unsigned int rng_state_;

    double nextNoise();
};

}  // namespace evtol
