#pragma once

namespace evtol {

// REQ-F-003, REQ-F-004, REQ-F-005, DESIGN-003
class PidController {
public:
    PidController(double kp, double ki, double kd, double min_output,
                  double max_output);

    double calculateError(double target, double current) const;
    double update(double error, double dt_s);
    void reset();

    double lastOutput() const;
    double integral() const;

    double kp() const;
    double ki() const;
    double kd() const;
    double minOutput() const;
    double maxOutput() const;

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;
    double integral_;
    double previous_error_;
    double last_output_;
    bool has_previous_error_;
};

}  // namespace evtol
