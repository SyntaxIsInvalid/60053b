#include "pid.hpp" 

namespace abclib::control {
    double PID::compute(double error, double dt) {
        if (prev_dt > 0) {
            integral += dt * (error + prev_error) / 2;
            double derivative = (error - prev_error) / dt;

            double output = constants.kP * error + constants.kI * integral + constants.kD * derivative;
            prev_error = error;
            prev_dt = dt;

            return output;
        }

        prev_error = error;
        prev_dt = dt;

        return constants.kP * error;
    }

    void PID::reset() {
        integral = 0;
        prev_error = 0;
        prev_dt = 0;
    }
}