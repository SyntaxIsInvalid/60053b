#include "pid.hpp" 

namespace abclib::control {
    double PID::compute(double error, double dt) {
        if (prev_dt > 0) {
            // Calculate integral using trapezoidal rule
            integral += dt * (error + prev_error) / 2;

            // Calculate derivative using backward difference
            double derivative = (error - prev_error) / dt;

            // Compute PID output
            double output = constants.kP * error + constants.kI * integral + constants.kD * derivative;

            // Update previous error
            prev_error = error;
            prev_dt = dt;

            return output;
        }

        // First run: no derivative computation
        prev_error = error;
        prev_dt = dt;

        return constants.kP * error; // Proportional only
    }

    void PID::reset() {
        integral = 0;
        prev_error = 0;
        prev_dt = 0;
    }
}