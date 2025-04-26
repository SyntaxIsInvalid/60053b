#pragma once

/**
 * @brief Constants for PID controller tuning
 */

namespace abclib::control {
    struct PIDConstants {
        double kP;  // Proportional gain
        double kI;  // Integral gain
        double kD;  // Derivative gain

        /**
         * @brief Constructor for PID constants
         * @param p Proportional gain
         * @param i Integral gain
         * @param d Derivative gain
         */
        PIDConstants(double p, double i, double d)
            : kP(p), kI(i), kD(d) {}
    };

    /**
     * @brief PID controller implementation
     */
    class PID {
    private:
        PIDConstants constants;  // PID gains
        double integral = 0;     // Accumulated integral term
        double prev_error = 0;   // Previous error value for derivative calculation
        double prev_dt = 0;      // Previous time step

    public:
        /**
         * @brief Constructor
         * @param pidConstants The PID gains
         */
        PID(const PIDConstants& pidConstants)
            : constants(pidConstants) {}

        /**
         * @brief Computes the PID control output
         * @param error Current error value
         * @param dt Time step since last computation
         * @return The control output
         */
        double compute(double error, double dt);

        /**
         * @brief Resets the controller state
         */
        void reset();
    };
}