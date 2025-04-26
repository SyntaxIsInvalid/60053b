#pragma once

#include <Eigen/Dense>
#include <vector>
#include "advanced_motor.hpp"
#include "pid.hpp"
#include "ukf.hpp"

namespace abclib::hardware {
    using Vec = Eigen::VectorXd;
    using Mat = Eigen::MatrixXd;

    struct motor_group_config {
        double kS = 0;    // Static friction constant
        double kV = 0;    // Velocity constant
        double kA = 0;    // Acceleration constant

        // PID constants, position
        double kPs = 0;
        double kIs = 0;
        double kDs = 0;

        // PID constants, velocity
        double kPv = 0;
        double kIv = 0;
        double kDv = 0;

        // UKF parameters
        bool use_ukf = false;
        double pos_process_noise = 0.01;
        double vel_process_noise = 0.1;
        double acc_process_noise = 1.0;
        double pos_measurement_noise = 0.1;
        double vel_measurement_noise = 0.2;
        double alpha = 1.0;
        double beta = 2.0;
        double kappa = 0.0;

        const double group_gearing = 1;
        pros::Rotation* rotation = nullptr;
    };

    class AdvancedMotorGroup {
    private:
        std::vector<AdvancedMotor*> motors;
        pros::Rotation* rotation_sensor;
        control::PID position_pid;
        control::PID velocity_pid;

        motor_group_config group_config;
        double ticks;
        double further_gearing;
        bool use_feedforward;
        bool using_encoder;

        // UKF members
        estimation::UKF* ukf = nullptr;
        bool use_ukf = false;
        double last_time = 0;

        Vec processModel(const Vec& state, double dt);
        Vec measurementModel(const Vec& state);
        void init_ukf();

    public:
        AdvancedMotorGroup(
            const std::vector<AdvancedMotor*>& motors_list,
            const motor_group_config& config,
            bool enable_feedforward = true);
        ~AdvancedMotorGroup();

        void enable_ukf(bool enable);
        bool is_using_ukf() const;
        void update_ukf();
        double get_estimated_position();
        double get_estimated_velocity();
        double get_estimated_acceleration();
        void reset_position();
        void move_voltage(double voltage);
        void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);
        void brake();
        double get_ticks();
        double get_raw_velocity();
        double get_position();
        void set_feedforward(bool enable);
        bool is_using_feedforward() const;
        double get_current_draw();
        void rotate_to(double target_degrees, double timeout,
                       double min_voltage = 0, double max_voltage = 127);
        void rotate_to_task(double target_degrees, double timeout,
                            double min_voltage = 0, double max_voltage = 127);
        void rotate_to_velocity(double target_velocity, double timeout,
                                 double min_voltage = 0, double max_voltage = 127);
        void rotate_to_velocity_task(double target_velocity, double timeout,
                                      double min_voltage = 0, double max_voltage = 127);
    };
}