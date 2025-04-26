#pragma once

#include "api.h"
#include <cmath>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include "pid.hpp"
#include "ukf.hpp"

namespace abclib::hardware {

struct MotorConfig {
    double kS = 0.0;    // Static friction constant
    double kV = 0.0;    // Velocity constant
    double kA = 0.0;    // Acceleration constant

    // PID constants for position control
    double kPs = 0.0;
    double kIs = 0.0;
    double kDs = 0.0;

    // PID constants for velocity control
    double kPv = 0.0;
    double kIv = 0.0;
    double kDv = 0.0;

    // UKF parameters
    bool use_ukf = false;
    double pos_process_noise = 0.01;
    double vel_process_noise = 0.1;
    double acc_process_noise = 1.0;
    double pos_measurement_noise = 0.1;
    double vel_measurement_noise = 0.2;
    double alpha = 1.0;
    double beta  = 2.0;
    double kappa = 0.0;

    pros::Rotation* rotation = nullptr;
};

class AdvancedMotor {
public:
    AdvancedMotor(pros::Motor* base_motor,
                  const MotorConfig& motor_config,
                  double gearing = 1.0,
                  bool enable_feedforward = true);
    ~AdvancedMotor();

    void enable_ukf(bool enable);
    bool is_using_ukf() const;
    void update_ukf();

    static double find_ticks(pros::Motor* motor, double further_gearing);

    double get_estimated_position();
    double get_estimated_velocity();
    double get_estimated_acceleration();

    void reset_position();
    double get_ticks() const;
    double get_current_draw() const;

    void move_voltage(double voltage);
    void brake();
    void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);

    double get_raw_velocity() const;
    double get_position() const;

    void rotate_to(double target_degrees,
                   double timeout,
                   double min_voltage = 0.0,
                   double max_voltage = 127.0);
    void rotate_to_task(double target_degrees,
                        double timeout,
                        double min_voltage = 0.0,
                        double max_voltage = 127.0);

    void rotate_to_velocity(double target_velocity,
                             double timeout,
                             double min_voltage = 0.0,
                             double max_voltage = 127.0);
    void rotate_to_velocity_task(double target_velocity,
                                  double timeout,
                                  double min_voltage = 0.0,
                                  double max_voltage = 127.0);

    void set_feedforward(bool enable);
    bool is_using_feedforward() const;

private:
    pros::Motor* motor_;
    pros::Rotation* rotation_sensor_;
    control::PID position_pid_;
    control::PID velocity_pid_;
    MotorConfig config_;
    double ticks_;
    double gearing_;
    bool using_encoder_;
    bool use_feedforward_;

    std::unique_ptr<estimation::UKF> ukf_;
    bool use_ukf_;
    double last_time_;

    Eigen::VectorXd processModel(const Eigen::VectorXd& state, double dt);
    Eigen::VectorXd measurementModel(const Eigen::VectorXd& state);

    void init_ukf();
};

} // namespace abclib::hardware
