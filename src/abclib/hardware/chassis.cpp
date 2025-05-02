#include "abclib/hardware/chassis.hpp"
#include "api.h"

using namespace abclib;

namespace abclib::hardware {

    Chassis::Chassis(ChassisConfig chassis_config, Sensors sensors, const control::PIDConstants lateral_constants, const control::PIDConstants angular_constants): 
        left_motors(chassis_config.left),
        right_motors(chassis_config.right),
        wheel_diameter(chassis_config.diameter),
        imu(sensors.imu),
        lateral_pid(lateral_constants),
        angular_pid(angular_constants),
        ticks(chassis_config.left -> get_ticks()),
        odom(sensors.y_encoder, sensors.x_encoder, sensors.imu) {}

    void Chassis::drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient) {
        // Scale the inputs using the coefficients
        double scaled_throttle = throttle * throttle_coefficient;
        double scaled_turn = turn * turn_coefficient;

        // Compute motor power for left and right sides
        double left_power = scaled_throttle + scaled_turn;
        double right_power = scaled_throttle - scaled_turn;

        // Apply a deadband to avoid jitter
        if (fabs(scaled_throttle) + fabs(scaled_turn) < 2) {
            left_motors -> brake();
            right_motors -> brake();
        } else {
            // Apply motor power
            left_motors -> move_voltage(left_power);
            right_motors -> move_voltage(right_power);
        }
    }

    void Chassis::move_left_motors(double voltage) {
        left_motors -> move_voltage(voltage);
    }

    void Chassis::move_right_motors(double voltage) {
        right_motors -> move_voltage(voltage);
    }

    void Chassis::calibrate() {
        left_motors -> reset_position();
        right_motors -> reset_position();
        imu -> reset();
        odom.init();
    }

    void Chassis::reset_chassis_position() {
        left_motors -> reset_position();
        right_motors -> reset_position();
        imu -> set_heading(0);
    }

    double Chassis::get_heading() {
        return imu -> get_heading();
    }

    void Chassis::drive_straight_relative(double target_distance, double timeout,
        double lateral_min, double lateral_max,
        double angular_min, double angular_max,
        bool reset_position) {
        std::uint32_t start_time = pros::millis();
        const double threshold = 0.5;
        const double dt = 0.01; // 100Hz

        // Reset position if requested
        if (reset_position) {
            reset_chassis_position();
        }

        // Get starting pose from odometry
        estimation::Pose start_pose = odom.getPose();
        double start_heading = start_pose.theta;
        double initial_x = start_pose.x;
        double initial_y = start_pose.y;

        // Reset PID controllers
        lateral_pid.reset();
        angular_pid.reset();

        while ((pros::millis() - start_time) < timeout) {
            // Get current position from odometry
            estimation::Pose current_pose = odom.getPose();

            // Calculate distance traveled using vector projection for accuracy
            double dx = current_pose.x - initial_x;
            double dy = current_pose.y - initial_y;
            double distance_traveled = dx * std::cos(start_heading) + dy * std::sin(start_heading);

            // Check if we've reached the target
            if (fabs(target_distance - distance_traveled) <= threshold) {
                break;
            }

            // Calculate lateral error and PID output
            double lateral_error = target_distance - distance_traveled;
            double lateral_output = lateral_pid.compute(lateral_error, dt);

            // Calculate angular error (maintain original heading) and PID output
            double angular_error = start_heading - current_pose.theta;
            // Normalize heading error to [-PI, PI]
            angular_error = util::normalizeAngle(angular_error);
            double angular_output = angular_pid.compute(angular_error, dt);

            // Apply min/max limits for forward/backward movement
            double lateral_abs = std::abs(lateral_output);
            lateral_abs = std::clamp(lateral_abs, lateral_min, lateral_max);
            lateral_output = (lateral_error >= 0) ? lateral_abs : -lateral_abs;

            // Apply min/max limits for turning
            double angular_abs = std::abs(angular_output);
            angular_abs = std::clamp(angular_abs, angular_min, angular_max);
            angular_output = (angular_error >= 0) ? angular_abs : -angular_abs;

            // Send power to motors
            move_left_motors(lateral_output - angular_output);
            move_right_motors(lateral_output + angular_output);

            pros::delay(10);
        }

        // Stop motors when done
        left_motors -> brake();
        right_motors -> brake();
    }

    void Chassis::turn_to_heading(double target_heading, double timeout,
        double angular_min, double angular_max, bool reset_position) {
        std::uint32_t start_time = pros::millis();
        const double threshold = 1.0; // Degree threshold for completion
        const double dt = 0.01; // 100Hz

        // Convert target heading to radians for internal calculations
        double target_heading_rad = target_heading * M_PI / 180.0;

        // Reset position if requested
        if (reset_position) {
            imu -> set_heading(0);
        }

        // Reset PID controller
        angular_pid.reset();

        while ((pros::millis() - start_time) < timeout) {
            // Get current heading in radians
            double current_heading_deg = get_heading();
            double current_heading_rad = current_heading_deg * M_PI / 180.0;

            // Calculate angular error (in radians)
            double angular_error = target_heading_rad - current_heading_rad;

            // Normalize heading error to [-PI, PI] for shortest turn direction
            angular_error = util::normalizeAngle(angular_error);

            // Check if we've reached the target
            if (fabs(angular_error * 180.0 / M_PI) <= threshold) {
                break;
            }

            // Calculate PID output
            double angular_output = angular_pid.compute(angular_error, dt);

            // Apply min/max limits
            double angular_abs = std::abs(angular_output);
            angular_abs = std::clamp(angular_abs, angular_min, angular_max);
            angular_output = (angular_error >= 0) ? angular_abs : -angular_abs;

            // Apply turn power to motors (opposite directions for turning)
            move_left_motors(-angular_output);
            move_right_motors(angular_output);

            pros::delay(10);
        }

        // Stop motors when done
        left_motors -> brake();
        right_motors -> brake();
    }

    void Chassis::turn_relative(double angle_delta, double timeout, double angular_min, double angular_max) {
        // Get current heading
        double current_heading = get_heading();

        // Calculate target heading by adding the delta
        double target_heading = current_heading + angle_delta;

        // Normalize the target heading using normalizeAngle (convert to radians, normalize, convert back)
        target_heading = util::normalizeAngle(util::deg_to_rad(target_heading)) * 180.0 / M_PI;

        // Call the absolute turning function
        turn_to_heading(target_heading, timeout, angular_min, angular_max, false);
    }

    void Chassis::euclidean_move_to_pose(
        double target_x, double target_y, double target_heading,
        double total_timeout,
        double turn1_timeout,  // First turn timeout
        double drive_timeout,  // Drive timeout
        double turn2_timeout,  // Final turn timeout
        double lateral_min, double lateral_max,
        double angular_min, double angular_max) {
        
        uint32_t start_time = pros::millis();
        uint32_t elapsed_time;
        
        // Get current position and calculate path
        estimation::Pose current_pose = odom.getPose();
        double dx = target_x - current_pose.x;
        double dy = target_y - current_pose.y;
        double angle_to_target_rad = std::atan2(dy, dx);
        double angle_to_target_deg = angle_to_target_rad * 180.0 / M_PI;
        double distance_to_target = std::sqrt(dx*dx + dy*dy);
    
        // 1. Turn to face the target
        turn_to_heading(angle_to_target_deg, turn1_timeout, angular_min, angular_max);
        
        // Check total timeout
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout) return;
        
        // 2. Drive straight to the target position
        drive_straight_relative(distance_to_target, drive_timeout, lateral_min, lateral_max);
        
        // Check total timeout again
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout) return;
        
        // 3. Turn to the final heading
        turn_to_heading(target_heading, turn2_timeout, angular_min, angular_max);
    }
}