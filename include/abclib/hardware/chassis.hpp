#pragma once

#include "api.h"
#include "hardware/motor_group.hpp"
#include "tracking_wheel.hpp"
#include "estimation/odometry.hpp"
#include "util.hpp"

namespace abclib::hardware {
    struct ChassisConfig {
        hardware::AdvancedMotorGroup* left;
        hardware::AdvancedMotorGroup* right;
        double diameter;
        double track_width;
    };

    struct Sensors {
        pros::IMU* imu;
        hardware::TrackingWheel* y_encoder;
        hardware::TrackingWheel* x_encoder;
    };

    class Chassis {
        private: 
            AdvancedMotorGroup* left_motors;
            AdvancedMotorGroup* right_motors;
            pros::IMU* imu;

            std::uint32_t current_time;
            std::uint32_t previous_time;
            double delta_time;

            control::PID lateral_pid;
            control::PID angular_pid;
            estimation::Odometry odom;

            double wheel_diameter;
            double ticks;

        public: 
            Chassis(ChassisConfig chassis_config, Sensors sensors, 
                   const control::PIDConstants lateral_constants, 
                   const control::PIDConstants angular_constants);

            void drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient);
            
            void move_left_motors(double voltage);
            void move_right_motors(double voltage);
            
            void calibrate();
            void reset_chassis_position();
            double get_heading();

            void drive_straight_relative(double target_distance, double timeout = 3000,
                double lateral_min = 0, double lateral_max = 127,
                double angular_min = 0, double angular_max = 60,
                bool reset_position = false);

            void turn_to_heading(double target_heading, double timeout = 2000,
                double angular_min = 0, double angular_max = 60, 
                bool reset_position = false);
    
            void turn_relative(double angle_delta, double timeout = 2000, 
                double angular_min = 0, double angular_max = 60);

            void euclidean_move_to_pose(
                double target_x, double target_y, double target_heading,
                double total_timeout = 5000,
                double turn1_timeout = 1500,  // First turn timeout
                double drive_timeout = 2500,  // Drive timeout
                double turn2_timeout = 1000,  // Final turn timeout
                double lateral_min = 0, double lateral_max = 127,
                double angular_min = 0, double angular_max = 60);
    };
}