#pragma once
#include "api.h"

namespace abclib::hardware {
    class TrackingWheel;  // Forward declaration
}

namespace abclib::estimation {
    struct Pose {
        double x = 0.0;       // x position
        double y = 0.0;       // y position
        double theta = 0.0;   // orientation in radians
        
        Pose() = default;
        Pose(double x_pos, double y_pos, double orientation);
    };

    class Odometry {
        private:
            hardware::TrackingWheel* vertical;     // Y-axis tracking wheel
            hardware::TrackingWheel* horizontal;   // X-axis tracking wheel  
            pros::IMU* imu;
            
            // Current pose
            Pose current_pose;
            
            // Previous sensor values
            double prev_vertical = 0.0;
            double prev_horizontal = 0.0;
            double prev_imu = 0.0;
            
            // Orientation at last reset
            double theta_reset = 0.0;
            
            // Background task pointer
            pros::Task* tracking_task = nullptr;
            
        public:
            Odometry(hardware::TrackingWheel* vertical_wheel, hardware::TrackingWheel* horizontal_wheel, pros::IMU* imu_sensor);
            ~Odometry();
            
            void init();
            void stop();
            void reset(double x = 0.0, double y = 0.0, double theta = 0.0);
            Pose getPose() const;
            void update();
    };
}