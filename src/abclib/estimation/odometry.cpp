#include "odometry.hpp"
#include "tracking_wheel.hpp"
#include <cmath>

namespace abclib::estimation {
    Pose::Pose(double x_pos, double y_pos, double orientation) 
        : x(x_pos), y(y_pos), theta(orientation) {}

    Odometry::Odometry(hardware::TrackingWheel* vertical_wheel, hardware::TrackingWheel* horizontal_wheel, pros::IMU* imu_sensor)
        : vertical(vertical_wheel), horizontal(horizontal_wheel), imu(imu_sensor) {
        
        // Initialize previous values
        if (vertical) prev_vertical = vertical->get_distance();
        if (horizontal) prev_horizontal = horizontal->get_distance();
        if (imu) prev_imu = imu->get_rotation() * M_PI / 180.0; // Convert to radians
    }
    
    Odometry::~Odometry() {
        if (tracking_task != nullptr) {
            delete tracking_task;
            tracking_task = nullptr;
        }
    }
    
    void Odometry::init() {
        if (tracking_task == nullptr) {
            tracking_task = new pros::Task([this] {
                while (true) {
                    this->update();
                    pros::delay(10); // 10ms delay (100Hz update rate)
                }
            });
        }
    }
    
    void Odometry::stop() {
        if (tracking_task != nullptr) {
            delete tracking_task;
            tracking_task = nullptr;
        }
    }
    
    void Odometry::reset(double x, double y, double theta) {
        // Reset the pose
        current_pose = Pose(x, y, theta);
        
        // Store current values as previous values
        if (vertical) prev_vertical = vertical->get_distance();
        if (horizontal) prev_horizontal = horizontal->get_distance();
        if (imu) prev_imu = imu->get_rotation() * M_PI / 180.0;
        
        // Store current orientation as reset orientation
        theta_reset = theta;
    }
    
    Pose Odometry::getPose() const {
        return current_pose;
    }
    
    void Odometry::update() {
        // Get current sensor values
        double verticalRaw = vertical ? vertical->get_distance() : 0.0;
        double horizontalRaw = horizontal ? horizontal->get_distance() : 0.0;
        double imuRaw = imu ? (imu->get_rotation() * M_PI / 180.0) : 0.0; // Convert to radians
        
        // Calculate sensor value changes
        double deltaVertical = verticalRaw - prev_vertical;
        double deltaHorizontal = horizontalRaw - prev_horizontal;
        double deltaImu = imuRaw - prev_imu;
        
        // Update previous values
        prev_vertical = verticalRaw;
        prev_horizontal = horizontalRaw;
        prev_imu = imuRaw;
        
        // Calculate heading using IMU
        double heading = current_pose.theta + deltaImu;
        double deltaHeading = deltaImu;
        double avgHeading = current_pose.theta + deltaHeading / 2;
        
        // Get offsets
        double verticalOffset = vertical ? vertical->get_offset() : 0.0;
        double horizontalOffset = horizontal ? horizontal->get_offset() : 0.0;
        
        // Calculate local x and y
        double localX = 0.0;
        double localY = 0.0;
        if (std::abs(deltaHeading) < 1e-6) { // prevent divide by 0
            localX = deltaHorizontal;
            localY = deltaVertical;
        } else {
            localX = 2 * std::sin(deltaHeading / 2) * (deltaHorizontal / deltaHeading + horizontalOffset);
            localY = 2 * std::sin(deltaHeading / 2) * (deltaVertical / deltaHeading + verticalOffset);
        }
        
        // Calculate global x and y
        current_pose.x += localY * std::sin(avgHeading);
        current_pose.y += localY * std::cos(avgHeading);
        current_pose.x += localX * -std::cos(avgHeading);
        current_pose.y += localX * std::sin(avgHeading);
        current_pose.theta = heading;
    }
}