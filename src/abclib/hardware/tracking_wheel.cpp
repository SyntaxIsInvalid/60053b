#include "tracking_wheel.hpp"
#include <cmath>

namespace abclib::hardware {
    TrackingWheel::TrackingWheel(pros::Rotation* rotation_sensor, double diameter, double offset):
        rotation_sensor(rotation_sensor),
        diameter(diameter),
        radius(diameter/2),
        offset(offset)
    {}

    double TrackingWheel::get_distance() {
        return radius * rotation_sensor->get_angle() * M_PI / 180.0;
    }

    double TrackingWheel::get_offset() {
        return offset;
    }

    void TrackingWheel::reset() {
        rotation_sensor->reset();
    }
}