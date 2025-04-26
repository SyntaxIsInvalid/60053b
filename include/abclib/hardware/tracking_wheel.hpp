#pragma once
#include "api.h"

namespace abclib::hardware {
    class TrackingWheel {
        private:
            pros::Rotation* rotation_sensor;
            double diameter, radius;
            double offset;
        public:
            TrackingWheel(pros::Rotation* rotation_sensor, double diameter, double offset);
            
            double get_distance();
            double get_offset();
            void reset();
    };
}