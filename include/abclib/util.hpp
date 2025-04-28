#pragma once

#include <cmath>

namespace util {
    inline double normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI); // Shift the range to [0, 2π)
        if (angle < 0.0) {
            angle += 2.0 * M_PI; // Handle negative angles
        }
        return angle - M_PI; // Shift the range back to [-π, π)
    }

    // Convert degrees to radians
    inline double deg_to_rad(double degree) {
        return degree * M_PI/180;
    }

    // Convert radians to degrees
    inline double rad_to_deg(double radian) {
        return radian * 180/M_PI;
    }

    inline static int sgn(double x) {
        return (x > 0) - (x < 0);
    }
}