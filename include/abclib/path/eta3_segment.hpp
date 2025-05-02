#pragma once

#include <array>
#include <functional>
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

class Eta3PathSegment {
public:
    Eta3PathSegment(const Vec3& start,
                    const Vec3& end,
                    const std::array<double,6>& eta,
                    const std::array<double,4>& kappa);

    Vec2 calcPoint(double u) const;
    Vec2 calcDeriv(double u, int order = 1) const;

    double getLength() const { return segment_length_; }
    double getLengthError() const { return length_error_estimate_; }
    double arcLength(double u) const;

private:
    Vec3 start_pose_;
    Vec3 end_pose_;
    Eigen::Matrix<double,2,8> coeffs_;
    std::function<double(double)> s_dot_;
    double segment_length_;
    double length_error_estimate_;

    void computeCoeffs(const std::array<double,6>& eta,
                       const std::array<double,4>& kappa);
    void computeLength();
};
