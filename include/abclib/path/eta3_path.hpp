#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <tuple>
#include "eta3_segment.hpp"
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

class Eta3Path {
public:
    explicit Eta3Path(std::vector<Eta3PathSegment>&& segments);

    double getTotalLength() const;

    double getTotalLengthError() const;

    Vec2 calcPathPoint(double u) const;

    std::tuple<double, double, double> calcPathPose(double u) const;

    int numSegments() const { return static_cast<int>(segs_.size()); }
    const Eta3PathSegment& getSegment(int i) const { return segs_.at(i); }

private:
    std::vector<Eta3PathSegment> segs_;
};
