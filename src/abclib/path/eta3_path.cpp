#include "abclib/path/eta3_path.hpp"
#include <cmath>
#include <iostream>

Eta3Path::Eta3Path(std::vector<Eta3PathSegment>&& segments)
        : segs_(std::move(segments)) {}

double Eta3Path::getTotalLength() const {
    double sum = 0.0;
    for (const auto& seg : segs_) {
        double L = seg.getLength();
        if (std::isfinite(L)) {
            sum += L;
        } else {
            std::cerr << "Warning: NaN segment length\n";
        }
    }
    return sum;
}

double Eta3Path::getTotalLengthError() const {
    double sum = 0.0;
    for (const auto& seg : segs_) {
        double E = seg.getLengthError();
        if (std::isfinite(E)) {
            sum += E;
        } else {
            std::cerr << "Warning: NaN segment error\n";
        }
    }
    return sum;
}

Vec2 Eta3Path::calcPathPoint(double u) const {
    int n = static_cast<int>(segs_.size());
    int idx;
    double ul;

    if (std::abs(u - n) < 1e-9) {
        idx = n - 1;
        ul  = 1.0;
    } else {
        idx = static_cast<int>(std::floor(u));
        idx = std::clamp(idx, 0, n - 1);
        ul  = std::clamp(u - idx, 0.0, 1.0);
    }
    return segs_[idx].calcPoint(ul);
}

std::tuple<double, double, double> Eta3Path::calcPathPose(double u) const {
    int n = static_cast<int>(segs_.size());
    int idx;
    double ul;

    if (std::abs(u - n) < 1e-9) {
        idx = n - 1;
        ul  = 1.0;
    } else {
        idx = static_cast<int>(std::floor(u));
        idx = std::clamp(idx, 0, n - 1);
        ul  = std::clamp(u - idx, 0.0, 1.0);
    }
    Vec2 p = segs_[idx].calcPoint(ul);
    Vec2 d = segs_[idx].calcDeriv(ul, 1);
    return { p.x(), p.y(), std::atan2(d.y(), d.x()) };
}