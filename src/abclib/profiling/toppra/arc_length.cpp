#include "abclib/profiling/toppra/arc_length.hpp"

ArcLengthPath::ArcLengthPath(const Eta3Path& path) : path_(path) {
    int n = path_.numSegments();
    cum_length_.resize(n+1);
    cum_length_[0] = 0.0;
    for (int i = 0; i < n; ++i) {
        cum_length_[i+1] = cum_length_[i] + path_.getSegment(i).getLength();
    }
}

double ArcLengthPath::totalLength() const {
    return cum_length_.back();
}

void ArcLengthPath::locate(double s, int& idx, double& u) const {
    double S = totalLength();
    s = std::clamp(s, 0.0, S);

    // find segment via upper_bound
    auto it = std::upper_bound(cum_length_.begin(), cum_length_.end(), s);
    int raw = static_cast<int>(it - cum_length_.begin()) - 1;

    // clamp to [0, numSegments-1]
    idx = std::clamp(raw, 0, path_.numSegments() - 1);

    // compute local arc-length within that segment
    double s0 = cum_length_[idx];
    double s_local = s - s0;

    // invert true arc-length to get u in [0,1]
    u = invertSegmentParam(idx, s_local);
}

// Invert arc-length within segment using bisection on partial length
double ArcLengthPath::invertSegmentParam(int idx, double s_local) const {
    const auto& seg = path_.getSegment(idx);
    double L = seg.getLength();
    double lower = 0.0, upper = 1.0, um = std::clamp(s_local/L, 0.0, 1.0);
    for (int it = 0; it < 15; ++it) {
        um = 0.5*(lower + upper);
        double len = seg.arcLength(um);
        if (len > s_local) upper = um; else lower = um;
    }
    return 0.5*(lower + upper);
}

Eigen::Vector3d ArcLengthPath::q(double s) const {
    int idx; double u;
    locate(s, idx, u);
    auto [x, y, theta] = path_.calcPathPose(double(idx) + u);
    return {x, y, theta};
}

Eigen::Vector3d ArcLengthPath::qdot(double s) const {
    int idx; double u;
    locate(s, idx, u);
    const auto& seg = path_.getSegment(idx);
    auto dpdu = seg.calcDeriv(u, 1);
    double dsdu = dpdu.norm();
    // normalized derivative
    Eigen::Vector2d dpos = dpdu / dsdu;
    // curvature = (x' y'' - y' x'') / |p'|^3
    auto d2pdu2 = seg.calcDeriv(u, 2);
    double kappa = (dpdu.x()*d2pdu2.y() - dpdu.y()*d2pdu2.x())
                   / std::pow(dsdu, 3);
    return {dpos.x(), dpos.y(), kappa};
}

Eigen::Vector3d ArcLengthPath::qddot(double s) const {
    int idx; double u;
    locate(s, idx, u);
    const auto& seg = path_.getSegment(idx);
    auto dpdu = seg.calcDeriv(u, 1);
    auto d2pdu2 = seg.calcDeriv(u, 2);
    double dsdu = dpdu.norm();
    double d2sdu2 = dpdu.dot(d2pdu2) / dsdu;
    // second derivative w.r.t s: (p''*dsdu - p'*d2sdu2)/(dsdu^3)
    Eigen::Vector2d d2pdds2 = (d2pdu2*dsdu - dpdu*d2sdu2)
                              / std::pow(dsdu, 3);
    return {d2pdds2.x(), d2pdds2.y(), 0.0};
}
