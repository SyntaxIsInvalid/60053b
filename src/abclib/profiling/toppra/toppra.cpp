#include "abclib/profiling/toppra/toppra.hpp"
#include <algorithm>
#include <cmath>


TOPPRA::TOPPRA(const ArcLengthPath& path,
               const std::vector<ConstraintStage>& stages,
               int N)
        : path_(path), stages_(stages), N_(N)
{
    initGrid();
    K_.resize(N_+1);
    x_.resize(N_+1);
    u_.resize(N_);
    t_.resize(N_+1);
}

void TOPPRA::initGrid() {
    s_.resize(N_+1);
    double S = path_.totalLength();
    for (int i = 0; i <= N_; ++i) {
        s_[i] = S * double(i) / N_;
    }
}

void TOPPRA::computeControllableSets(double sd_start, double sd_end) {
    // backward pass
    K_[N_] = {sd_end*sd_end, sd_end*sd_end};
    for (int i = N_-1; i >= 0; --i) {
        double Δ = s_[i+1] - s_[i];
        K_[i] = computeKi(stages_[i], K_[i+1], Δ);
        if (K_[i].first > K_[i].second) {
            throw std::runtime_error("Infeasible at stage " + std::to_string(i));
        }
    }
    // check start speed
    if (sd_start*sd_start < K_[0].first - kEps ||
        sd_start*sd_start > K_[0].second + kEps) {
        throw std::runtime_error("Start speed not in K[0]");
    }
}

void TOPPRA::computeTimeOptimalProfile(double sd_start, double sd_end) {
    // forward pass
    x_[0] = sd_start*sd_start;
    t_[0] = 0.0;
    for (int i = 0; i < N_; ++i) {
        double Δ = s_[i+1] - s_[i];
        auto [u_min, u_max] = stages_[i].admissibleU(x_[i]);
        // clamp interval to ensure x+2 \delta u \in K_[i+1]
        double l = (K_[i+1].first - x_[i])/(2*Δ);
        double h = (K_[i+1].second - x_[i])/(2*Δ);
        double umax = std::min(u_max, h);
        double uopt = umax;
        uopt = std::max(uopt, l);  // ensure >= l
        u_[i] = uopt;
        x_[i+1] = x_[i] + 2*Δ*uopt;
        // time increment
        double v0 = std::sqrt(x_[i]);
        double v1 = std::sqrt(x_[i+1]);
        t_[i+1] = t_[i] + 2*Δ/(v0 + v1);
    }
}

std::pair<double,double> TOPPRA::computeKi(
        const ConstraintStage& st,
        const std::pair<double,double>& Knext,
        double delta)
{
    const double lnext = Knext.first;
    const double hnext = Knext.second;

    // start with admissible X_i
    auto [xmin, xmax] = st.admissibleX();

    // 1) enforce x + 2 \delta * u >= lnext for alpha >0
    for (size_t j = 0; j < st.alpha.size(); ++j) {
        double a = st.alpha[j];
        if (a <= kEps) continue;
        double b = st.beta[j], c = st.gamma[j];
        double denom = 1.0 - 2.0*delta*b/a;
        double rhs   = lnext + 2.0*delta*c/a;
        if (std::abs(denom) < kEps) {
            if (rhs > 0.0) return {1.0, 0.0};
        } else if (denom > 0.0) {
            xmin = std::max(xmin, rhs/denom);
        } else {
            xmax = std::min(xmax, rhs/denom);
        }
    }

    // 2) enforce x + 2 \delta * u <= hnext for <0
    for (size_t j = 0; j < st.alpha.size(); ++j) {
        double a = st.alpha[j];
        if (a >= -kEps) continue;
        double b = st.beta[j], c = st.gamma[j];
        double denom = 1.0 - 2.0*delta*b/a;
        double rhs   = hnext + 2.0*delta*c/a;
        if (std::abs(denom) < kEps) {
            if (rhs < 0.0) return {1.0, 0.0};
        } else if (denom > 0.0) {
            xmax = std::min(xmax, rhs/denom);
        } else {
            xmin = std::max(xmin, rhs/denom);
        }
    }

    return {xmin, xmax};
}
