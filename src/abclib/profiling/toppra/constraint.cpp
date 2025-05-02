#include "abclib/profiling/toppra/constraint.hpp"
#include <limits>
#include <algorithm>

std::pair<double,double> ConstraintStage::admissibleX() const {
    double x_min = -std::numeric_limits<double>::infinity();
    double x_max =  std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < alpha.size(); ++j) {
        const double a = alpha[j];
        const double b = beta[j];
        const double c = gamma[j];

        // If alpha != 0, we can always pick u to satisfy alpha * u + beta * x + gamma <= 0
        if (std::abs(a) > 1e-12) {
            continue;
        }
        // Now alpha approx 0 implies constraint reduces to beta x + gamma <= 0
        if (std::abs(b) < 1e-12) {
            // implies c <= 0 must hold
            if (c > 0) {
                // empty interval
                return {1.0, 0.0};
            } else {
                continue;
            }
        }
        // beta x + gamma <= 0  implies  x <=  –gamma/beta  or  x >= –gamma/beta
        const double bound = -c / b;
        if (b > 0) {
            x_max = std::min(x_max, bound);
        } else {
            x_min = std::max(x_min, bound);
        }
    }
    return {x_min, x_max};
}


std::pair<double, double> ConstraintStage::admissibleU(double x) const {
    double u_min = -std::numeric_limits<double>::infinity();
    double u_max =  std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < alpha.size(); ++j) {
        double a = alpha[j], b = beta[j], c = gamma[j];
        double rhs = -b * x - c;
        if (std::abs(a) < 1e-12) {
            if (rhs < 0) return {1, 0}; // empty
            continue;
        }
        double bound = rhs / a;
        if (a > 0) {
            u_max = std::min(u_max, bound);
        } else {
            u_min = std::max(u_min, bound);
        }
    }
    return {u_min, u_max};
}
