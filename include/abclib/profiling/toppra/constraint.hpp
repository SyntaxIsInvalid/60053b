#pragma once

#include <vector>
#include <utility>

// A single stage's linear constraints on (u, x):
//   alpha_j * u + beta_j * x + gamma_j <= 0
struct ConstraintStage {
    std::vector<double> alpha;  // coefficients on acceleration u
    std::vector<double> beta;   // coefficients on squared-speed x
    std::vector<double> gamma;  // constant terms

    // Compute admissible x range [x_min, x_max]
    // such that exists u satisfying all inequalities.
    std::pair<double, double> admissibleX() const;

    // Given a fixed x, compute admissible u range [u_min, u_max]
    std::pair<double, double> admissibleU(double x) const;
};
