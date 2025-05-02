#pragma once

#include <vector>
#include <utility>
#include "abclib/profiling/toppra/arc_length.hpp"
#include "abclib/profiling/toppra/constraint.hpp"
#include <stdexcept>


class TOPPRA {
public:
    // Constructor: takes your path and per-stage constraints, plus # of grid points N
    TOPPRA(const ArcLengthPath& path,
           const std::vector<ConstraintStage>& stages,
           int N);

    // Compute the backward‐pass controllable sets K[0…N]
    // sd_start, sd_end are start/end speed limits (e.g. 0 for rest-to-rest)
    void computeControllableSets(double sd_start, double sd_end);

    // Compute forward‐pass optimal (u,x) and time‐stamps t[i]
    void computeTimeOptimalProfile(double sd_start, double sd_end);

    // Access results
    const std::vector<double>& timeStamps() const { return t_; }
    const std::vector<double>& positions()  const { return s_; }
    const std::vector<double>& speeds2()    const { return x_; }
    const std::vector<double>& accels()     const { return u_; }

private:
    ArcLengthPath        path_;
    std::vector<ConstraintStage> stages_;
    int                  N_;

    // grids and results
    std::vector<double>  s_;    // s[0…N]
    std::vector<double>  t_;    // t[0…N]
    std::vector<double>  x_;    // squared speeds x[i]=sd_i^2
    std::vector<double>  u_;    // accelerations u[i]

    // controllable sets [ℓ_i,h_i]
    std::vector<std::pair<double,double>> K_;

    // helper to build uniform s-grid
    void initGrid();

    // the computeKi function from earlier
    static std::pair<double,double> computeKi(
            const ConstraintStage& st,
            const std::pair<double,double>& Knext,
            double delta);

    // small epsilon
    static constexpr double kEps = 1e-12;
};
