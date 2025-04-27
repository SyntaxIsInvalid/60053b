#include "eta3_segment.hpp"
#include <boost/math/quadrature/gauss_kronrod.hpp>
#include <cmath>
#include <iostream>

using Mat2x7 = Eigen::Matrix<double,2,7>;
using Vec7    = Eigen::Matrix<double,7,1>;
using Vec6    = Eigen::Matrix<double,6,1>;

Eta3PathSegment::Eta3PathSegment(const Vec3& start,
                                 const Vec3& end,
                                 const std::array<double,6>& eta,
                                 const std::array<double,4>& kappa)
        : start_pose_(start),
          end_pose_(end),
          coeffs_(Eigen::Matrix<double,2,8>::Zero()),
          segment_length_(std::numeric_limits<double>::quiet_NaN()),
          length_error_estimate_(std::numeric_limits<double>::quiet_NaN())
{
    computeCoeffs(eta, kappa);
    computeLength();
}

void Eta3PathSegment::computeCoeffs(const std::array<double,6>& eta,
                                    const std::array<double,4>& kappa)
{
    double ca = std::cos(start_pose_[2]);
    double sa = std::sin(start_pose_[2]);
    double cb = std::cos(end_pose_[2]);
    double sb = std::sin(end_pose_[2]);

    // constant & linear
    coeffs_(0,0) = start_pose_[0];
    coeffs_(1,0) = start_pose_[1];
    coeffs_(0,1) = eta[0] * ca;
    coeffs_(1,1) = eta[0] * sa;

    // quadratic
    coeffs_(0,2) = 0.5*eta[2]*ca - 0.5*eta[0]*eta[0]*kappa[0]*sa;
    coeffs_(1,2) = 0.5*eta[2]*sa + 0.5*eta[0]*eta[0]*kappa[0]*ca;

    // cubic
    coeffs_(0,3) = (eta[4]/6.0)*ca
                   - ((eta[0]*eta[0]*eta[0]*kappa[1] + 3*eta[0]*eta[2]*kappa[0]) / 6.0) * sa;
    coeffs_(1,3) = (eta[4]/6.0)*sa
                   + ((eta[0]*eta[0]*eta[0]*kappa[1] + 3*eta[0]*eta[2]*kappa[0]) / 6.0) * ca;

    // quartic
    {
        double t1 = 35*(end_pose_[0]-start_pose_[0]);
        double t2 = (20*eta[0] + 5*eta[2] + 2.0/3*eta[4])*ca;
        double t3 = (5*eta[0]*eta[0]*kappa[0] + 2.0/3*eta[0]*eta[0]*eta[0]*kappa[1]
                     + 2*eta[0]*eta[2]*kappa[0])*sa;
        double t4 = (15*eta[1] - 2.5*eta[3] + 1.0/6*eta[5])*cb;
        double t5 = (2.5*eta[1]*eta[1]*kappa[2] - (1.0/6)*eta[1]*eta[1]*eta[1]*kappa[3]
                     - 0.5*eta[1]*eta[3]*kappa[2])*sb;
        coeffs_(0,4) = t1 - t2 + t3 - t4 - t5;
    }
    {
        double t1 = 35*(end_pose_[1]-start_pose_[1]);
        double t2 = (20*eta[0] + 5*eta[2] + 2.0/3*eta[4])*sa;
        double t3 = (5*eta[0]*eta[0]*kappa[0] + 2.0/3*eta[0]*eta[0]*eta[0]*kappa[1]
                     + 2*eta[0]*eta[2]*kappa[0])*ca;
        double t4 = (15*eta[1] - 2.5*eta[3] + 1.0/6*eta[5])*sb;
        double t5 = (2.5*eta[1]*eta[1]*kappa[2] - (1.0/6)*eta[1]*eta[1]*eta[1]*kappa[3]
                     - 0.5*eta[1]*eta[3]*kappa[2])*cb;
        coeffs_(1,4) = t1 - t2 - t3 - t4 + t5;
    }

    // quintic
    {
        double t1 = -84*(end_pose_[0]-start_pose_[0]);
        double t2 = (45*eta[0] + 10*eta[2] + eta[4])*ca;
        double t3 = (10*eta[0]*eta[0]*kappa[0] + eta[0]*eta[0]*eta[0]*kappa[1]
                     + 3*eta[0]*eta[2]*kappa[0])*sa;
        double t4 = (39*eta[1] - 7*eta[3] + 0.5*eta[5])*cb;
        double t5 = (7*eta[1]*eta[1]*kappa[2] - 0.5*eta[1]*eta[1]*eta[1]*kappa[3]
                     - 1.5*eta[1]*eta[3]*kappa[2])*sb;
        coeffs_(0,5) = t1 + t2 - t3 + t4 + t5;
    }
    {
        double t1 = -84*(end_pose_[1]-start_pose_[1]);
        double t2 = (45*eta[0] + 10*eta[2] + eta[4])*sa;
        double t3 = (10*eta[0]*eta[0]*kappa[0] + eta[0]*eta[0]*eta[0]*kappa[1]
                     + 3*eta[0]*eta[2]*kappa[0])*ca;
        double t4 = (39*eta[1] - 7*eta[3] + 0.5*eta[5])*sb;
        double t5 = -(7*eta[1]*eta[1]*kappa[2] - 0.5*eta[1]*eta[1]*eta[1]*kappa[3]
                      - 1.5*eta[1]*eta[3]*kappa[2])*cb;
        coeffs_(1,5) = t1 + t2 + t3 + t4 + t5;
    }

    // sextic
    {
        double t1 = 70*(end_pose_[0]-start_pose_[0]);
        double t2 = (36*eta[0] + 7.5*eta[2] + 2.0/3*eta[4])*ca;
        double t3 = (7.5*eta[0]*eta[0]*kappa[0] + 2.0/3*eta[0]*eta[0]*eta[0]*kappa[1]
                     + 2*eta[0]*eta[2]*kappa[0])*sa;
        double t4 = (34*eta[1] - 6.5*eta[3] + 0.5*eta[5])*cb;
        double t5 = -(6.5*eta[1]*eta[1]*kappa[2] - 0.5*eta[1]*eta[1]*eta[1]*kappa[3]
                      - 1.5*eta[1]*eta[3]*kappa[2])*sb;
        coeffs_(0,6) = t1 - t2 + t3 - t4 + t5;
    }
    {
        double t1 = 70*(end_pose_[1]-start_pose_[1]);
        double t2 = -(36*eta[0] + 7.5*eta[2] + 2.0/3*eta[4])*sa;
        double t3 = -(7.5*eta[0]*eta[0]*kappa[0] + 2.0/3*eta[0]*eta[0]*eta[0]*kappa[1]
                      + 2*eta[0]*eta[2]*kappa[0])*ca;
        double t4 = -(34*eta[1] - 6.5*eta[3] + 0.5*eta[5])*sb;
        double t5 =  (6.5*eta[1]*eta[1]*kappa[2] - 0.5*eta[1]*eta[1]*eta[1]*kappa[3]
                      - 1.5*eta[1]*eta[3]*kappa[2])*cb;
        coeffs_(1,6) = t1 + t2 + t3 + t4 + t5;
    }

    // septic
    {
        double t1 = -20*(end_pose_[0]-start_pose_[0]);
        double t2 = (10*eta[0] + 2*eta[2] + 1.0/6*eta[4])*ca;
        double t3 = -(2*eta[0]*eta[0]*kappa[0] + 1.0/6*eta[0]*eta[0]*eta[0]*kappa[1]
                      + 0.5*eta[0]*eta[2]*kappa[0])*sa;
        double t4 = (10*eta[1] - 2*eta[3] + 1.0/6*eta[5])*cb;
        double t5 = (2*eta[1]*eta[1]*kappa[2] - 1.0/6*eta[1]*eta[1]*eta[1]*kappa[3]
                     - 0.5*eta[1]*eta[3]*kappa[2])*sb;
        coeffs_(0,7) = t1 + t2 + t3 + t4 + t5;
    }
    {
        double t1 = -20*(end_pose_[1]-start_pose_[1]);
        double t2 = (10*eta[0] + 2*eta[2] + 1.0/6*eta[4])*sa;
        double t3 = (2*eta[0]*eta[0]*kappa[0] + 1.0/6*eta[0]*eta[0]*eta[0]*kappa[1]
                     + 0.5*eta[0]*eta[2]*kappa[0])*ca;
        double t4 = (10*eta[1] - 2*eta[3] + 1.0/6*eta[5])*sb;
        double t5 = -(2*eta[1]*eta[1]*kappa[2] - 1.0/6*eta[1]*eta[1]*eta[1]*kappa[3]
                      - 0.5*eta[1]*eta[3]*kappa[2])*cb;
        coeffs_(1,7) = t1 + t2 + t3 + t4 + t5;
    }
}

void Eta3PathSegment::computeLength()
{
    // Build s_dot integrand
    Mat2x7 D1 = coeffs_.block<2,7>(0,1);
    s_dot_ = [D1](double u) {
        Vec7 p;
        for(int i = 0; i < 7; ++i) p[i] = (i+1) * std::pow(u, i);
        Vec2 v = D1 * p;
        return std::max(v.norm(), 1e-9);
    };

    try {
        boost::math::quadrature::gauss_kronrod<double,15> integrator;
        segment_length_ = integrator.integrate(s_dot_, 0.0, 1.0, 1e-9);
        length_error_estimate_ = 1e-9;
    } catch (const std::exception &e) {
        std::cerr << "Warning: Integration failed: " << e.what() << std::endl;
    }
}

Vec2 Eta3PathSegment::calcPoint(double u) const
{
    u = std::clamp(u, 0.0, 1.0);
    Eigen::Matrix<double,8,1> up;
    for(int i = 0; i < 8; ++i) up[i] = std::pow(u, i);
    return coeffs_ * up;
}

Vec2 Eta3PathSegment::calcDeriv(double u, int order) const
{
    u = std::clamp(u, 0.0, 1.0);
    if (order == 1) {
        Vec7 d;
        for(int i = 0; i < 7; ++i) d[i] = (i+1) * std::pow(u, i);
        return coeffs_.block<2,7>(0,1) * d;
    } else {
        Vec6 d2;
        d2 << 2, 6*u, 12*u*u, 20*std::pow(u,3), 30*std::pow(u,4), 42*std::pow(u,5);
        return coeffs_.block<2,6>(0,2) * d2;
    }
}
