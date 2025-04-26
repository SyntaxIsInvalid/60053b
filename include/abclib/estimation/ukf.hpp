#pragma once

#include <Eigen/Dense>
#include <vector>
#include <functional>

namespace abclib::estimation {

class UKF {
public:
    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    // Constructor: initialize dimensions, state, covariances, and weights
    UKF(size_t state_dim, size_t measurement_dim);

    // Tuning parameters
    void setAlpha(double a);
    void setBeta(double b);
    void setKappa(double k);

    // Noise covariances
    void setProcessNoise(const Matrix& Q);
    void setMeasurementNoise(const Matrix& R);

    // State accessors
    void setState(const Vector& x);
    void setCovariance(const Matrix& P);
    const Vector& state() const;
    const Matrix& covariance() const;

    // UKF steps
    void predict(const std::function<Vector(const Vector&, double)>& f, double dt);
    void update(const Vector& z, const std::function<Vector(const Vector&)>& h);

private:
    // Internal utilities
    double computeLambda() const;
    void updateWeights();
    void generateSigmaPoints();

    size_t state_dim_;
    size_t measurement_dim_;
    size_t n_sigma_;

    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;

    Vector x_;
    Matrix P_;
    Matrix Q_;
    Matrix R_;

    std::vector<Vector> sigma_points_;
    std::vector<double> weights_mean_;
    std::vector<double> weights_cov_;
};

} // namespace abclib::estimation
