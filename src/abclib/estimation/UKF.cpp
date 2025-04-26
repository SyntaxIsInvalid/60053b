#include "ukf.hpp"
#include <stdexcept>
#include <Eigen/Dense>

namespace abclib::estimation {

UKF::UKF(size_t state_dim, size_t measurement_dim)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      x_(state_dim),
      P_(Matrix::Identity(state_dim, state_dim)),
      Q_(Matrix::Zero(state_dim, state_dim)),
      R_(Matrix::Zero(measurement_dim, measurement_dim)),
      alpha_(1.0),
      beta_(2.0),
      kappa_(0.0)
{
    lambda_ = computeLambda();
    n_sigma_ = 2 * state_dim_ + 1;
    sigma_points_.resize(n_sigma_, Vector(state_dim_));
    weights_mean_.resize(n_sigma_);
    weights_cov_.resize(n_sigma_);
    updateWeights();
}

// Parameter setters
void UKF::setAlpha(double a) {
    alpha_ = a;
    lambda_ = computeLambda();
    updateWeights();
}

void UKF::setBeta(double b) {
    beta_ = b;
    lambda_ = computeLambda();
    updateWeights();
}

void UKF::setKappa(double k) {
    kappa_ = k;
    lambda_ = computeLambda();
    updateWeights();
}

void UKF::setProcessNoise(const Matrix& Q) {
    if (Q.rows() != state_dim_ || Q.cols() != state_dim_)
        throw std::invalid_argument("Process noise Q dimension mismatch");
    Q_ = Q;
}

void UKF::setMeasurementNoise(const Matrix& R) {
    if (R.rows() != measurement_dim_ || R.cols() != measurement_dim_)
        throw std::invalid_argument("Measurement noise R dimension mismatch");
    R_ = R;
}

void UKF::setState(const Vector& x) {
    if ((size_t)x.size() != state_dim_)
        throw std::invalid_argument("State dimension mismatch");
    x_ = x;
}

void UKF::setCovariance(const Matrix& P) {
    if (P.rows() != state_dim_ || P.cols() != state_dim_)
        throw std::invalid_argument("Covariance dimension mismatch");
    P_ = P;
}

// Accessors
const UKF::Vector& UKF::state() const {
    return x_;
}

const UKF::Matrix& UKF::covariance() const {
    return P_;
}

// Prediction step
void UKF::predict(const std::function<Vector(const Vector&, double)>& f, double dt) {
    generateSigmaPoints();
    std::vector<Vector> prop(n_sigma_);
    for (size_t i = 0; i < n_sigma_; ++i) {
        prop[i] = f(sigma_points_[i], dt);
    }
    // Mean
    x_.setZero();
    for (size_t i = 0; i < n_sigma_; ++i) {
        x_ += weights_mean_[i] * prop[i];
    }
    // Covariance
    P_.setZero();
    for (size_t i = 0; i < n_sigma_; ++i) {
        Vector d = prop[i] - x_;
        P_ += weights_cov_[i] * (d * d.transpose());
    }
    P_ += Q_;
    sigma_points_ = std::move(prop);
}

// Update step
void UKF::update(const Vector& z, const std::function<Vector(const Vector&)>& h) {
    if ((size_t)z.size() != measurement_dim_)
        throw std::invalid_argument("Measurement dimension mismatch");

    // Transform sigma points into measurement space
    std::vector<Vector> sig_z(n_sigma_, Vector(measurement_dim_));
    for (size_t i = 0; i < n_sigma_; ++i) {
        sig_z[i] = h(sigma_points_[i]);
    }
    // Predicted measurement mean
    Vector z_pred = Vector::Zero(measurement_dim_);
    for (size_t i = 0; i < n_sigma_; ++i) {
        z_pred += weights_mean_[i] * sig_z[i];
    }
    // Innovation covariance
    Matrix S = Matrix::Zero(measurement_dim_, measurement_dim_);
    for (size_t i = 0; i < n_sigma_; ++i) {
        Vector d = sig_z[i] - z_pred;
        S += weights_cov_[i] * (d * d.transpose());
    }
    S += R_;
    // Cross-covariance
    Matrix Pxz = Matrix::Zero(state_dim_, measurement_dim_);
    for (size_t i = 0; i < n_sigma_; ++i) {
        Vector dx = sigma_points_[i] - x_;
        Vector dz = sig_z[i] - z_pred;
        Pxz += weights_cov_[i] * (dx * dz.transpose());
    }
    // Kalman gain
    Eigen::LDLT<Matrix> ldlt(S);
    if (ldlt.info() != Eigen::Success)
        throw std::runtime_error("Innovation covariance not positive definite");
    Matrix K = Pxz * ldlt.solve(Matrix::Identity(measurement_dim_, measurement_dim_));
    // Update state and covariance
    x_ += K * (z - z_pred);
    P_ -= K * S * K.transpose();
}

// Private helpers

double UKF::computeLambda() const {
    return alpha_ * alpha_ * (state_dim_ + kappa_) - state_dim_;
}

void UKF::updateWeights() {
    lambda_ = computeLambda();
    double c = state_dim_ + lambda_;
    weights_mean_[0] = lambda_ / c;
    weights_cov_[0]  = weights_mean_[0] + (1 - alpha_ * alpha_ + beta_);
    double w = 0.5 / c;
    for (size_t i = 1; i < n_sigma_; ++i) {
        weights_mean_[i] = w;
        weights_cov_[i]  = w;
    }
}

void UKF::generateSigmaPoints() {
    double c = state_dim_ + lambda_;
    Eigen::LLT<Matrix> llt(c * P_);
    if (llt.info() != Eigen::Success)
        throw std::runtime_error("Covariance is not positive definite");
    Matrix L = llt.matrixL();
    sigma_points_[0] = x_;
    for (size_t i = 0; i < state_dim_; ++i) {
        Vector offset = L.col(i);
        sigma_points_[1 + i]            = x_ + offset;
        sigma_points_[1 + state_dim_ + i] = x_ - offset;
    }
}

} // namespace abclib::estimation

