#include "advanced_motor.hpp"
#include <cmath>
#include <algorithm>

namespace abclib::hardware {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

AdvancedMotor::AdvancedMotor(
    pros::Motor* base_motor,
    const MotorConfig& motor_config,
    double gearing,
    bool enable_feedforward)
    : motor_(base_motor),
      rotation_sensor_(motor_config.rotation),
      position_pid_(abclib::control::PIDConstants{
          motor_config.kPs,
          motor_config.kIs,
          motor_config.kDs
      }),
      velocity_pid_(abclib::control::PIDConstants{
          motor_config.kPv,
          motor_config.kIv,
          motor_config.kDv
      }),
      config_(motor_config),
      ticks_(find_ticks(base_motor, gearing)),
      gearing_(gearing),
      using_encoder_(rotation_sensor_ != nullptr),
      use_feedforward_(enable_feedforward),
      ukf_(nullptr),
      use_ukf_(motor_config.use_ukf),
      last_time_(pros::millis() * 1e-3)
{
    if (use_ukf_) {
        init_ukf();
    }
}

AdvancedMotor::~AdvancedMotor() = default;

void AdvancedMotor::init_ukf() {
    const size_t state_dim = 3;
    const size_t measurement_dim = using_encoder_ ? 2 : 1;

    ukf_ = std::make_unique<estimation::UKF>(state_dim, measurement_dim);
    ukf_->setAlpha(config_.alpha);
    ukf_->setBeta(config_.beta);
    ukf_->setKappa(config_.kappa);

    Vector initial_state(3);
    initial_state << get_position(), get_raw_velocity(), 0.0;
    ukf_->setState(initial_state);

    Matrix Q = Matrix::Zero(state_dim, state_dim);
    Q(0, 0) = config_.pos_process_noise;
    Q(1, 1) = config_.vel_process_noise;
    Q(2, 2) = config_.acc_process_noise;
    ukf_->setProcessNoise(Q);

    Matrix R = Matrix::Zero(measurement_dim, measurement_dim);
    R(0, 0) = config_.pos_measurement_noise;
    if (using_encoder_) {
        R(1, 1) = config_.vel_measurement_noise;
    }
    ukf_->setMeasurementNoise(R);
}

void AdvancedMotor::enable_ukf(bool enable) {
    if (enable && !ukf_) {
        init_ukf();
    }
    if (enable && !use_ukf_) {
        last_time_ = pros::millis() * 1e-3;
        Vector state(3);
        state << get_position(), get_raw_velocity(), 0.0;
        ukf_->setState(state);
    }
    use_ukf_ = enable;
}

bool AdvancedMotor::is_using_ukf() const {
    return use_ukf_;
}

void AdvancedMotor::update_ukf() {
    if (!use_ukf_ || !ukf_) return;

    double current_time = pros::millis() * 1e-3;
    double dt = current_time - last_time_;
    last_time_ = current_time;
    if (dt < 1e-3) return;

    ukf_->predict([this](const Vector& st, double d) {
        return this->processModel(st, d);
    }, dt);

    if (using_encoder_) {
        Vector meas(2);
        meas << get_position(), get_raw_velocity();
        ukf_->update(meas, [this](const Vector& st) {
            return this->measurementModel(st);
        });
    } else {
        Vector meas(1);
        meas << get_position();
        ukf_->update(meas, [this](const Vector& st) {
            return this->measurementModel(st);
        });
    }
}

// static
double AdvancedMotor::find_ticks(pros::Motor* motor, double further_gearing) {
    switch (motor->get_gearing()) {
        case pros::MotorGearset::blue:  return 300 * further_gearing;
        case pros::MotorGearset::green: return 900 * further_gearing;
        case pros::MotorGearset::red:   return 1800 * further_gearing;
    }
    return 0;
}

Vector AdvancedMotor::processModel(const Vector& state, double dt) {
    double theta = state(0);
    double omega = state(1);
    double alpha = state(2);
    Vector next(3);
    next(0) = theta + omega * dt + 0.5 * alpha * dt * dt;
    next(1) = omega + alpha * dt;
    next(2) = alpha;
    return next;
}

Vector AdvancedMotor::measurementModel(const Vector& state) {
    if (using_encoder_) {
        Vector m(2);
        m << state(0), state(1);
        return m;
    } else {
        Vector m(1);
        m << state(0);
        return m;
    }
}

// Accessors
 double AdvancedMotor::get_estimated_position() {
    if (use_ukf_ && ukf_) {
        update_ukf();
        return ukf_->state()(0);
    }
    return get_position();
}

 double AdvancedMotor::get_estimated_velocity() {
    if (use_ukf_ && ukf_) {
        update_ukf();
        return ukf_->state()(1);
    }
    return get_raw_velocity();
}

 double AdvancedMotor::get_estimated_acceleration() {
    if (use_ukf_ && ukf_) {
        update_ukf();
        return ukf_->state()(2);
    }
    return 0.0;
}

void AdvancedMotor::reset_position() {
    motor_->tare_position();
    if (using_encoder_) rotation_sensor_->reset_position();
    if (use_ukf_ && ukf_) {
        Vector s = ukf_->state();
        s(0) = 0.0;
        ukf_->setState(s);
    }
}

// const accessors
 double AdvancedMotor::get_ticks() const {
    return ticks_;
}

 double AdvancedMotor::get_current_draw() const {
    return motor_->get_current_draw();
}

void AdvancedMotor::move_voltage(double voltage) {
    motor_->move(voltage);
}

void AdvancedMotor::brake() {
    motor_->brake();
}

void AdvancedMotor::set_brake_mode(pros::motor_brake_mode_e_t brake_mode) {
    motor_->set_brake_mode(brake_mode);
}

 double AdvancedMotor::get_raw_velocity() const {
    return using_encoder_ ? rotation_sensor_->get_velocity()
                          : motor_->get_actual_velocity();
}

 double AdvancedMotor::get_position() const {
    return using_encoder_ ? rotation_sensor_->get_position()
                          : motor_->get_position();
}

void AdvancedMotor::rotate_to(double target_degrees, double timeout,
                              double min_voltage, double max_voltage) {
    reset_position();
    uint32_t start = pros::millis();
    const double threshold = 1.0;
    const double dt = 0.01;
    position_pid_.reset();
    while ((pros::millis() - start) < timeout) {
        double pos = get_estimated_position();
        double vel = get_estimated_velocity();
        if (std::fabs(target_degrees - pos) <= threshold &&
            std::fabs(vel) < 1.0) break;
        double err = target_degrees - pos;
        double out = position_pid_.compute(err, dt);
        if (use_feedforward_ && use_ukf_) {
            out += config_.kA * get_estimated_acceleration();
        }
        out = std::clamp(out, min_voltage, max_voltage);
        move_voltage(out);
        pros::delay(10);
    }
    brake();
}

void AdvancedMotor::rotate_to_task(double target_degrees,
    double timeout,
    double min_voltage,
    double max_voltage) {pros::Task([ this, target_degrees, timeout, min_voltage, max_voltage ]() {
        this->rotate_to(target_degrees, timeout, min_voltage, max_voltage);
    });
}

void AdvancedMotor::rotate_to_velocity(double target_velocity, double timeout,
                                       double min_voltage, double max_voltage) {
    uint32_t start = pros::millis();
    const double threshold = 1.0;
    const double dt = 0.01;
    velocity_pid_.reset();
    while ((pros::millis() - start) < timeout) {
        double vel = get_estimated_velocity();
        if (std::fabs(target_velocity - vel) <= threshold) break;
        double err = target_velocity - vel;
        double out = velocity_pid_.compute(err, dt);
        if (use_feedforward_) {
            double ff = config_.kS * ((target_velocity > 0) - (target_velocity < 0));
            ff += config_.kV * target_velocity;
            if (use_ukf_) ff += config_.kA * get_estimated_acceleration();
            out += ff;
        }
        out = std::clamp(out, min_voltage, max_voltage);
        move_voltage(out);
        pros::delay(10);
    }
}

void AdvancedMotor::rotate_to_velocity_task(double target_velocity, double timeout, double min_voltage, double max_voltage) {
    pros::Task([ this, target_velocity, timeout, min_voltage, max_voltage ]() {
        this->rotate_to_velocity(target_velocity, timeout, min_voltage, max_voltage);
    });
}

void AdvancedMotor::set_feedforward(bool enable) {
    use_feedforward_ = enable;
}

bool AdvancedMotor::is_using_feedforward() const {
    return use_feedforward_;
}

} // namespace abclib::hardware
