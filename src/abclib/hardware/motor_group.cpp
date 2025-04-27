#include "motor_group.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::hardware {
    using Vec = Eigen::VectorXd;
    using Mat = Eigen::MatrixXd;

    AdvancedMotorGroup::AdvancedMotorGroup(
        const std::vector<AdvancedMotor*>& motors_list,
        const motor_group_config& config,
        bool enable_feedforward):
        motors(motors_list),
        rotation_sensor(config.rotation),
        further_gearing(config.group_gearing),
        group_config(config),
        using_encoder(config.rotation != nullptr),
        position_pid(control::PIDConstants(config.kPs, config.kIs, config.kDs)),
        velocity_pid(control::PIDConstants(config.kPv, config.kIv, config.kDv)),
        use_feedforward(enable_feedforward),
        use_ukf(config.use_ukf),
        last_time(pros::millis() * 1e-3)
    {
        if (!motors.empty()) {
            ticks = motors[0]->get_ticks() * further_gearing;
        } else {
            ticks = 0;
        }
        if (use_ukf) init_ukf();
    }

    AdvancedMotorGroup::~AdvancedMotorGroup() {
        delete ukf;
        ukf = nullptr;
    }

    void AdvancedMotorGroup::enable_ukf(bool enable) {
        if (enable && !ukf) init_ukf();
        if (enable && !use_ukf) {
            last_time = pros::millis() * 1e-3;
            Vec state(3);
            state << get_position(), get_raw_velocity(), 0.0;
            ukf->setState(state);
        }
        use_ukf = enable;
    }

    bool AdvancedMotorGroup::is_using_ukf() const {
        return use_ukf;
    }

    void AdvancedMotorGroup::update_ukf() {
        if (!use_ukf || !ukf) return;
        double current_time = pros::millis() * 1e-3;
        double dt = current_time - last_time;
        last_time = current_time;
        if (dt < 1e-3) return;
        ukf->predict([this](const Vec& s, double d) { return processModel(s, d); }, dt);
        if (using_encoder) {
            Vec meas(2);
            meas << get_position(), get_raw_velocity();
            ukf->update(meas, [this](const Vec& s) { return measurementModel(s); });
        } else {
            Vec meas(1);
            meas << get_position();
            ukf->update(meas, [this](const Vec& s) { return measurementModel(s); });
        }
    }

    double AdvancedMotorGroup::get_estimated_position() {
        if (use_ukf && ukf) { update_ukf(); return ukf->state()(0); }
        return get_position();
    }

    double AdvancedMotorGroup::get_estimated_velocity() {
        if (use_ukf && ukf) { update_ukf(); return ukf->state()(1); }
        return get_raw_velocity();
    }

    double AdvancedMotorGroup::get_estimated_acceleration() {
        if (use_ukf && ukf) { update_ukf(); return ukf->state()(2); }
        return 0.0;
    }

    void AdvancedMotorGroup::reset_position() {
        for (auto m : motors) m->reset_position();
        if (using_encoder) rotation_sensor->reset_position();
        if (use_ukf && ukf) {
            Vec s = ukf->state(); s(0) = 0.0; ukf->setState(s);
        }
    }

    void AdvancedMotorGroup::move_voltage(double voltage) {
        for (auto m : motors) m->move_voltage(voltage);
    }

    void AdvancedMotorGroup::set_brake_mode(pros::motor_brake_mode_e_t mode) {
        for (auto m : motors) m->set_brake_mode(mode);
    }

    void AdvancedMotorGroup::brake() {
        for (auto m : motors) m->brake();
    }

    double AdvancedMotorGroup::get_ticks() { return ticks; }

    double AdvancedMotorGroup::get_raw_velocity() {
        if (using_encoder) return rotation_sensor->get_velocity();
        if (motors.empty()) return 0.0;
        double sum = 0.0;
        for (auto m : motors) sum += m->get_raw_velocity();
        return sum / motors.size();
    }

    double AdvancedMotorGroup::get_position() {
        if (using_encoder) return rotation_sensor->get_position();
        if (motors.empty()) return 0.0;
        double sum = 0.0;
        for (auto m : motors) sum += m->get_position();
        return sum / motors.size();
    }

    void AdvancedMotorGroup::set_feedforward(bool enable) {
        use_feedforward = enable;
        for (auto m : motors) m->set_feedforward(enable);
    }

    bool AdvancedMotorGroup::is_using_feedforward() const { return use_feedforward; }

    double AdvancedMotorGroup::get_current_draw() {
        double sum = 0.0;
        for (auto m : motors) sum += m->get_current_draw();
        return sum;
    }

    void AdvancedMotorGroup::rotate_to(double target_degrees, double timeout,
                                      double min_voltage, double max_voltage) {
        reset_position();
        auto start = pros::millis();
        const double thresh = 1.0, dt = 0.01;
        position_pid.reset();
        while ((pros::millis() - start) < timeout) {
            double pos = get_estimated_position();
            double vel = get_estimated_velocity();
            if (std::fabs(target_degrees - pos) <= thresh && std::fabs(vel) < 1.0) break;
            double err = target_degrees - pos;
            double out = position_pid.compute(err, dt);
            if (use_feedforward && use_ukf) out += group_config.kA * get_estimated_acceleration();
            out = std::clamp(out, min_voltage, max_voltage);
            move_voltage(out);
            pros::delay(10);
        }
        brake();
    }

    void AdvancedMotorGroup::rotate_to_task(double target_degrees, double timeout, double min_voltage, double max_voltage) {
        pros::Task([ this, target_degrees, timeout, min_voltage, max_voltage ]() {
            this->rotate_to(target_degrees, timeout, min_voltage, max_voltage);
        });
    }

    void AdvancedMotorGroup::rotate_to_velocity(double target_velocity, double timeout,
                                                double min_voltage, double max_voltage) {
        auto start = pros::millis();
        const double thresh = 1.0, dt = 0.01;
        velocity_pid.reset();
        while ((pros::millis() - start) < timeout) {
            double vel = get_estimated_velocity();
            if (std::fabs(target_velocity - vel) <= thresh) break;
            double err = target_velocity - vel;
            double out = velocity_pid.compute(err, dt);
            if (use_feedforward) {
                double ff = group_config.kS * ((target_velocity > 0) - (target_velocity < 0));
                ff += group_config.kV * target_velocity;
                if (use_ukf) ff += group_config.kA * get_estimated_acceleration();
                out += ff;
            }
            out = std::clamp(out, min_voltage, max_voltage);
            move_voltage(out);
            pros::delay(10);
        }
    }

    void AdvancedMotorGroup::rotate_to_velocity_task(double target_velocity, double timeout, double min_voltage, double max_voltage) 
        {pros::Task([ this, target_velocity, timeout, min_voltage, max_voltage ]() {
            this->rotate_to_velocity(target_velocity, timeout, min_voltage, max_voltage);
        });
    }

    Vec AdvancedMotorGroup::processModel(const Vec& state, double dt) {
        Vec ns(3);
        ns(0) = state(0) + state(1) * dt + 0.5 * state(2) * dt * dt;
        ns(1) = state(1) + state(2) * dt;
        ns(2) = state(2);
        return ns;
    }

    Vec AdvancedMotorGroup::measurementModel(const Vec& state) {
        if (using_encoder) {
            Vec m(2);
            m << state(0), state(1);
            return m;
        } else {
            Vec m(1);
            m << state(0);
            return m;
        }
    }

    void AdvancedMotorGroup::init_ukf() {
        const size_t sd = 3;
        const size_t md = using_encoder ? 2 : 1;
        ukf = new estimation::UKF(sd, md);
        ukf->setAlpha(group_config.alpha);
        ukf->setBeta(group_config.beta);
        ukf->setKappa(group_config.kappa);
        Vec s(3);
        s << get_position(), get_raw_velocity(), 0.0;
        ukf->setState(s);
        Mat Q = Mat::Zero(sd, sd);
        Q(0,0)=group_config.pos_process_noise;
        Q(1,1)=group_config.vel_process_noise;
        Q(2,2)=group_config.acc_process_noise;
        ukf->setProcessNoise(Q);
        Mat R = Mat::Zero(md, md);
        R(0,0)=group_config.pos_measurement_noise;
        if(using_encoder) R(1,1)=group_config.vel_measurement_noise;
        ukf->setMeasurementNoise(R);
    }
} // namespace abclib::hardware
