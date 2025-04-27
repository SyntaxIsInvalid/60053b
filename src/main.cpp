#include "main.h"
#include <Eigen/Dense>
#include "abclib/abclib.hpp"
using namespace abclib;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor FR(1, pros::MotorGearset::blue);
pros::Motor MR(2, pros::MotorGearset::blue);
pros::Motor BR(3, pros::MotorGearset::blue);

pros::Motor FL(4, pros::MotorGearset::blue);
pros::Motor ML(5, pros::MotorGearset::blue);
pros::Motor BL(6, pros::MotorGearset::blue);

hardware::MotorConfig left_motor_group_config = {};
hardware::MotorConfig right_motor_group_config = {};

hardware::AdvancedMotor advanced_FR(&FR, right_motor_group_config);
hardware::AdvancedMotor advanced_MR(&MR, right_motor_group_config);
hardware::AdvancedMotor advanced_BR(&BR, right_motor_group_config);

hardware::AdvancedMotor advanced_FL(&FL, left_motor_group_config);
hardware::AdvancedMotor advanced_ML(&ML, left_motor_group_config);
hardware::AdvancedMotor advanced_BL(&BL, left_motor_group_config);

hardware::motor_group_config right_group_config = {};
hardware::motor_group_config left_group_config = {};
std::vector<hardware::AdvancedMotor*> right_motor_group = {&advanced_FR, &advanced_MR, &advanced_BR};
std::vector<hardware::AdvancedMotor*> left_motor_group = {&advanced_FL, &advanced_ML, &advanced_BL};
hardware::AdvancedMotorGroup rightAdvMotorGroup(right_motor_group, right_group_config);
hardware::AdvancedMotorGroup leftAdvMotorGroup(left_motor_group, left_group_config);

pros::IMU imu(7);
pros::Rotation x_rotation(8);
pros::Rotation y_rotation(9);
hardware::TrackingWheel x_tracker(&x_rotation, 2, 2);
hardware::TrackingWheel y_tracker(&y_rotation, 2, 2);

hardware::ChassisConfig chassis_constant(
	&rightAdvMotorGroup,
	&leftAdvMotorGroup,
	2.75,
	12
);

hardware::Sensors sensors {
	&imu,
	&x_tracker,
	&y_tracker
};

control::PIDConstants lateral_pid(
    1.0,  // kP - proportional
    0,  // kI - integral
    0   // kD - derivative
);

control::PIDConstants angular_pid(
    0,  // kP - proportional
    0,  // kI - integral
    0   // kD - derivative
);

hardware::Chassis drive_train(chassis_constant, sensors, lateral_pid, angular_pid);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	Eigen::Vector3d v(1,2,3);
	std::cout << 1 << "\n";
	while (1) {
		int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		drive_train.drive(throttle, turn, 1, .65);
		pros::delay(20);                               // Run for 20 ms then update
	}
}