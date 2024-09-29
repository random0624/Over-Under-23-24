#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut h1('A', false);
pros::ADIDigitalOut h2('H', false);
pros::ADIDigitalOut h3('B', false);
pros::ADIDigitalOut h4('C', false);
pros::ADIDigitalOut piston_left('D', false);
pros::ADIDigitalOut piston_right('G', false);
pros::ADIDigitalOut locking_right('F', false);
pros::ADIDigitalOut locking_left('E', false);

pros::Motor left_frt(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor left_bck(15, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor left_up(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor right_frt(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor right_bck(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor right_up(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor kicker_1(4, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor kicker_2(19, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor intake(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Imu gyro(17);


pros::MotorGroup leftMotors({left_frt, left_bck});
pros::MotorGroup rightMotors({right_frt, right_bck});

ASSET(example_txt);

lemlib::Drivetrain drivetrain{
    &leftMotors, // left drivetrain motors
    &rightMotors, // right drivetrain motors
    9.5, // track width
    4.0, // wheel diameter
    300, // wheel rpm
    8
};

lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &gyro // inertial sensor
};

lemlib::ControllerSettings lateralController {
    26, // kP
    0,  //kI
    60, // kD
    3,  //anti windup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    127 // slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {
    3.5, // kP
    0,   //kI
    30, // kD
    3,  //Anti Windup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
void drive(float l, float r){
	left_frt.move(l);
	left_bck.move(l);
	right_frt.move(r);
	right_bck.move(r);
}
void brake(){
	left_frt.move(0);
	left_bck.move(0);
	right_frt.move(0);
	right_bck.move(0);
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
 
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
	chassis.setPose(0,0,0);
    pros::Task screenTask(screen); // create a task to print the position to the screen
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
ASSET(matchloads_txt);
ASSET(long_txt);
ASSET(preloads_txt);
ASSET(score1_txt);
ASSET(score2_txt);
ASSET(score3_txt);
ASSET(score4_txt);
ASSET(score5_txt);
ASSET(score6_txt);
void autonomous() {
    left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    /*chassis.turnTo(53, 53, 1000); // turn to the point (53, 53) with a timeout of 1000 ms
    chassis.turnTo(-20, 32, 1500, true); // turn to the point (-20, 32) with the back of the robot facing the point, and a timeout of 1500 ms
    chassis.turnTo(10, 0, 1000, false, 50); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

	chassis.moveTo(53, 53, 1000); // move to the point (53, 53) with a timeout of 1000 ms
    chassis.moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50*/
    chassis.setPose(-45,-58,-30);
    chassis.follow(preloads_txt, 15, 1500);
    pros::delay(800);
    chassis.follow(matchloads_txt, 11, 900, false);
    pros::delay(1500);
    chassis.turnTo(45,0,1000);
    pros::delay(500);
    piston_right.set_value(true);
    pros::delay(1000);
    piston_right.set_value(false);
    chassis.follow(long_txt, 9.8, 5800);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.moveToPose(57.592, -13,0,1000);
    chassis.follow(score1_txt, 11, 5000, false);
    pros::delay(500);
    chassis.follow(score2_txt, 11, 3000, false);
    piston_left.set_value(true);
    piston_right.set_value(true);
    pros::delay(2000);
    chassis.follow(score3_txt,11,5000,true);
    piston_left.set_value(false);
    piston_right.set_value(false);
    pros::delay(2000);
    chassis.follow(score4_txt,11,5000,false);
    pros::delay(2000);
    piston_left.set_value(false);
    piston_right.set_value(false);
    chassis.follow(score5_txt,12,5000,false);
    pros::delay(2000);
    piston_left.set_value(true);
    piston_right.set_value(true);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 void MoveBaseTank(){
	float l;
	float r;
	l = (master.get_analog(ANALOG_LEFT_Y));
	r = (master.get_analog(ANALOG_RIGHT_Y));
	drive(l, r);
	pros::screen::print(pros::E_TEXT_LARGE, 1, "Left Power: %3f", left_bck.get_actual_velocity());
	pros::screen::print(pros::E_TEXT_LARGE, 3, "Right Power: %3f", right_bck.get_actual_velocity());
}
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		MoveBaseTank();

		pros::delay(20);
	}
}
