
#include "main.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include "Pragma.h"
#include "pros/screen.hpp"

void initialize() { 
	//pros::lcd::initialize();
	//on_center_button();
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void shoot(float s){
	kicker_1.move(s);
	kicker_2.move(s);
}

void pidTurn(float target, int time){
	gyro.tare_rotation();
	float kp = 1.05;
	float ki = 0;
	float kd = 0.15;
	float error = 0;
	float errorTotal = 0;
	float errorLast = 0;
	float pTerm = 0;
	float iTerm = 0;
	float dTerm = 0;
	float power = 0; 
	for (int i = 0; i < time; i+=20){
		errorLast = error;
		error = target - (gyro.get_rotation());
		errorTotal += error;
		pTerm = error * kp;
		iTerm = ki * errorTotal;
		dTerm = kd * (error - errorLast);
		power = pTerm + iTerm + dTerm;
		left_frt.move(power);
		left_bck.move(power);
		right_frt.move(-power);
		right_bck.move(-power);
		pros::delay(20);
	}
	brake();
}

void curveTurn(float target, float c, int time, bool forward) {
    gyro.tare_rotation();  // Reset gyro rotation to 0
    float kp = 4;
    float ki = 0;
    float kd = 0;
    float error = 0;
    float errorTotal = 0;
    float errorLast = 0;
    float pTerm = 0;
    float iTerm = 0;
    float dTerm = 0;
    float power = 0;
    int direction = forward ? 1 : -1;  // Determine direction multiplier

    for (int i = 0; i < time; i += 20) {
        errorLast = error;
        error = target - gyro.get_rotation();
        errorTotal += error;
        pTerm = error * kp;
        iTerm = ki * errorTotal;
        dTerm = kd * (error - errorLast);
        power = (pTerm + iTerm + dTerm) * direction;  // Apply direction multiplier here

        // Apply adjusted power to motors, considering the curve factor (c) and direction
        left_frt.move(power);
        left_bck.move(power);
        right_frt.move(power * c);  // Reverse the direction for one side to turn
        right_bck.move(power * c);

        pros::delay(20);
    }
    brake();  // Ensure you have defined a function to stop the motors
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

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
void autonomous() {
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	InchDrive(46, 120);
	pros::delay(400);
	/*intake.move(120);
	InchDrive(10, 100);
	pros::delay(300);
	InchDrive(-8, 100);
	intake.move(0);
	InchDrive(-54, 80);
	pros::delay(600);
	pidTurn(-51, 700);
	locking_left.set_value(true);
	pros::delay(200);
	drive(-58, -58);
	pros::delay(660);
	brake();
	pros::delay(300);
	pidTurn(-54, 800);
	locking_left.set_value(false);
	pros::delay(200);
	drive(-100, -100);
	pros::delay(600);
	brake();
	pros::delay(400);
	InchDrive(8, 100);
	pros::delay(300);
	pidTurn(172, 900);
	pros::delay(200);
	drive(100, 100);
	pros::delay(450);
	brake();
	pros::delay(300);
	InchDrive(-17, 80);
	pros::delay(400);
	pidTurn(-120, 1600);
	InchDrive(56, 80);
	pros::delay(300);
	pidTurn(45, 1000);
	InchDrive(42, 60);*/
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
void opcontrol() {
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	kicker_1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	kicker_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    double *x;
    double *y;
	bool piston_on = false;
	bool shooting = false;
	bool hang_out = false;
	bool locking_on = false;

	while (true) {
		MoveBaseTank();
		
		if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))){
			if(!shooting){
				shoot(-95);
				shooting = true;
			}
			else{
				shoot(0);
				shooting = false;
			}
		}
		if((master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))){
			intake.move(127);
		}
		else if((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))){
			
			intake.move(-127);
		}
		else{
			intake.move(0);
		}

		if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))){
			if(!piston_on){
				piston_right.set_value(true);
				piston_left.set_value(true);
				piston_on = true;
			}
			else{
				
				piston_right.set_value(false);
				piston_left.set_value(false);
				piston_on = false;
			}
		}

		if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))){
			if(!locking_on){
				locking_right.set_value(true);
				locking_left.set_value(true);
				
				locking_on = true;
			}
			else{
				
				locking_right.set_value(false);
				locking_left.set_value(false);
				locking_on = false;
			}
		}
		
		if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))){
			if(!hang_out){
				h1.set_value(true);
				h2.set_value(true);
				h3.set_value(true);
				h4.set_value(true);
				
				hang_out = true;
			}
			else{
				
				h1.set_value(false);
				h2.set_value(false);
				h3.set_value(false);
				h4.set_value(false);
				hang_out = false;
			}
		}
	}
	pros::delay(20);
}
