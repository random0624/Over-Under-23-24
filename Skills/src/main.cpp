
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

void autoShoot(){
	while(true){
		shoot(-60);
	}
}

void pidTurn(float target, int time){
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
	gyro.tare_rotation();
	brake();
}

void curveTurnBck(float target, float c, int time){
	gyro.tare_rotation();
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
	for (int i = 0; i < time; i+=20){
		errorLast = error;
		error = target - (gyro.get_rotation());
		errorTotal += error;
		pTerm = error * kp;
		iTerm = ki * errorTotal;
		dTerm = kd * (error - errorLast);
		power = pTerm + iTerm + dTerm;
		left_frt.move(-power);
		left_bck.move(-power);
		right_frt.move(-power*c);
		right_bck.move(-power*c);
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


void autonomous() {
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	                			//Start shooting
	
	drive(96, 80);
	pros::delay(800);
	intake.move(-100);
	drive(96,80);
	pros::delay(200);
	intake.move(0);
	brake();
	pros::delay(400);
	InchDrive(-21, 110);
	pros::delay(300);
	pidTurn(106, 1000);
	locking_right.set_value(true);
	/*shoot(-85);
	pros::delay(28000);
	shoot(0);*/
	pidTurn(54, 1000);
	InchDrive(53, 100);
	pros::delay(200);
	pidTurn(-45, 1000);
	InchDrive(115, 100);
	piston_right.set_value(true);
	pros::delay(300);
	pidTurn(-60, 1000);
	drive(100, 115);
	pros::delay(1600);
	brake();
	pros::delay(300);
	InchDrive(-15, 100);
	pros::delay(300);
	drive(100, 100);
	pros::delay(600);
	brake();
	pros::delay(300);
	InchDrive(-17, 80);
	pros::delay(300);
	pidTurn(-85, 1000);
	pros::delay(300);
	InchDrive(60, 100);
	pros::delay(500);
	pidTurn(64, 1000);
	piston_left.set_value(true);
	piston_right.set_value(true);
	drive(120, 120);
	pros::delay(1000);
	brake();
	InchDrive(-15, 70);
	piston_left.set_value(false);
	piston_right.set_value(false);
	InchDrive(-40, 70);
	pros::delay(500);
	pidTurn(-96, 1100);
	InchDrive(40, 90);
	pros::delay(500);
	piston_left.set_value(true);
	piston_right.set_value(true);
	curveTurn(80, 0.65, 1200, true);
	pros::delay(500);
	drive(120, 120);
	pros::delay(1000);
	brake();
	pros::delay(600);
	InchDrive(-5, 80);
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
				shoot(-100);
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