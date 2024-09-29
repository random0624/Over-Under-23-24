#include "main.h"

void motor_hold(){
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void motor_coast(){
	left_frt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_bck.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_frt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_bck.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void brake(){
	left_frt.move(0);
	left_bck.move(0);
	right_frt.move(0);
	right_bck.move(0);
	right_up.move(0);
	left_up.move(0);
}

void drive(float l, float r){
	left_frt.move(l);
	left_bck.move(l);
	left_up.move(l);
	right_frt.move(r);
	right_bck.move(r);
	right_up.move(r);

}

void MoveBaseTank(){
	float l;
	float r;
	l = (master.get_analog(ANALOG_LEFT_Y));
	r = (master.get_analog(ANALOG_RIGHT_Y));
	drive(l, r);
	pros::screen::print(pros::E_TEXT_LARGE, 1, "Left Power: %3f", left_bck.get_actual_velocity());
	pros::screen::print(pros::E_TEXT_LARGE, 3, "Left Power: %3f", right_bck.get_actual_velocity());
}
