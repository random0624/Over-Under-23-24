#include "main.h"

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
}
