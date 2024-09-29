#include "api.h"
#ifndef _PROS_DRIVETRAIN_H_
#define _PROS_DRIVETRAIN_H_
void MoveBaseTank();


void drive(float l, float r);

void motor_coast();
void motor_hold();
void brake();
#endif