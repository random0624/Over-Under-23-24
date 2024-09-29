#include "main.h"

void InchDrive(float target, int speed) {
    left_bck.tare_position();
    float x = 0;
    
    while (fabs(x) <= fabs(target)) {
        x = left_bck.get_position() * 3.1415926 * 4.0;
        int direction = (target >= 0) ? 1 : -1; // Determine the direction of motion
        
        left_frt.move(direction * speed);
        left_bck.move(direction * speed);
        right_frt.move(direction * speed);
        right_bck.move(direction * speed);
        
        pros::delay(20);
    }
    
    brake();
}
