 #include "main.h"
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
pros::Motor right_bck(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor right_up(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor kicker_1(4, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor kicker_2(19, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor intake(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Imu gyro(20);