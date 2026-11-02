#include "main.h"
//pros::MotorGroup intake({1,2});
pros::MotorGroup lift({17, -13}); //Lift {L,R}
//pros::MotorGroup driveL({19,20}); //Left {F,B}
//pros::MotorGroup driveR({12,11}); //Right {F,B}
pros::Motor tilt(20);
ez::Piston piston('H');