#include "main.h"

//Hardware definitions
//pros::MotorGroup intake({1,2});
pros::MotorGroup lift({17, -13}, pros::v5::MotorGears::blue); //Lift {L,R}
//pros::MotorGroup driveL({19,20}); //Left {F,B}
//pros::MotorGroup driveR({12,11}); //Right {F,B}
pros::Motor tilt(20);
ez::Piston piston('H');


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Drivetrain control systems & config
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Chassis constructor, need to change values
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-18, -19},  // Left Chassis Ports Front/Back(negative port will reverse it!)
    {12, 11},  // Right Chassis Ports Front/Back(negative port will reverse it!)

    21,      // IMU Port
    3.25,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    200.0 // Wheel RPM = cartridge * (motor gear / wheel gear)
);  
// Uncomment the trackers you're using here!

// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Lift control systems
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Quick lift height initializer to find and set 0; do NOT run in another while with other functions because it has its own while
void liftInit() {
  lift.move(20); //rotates lift until significant power is drawn (string is taught)
  while ((lift.get_power(0) + lift.get_power(1))/2 < 0.5) {
    pros::delay(10);
  }
  lift.tare_position();
  lift.move(0);
}

//Height limits with slow zone
int lift_speed_limit(double limit_distance, double slow_zone, double min_speed) {
  if(limit_distance <= 0) return 0; //Stop at limit
  if(limit_distance >= slow_zone) return 127; //Full speed
  return (int)(min_speed + (127 - min_speed) * (limit_distance / slow_zone)); //Scale speed between min_speed and 127
}

//Cascade lift controls
void liftControl(int target_voltage) {
  //IMPORTANT - might have to change these constants now that cartridge is defined as blue
  constexpr double LIFT_MIN   = 20;      // bottom limit
  constexpr double LIFT_MAX   = 1600;   // top limit
  constexpr double SLOW_ZONE  = 250;    // distance from each end where slowing begins
  constexpr int    MIN_SPEED  = 20;     // Speed in slow zone
  constexpr int    RAMP_STEP  = 10;     // change per loop; lower = smoother

  static int lift_cmd = 0;  // current command, persists between loops

  double position = lift.get_position();
  int up_max   = lift_speed_limit(LIFT_MAX - position, SLOW_ZONE, MIN_SPEED);
  int down_max = lift_speed_limit(position - LIFT_MIN, SLOW_ZONE, MIN_SPEED);

  // Clamp the target to limits
  target_voltage = std::clamp(target_voltage, -down_max, up_max);

  // ---- Ramp toward target ----
  if (lift_cmd < target_voltage) {
    lift_cmd = std::min(lift_cmd + RAMP_STEP, target_voltage);
  } else if (lift_cmd > target_voltage) {
    lift_cmd = std::max(lift_cmd - RAMP_STEP, target_voltage);
  }

  lift.move(lift_cmd);

}

//PID controller for lift (WIP)
ez::PID liftPIDController(
  0.1, //kP
  0.0, //kI
  0.0, //kD
  0.0, //start_i (error at which kI windup starts)
  "LiftPID" //name for debugging
);

pros::Motor liftStallCheck(17); //Duplicate port handler since passing a motor group doesn't work for exit_condition()
void waitForLift() {
  while (liftPIDController.exit_condition(liftStallCheck, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

void liftPIDTask_fn(void* param) {//WIP task to handle lift controls (ideally with PID)
  pros::delay(2000); //Wait for lift to initialize before running PID
  
  while (true) {
    liftControl(liftPIDController.compute(lift.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }
}

void manualLift() { //uses controller to give PID target during opcontrol
  constexpr double LIFT_INCREMENT = 10; //change in target pos in degrees per tick (~10ms)
  static double manualTarget = 0;
  
  if (master.get_digital(DIGITAL_R1)) { //inputs values into the PID controller to move lift
    manualTarget += LIFT_INCREMENT;
    liftPIDController.target_set(manualTarget);
  } 
  if (master.get_digital(DIGITAL_R2)) {
    manualTarget -= LIFT_INCREMENT;
    liftPIDController.target_set(manualTarget);
  } 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Wrist control systems
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tiltControl() {
  int tiltMax = 345;
  int tiltMin = 0;

  if(master.get_digital(DIGITAL_L1) && tilt.get_position() < tiltMax){
      tilt.move(40);
    }
  else if(master.get_digital(DIGITAL_L2) && tilt.get_position() > tiltMin) {
      tilt.move(-30);
    }
  else if(master.get_digital(DIGITAL_RIGHT)) {
      tilt.move_absolute(200,50); //Scoring Position (level)
    }
  else if (master.get_digital(DIGITAL_UP)) {
      tilt.move_absolute(340,50); //Up Position 
    }
  else if (master.get_digital(DIGITAL_LEFT)) {
      tilt.move_absolute(125,50); //Cone Position (angled down)
    }
  else {
      tilt.brake();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Macros and other controls
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//(**not tested yet) Simple grabbing macro to tilt wrist down onto pin and grab shortly after
void grabMacro() {
  tilt.move_absolute(200, 50);
  if (tilt.get_position() < 205) { //clamps once wrist is at desired angle
    piston.set(true);
    tilt.move(0);
  } 
}

//(**not tested yet) Simple scoring macro to drop cup on stack, lift up, and automatically tilt wrist up
//Potential for implementing an automatic lift lowering back to grabbing height after sensors detect bot moving away from stack

//later should use while loops and delay inside of a separate task to avoid weird if structures
void scoreMacro() {
  static bool contact = false;
  static int target = 0;
  static bool targetSet = false;

  if (!contact && (lift.get_power(0) + lift.get_power(1))/2 > 0.5) { //**need to tune value here for power when it hits stack
    liftControl(-40); //lowers lift until cup contacts stack and power dips
  } else if (!contact) {
    liftControl(0);
    contact = true;
  }
  
  if (contact) {
    if (!targetSet) {
      target = lift.get_position() + 200;
      targetSet = true;
    }
  
    piston.set(false);

    if (targetSet && abs(lift.get_position() - target) < 10) { //basically move.absolute() but with liftControl()
      liftControl(0); 
      target = 1600; //max height just in case some bug happens so it still goes up regardless
      targetSet = false;
    } else if (targetSet){
      liftControl(127); //probably overshoots but whatev its not a precise target
    }

    tilt.move_absolute(340, 50);
  }
  
}