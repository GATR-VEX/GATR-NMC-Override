#include "main.h"


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor, need to change values
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-18, -19},  // Left Chassis Ports Front/Back(negative port will reverse it!)
    {12, 11},  // Right Chassis Ports Front/Back(negative port will reverse it!)

    21,      // IMU Port
    3.25,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    200.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

//pros::Task Lift_task(liftTask); //can be removed if causing issues

//Quick lift height initializer to find and set 0; do NOT run in another while with other functions because it has its own while
void liftInit() {
  lift.move(20); //rotates lift until significant power is drawn (string is taught)
  while ((lift.get_power(0) + lift.get_power(1))/2 < 0.5) {
    pros::delay(10);
  }
  lift.tare_position();
  lift.move(0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(300);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  
  liftInit(); //sets lift strings taught
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //liftPID.exit_condition_set(80,50,300,150,500,500); //will need to change
  
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

//Height limits with slow zone
static int lift_speed_limit(double limit_distance, double slow_zone, double min_speed) {
  if(limit_distance <= 0) return 0; //Stop at limit
  if(limit_distance >= slow_zone) return 127; //Full speed
  return (int)(min_speed + (127 - min_speed) * (limit_distance / slow_zone)); //Scale speed between min_speed and 127
}

//Cascade lift controls
void liftControl(int target_voltage) {
  const double LIFT_MIN   = 20;      // bottom limit
  const double LIFT_MAX   = 1600;   // top limit
  const double SLOW_ZONE  = 250;    // distance from each end where slowing begins
  const int    MIN_SPEED  = 20;     // Speed in slow zone
  const int    RAMP_STEP  = 10;     // change per loop; lower = smoother

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OP CONTROL //////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    //chassis.opcontrol_tank();  // Tank control
     chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    /*
    // Lift control
    if(master.get_digital(DIGITAL_UP)){
      liftPID.target_set(500);
    }
    else if(master.get_digital(DIGITAL_DOWN)){
      liftPID.target_set(0);
    }
    if(!(countController%25)){
      master.print(0, 0, "%lf", lift.get_position());
    }
    countController++;
    */


    piston.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)); //Claw control

    tiltControl(); //Manual tilt control
  
    if (master.get_digital(DIGITAL_R1)) { //uses lift control to slew movement
    liftControl(127);
    } 
    else if (master.get_digital(DIGITAL_R2)) {
    liftControl(-40);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
