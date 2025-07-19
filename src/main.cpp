#include "main.h"
#include "autons.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-13, -1, -11},     // Left Chassis Ports (negative port will reverse it!)
    {10, 19, 20},  // Right Chassis Ports (negative port will reverse it!)

    6,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels



void gpsupdate(){
 // return; // Comment this out if you want to use GPS
  
  while(true){
    // gpsData = gps1.get_data();
    // chassis.odom_xy_set(gpsData.x, gpsData.y);
    chassis.odom_xy_set(gps1.get_position_x() * 39.3701, gps1.get_position_y() * 39.3701);
    pros::delay(500); // Add a small delay to prevent high CPU usage
  }
}

void distance_sensor_update() {
  // This function uses our estimated pose from ez templates built in odometry
  // to determine which wall the robot is facing. Then using the distance
  // sensor value, it can update one of the coordinates, since trigonometry
  // can give us the coordinate perpendicular to the wall.

  // Sensor offset from robot center (in inches)
  const double sensor_offset_x = -3;
  const double sensor_offset_y = -4.75;
  const double sensor_angle_offset_deg = 180;

  const double FIELD_HALF = 72.0; // Half field size in inches
  while (true) {
    double robot_x = chassis.odom_x_get();
    double robot_y = chassis.odom_y_get();
    double robot_theta_deg = chassis.odom_theta_get();
    double robot_theta_rad = robot_theta_deg * M_PI / 180.0;

    // Calculate sensor's global position, -robot_theta_rad is used since 
    // robot_theta_rad is the clockwise angle but the rotation formulas
    // assume counter-clockwise rotation.
    double sensor_global_x = robot_x + sensor_offset_x * cos(-robot_theta_rad) - sensor_offset_y * sin(-robot_theta_rad);
    double sensor_global_y = robot_y + sensor_offset_x * sin(-robot_theta_rad) + sensor_offset_y * cos(-robot_theta_rad);

    // Calculate sensor's global heading
    double sensor_global_theta_rad = robot_theta_rad + sensor_angle_offset_deg * M_PI / 180.0;

    double dist_mm = distance_sensor.get(); // Returns mm
    // VEX distance sensor valid range from website: 20mm to 2000mm
    if (dist_mm < 20 || dist_mm > 2000) {
      pros::delay(100);
      continue; // Skip this iteration if the value is invalid
    }
    double dist = dist_mm / 25.4; // Convert mm to inches

    // Calculate the direction vector based on the robot's orientation.
    // This is a unit vector (length = 1) showing the direction the robot is facing.
    double dx = sin(sensor_global_theta_rad);
    double dy = cos(sensor_global_theta_rad);

    // Calculate intersection t for each wall
    double t_top = (FIELD_HALF - sensor_global_y) / dy;
    double t_bottom = (-FIELD_HALF - sensor_global_y) / dy;
    double t_right = (FIELD_HALF - sensor_global_x) / dx;
    double t_left = (-FIELD_HALF - sensor_global_x) / dx;

    // Find the closest wall in front of the robot
    double t_min = 1000000.0; // Initialize to a large value
    std::string wall = "";
    if (t_top > 0 && t_top < t_min) { t_min = t_top; wall = "top"; }
    if (t_bottom > 0 && t_bottom < t_min) { t_min = t_bottom; wall = "bottom"; }
    if (t_right > 0 && t_right < t_min) { t_min = t_right; wall = "right"; }
    if (t_left > 0 && t_left < t_min) { t_min = t_left; wall = "left"; }

    // Update the perpendicular coordinate using the sensor value
    // Vector points towards the wall, so subtract the distance
    // times the direction vector to get the coordinate
    if (wall == "top") {
      sensor_global_y = FIELD_HALF - dist * dy;
    } else if (wall == "bottom") {
      sensor_global_y = -FIELD_HALF - dist * dy;
    } else if (wall == "right") {
      sensor_global_x = FIELD_HALF - dist * dx;
    } else if (wall == "left") {
      sensor_global_x = -FIELD_HALF - dist * dx;
    }

    // Debug print: which wall is detected and the distance
    ez::screen_print(std::string("Wall:") + wall + " Dist: " + std::to_string(dist) + " in", 6);

    // Convert sensor global position back to robot center, -robot_theta_rad is used since 
    // robot_theta_rad is the clockwise angle but the rotation formulas
    // assume counter-clockwise rotation.
    double updated_robot_x = sensor_global_x - (sensor_offset_x * cos(-robot_theta_rad) - sensor_offset_y * sin(-robot_theta_rad));
    double updated_robot_y = sensor_global_y - (sensor_offset_x * sin(-robot_theta_rad) + sensor_offset_y * cos(-robot_theta_rad));

    chassis.odom_xy_set(updated_robot_x, updated_robot_y);
    pros::delay(100);
  }
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
  //pros::Task background_task(gpsupdate); 
  pros::Task distance_sensor_task(distance_sensor_update);

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

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
      {"Localization Test\n\nThis will drive the bot to 2 points using ptp", localization_test},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
      {"Test automation \n\nThis will do some turns and then perform some movements.", testaut},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
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
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    // chassis.opcontrol_tank();  // Tank control
     chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    mogo_clamp.button_toggle(master.get_digital(DIGITAL_X));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }

}






