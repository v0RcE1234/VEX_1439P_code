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
    {-20, -3, -1},     // Left Chassis Ports (negative port will reverse it!)
    {10, 13, 11},  // Right Chassis Ports (negative port will reverse it!)

    18,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

// Helpers for MCL
double random_uniform(double min, double max) {
  return min + (double)rand() / RAND_MAX * (max - min);
}

double random_gaussian(double mean, double stddev) {
    // Use Box-Muller transform
    static bool hasSpare = false;
    static double spare;
    if (hasSpare) {
        hasSpare = false;
        return mean + stddev * spare;
    }

    hasSpare = true;
    double u, v, s;
    do {
        u = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        v = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);

    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    return mean + stddev * (u * s);
}

double gaussian(double x, double mean, double stddev) {
  double exponent = -((x - mean) * (x - mean)) / (2 * stddev * stddev);
  return (1.0 / (stddev * sqrt(2 * M_PI))) * exp(exponent);
}

double simulate_distance_sensor(double x, double y, double robot_heading) {
  // Simulate distance sensor reading based on robot position and heading
  // Assuming field boundaries at x = ±72, y = ±72

  // Sensor offset from robot center (in inches)
  const double sensor_offset_x = -4.75;
  const double sensor_offset_y = -8.5;
  const double sensor_angle_offset_deg = 180;

  const double FIELD_HALF = 72.0; // Half field size in inches

  // Calculate sensor's global position, -robot_theta_rad is used since 
  // robot_theta_rad is the clockwise angle but the rotation formulas
  // assume counter-clockwise rotation.
  double robot_theta_rad = robot_heading * M_PI / 180.0;
  double sensor_global_x = x + sensor_offset_x * cos(-robot_theta_rad) - sensor_offset_y * sin(-robot_theta_rad);
  double sensor_global_y = y + sensor_offset_x * sin(-robot_theta_rad) + sensor_offset_y * cos(-robot_theta_rad);

  // Calculate sensor's global heading
  double sensor_global_theta_rad = robot_theta_rad + sensor_angle_offset_deg * M_PI / 180.0;

  // Calculate the direction vector based on the robot's orientation. x uses sin since
  // robot heading is measured as clockwise from y axis
  double dx = sin(sensor_global_theta_rad);
  double dy = cos(sensor_global_theta_rad);

  // Calculate intersection t for each wall
  double t_top = (FIELD_HALF - sensor_global_y) / dy;
  double t_bottom = (-FIELD_HALF - sensor_global_y) / dy;
  double t_right = (FIELD_HALF - sensor_global_x) / dx;
  double t_left = (-FIELD_HALF - sensor_global_x) / dx;
  double t_min = 1000000.0; // Initialize to a large value
  if (t_top > 0 && t_top < t_min) t_min = t_top;
  if (t_bottom > 0 && t_bottom < t_min) t_min = t_bottom;
  if (t_right > 0 && t_right < t_min) t_min = t_right;
  if (t_left > 0 && t_left < t_min) t_min = t_left;

  return t_min; // Distance to closest wall
}

int find_idx_in_cumulative(double* cumulative_weights, double r, int size) {
  int i = 0;
  while (r > cumulative_weights[i] && i < size - 1) {
    i++;
  }
  return i;
}

double get_forward_movement() {
    static double last_left = 0.0;
    static double last_right = 0.0;

    // Get current drive sensor values (in inches)
    double curr_left = chassis.drive_sensor_left();
    double curr_right = chassis.drive_sensor_right();

    // Calculate average movement since last call
    double delta_left = curr_left - last_left;
    double delta_right = curr_right - last_right;
    double delta_forward = (delta_left + delta_right) / 2.0;

    // Update last values
    last_left = curr_left;
    last_right = curr_right;

    return delta_forward;
}

double get_inertial_heading() {
    return chassis.odom_theta_get();
}

const int NUM_PARTICLES = 100;
typedef struct {
  double x;
  double y;
  double weight;
} Particle;
Particle particles[NUM_PARTICLES];
bool reset_particles_flag = false;
double reset_x, reset_y, reset_spread;

void mcl() {
  // Monte Carlo Localization
  int SIGMA = 2.0; // Assumed sensor noise std in inches
  int SIGMA_GPS = 6.0; // Assumed GPS noise std in inches
  int PERIOD_MS = 100; // Update period in ms

  // Initialization
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = (double)(rand() % 144) - 72; // Random x in [-72, 72]
    particles[i].y = (double)(rand() % 144) - 72; // Random y in [-72, 72]
    particles[i].weight = 1.0 / NUM_PARTICLES;
  }
  gps1.set_offset(-6.25 / 39.3701, 5.75 / 39.3701); // Offset from center of robot to GPS in meters. ASSUME ROBOT IS FACING GPS'S DIRECTION

  // Main loop
  while (true) {
    // Start timing
    int start_time = pros::millis();

    // Handle reset request
    if (reset_particles_flag) {
      for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].x = reset_x + random_gaussian(0, reset_spread);
        particles[i].y = reset_y + random_gaussian(0, reset_spread);
        particles[i].weight = 1.0 / NUM_PARTICLES;
      }
      reset_particles_flag = false;
    }

    // Step 1: Get robot movement and inertial heading
    double delta_distance = get_forward_movement();
    double robot_heading = get_inertial_heading();

    // Step 2: Motion Update
    for (int i = 0; i < NUM_PARTICLES; i++) {
      // Add noise to movement
      double noisy_distance = delta_distance + random_gaussian(0, 0.5);
      // IMPORTANT: sin is used for x since robot heading is measured as 
      // clockwise from y axis
      particles[i].x += noisy_distance * sin(robot_heading * M_PI / 180.0);
      particles[i].y += noisy_distance * cos(robot_heading * M_PI / 180.0);
      // Keep particles within field bounds
      if (particles[i].x < -72) particles[i].x = -72;
      if (particles[i].x > 72) particles[i].x = 72;
      if (particles[i].y < -72) particles[i].y = -72;
      if (particles[i].y > 72) particles[i].y = 72;
    }

    // Step 3: Sensor readings from robot
    double dist_mm = distance_sensor.get(); // Get value in mm
    if (dist_mm < 20 || dist_mm > 2000) {
      pros::delay(100);
      continue; // Skip this iteration if the value is invalid
    }
    double sensor_distance = dist_mm / 25.4; // Convert mm to inches

    double gps_x = gps1.get_position_x() * 39.3701; // meters to inches
    double gps_y = gps1.get_position_y() * 39.3701; // meters to inches
    static int stuck_count = 0;
    
    // GPS validity checks
    static double last_gps_x = 0, last_gps_y = 0;
    const int STUCK_THRESHOLD = 10;
    const double STUCK_EPSILON = 0.01;

    if (fabs(gps_x - last_gps_x) < STUCK_EPSILON && fabs(gps_y - last_gps_y) < STUCK_EPSILON) {
      stuck_count++;
    } else {
      stuck_count = 0;
    }
    last_gps_x = gps_x;
    last_gps_y = gps_y;

    bool gps_valid = true;
    if (stuck_count > STUCK_THRESHOLD) {
      gps_valid = false;
      ez::screen_print("GPS STUCK!", 4);
    }
    if (gps_x < -72 || gps_x > 72 || gps_y < -72 || gps_y > 72) {
      gps_valid = false;
      ez::screen_print("GPS OUT OF BOUNDS!", 4);
    }
    if (std::isnan(gps_x) || std::isnan(gps_y)){
      gps_valid = false;
      ez::screen_print("GPS NAN!", 4);
    }

    // Step 4: Weighting
    double total_weight = 0.0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      double x = particles[i].x;
      double y = particles[i].y;
      
      double expected_distance = simulate_distance_sensor(x, y, robot_heading);

      double weight = 1.0;
      double error = sensor_distance - expected_distance;
      double prob = gaussian(error, 0, SIGMA);
      weight *= prob;

      // GPS weighting
      if (gps_valid) {
        double gps_error_x = gps_x - x;
        double gps_error_y = gps_y - y;
        double gps_prob_x = gaussian(gps_error_x, 0, SIGMA_GPS);
        double gps_prob_y = gaussian(gps_error_y, 0, SIGMA_GPS);
        weight *= gps_prob_x * gps_prob_y;
      }

      particles[i].weight = weight;
      total_weight += weight;
    }

    // Step 5: Normalize weights
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particles[i].weight /= total_weight;
    }

    // Step 6: Resample Particles
    Particle new_particles[NUM_PARTICLES];
    double cumulative_weights[NUM_PARTICLES];
    double cumulative = 0.0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      cumulative += particles[i].weight;
      cumulative_weights[i] = cumulative;
    }

    for (int i = 0; i < NUM_PARTICLES; i++) {
      double r = random_uniform(0, 1);
      int idx = find_idx_in_cumulative(cumulative_weights, r, NUM_PARTICLES);
      Particle chosen = particles[idx];
      new_particles[i] = {chosen.x, chosen.y, 1.0 / NUM_PARTICLES}; // Reset weight
    }

    for (int i = 0; i < NUM_PARTICLES; i++) {
      particles[i] = new_particles[i];
    }

    // Step 7: Estimate Position from particle average
    double avg_x = 0.0;
    double avg_y = 0.0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      avg_x += particles[i].x;
      avg_y += particles[i].y;
    }
    avg_x /= NUM_PARTICLES;
    avg_y /= NUM_PARTICLES;
    chassis.odom_xy_set(avg_x, avg_y);
    
    // Debug print
    ez::screen_print("Est Pos x: " + std::to_string(avg_x) +
                     " y: " + std::to_string(avg_y), 5);
    // GPS debug print
    ez::screen_print("GPS x: " + std::to_string(gps_x) +
                     " y: " + std::to_string(gps_y), 6);
    // Distance sensor debug print
    ez::screen_print("DS: " + std::to_string(sensor_distance) +
                     " Exp: " + std::to_string(simulate_distance_sensor(avg_x, avg_y, robot_heading)), 7);
    // // GPS validity check
    // if (std::isnan(gps_x) || std::isnan(gps_y)) {
    //     ez::screen_print("GPS INVALID!", 4);
    // }

    // Delay to maintain loop period
    int elapsed = pros::millis() - start_time;
    if (elapsed < PERIOD_MS) {
      pros::delay(PERIOD_MS - elapsed);
    }
  }
}

void request_particle_init(double start_x, double start_y, double spread = 0.0) {
  reset_x = start_x;
  reset_y = start_y;
  reset_spread = spread;
  reset_particles_flag = true;
}

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
  // pros::Task distance_sensor_task(distance_sensor_update);

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

  // Start tasks
  pros::Task mcl_task(mcl);
  // Initalize all cordinate systems to starting pose of robot. Following code assumes 0, 0, 0, but this
  // templated should be changed in each autonomous to the desired starting pose
  request_particle_init(0.0, 24.0); // Robot starts at (0,0) with no spread
  gps1.set_position(0, 24 / 39.3701, 270); // FOR ORIENTATION, ASSUME ROBOT IS FACING GPS'S DIRECTION. EX, IF GPS IS MOUNTED ON LEFT, ORIENTATION IS ORIENTATION OF ROBOT - 90
  chassis.odom_xyt_set(0_in, 24_in, 0_deg);
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

    // mogo_clamp.button_toggle(master.get_digital(DIGITAL_X));

    // if (master.get_digital(DIGITAL_L1)) {
    //   basket.move(127);
    // } 
    // else if (master.get_digital(DIGITAL_L2)) {
    //   basket.move(-127);
    // } 
    // else {
    //   basket.move(0);
    // }

    // if (master.get_digital(DIGITAL_R1)) {
    //   intake_bottom.move(127);
    // } 
    // else if (master.get_digital(DIGITAL_R2)) {
    //   intake_bottom.move(-127);
    // } 
    // else {
    //   intake_bottom.move(0);
    // }

    // if (master.get_digital(DIGITAL_UP)) {
    //   intake_top.move(127);
    // } 
    // else if (master.get_digital(DIGITAL_DOWN)) {
    //   intake_top.move(-127);
    // } 
    // else {
    //   intake_top.move(0);
    // }

    if (master.get_digital(DIGITAL_R1)) { // to basket
      intake_bottom.move(127);
      basket.move(127);
      intake_top.move(127);
    } 
    else if (master.get_digital(DIGITAL_R2)) { // low goal
      intake_bottom.move(-127);
      basket.move(-127);
      intake_top.move(127);
    }
    else if (master.get_digital(DIGITAL_L1)) { // mid goal
      intake_bottom.move(127);
      basket.move(-127);
      intake_top.move(127);
    } else if (master.get_digital(DIGITAL_L2)) { // top goal
      intake_bottom.move(127);
      basket.move(-127);
      intake_top.move(-127);
    } else {
      intake_bottom.move(0);
      basket.move(0);
      intake_top.move(0);
    }
    

    // mid_goal_piston.button_toggle(master.get_digital(DIGITAL_A));
    aligner.button_toggle(master.get_digital(DIGITAL_B));
    matchloader.button_toggle(master.get_digital(DIGITAL_A));


    if (master.get_digital(DIGITAL_L1)) {
      intake.move(127);
    } 
    else if (master.get_digital(DIGITAL_L2)) {
      intake.move(-127);
    } 
    else {
      intake.move(0);
    }


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }

}





#include <cmath>
#include <cstdlib>

// Generate Gaussian random number with given mean (μ) and stddev (σ)
double random_gaussian(double mean, double stddev) {
    // Box-Muller transform
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0); // avoid log(0)
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);

    double z0 = sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2); // standard normal ~N(0,1)

    return mean + z0 * stddev;
}

double gaussian(double x, double mean, double stddev) {
    double exponent = -0.5 * pow((x - mean) / stddev, 2);
    return (1.0 / (stddev * sqrt(2 * M_PI))) * exp(exponent);
}



double get_forward_movement_from_encoders() {
    static double last_x = chassis.odom_x_get();
    static double last_y = chassis.odom_y_get();

    double current_x = chassis.odom_x_get();
    double current_y = chassis.odom_y_get();

    double dx = current_x - last_x;
    double dy = current_y - last_y;
    double theta = chassis.odom_theta_get() * M_PI / 180.0;

    double forward_movement = dx * cos(theta) + dy * sin(theta);

    last_x = current_x;
    last_y = current_y;

    return forward_movement; 
}

const double FIELD_HALF = 72.0; // Half of a 144-inch field

double simulate_distance_sensor(double x, double y, double theta) {
    // Ray direction vector
    double sensor_offset_x = 1;
    double sensor_offset_y = 1;
    sensor_offset_theta = 0 * 180/3.14; // radians

    // Rotate the sensor's local offset by the robot's orientation
    x = x + sensor_offset_x * cos(theta) + sensor_offset_y * sin(theta);
    y = y -  sensor_offset_x * sin(theta) + sensor_offset_y * cos(theta);
    
    // Adjust the sensor's orientation
    theta = theta + sensor_offset_theta;

    double dx = cos(theta);  // x-component
    double dy = -   sin(theta);  // y-component


    double t_min = 1e6; // large number for min distance

    // Check intersection with top wall (y = +FIELD_HALF)
    if (dy != 0) {
        double t_top = (FIELD_HALF - y) / dy;
        if (t_top > 0 && t_top < t_min) t_min = t_top;
    }

    // Bottom wall (y = -FIELD_HALF)
    if (dy != 0) {
        double t_bottom = (-FIELD_HALF - y) / dy;
        if (t_bottom > 0 && t_bottom < t_min) t_min = t_bottom;
    }

    // Right wall (x = +FIELD_HALF)
    if (dx != 0) {
        double t_right = (FIELD_HALF - x) / dx;
        if (t_right > 0 && t_right < t_min) t_min = t_right;
    }

    // Left wall (x = -FIELD_HALF)
    if (dx != 0) {
        double t_left = (-FIELD_HALF - x) / dx;
        if (t_left > 0 && t_left < t_min) t_min = t_left;
    }

    return t_min; // distance to closest wall
}

















void mcl() {
  //constants
  const int NUM_PARTICLES = 50;
  const double SIGMA = 2.0; //Assumed standard deviation of the sensor noise in inches

  // particle structure: [x, y, weight]
  // no theta, since we assume the imu is always correct
  typedef struct particle {
    double x;
    double y;
    double weight;
  } particle;

  // initilization
  particle particle_array[NUM_PARTICLES];
  for (int i = 0; i < NUM_PARTICLES; i++) {
    double x = rand() * 144 - 72; // Random x position between -72 and 72
    double y = rand() * 144 - 72; // Random y position between -72 and 72
    double weight = 1.0 / NUM_PARTICLES; // Initial weight is uniform
    particle_array[i] = {x, y, weight};
  }
  // Main loop
  while(true){
    //step 1: get robot movement and initial heading
    double delta_distance = get_forward_movement_from_encoders(); // Get the forward movement from encoders
    double robot_heading = (90 - chassis.odom_theta_get()) * M_PI/180; // Get the robot's heading from the IMU
    
    //step 2: motion update
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particle p = particle_array[i];
      double x = p.x;
      double y = p.y;
      double w = p.weight;
      // add some noise to the distance moved
      double noisy_distance = delta_distance + random_gaussian(0, 0.5);
      double dx = noisy_distance * cos(robot_heading);
      double dy = noisy_distance * sin(robot_heading);
      p.x += dx;
      p.y += dy;
    }


    //step 3: simulate sensor values from this pose, weighting
    double total_weight = 0.0;
    for(int i = 0; i < NUM_PARTICLES; i++) {
      particle p = particle_array[i];
      double x = p.x;
      double y = p.y;
      double theta = robot_heading;
      
      double weight = 1.0;
      double distance_sensor_error = distance_sensor.get() / 25.4 - simulate_distance_sensor(x, y, theta); // Convert mm to inches
      double prob = gaussian(distance_sensor_error, 0, SIGMA); // Calculate the probability of the sensor value given the pose
      weight *= prob; // Update the weight of the particle 
      
      p.weight = weight; // Update the particle's weight 
    }

    //step 4: normalize weights
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particle p = particle_array[i];
      p.weight /= total_weight; // Normalize the weight
    }

    //step 5: resample particles
    particle new_particles[NUM_PARTICLES];
    double cummulative_weights[NUM_PARTICLES];
    double cummulative_weight = 0.0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
      particle p = particle_array[i];
      cummulative_weight += p.weight;
      cummulative_weights[i] = cummulative_weight;
    }
    for (int i = 0; i < NUM_PARTICLES; i++) {
    double r = ((double) rand()) / RAND_MAX; // Random number between 0 and 1
    int j = 0;
    while (j < NUM_PARTICLES - 1 && r > cummulative_weights[j]) {
        j++;
    }
    new_particles[i] = particle_array[j];
}
    // Copy new particles back to particle_array
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particle_array[i] = new_particles[i];
    }



    //step 6: estimate
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particle p = particle_array[i];
      estimated_x += p.x * p.weight; // Weighted average of x positions
      estimated_y += p.y * p.weight; // Weighted average of y positions
    }

  }

}







  







