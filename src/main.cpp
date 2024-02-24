#include "main.h"

#include "autons.hpp"

// Drive motors.  These are used in GUI and Template
std::vector<int> left_motors = {-2, -1, -11, -15};
std::vector<int> right_motors = {16, 6, 7, 8};

// Chassis constructor
ez::Drive chassis(
    left_motors,   // Left motors
    right_motors,  // Right motors
    21,            // IMU Port
    3.25,          // Wheel size
    600,           // Cart RPM
    1.6667,        // Gear ratio
    8.93           // Drive width
);

// GUI Constructor
ez::GUI display(
    {{pros::Motor(left_motors[0]), "left 1"},
     {pros::Motor(left_motors[1]), "left 2"},
     {pros::Motor(right_motors[1]), "right 2"},
     {pros::Motor(right_motors[0]), "right 1"},
     {pros::Motor(left_motors[2]), "left 3"},
     {pros::Motor(left_motors[3]), "left 4"},
     {pros::Motor(right_motors[3]), "right 4"},
     {pros::Motor(right_motors[2]), "right 3"},
     {intake[0], "l inta"},
     {intake[1], "r inta"}},

    {//{"defense 2 ball, grab farther ball and knock closer ball", defense2ballinterupt},
     {" offense 3 ball, touches bar ", offense3ball},
     {" offense 2 ball, touches bar ", offense2ball},
     {"offense 3 ball, doesnt touch bar", offense3ball_nobar},
     {"offense 2 ball, doesnt touch bar", offense2ball_nobar},
     {"defense 2 ball, grab closer ball, touches bar", defense2ball},
     {"defense 2 ball interrupt, touches bar", defense2ballinterupt_barrier},
     {"defense 2 ball, grab closer ball, doesnt touch bar", defense2ball_nobar},
     {"defense 2 ball interrupt, doesnt touch bar", defense2ballinterupt_barrier_nobar},
     {" drive forward and come back ", drive_example},  // ez-gui bug with "drive forward and come back" doesn't wiggle
     {"turn 3 times", turn_example},
     {"drive forward, turn, come back", drive_and_turn},
     {"slow down during drive", wait_until_change_speed},
     {"swing in 'S' curve", swing_example},
     {"combining all 3 movements", combining_movements},
     {"after driving forward, robot performs differently if interfered or not", interfered_example}});

// Initialize
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(1.5);  // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants();                           // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Initialize chassis and auton selector
  chassis.initialize();
  // ez::as::initialize();
  display.enable();
  master.rumble(".");

  // Initialize subsystems
  intake_init();
}

// Auton
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({0, 0, 0});
  chassis.drive_odom_enable(true);
  chassis.SPACING = 0.5;
  pros::delay(10);

  // ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
  display.auton_call();
  /*double x = 12;
  double y = 24;
  int speed = 60;

  chassis.pid_odom_smooth_pp_set({{{0, y}, fwd, speed},
                                  {{x, y}, fwd, speed},
                                  {{x, 0}, fwd, speed},
                                  {{0, 0}, fwd, speed}},
                                 true);
  chassis.pid_wait();*/

  // chassis.pid_odom_injected_pp_set({{{0, 7}, fwd, 110}}, false);
  // chassis.pid_wait_until(4_in);
  // chassis.pid_wait();
}

// Opcontrol
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.drive_odom_enable(false);

  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X)) {
        chassis.pid_tuner_toggle();
        if (chassis.pid_tuner_enabled())
          pros::lcd::set_background_color(LV_COLOR_HEX(0xFFC0CB));
      }

      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) {
        autonomous();
        chassis.drive_brake_set(MOTOR_BRAKE_COAST);
        chassis.drive_odom_enable(false);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate

      // Modifying track width
      //  only allow this to run when PID Tuner is disabled
      if (!chassis.pid_tuner_enabled()) {
        // When A is pressed, increase track width
        if (master.get_digital_new_press(DIGITAL_A)) {
          chassis.drive_width_set(chassis.drive_width_get() + 0.1);
          printf("%.2f\n", chassis.drive_width_get());
        }
        // When Y is pressed, decrease track width
        else if (master.get_digital_new_press(DIGITAL_Y)) {
          chassis.drive_width_set(chassis.drive_width_get() - 0.1);
          printf("%.2f\n", chassis.drive_width_get());
        }
      }
    }

    intake_opcontrol();

    chassis.opcontrol_tank();  // Tank control

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

void disabled() {}
void competition_initialize() {}