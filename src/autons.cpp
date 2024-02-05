#include "main.h"

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 50);
  chassis.pid_drive_constants_set(25, 0, 100);
  chassis.pid_turn_constants_set(5, 0.003, 35, 15);
  chassis.pid_swing_constants_set(7, 0, 45);

  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 1_in, 300_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(100_ms, 1_in, 300_ms, 3_in, 500_ms, 500_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Offense Start
///
void offense_odom_start() {
  // Set new position
  chassis.odom_pose_set({0, 0, -15});
}

void offense() {
  offense_odom_start();             // Setup offense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  set_intake(80);
  chassis.pid_odom_injected_pp_set({{{-9, 33}, fwd, 110}},
                                   false);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 127);
  chassis.pid_wait_until(80_deg);
  set_intake(-127);
  pros::delay(150);

  chassis.pid_turn_set(-80, 127);
  chassis.pid_wait();

  set_intake(80);
  chassis.pid_drive_set(8, 127);
  chassis.pid_wait();

  chassis.pid_odom_smooth_pp_set({{{-5, 16}, rev, 110},
                                  {{2, 0}, rev, 90}},
                                 false);
  chassis.pid_wait();
}

///
// Defense Start and End
///
void defense_odom_start() {
  // Set new position
  chassis.odom_pose_set({0, 0, 15});
}

void defense_end(int time_start) {
  // Come back to face the lane, knocking the closer center ball on the way
  chassis.pid_odom_smooth_pp_set({{{4, 16}, rev, 110},
                                  {{0, 0}, rev, 80},
                                  {{-6, -3}, rev, 60}},
                                 false);
  chassis.pid_wait();

  set_intake(30);

  // Face the offensive side
  chassis.pid_turn_set(95, TURN_SPEED);
  chassis.pid_wait();

  // Wait before pushing balls over to offensive side so we don't mess opponents up
  while (pros::millis() - time_start < 13500) {
    pros::delay(util::DELAY_TIME);
  }

  set_intake(-127);
  chassis.pid_drive_set(24, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Defense 2 Ball
///
void defense2ball() {
  defense_odom_start();             // Setup defense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Rush towards closer center ball
  set_intake(80);
  chassis.pid_odom_injected_pp_set({{{8, 32}, fwd, 110}},
                                   true);
  chassis.pid_wait();

  // Run the end of the function
  defense_end(time_start);
}

///
// Defense 2 Ball with Interrupt
///
void defense2ballinterupt() {
  defense_odom_start();             // Setup defense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Rush towards farther center back
  set_intake(80);
  chassis.pid_odom_smooth_pp_set({{{4, 16}, fwd, 110},
                                  {{8, 28}, fwd, 110},
                                  {{13, 30}, fwd, 110},
                                  {{18, 34}, fwd, 110}},
                                 false);
  set_intake(80);
  chassis.pid_wait();

  // RUn the end of the function
  defense_end(time_start);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .