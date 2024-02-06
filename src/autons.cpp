#include "EZ-Template/util.hpp"
#include "main.h"

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;
const int INTAKE_SPEED = 127;
const int INTAKE_HOLD = 30;
const int OUTAKE_SPEED = -127;

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
void offense_end() {
  // Drive over to the lane and spit the ball out
  pose first_pos = {-5, 16};
  std::vector<pose> bm1 = ez::util::boomerang(first_pos, {0, -4, -70}, 0.375);
  chassis.pid_odom_smooth_pp_set({{first_pos, rev, 110},
                                  {bm1[0], rev, 80},
                                  {bm1[1], rev, 80}},
                                 false);
  chassis.pid_wait_until_pp(2);
  set_intake(OUTAKE_SPEED);
  chassis.pid_wait();

  // Go in the lane to intake the ball
  set_intake(INTAKE_SPEED);
  chassis.pid_odom_smooth_pp_set({{{-4, -9}, fwd, 80},
                                  {{-24, -8}, fwd, 80}},
                                 false);
  chassis.pid_wait();

  // Go back to push balls into the goal
  pose start = {14, 12};
  double dist1 = 8.0;
  chassis.pid_odom_smooth_pp_set({{{0, -7}, rev, 80},
                                  {{4, -5}, rev, 80},
                                  {start, rev, 80}},
                                 false);
  chassis.pid_wait_until_pp(2);
  set_intake(INTAKE_HOLD);
  chassis.pid_wait();

  // Push balls into the goal
  chassis.pid_odom_smooth_pp_set({{{start.x, start.y + dist1}, rev, 80}},
                                 false);
  chassis.pid_wait();

  chassis.pid_odom_smooth_pp_set({{start, fwd, 80}},
                                 false);
  chassis.pid_wait();

  // Come back and flip around to outtake and score the other ball
  double dist2 = 8.0;
  double tx_cx = start.x - chassis.odom_target.x;
  double m = 0.0;
  double angle = 0.0;
  if (tx_cx != 0) {
    m = (start.y - chassis.odom_target.y) / tx_cx;
    angle = 90.0 - util::to_deg(atan(m));
  }
  pose face_point = util::vector_off_point(chassis.LOOK_AHEAD, {start.x, start.y + dist2, angle});
  chassis.pid_turn_set(face_point, fwd, TURN_SPEED);
  chassis.pid_wait();

  set_intake(OUTAKE_SPEED);
  pros::delay(300);

  chassis.pid_odom_injected_pp_set({{{start.x, start.y - 5.0}, rev, 110}},
                                   false);
  chassis.pid_wait();

  chassis.pid_odom_injected_pp_set({{{start.x, start.y + dist2}, fwd, 110}},
                                   false);
  chassis.pid_wait();

  chassis.pid_odom_injected_pp_set({{start, rev, 80}},
                                   false);
  chassis.pid_wait();
}

void offense3ball() {
  offense_odom_start();             // Setup offense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Grab center center ball
  set_intake(INTAKE_SPEED);
  chassis.pid_odom_injected_pp_set({{{-9, 33}, fwd, 110}},
                                   false);
  chassis.pid_wait();

  chassis.pid_odom_injected_pp_set({{{-9, 24}, rev, 80}},
                                   false);
  chassis.pid_wait();

  // Spit out the ball towards corner
  chassis.pid_turn_set(125, TURN_SPEED);
  chassis.pid_wait_until(110);
  set_intake(OUTAKE_SPEED);
  chassis.pid_wait();
  pros::delay(250);

  // Intake the other ball against the barrier
  set_intake(INTAKE_SPEED);
  chassis.pid_odom_injected_pp_set({{{-21, 21}, fwd, 80}},
                                   false);
  chassis.pid_wait();

  offense_end();
}

void offense2ball() {
  offense_odom_start();             // Setup offense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Grab center center ball
  set_intake(INTAKE_SPEED);
  chassis.pid_odom_injected_pp_set({{{-9, 33}, fwd, 110}},
                                   false);
  chassis.pid_wait();

  offense_end();
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

  set_intake(INTAKE_HOLD);

  // Face the offensive side
  chassis.pid_turn_set(95, TURN_SPEED);
  chassis.pid_wait();

  // Wait before pushing balls over to offensive side so we don't mess opponents up
  while (pros::millis() - time_start < 13500) {
    pros::delay(util::DELAY_TIME);
  }

  set_intake(OUTAKE_SPEED);
  chassis.pid_drive_set(24, DRIVE_SPEED);
  chassis.pid_wait();
}

// Defense 2 Ball
void defense2ball() {
  defense_odom_start();             // Setup defense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Rush towards closer center ball
  chassis.pid_odom_injected_pp_set({{{8, 32}, fwd, 110}},
                                   false);
  set_intake(INTAKE_SPEED);
  chassis.pid_wait();

  // Run the end of the function
  defense_end(time_start);
}

// Defense 2 Ball with Interrupt
void defense2ballinterupt() {
  defense_odom_start();             // Setup defense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Rush towards farther center back
  chassis.pid_odom_smooth_pp_set({{{4, 16}, fwd, 110},
                                  {{8, 28}, fwd, 110},
                                  {{13, 30}, fwd, 110},
                                  {{18, 34}, fwd, 110}},
                                 false);
  set_intake(INTAKE_SPEED);
  chassis.pid_wait();

  // Run the end of the function
  defense_end(time_start);
}

void defense2ballinterupt_barrier() {
  defense_odom_start();             // Setup defense starting position
  int time_start = pros::millis();  // Timestamp when this auto started

  // Rush towards closer center ball
  chassis.pid_odom_injected_pp_set({{{8, 33}, fwd, 110}},
                                   false);
  set_intake(INTAKE_SPEED);
  chassis.pid_wait();

  // Rush towards closer center ball
  chassis.pid_odom_injected_pp_set({{{20.5, 33}, fwd, 70}},
                                   false);
  pros::delay(150);
  set_intake(OUTAKE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);

  // Run the end of the function
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