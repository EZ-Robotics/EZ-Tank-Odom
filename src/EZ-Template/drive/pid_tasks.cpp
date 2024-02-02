/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"
#include "pros/misc.hpp"

using namespace ez;

void Drive::ez_auto_task() {
  int timer = 0;
  while (true) {
    if (timer >= ez::util::DELAY_TIME) {
      // Autonomous PID
      switch (drive_mode_get()) {
        case DRIVE:
          drive_pid_task();
          break;
        case TURN ... TURN_TO_POINT:
          turn_pid_task();
          break;
        case SWING:
          swing_pid_task();
          break;
        case POINT_TO_POINT:
          ptp_task();
        case DISABLE:
          break;
        default:
          break;
      }

      util::AUTON_RAN = drive_mode_get() != DISABLE ? true : false;

      timer = 0;
    }

    ez_tracking_task();
    pros::delay(1);
    timer++;
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  headingPID.compute(drive_imu_get());

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());

  // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double l_drive_out = util::clamp(leftPID.output, slew_left.output(), -slew_left.output());
  double r_drive_out = util::clamp(rightPID.output, slew_right.output(), -slew_right.output());

  // Toggle heading
  double gyro_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + gyro_out;
  double r_out = r_drive_out - gyro_out;

  // Vector scaling
  double max_slew_out = fmin(slew_left.output(), slew_right.output());
  if (fabs(l_out) > max_slew_out || fabs(r_out) > max_slew_out) {
    if (fabs(l_out) > fabs(r_out)) {
      r_out = r_out * (max_slew_out / fabs(l_out));
      l_out = util::clamp(l_out, max_slew_out, -max_slew_out);
    } else {
      l_out = l_out * (max_slew_out / fabs(r_out));
      r_out = util::clamp(r_out, max_slew_out, -max_slew_out);
    }
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID
  // turnPID.compute(drive_imu_get());

  // Compute PID if it's a normal turn
  if (mode == TURN) {
    turnPID.compute(drive_imu_get());
  }
  // Compute PID if we're turning to point
  else {
    int add = current_turn_type == REV ? 180 : 0;
    double a_target = util::absolute_angle_to_point(turn_to_point_target, odom_current) + add;
    turnPID.target_set(util::wrap_angle(a_target - drive_imu_get()));
    turnPID.compute(0);
  }

  // Compute slew
  slew_turn.iterate(drive_imu_get());

  // Clip gyroPID to max speed
  double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.target_get()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (pid_turn_min_get() != 0)
      gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  // Compute PID
  swingPID.compute(drive_imu_get());
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  // Compute slew
  double current = slew_swing_using_angle ? drive_imu_get() : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.iterate(current);

  // Clip swingPID to max speed
  double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.target_get()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (pid_swing_min_get() != 0)
      swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
  }

  // Set the motors powers, and decide what to do with the "still" side of the drive
  double opposite_output = 0;
  double scale = swing_out / max_speed;
  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
      private_drive_set(swing_out, opposite_output);
    } else if (current_swing == RIGHT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
      private_drive_set(opposite_output, -swing_out);
    }
  }
}

int Drive::is_past_target() {
  double distance_to_target = util::distance_to_point(odom_target, odom_current);
  double fake_y = (odom_target.y - odom_current.y);
  double new_y = fake_y * fabs(distance_to_target / fake_y);
  return util::sgn(new_y);
}

std::vector<pose> Drive::find_point_to_face(bool set_global) {
  double tx_cx = odom_target.x - odom_current.x;
  double m = 0.0;
  if (tx_cx != 0)
    m = (odom_target.y - odom_current.y) / tx_cx;
  double angle = (sin(util::to_rad(m)));
  pose ptf1 = util::vector_off_point(24.0, {odom_target.x, odom_target.y, angle});
  pose ptf2 = util::vector_off_point(24.0, {odom_target.x, odom_target.y, angle + 180});
  if (set_global) {
    double ptf1_dist = util::distance_to_point(ptf1, odom_current);
    double ptf2_dist = util::distance_to_point(ptf2, odom_current);
    if (ptf1_dist > ptf2_dist)
      ptf1_running = true;
    else
      ptf1_running = false;
  }
  return {ptf1, ptf2};
}

// Odom To Point Task
void Drive::ptp_task() {
  // double b = odom_target.y - (m * odom_target.x);
  pose point_to_face;
  pose ptf1 = find_point_to_face()[0];
  pose ptf2 = find_point_to_face()[1];

  if (ptf1_running)
    point_to_face = ptf1;
  else
    point_to_face = ptf2;

  int add = current_turn_type == REV ? 180 : 0;
  double a_target = util::absolute_angle_to_point(point_to_face, odom_current) + add;
  headingPID.target_set(util::wrap_angle(a_target - drive_imu_get()));
  headingPID.compute(0);

  // Error for xy pid
  double distance_to_target = util::distance_to_point(odom_target, odom_current);

  // Check to see if we've passed target
  double fake_y = odom_target.y - odom_current.y;
  double new_y = 0.0;
  if (fake_y != 0)
    new_y = fake_y * fabs(distance_to_target / fake_y);
  int dir = (current_turn_type == REV ? -1 : 1);           // If we're going backwards, add a -1
  int flipped = is_past_target() != past_target ? -1 : 1;  // Check if we've flipped directions to what we started

  // Compute xy PID
  xyPID.target_set(distance_to_target * dir * flipped);
  xyPID.compute(0);

  // Force code to prioritize turning
  double xy_out = xyPID.output * cos(util::to_rad(util::wrap_angle(a_target - drive_imu_get())));

  // Clip gyroPID to max speed
  double left_output = xy_out + headingPID.output;
  double right_output = xy_out - headingPID.output;

  // Vector scaling
  double biggest = pid_speed_max_get();
  if (fabs(left_output) > biggest || fabs(right_output) > biggest) {
    if (fabs(left_output) > fabs(right_output)) {
      right_output = right_output * (biggest / fabs(left_output));
      left_output = util::clamp(left_output, biggest, -biggest);
    } else {
      left_output = left_output * (biggest / fabs(right_output));
      right_output = util::clamp(right_output, biggest, -biggest);
    }
  }

  printf("xy(%.2f, %.2f, %.2f)   xyPID: %.2f   aPID: %.2f     fakey: %.2f   dir: %i   sgn: %i\n", odom_current.x, odom_current.y, odom_current.theta, xyPID.target_get(), headingPID.target_get(), fake_y, dir, flipped);

  // if (drive_toggle)
  private_drive_set(left_output, right_output);
  // private_drive_set(headingPID.output, -headingPID.output);

  /*
  pose i = util::vector_off_point(18, {0, 0, a_target + 90});
  pose o = util::vector_off_point(18, {i.x, i.y, a_target});
  double dot_product = (i.x * fake_current.x) + (i.y * fake_current.y) + (i.x * fake_current.x) + (i.y * fake_current.y);
  // printf("i(%.2f, %.2f, %.2f)   o(%.2f, %.2f, %.2f)   cur(%.2f, %.2f, %.2f)      product:%.2f\n", i.x, i.y, i.theta, o.x, o.y, o.theta, fake_current.x, fake_current.y, fake_current.theta, dot_product);

  pose b = util::vector_off_point(24, {odom_target.x, odom_target.y, a_target + 90});
  pose a = util::vector_off_point(24, {odom_target.x, odom_target.y, a_target - 90});
  pose c = odom_current;
  int there = util::sgn(((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)));  // cross product to decide if above/below line

  bool passed_target;
  if (there == 1)
    passed_target = current_turn_type == FWD ? true : false;
  else if (there == -1)
    passed_target = current_turn_type == REV ? true : false;
  // printf("Passed Target: %i   There: %i      xy %.2f\n", passed_target, there, xyPID.target_get());
*/
}