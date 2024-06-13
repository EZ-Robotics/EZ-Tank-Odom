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
          break;
        case PURE_PURSUIT:
          pp_task();
          break;
        case BOOMERANG_PTP:
          boomerang_task();
          break;
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
    int add = current_turn_type == REV ? 180 : 0;                                                       // Decide if going fwd or rev
    double a_target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_current) + add;  // Calculate the point for angle to face
    turnPID.target_set(util::wrap_angle(a_target - drive_imu_get()));                                   // Constrain error to -180 to 180
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

float getCurvature(pose current, pose target) {
  // calculate whether the pose is on the left or right side of the circle
  float side = util::sgn(std::sin(current.theta) * (target.x - current.x) - std::cos(current.theta) * (target.y - current.y));
  // calculate center point and radius
  float a = -std::tan(current.theta);
  float c = std::tan(current.theta) * current.x - current.y;
  float x = std::fabs(a * target.x + target.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(target.x - current.x, target.y - current.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}

// Odom To Point Task
void Drive::ptp_task() {
  // if (util::distance_to_point(odom_target, odom_current) > LOOK_AHEAD / 2.0) {
  //   find_point_to_face(odom_current, odom_target, false);
  // }

  // Compute angle
  int add = current_turn_type == REV ? 180 : 0;                              // Decide if going fwd or rev
  pose ptf = point_to_face[!ptf1_running];                                   // Target point to face
  double a_target = util::absolute_angle_to_point(ptf, odom_current) + add;  // Calculate the point for angle to face
  double wrapped_a_target = util::wrap_angle(a_target - drive_imu_get());    // Constrain error to -180 to

  double dist_to_tar = fabs(util::distance_to_point(odom_target, odom_current));
  // double leg = /*1.0 - */ util::clamp(fabs((dist_to_tar * sin(util::to_rad(aPID.target_get()))) / 2.0), 1.0, 0.0);
  double close_to_target = util::clamp(dist_to_tar / 24.0, 1.0, 0.0);
  // double scale = leg * close_to_target;
  //  if (leg == 0)
  //    scale = 1.0;
  //  else
  //    scale = leg / close_to_target;
  double leg = fabs(dist_to_tar * sin(util::to_rad(wrapped_a_target))) * util::sgn(wrapped_a_target);
  // double k = 10.0;
  // double steer = aPID.output + fabs(util::to_deg(atan((k * leg) / fabs(last_l_out + last_r_out)))) * util::sgn(aPID.output);
  double k = 0.5;
  // leg = (int)velocity_current == 0 ? 0 : fabs(util::to_deg(atan((k * leg) / velocity_current))) * util::sgn(aPID.output);
  double stan = /*(int)velocity_current == 0 ? 0 :*/ fabs(util::to_deg(atan((k * leg) / (1 + velocity_current)))) * util::sgn(aPID.target_get());
  double steer = (aPID.output + stan);
  // aPID.output = steer;
  //  aPID.target_set(steer);
  //  aPID.compute(0);

  //aPID.target_set(wrapped_a_target + (leg*3));
  aPID.target_set(leg * 2);
  aPID.cur = drive_imu_get();
  aPID.compute(0);

  double fake_target = is_past_target(odom_target, odom_current);

  // Decide if we've past the target or not
  int dir = (current_turn_type == REV ? -1 : 1);                            // If we're going backwards, add a -1
  int flipped = util::sgn(fake_target) != util::sgn(past_target) ? -1 : 1;  // Check if we've flipped directions to what we started

  // Compute xy PID
  double temp_target = fabs(fake_target);  // Use this instead of distance formula to fix impossible movements
  xyPID.target_set(temp_target * dir * flipped);
  xyPID.cur = odom_current.x + odom_current.y;
  xyPID.compute(0);

  // Prioritize turning by scaling xy_out down
  double cos_scale = fabs(cos(util::to_rad(wrapped_a_target)));
  double scale = (cos_scale / close_to_target) / (1.0 / close_to_target);
  scale = close_to_target / (close_to_target * cos_scale);
  // double horiz_scale = fabs(is_past_target(odom_target, odom_current));
  double xy_out = xyPID.output;
  double a_out = aPID.output;
  // if (scale != 0)
  // a_out /= scale;

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());

  // Vector scaling so nothing can be larger then max speed
  double max_slew_out = fmin(slew_left.output(), slew_right.output());

  //xy_out = util::clamp(xy_out, max_slew_out, -max_slew_out);
  //a_out = util::clamp(a_out, max_slew_out, -max_slew_out);

  // constrain lateral output by the max speed it can travel at without
  // slipping
  double radius = 1.0 / fabs(getCurvature(odom_current, odom_target));
  double maxSlipSpeed = sqrt(radius * 5000);
  xy_out = std::clamp(xy_out, -maxSlipSpeed, maxSlipSpeed);
  // prioritize angular movement over lateral movement
  double overturn = fabs(a_out) + fabs(xy_out) - max_slew_out;
  if (overturn > 0) xy_out -= xy_out > 0 ? overturn : -overturn;

  // Raw outputs
  double l_out = xy_out + a_out;
  double r_out = xy_out - a_out;

  // When left and right slew are disabled, scale max speed by the turn scaler so robot goes slower in curves
  if (fabs(l_out) > max_slew_out || fabs(r_out) > max_slew_out) {
    if (fabs(l_out) > fabs(r_out)) {
      r_out = r_out * (max_slew_out / fabs(l_out));
      l_out = util::clamp(l_out, max_slew_out, -max_slew_out);
    } else {
      l_out = l_out * (max_slew_out / fabs(r_out));
      r_out = util::clamp(r_out, max_slew_out, -max_slew_out);
    }
  }

  // Curvature Drive from
  // https://github.com/OkapiLib/OkapiLib/blob/54995fd390aaf4d4949a516a76580c50b394912f/src/api/chassis/model/skidSteerModel.cpp#L140-L169
/*
  double forward = xy_out / (127.0);
  double curve = a_out / (127.0);

  // double left_speed = forward + fabs(forward) * curve;
  // double right_speed = forward - fabs(forward) * curve;
  double left_speed = forward + curve;
  double right_speed = forward - curve;

  // normalizes output
  double faster_side = fmax(fabs(left_speed), fabs(right_speed));
  if (faster_side > 1.0) {
    left_speed /= faster_side;
    right_speed /= faster_side;
  }

  left_speed = left_speed * max_slew_out;
  right_speed = right_speed * max_slew_out;
  l_out = left_speed;
  r_out = right_speed;
  last_l_out = l_out;
  last_r_out = r_out;
*/

  // printf("cos %.2f   max_slew_out %.2f      headingerr: %.2f\n", cos_scale, max_slew_out, aPID.target_get());
  // printf("lr(%.2f, %.2f)   xy_raw: %.2f   xy_out: %.2f   heading_out: %.2f      cos: %.2f   max_slew_out: %.2f\n", l_out, r_out, xyPID.output, xy_out, aPID.output, cos_scale, max_slew_out);
  printf("xyt(%.2f, %.2f, %.2f)   lr(%.2f, %.2f)   xya(%.2f, %.2f)   maxslip: %.2f   xyPID: %.2f   aPID: %.2f   steer: %.2f   leg: %.2f   st: %.2f   fake_target: %.2f   cos_scale; %.2f   ptf(%.2f, %.2f)   vel: %.2f\n", odom_current.x, odom_current.y, odom_current.theta, l_out, r_out, xy_out, a_out, maxSlipSpeed, xyPID.target_get(), wrapped_a_target, steer, leg, stan, fake_target, cos_scale, ptf.x, ptf.y, velocity_current /*, fake_x, fake_y, util::to_deg(fake_angle)fake_xy(%.2f, %.2f, %.2f)*/);

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);

  // This is for wait_until
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::boomerang_task() {
  static bool close;
  int add = current_turn_type == REV ? 180 : 0;  // Decide if going fwd or rev
  bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;

  // pose target = pp_movements[pp_index].target;
  pose real_target = pp_movements[pp_index].target;
  pose closer_target = util::vector_off_point(LOOK_AHEAD - 3.5, {real_target.x, real_target.y, real_target.theta + add - 180});
  closer_target.theta = real_target.theta;
  pose target = closer_target;

  /*
      target = real_target;
    raw_pid_odom_ptp_set({target, pp_movements[pp_index].turn_type, pp_movements[pp_index].max_xy_speed}, slew_on);
  else if (!close) {
    target = closer_target;
  }
  */

  double r_theta = atan2(target.x - odom_current.x, target.y - odom_current.y);
  double j = fabs(util::to_deg(r_theta) - target.theta);
  double h = (24.0 / (90.0 / j));
  pose carrot = util::vector_off_point(h, {target.x, target.y, target.theta + add - 180});

  if (util::distance_to_point(closer_target, odom_current) < 5.0 && !close) {
    carrot = real_target;
    raw_pid_odom_ptp_set({carrot, pp_movements[pp_index].turn_type, pp_movements[pp_index].max_xy_speed}, slew_on);
    close = true;
  }

  std::vector<pose> bm = util::boomerang(odom_current, pp_movements[pp_index].target, 0.5);
  carrot = bm[0];

  if (!close) {
    raw_pid_odom_ptp_set({carrot, pp_movements[pp_index].turn_type, pp_movements[pp_index].max_xy_speed}, slew_on);
  }
  // printf("current(%.2f, %.2f)   target(%.2f, %.2f)   r_theta: %.2f   j: %.2f   h: %.2f   carrot(%.2f, %.2f)\n", odom_current.x, odom_current.y, target.x, target.y, r_theta, j, h, carrot.x, carrot.y);
  ptp_task();
}

void Drive::pp_task() {
  if (fabs(util::distance_to_point(pp_movements[pp_index].target, odom_current)) < LOOK_AHEAD) {
    if (pp_index < pp_movements.size() - 1) {
      pp_index = pp_index >= pp_movements.size() - 1 ? pp_index : pp_index + 1;
      bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;

      // if (pp_movements[pp_index].target.theta != errno)
      //   boomerang_task();
      // else
      raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);
    }
  }

  ptp_task();
}