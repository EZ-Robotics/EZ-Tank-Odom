#include "main.h"

void intake_init() {
  intake.set_brake_modes(MOTOR_BRAKE_HOLD);
}

void set_intake(int input) {
  intake = input;
}

void intake_opcontrol() {
  if (master.get_digital(DIGITAL_R1)) {
    set_intake(127);
  } else if (master.get_digital(DIGITAL_R2)) {
    set_intake(-127);
  } else {
    set_intake(0);
  }
}