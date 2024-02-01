#pragma once

#include "api.h"

inline pros::MotorGroup intake({17, -18});
void intake_init();
void set_intake(int input);
void intake_opcontrol();