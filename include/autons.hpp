#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

void defense2ball();
void defense2ballinterupt();
void defense2ballinterupt_barrier();
void defense2ball_nobar();
void defense2ballinterupt_barrier_nobar();

void offense2ball();
void offense3ball();
void offense3ball_nobar();
void offense2ball_nobar();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();

void default_constants();