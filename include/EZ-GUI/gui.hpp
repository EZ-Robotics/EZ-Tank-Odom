/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>

#include "api.h"
#include "pros/misc.h"

namespace ez {

/**
 * Is the SD card plugged in?
 */
const bool IS_SD_CARD = pros::usd::is_installed();

struct gui_motor_name {
  pros::Motor motor;
  std::string name;
};

struct gui_int_name {
  int motor_port;
  std::string name;
};

struct auton_and_name {
  std::string name;
  std::function<void()> auton_call;
};

class GUI {
 public:
  GUI(std::vector<gui_motor_name> motor_name, lv_color_t accent_color = LV_COLOR_HEX(0xFFC0CB));
  GUI(std::vector<gui_motor_name> motor_name, std::vector<auton_and_name> autons, lv_color_t accent_color = LV_COLOR_HEX(0xFFC0CB));
  GUI(std::vector<gui_int_name> int_name, lv_color_t accent_color = LV_COLOR_HEX(0xFFC0CB));
  GUI(std::vector<gui_int_name> int_name, std::vector<auton_and_name> autons, lv_color_t accent_color = LV_COLOR_HEX(0xFFC0CB));

  void screen_task();
  void disable();
  void enable();
  bool enabled();

  void selector_text_initialize();
  void selector_text_set(std::string text);
  void selector_text_wiggle_toggle(bool toggle);
  void selector_text_hide(bool hidden);
  void selector_buttons_hide(bool hidden);
  void selector_buttons_initialize();

  void auton_enable();
  void auton_disable();
  bool auton_button_screen_right();
  bool auton_button_screen_right_new();
  bool auton_button_screen_left();
  bool auton_button_screen_left_new();
  bool auton_button_left();
  bool auton_button_right();
  bool auton_button_left_new();
  bool auton_button_right_new();
  void auton_page_up();
  void auton_page_down();
  void auton_call();
  void auton_button_limitswitch_initialize(pros::ADIDigitalIn* limitswitch_right, pros::ADIDigitalIn* limitswitch_left = nullptr);
  int auton_amount_get();
  int auton_page_current_get();
  void auton_print();

  void background_hide(bool hidden);

  void motor_boxes_hide(bool hidden);

  bool left();
  bool right();

  void pong_enable();
  void pong_disable();
  bool pong_enabled();
  void pong_loop(std::int32_t ana_stick_L, std::int32_t ana_stick_R);

 private:
  bool gui_enabled = false;
  bool gui_initialized = false;
  void styles_initialize();
  lv_color_t ACCENT_COLOR = LV_COLOR_HEX(0xFFC0CB);
  lv_color_t BACKGROUND_COLOR = LV_COLOR_BLACK;
  pros::Task screenTask;

  pros::ADIDigitalIn* auton_button_limitswitch_left = nullptr;
  pros::ADIDigitalIn* auton_button_limitswitch_right = nullptr;
  void auton_sd_initialize();
  void auton_sd_update();
  bool auton_button_screen_last_right = false;
  bool auton_button_screen_last_left = false;
  bool auton_button_last_right = false;
  bool auton_button_last_left = false;
  int amount_of_autos = 0;
  int auton_page_current = 0;
  std::vector<auton_and_name> autons_and_names;
  bool auton_enabled = false;
  bool auton_selector_last = false;
  bool auton_button_limitswitch_using = false;

  void selector_text_normal_set(std::string text);
  void selector_text_wiggle_set(std::string text);
  lv_obj_t* selector_text;
  lv_obj_t* selector_left;
  lv_obj_t* selector_right;
  lv_style_t selector_text_style;
  lv_style_t selector_button_style;
  bool selector_wiggle_text_enabled = true;

  lv_style_t background_style;
  void background_initialize();
  lv_obj_t* background;

  lv_style_t box_style;
  lv_style_t box_txt_style;
  void motor_boxes_initialize();
  void motor_boxes_calculate();
  void motor_boxes_update();
  struct box {
    int x1 = 0;
    int x2 = 0;
    int y1 = 0;
    int y2 = 0;
  };
  struct motor_display_constants_t {
    int rows = 0;
    int columns = 0;
    int box_width = 0;
    int box_height = 0;
    int boarder = 0;
  };
  motor_display_constants_t motor_display_constants;
  int motor_name_length_max = 0;
  std::vector<pros::Motor> motors;
  std::vector<std::string> motor_names;
  std::vector<lv_obj_t*> motor_boxes_obj;
  std::vector<lv_obj_t*> motor_names_obj;
  std::vector<double> motor_temps;
  std::vector<box> motor_box_positions;
  bool motor_boxes_hidden = true;
  bool last_motor_box_hidden = false;

  bool pong_initialized = false;
  bool is_pong_enabled = false;
  lv_obj_t* pong_paddle_left;
  lv_obj_t* pong_paddle_right;
  lv_obj_t* pong_ball;
  void lose_condition();
  bool checkCollision(std::int16_t padX,
                      std::int16_t padY,
                      std::int16_t padWidth,
                      std::int16_t padHeight,
                      std::int16_t bX,
                      std::int16_t bY);
  void stickMovement(std::int32_t ana_stick_L, std::int32_t ana_stick_R);
  void ballMovement();
  void pong_update(std::int32_t ana_stick_L, std::int32_t ana_stick_R);
  void pong_draw();
  void pong_initialize();
  int pong_score_last = -1;
};
}  // namespace ez