#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui/ui.h"
#include <RAK14014_FT6336U.h>
#include <variables.h>

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf_1[ screenWidth * screenHeight / 10 ];
static lv_color_t buf_2[ screenWidth * screenHeight / 10 ];

void display_init();
void backlight_init();

void lvgl_run();
void lvgl_tmp102_refresh();
void lvgl_max6675_refresh();
void lvgl_clock_refresh();

#endif