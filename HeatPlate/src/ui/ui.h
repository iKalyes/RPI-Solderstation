// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: HeatPlate

#ifndef _HEATPLATE_UI_H
#define _HEATPLATE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"

void SettingRollingDown_Animation(lv_obj_t * TargetObject, int delay);
void SettingRollingUp_Animation(lv_obj_t * TargetObject, int delay);
void SettingIN_Animation(lv_obj_t * TargetObject, int delay);
void SettingOUT_Animation(lv_obj_t * TargetObject, int delay);

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
void ui_event_Setting(lv_event_t * e);
extern lv_obj_t * ui_Setting;
extern lv_obj_t * ui_Image6;
extern lv_obj_t * ui_SET1;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_SET2;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_SET3;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_SET5;
extern lv_obj_t * ui_SET6;
extern lv_obj_t * ui_SET7;
extern lv_obj_t * ui_SET8;
extern lv_obj_t * ui_SET9;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_setting_png);    // assets/setting.png

// FONTS
LV_FONT_DECLARE(ui_font_SarasaMonoASCII18);

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
