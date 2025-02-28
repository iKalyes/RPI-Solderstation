#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include <Arduino.h>

#include <lvgl.h>
#include "ui/ui.h"
#include "ui/ui_helpers.h"
#include "ui/ui_events.h"
#include "ui/ui_theme_manager.h"
#include "ui/ui_themes.h"

//clock//
extern uint8_t timer_second;
extern uint8_t timer_minute;
//temperature//
extern float room_temperature;
extern float heater_temperature;
extern int heater_status;
//custom curve//
extern uint16_t stage1_temp;
extern uint16_t stage1_time;
extern uint16_t stage2_temp;
extern uint16_t stage2_time;
extern uint16_t stage3_temp;
extern uint16_t stage3_time;
extern uint16_t stage4_temp;
extern uint16_t stage4_time;
extern uint16_t stage5_temp;
extern uint16_t stage5_time;
//main screen//
extern uint8_t buzzer_status;
extern uint8_t clock_status;
extern uint8_t tempset_status;
//setting screen//
extern uint16_t temp_limited;
extern uint16_t sleep_time;
extern uint8_t brightness;
//tempset screen//
extern uint16_t SetTemp;
extern uint16_t PIDSetTemp;
//pidcalibration screen//
extern lv_timer_t* chart_update_timer;
extern lv_chart_series_t * ui_TempChart_TempSeries;
extern lv_chart_series_t * ui_TempChart_DutySeries;

#endif