#include <variables.h>

//clock//
uint8_t timer_second;
uint8_t timer_minute;
//temperature//
float room_temperature;
float heater_temperature;
int heater_status;
//custom curve//
uint16_t stage1_temp;
uint16_t stage1_time;
uint16_t stage2_temp;
uint16_t stage2_time;
uint16_t stage3_temp;
uint16_t stage3_time;
uint16_t stage4_temp;
uint16_t stage4_time;
uint16_t stage5_temp;
uint16_t stage5_time;
//main screen//
uint8_t buzzer_status;
uint8_t clock_status;
uint8_t tempset_status;
uint8_t heating_status;
//setting screen//
uint16_t temp_limited;
uint16_t sleep_time;
uint8_t brightness;
//tempset screen//
uint16_t SetTemp;
//pidcalibration screen//
lv_timer_t* chart_update_timer;
lv_chart_series_t * ui_TempChart_TempSeries;
lv_chart_series_t * ui_TempChart_DutySeries;
//heater-run//
float all_Kp;
float all_Ki;
float all_Kd;
