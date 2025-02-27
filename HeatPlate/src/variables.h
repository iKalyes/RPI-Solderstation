#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include <Arduino.h>

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

#endif