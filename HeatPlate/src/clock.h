#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>
#include <ui-main.h>

void clock_run(void *param);

extern uint8_t timer_second;
extern uint8_t timer_minute;

#endif