#ifndef _TMP102_H_
#define _TMP102_H_

#include <SparkFunTMP102.h>
#include <lvgl.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <task.h>

void TMP102_init();
void TMP102_Read(void *param);

extern float room_temperature;

#endif