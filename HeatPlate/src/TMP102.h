#ifndef _TMP102_H_
#define _TMP102_H_

#include <SparkFunTMP102.h>
#include <lvgl.h>
#include <Wire.h>

void TMP102_init();
void TMP102_Read();

extern float room_temperature;

#endif