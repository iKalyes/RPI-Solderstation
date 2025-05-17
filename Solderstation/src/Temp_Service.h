#ifndef _TEMP_SERVICE_H
#define _TEMP_SERVICE_H

#include <Arduino.h>
#include <lvgl.h>
#include <ui/ui.h>
#include <variables.h>

#define Soldering_Pin 29
#define Heatgun_Pin 28

void Temp_Init();
void Temp_Task();

#endif