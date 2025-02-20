#ifndef _MAIN_H_
#define _MAIN_H_

#include <TFT_eSPI.h>

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

#include <RotaryEncoder.h>
#include <display.h>
#include <flash.h>

#define LVGL_TASK_HANDLER_PRIORITY (tskIDLE_PRIORITY + 3)
#define LVGL_TASK_HANDLER_STACK_SIZE (1024)

#endif