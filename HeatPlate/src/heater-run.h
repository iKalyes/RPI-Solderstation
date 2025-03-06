#ifndef _HEATER_RUN_H_
#define _HEATER_RUN_H_

#include <Arduino.h>
#include <variables.h>

#include "hardware/pwm.h"
#include "sTune/src/sTune.h"
#include "QuickPID/src/QuickPID.h"

void heater_tempset();
void heater_templimit();

void heater_init();
void heater_run();
void heater_stop();

void fan_init();
void fan_on();
void fan_off();

#endif