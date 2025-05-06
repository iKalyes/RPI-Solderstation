#ifndef _HEATER_RUN_H_
#define _HEATER_RUN_H_

#include <Arduino.h>
#include <variables.h>

#include "hardware/pwm.h"
#include "sTune/src/sTune.h"
#include "QuickPID/src/QuickPID.h"
#include "ui-pidsetting.h"
#include <flash.h>

void heater_tempset();
void heater_templimit();

void PID_flag_True();
void PID_flag_False();

void heater_init();
void heater_run();
void heater_stop();
void SetStatusRunPid();
void SetStatusSample();

void fan_init();
void fan_on();
void fan_off();

#endif