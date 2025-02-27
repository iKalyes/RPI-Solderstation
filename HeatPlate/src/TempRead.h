#ifndef _TEMPREAD_H_
#define _TEMPREAD_H_

#include <SparkFunTMP102.h>
#include <MAX6675.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <task.h>

#include <variables.h>

void TMP102_init();
void MAX6675_init();
void TMP102_Read(void *param);
void MAX6675_Read(void *param);

#endif