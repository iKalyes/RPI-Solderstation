#ifndef _VARIABLES_H
#define _VARIABLES_H

#include <Arduino.h>

extern uint16_t SolderingTargetTemp;
extern uint16_t SolderingTargetTempMin;
extern uint16_t SolderingTargetTempMax;
extern uint16_t SolderingStandbyTemp;
extern uint16_t SolderingStandbyTime;

extern uint16_t HeatgunTargetTemp;
extern uint16_t HeatgunTargetTempMin;
extern uint16_t HeatgunTargetTempMax;

extern uint16_t HeatgunWindSpeed;
extern uint16_t HeatgunWindSpeedMin;
extern uint16_t HeatgunWindSpeedMax;

extern float SolderingTempParameter;
extern float HeatgunTempParameter;

extern float SolderingTemp;
extern float HeatgunTemp;

extern uint8_t Brightness;

#endif