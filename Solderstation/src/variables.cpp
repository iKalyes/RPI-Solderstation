#include <variables.h>

uint16_t SolderingTargetTemp = 100;
uint16_t SolderingTargetTempMin = 50;
uint16_t SolderingTargetTempMax = 400;
uint16_t SolderingStandbyTemp = 100;
uint16_t SolderingStandbyTime = 10; // Minutes

uint16_t HeatgunTargetTemp = 100;
uint16_t HeatgunTargetTempMin = 100;
uint16_t HeatgunTargetTempMax = 400;

uint16_t HeatgunWindSpeed = 50;
uint16_t HeatgunWindSpeedMin = 10;
uint16_t HeatgunWindSpeedMax = 100;

float SolderingTempParameter = 0;
float HeatgunTempParameter = 0;

float SolderingTemp;
float HeatgunTemp;

uint8_t Brightness;