#ifndef _FLASH_H_
#define _FLASH_H_ 

#include <EEPROM.h>
#include <ui-main.h>
#include <ui-setting.h>
#include <ui-tempset.h>
#include <ui-customcurve.h>
#include <ui-pidcalibration.h>

void WriteFlash();
void ReadFlash();
void ClearFlash();

#endif