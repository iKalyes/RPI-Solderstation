#ifndef _FLASH_H_
#define _FLASH_H_ 

#include <EEPROM.h>
#include <variables.h>

void WriteFlash();
void ReadFlash();
void ClearFlash();
void ReadPID();
void WritePID();

#endif