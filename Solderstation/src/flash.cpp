#include <flash.h>

void WriteFlash() {
    EEPROM.begin(256);

    EEPROM.end();
}

void ReadFlash() {
    EEPROM.begin(256);

    EEPROM.end();
}

void ClearFlash(){
    EEPROM.begin(256);
    for (int i = 0; i < 256; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.end();
}