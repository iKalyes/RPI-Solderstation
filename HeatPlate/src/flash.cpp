#include <flash.h>

void WriteFlash() {
    EEPROM.begin(256);
    EEPROM.write(0, buzzer_state); //uint8_t
    EEPROM.write(1, brightness); //uint8_t
    EEPROM.put(2, temp_limited); //uint16_t
    EEPROM.put(4, sleep_time); //uint16_t
    EEPROM.put(6, SetTemp); //uint16_t
    EEPROM.end();
}

void ReadFlash() {
    EEPROM.begin(256);
    buzzer_state = EEPROM.read(0);
    brightness = EEPROM.read(1);
    EEPROM.get(2, temp_limited);
    EEPROM.get(4, sleep_time);
    EEPROM.get(6, SetTemp);
    EEPROM.end();
}

void ClearFlash(){
    EEPROM.begin(256);
    for (int i = 0; i < 256; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.end();
}