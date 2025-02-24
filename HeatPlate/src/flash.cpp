#include <flash.h>

void WriteFlash() {
    EEPROM.begin(256);

    EEPROM.write(0, buzzer_status); //uint8_t
    EEPROM.write(1, brightness); //uint8_t

    EEPROM.put(2, temp_limited); //uint16_t
    EEPROM.put(4, sleep_time); //uint16_t
    EEPROM.put(6, SetTemp); //uint16_t

    EEPROM.put(8, stage1_temp); //uint16_t
    EEPROM.put(10, stage1_time); //uint16_t
    EEPROM.put(12, stage2_temp); //uint16_t
    EEPROM.put(14, stage2_time); //uint16_t
    EEPROM.put(16, stage3_temp); //uint16_t
    EEPROM.put(18, stage3_time); //uint16_t
    EEPROM.put(20, stage4_temp); //uint16_t
    EEPROM.put(22, stage4_time); //uint16_t
    EEPROM.put(24, stage5_temp); //uint16_t
    EEPROM.put(26, stage5_time); //uint16_t

    EEPROM.end();
}

void ReadFlash() {
    EEPROM.begin(256);

    buzzer_status = EEPROM.read(0);
    brightness = EEPROM.read(1);

    EEPROM.get(2, temp_limited);
    EEPROM.get(4, sleep_time);
    EEPROM.get(6, SetTemp);

    EEPROM.get(8, stage1_temp);
    EEPROM.get(10, stage1_time);
    EEPROM.get(12, stage2_temp);
    EEPROM.get(14, stage2_time);
    EEPROM.get(16, stage3_temp);
    EEPROM.get(18, stage3_time);
    EEPROM.get(20, stage4_temp);
    EEPROM.get(22, stage4_time);
    EEPROM.get(24, stage5_temp);
    EEPROM.get(26, stage5_time);

    EEPROM.end();
}

void ClearFlash(){
    EEPROM.begin(256);
    for (int i = 0; i < 256; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.end();
}