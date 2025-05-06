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

void WritePID()
{
    EEPROM.begin(256);
    uint32_t Kp = (uint32_t)(all_Kp * 1000.0f + 0.5f); 
    uint32_t Ki = (uint32_t)(all_Ki * 1000.0f + 0.5f); 
    uint32_t Kd = (uint32_t)(all_Kd * 1000.0f + 0.5f); 

    EEPROM.put(28, Kp); //uint32_t
    EEPROM.put(32, Ki); //uint32_t
    EEPROM.put(36, Kd); //uint32_t

    EEPROM.end();
}

void ReadPID()
{
    EEPROM.begin(256);
    uint32_t Kp; 
    uint32_t Ki; 
    uint32_t Kd; 

    EEPROM.get(28, Kp);
    EEPROM.get(32, Ki);
    EEPROM.get(36, Kd);

    all_Kp = Kp / 1000.0f;
    all_Ki = Ki / 1000.0f;
    all_Kd = Kd / 1000.0f;

    EEPROM.end();
}

void ClearFlash(){
    EEPROM.begin(256);
    for (int i = 0; i < 256; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.end();
}