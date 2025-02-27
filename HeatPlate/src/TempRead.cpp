#include <tempread.h>

TMP102 sensor0;
MAX6675 sensor1(24, 25, 26, &SPI1, 1000000);

void TMP102_init()
{
    Wire1.setSDA(18);
    Wire1.setSCL(19);
    Wire1.begin();
    
    sensor0.begin();

    // set the number of consecutive faults before triggering alarm.
    // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
    sensor0.setFault(0);  // Trigger alarm immediately
    // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
    sensor0.setAlertPolarity(1); // Active HIGH
    // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
    sensor0.setAlertMode(0); // Comparator Mode.
    // set the Conversion Rate (how quickly the sensor gets a new reading)
    //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
    sensor0.setConversionRate(2);
    //set Extended Mode.
    //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
    sensor0.setExtendedMode(0);
    sensor0.wakeup();
}

void MAX6675_init()
{
    sensor1.begin();
}

void TMP102_Read(void *param)
{
    (void) param;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(250);
    xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        room_temperature = sensor0.readTempC();
    }
    vTaskDelete(NULL);
}

void MAX6675_Read(void *param)
{
    (void) param;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(250);
    xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        heater_status = sensor1.read();
        heater_temperature = sensor1.getTemperature();
    }
    vTaskDelete(NULL);
}