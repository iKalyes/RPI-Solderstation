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
    const TickType_t xPeriod = pdMS_TO_TICKS(200);
    xLastWakeTime = xTaskGetTickCount();

    // 初始化滤波相关变量
    float smoothed_temp = 0.0f;
    bool first_reading = true;
    const float alpha = 0.1f;  // 滤波系数(0-1)：越小越平滑但响应越慢

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        float raw_temp = sensor0.readTempC();
        
        // 应用指数加权移动平均滤波
        if (first_reading) {
            smoothed_temp = raw_temp;  // 第一次读取直接使用原始值
            first_reading = false;
        } else {
            // 新值占alpha比例，历史值占(1-alpha)比例
            smoothed_temp = alpha * raw_temp + (1.0f - alpha) * smoothed_temp;
        }
        
        room_temperature = smoothed_temp;
    }
    vTaskDelete(NULL);
}

void MAX6675_Read(void *param)
{
    (void) param;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(200);
    xLastWakeTime = xTaskGetTickCount();
    
    // 初始化滤波相关变量
    float smoothed_temp = 0.0f;
    bool first_reading = true;
    const float alpha = 0.5f;  // 滤波系数(0-1)：越小越平滑但响应越慢
    
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        heater_status = sensor1.read();
        float raw_temp = sensor1.getTemperature();
        
        // 应用指数加权移动平均滤波
        if (first_reading) {
            smoothed_temp = raw_temp;  // 第一次读取直接使用原始值
            first_reading = false;
        } else {
            // 新值占alpha比例，历史值占(1-alpha)比例
            smoothed_temp = alpha * raw_temp + (1.0f - alpha) * smoothed_temp;
        }
        
        heater_temperature = smoothed_temp;
    }
    vTaskDelete(NULL);
}