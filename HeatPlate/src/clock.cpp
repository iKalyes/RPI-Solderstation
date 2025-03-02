#include <clock.h>

uint8_t timer_200ms;

void clock_run(void *param)
{
    (void) param;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(200);
    xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        if(clock_status == 1)
        {
        timer_200ms++;
            if(timer_200ms == 5)
            {
                timer_second++;
                timer_200ms = 0;
                    if(timer_second == 60)
                    {
                        timer_minute++;
                        timer_second = 0;
                    }
            }
        }
        else
        {
            timer_200ms = 0;
            timer_second = 0;
            timer_minute = 0;
        }
    }
    vTaskDelete(NULL);
}