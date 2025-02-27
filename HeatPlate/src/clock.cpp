#include <clock.h>

uint8_t timer_250ms;

void clock_run(void *param)
{
    (void) param;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(250);
    xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        if(clock_status == 1)
        {
        timer_250ms++;
            if(timer_250ms == 4)
            {
                timer_second++;
                timer_250ms = 0;
                    if(timer_second == 60)
                    {
                        timer_minute++;
                        timer_second = 0;
                    }
            }
        }
        else
        {
            timer_250ms = 0;
            timer_second = 0;
            timer_minute = 0;
        }
    }
    vTaskDelete(NULL);
}