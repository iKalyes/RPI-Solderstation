#include <MAX6675_Service.h>

MAX6675 sensor1(MAX6675_SO_PIN, MAX6675_CS1_PIN, MAX6675_CLK_PIN, &SPI1, 1000000);
MAX6675 sensor2(MAX6675_SO_PIN, MAX6675_CS2_PIN, MAX6675_CLK_PIN, &SPI1, 1000000);
lv_timer_t* MAX6675_Timer;

void MAX6675_Init()
{
    sensor1.begin();
    sensor2.begin();
    MAX6675_Timer = lv_timer_create(MAX6675_Task, 200, NULL);
}

void MAX6675_Task(lv_timer_t *timer)
{
        if(Soldering_Enabled == false)
        {
            Soldering_Status = sensor1.read();
            Soldering_Temp = (int)sensor1.getTemperature();
            if (Soldering_Status == 0)
            {
                lv_label_set_text_fmt(ui_SolderingTemp, "%.3d", Soldering_Temp);
            }
            else
            {
                lv_label_set_text(ui_SolderingTemp, "ERR");
            }
        }
        Heatgun_Status = sensor2.read();
        Heatgun_Temp = (int)sensor2.getTemperature();
        if (Heatgun_Status == 0)
        {
            lv_label_set_text_fmt(ui_HeatgunTemp, "%.3d", Heatgun_Temp);
        }
        else
        {
            lv_label_set_text(ui_HeatgunTemp, "ERR");
        }
}