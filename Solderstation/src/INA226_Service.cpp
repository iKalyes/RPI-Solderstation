#include <INA226_Service.h>

lv_timer_t* INA226_Timer;

INA226 INA(0x40, &Wire1);

void INA226_Init()
{
  Wire1.setSCL(INA226_SCL);
  Wire1.setSDA(INA226_SDA);
  Wire1.begin();
  INA.begin();
  INA.setMaxCurrentShunt(10, 0.01);
  INA226_Timer = lv_timer_create(INA226_Task, 200, NULL);
}


void INA226_Task(lv_timer_t *timer)
{
    // 电压：2位整数，2位小数
    int voltage_full = round(INA.getBusVoltage() * 100);
    int voltage_int = voltage_full / 100;
    int voltage_frac = voltage_full % 100;
    
    // 电流：1位整数，2位小数
    int current_full = round(INA.getCurrent() * 1000);
    int current_int = current_full / 1000;
    int current_frac = current_full % 1000;
    
    // 功率：3位整数，1位小数
    int power_full = round(INA.getPower() * 10);
    int power_int = power_full / 10;
    int power_frac = power_full % 10;

    if(voltage_full >= 0 && current_full >= 0 && power_full >= 0)
    {
        lv_label_set_text_fmt(ui_INA226Voltage, "%02d.%02dV", voltage_int, voltage_frac);
        lv_label_set_text_fmt(ui_INA226Current, "%01d.%03dA", current_int, current_frac);
        lv_label_set_text_fmt(ui_INA226Power, "%03d.%01dW", power_int, power_frac);
    }
    else
    {
        lv_label_set_text(ui_INA226Voltage, "00.00V");
        lv_label_set_text(ui_INA226Current, "0.000A");
        lv_label_set_text(ui_INA226Power, "000.0W");
    }
}

