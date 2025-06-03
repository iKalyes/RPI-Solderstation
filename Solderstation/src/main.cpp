#include <main.h>

void setup()
{
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  ReadFlash();
  ReadPID();

  display_init();
  encoder_lvgl_init();
  lvgl_group_init();
  INA226_Init();
  TMP102_Init();
  MAX6675_Init();
}

void loop()
{
  lvgl_task_handler();
}

void setup1()
{
  Soldering_GPIO_Init();
  Heatgun_GPIO_Init();
  Cooling_FAN_GPIO_Init();
  Buzzer_GPIO_Init();
}

void loop1()
{

}
