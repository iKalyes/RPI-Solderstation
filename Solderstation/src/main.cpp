/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <main.h>

void setup()
{
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  display_init();
  encoder_lvgl_init();
  lvgl_group_init();
  INA226_Init();
  TMP102_Init();
}

void loop()
{
  lvgl_task_handler();
}

void setup1()
{

}

void loop1()
{

}
