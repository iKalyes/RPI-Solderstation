/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <main.h>

void setup()
{
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  display_init();
}

void loop()
{
  lvgl_task_handler();
}
