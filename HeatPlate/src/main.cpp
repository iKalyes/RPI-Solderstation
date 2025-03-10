/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <main.h>

void vApplicationTickHook()
{
  encoder_tick();
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    ReadFlash();
    display_init();
    backlight_init();
    update_chart_init();
}

void loop()
{
    lvgl_tmp102_refresh();
    lvgl_max6675_refresh();
    lvgl_clock_refresh();
    lvgl_run();
}

void setup1()
{
  TMP102_init();
  MAX6675_init();
  heater_init();
  fan_init();
  xTaskCreate(clock_run, "clock_run", 128, NULL, 4, NULL);
  xTaskCreate(TMP102_Read, "TMP102_Read", 128, NULL, 3, NULL);
  xTaskCreate(MAX6675_Read, "MAX6675_Read", 128, NULL, 5, NULL);
}

void loop1()
{
  heater_run();
  if(timer_minute >= sleep_time)
  {
    heater_stop();
    clock_status = 0;
  }
}