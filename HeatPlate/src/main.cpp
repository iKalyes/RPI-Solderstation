/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <main.h>

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    ReadFlash();
    backlight_init();
    display_init();
}

void loop()
{
    lvgl_tmp102_refresh();
    lvgl_run();
}


RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup1()
{
  TMP102_init();
  xTaskCreate(TMP102_Read, "TMP102_Read", 512, NULL, 1, NULL);
  encoder = new RotaryEncoder(16, 17, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(16), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), checkPosition, CHANGE);
}

void loop1()
{
  static int pos = 0;

  encoder->tick(); // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    //Serial.print("pos:");
    //Serial.print(newPos);
    //Serial.print(" dir:");
    //Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  }
}