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
    lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(5);
}


RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup1()
{

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
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  }
  }