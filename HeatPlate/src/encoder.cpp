#include <encoder.h>

RotaryEncoder *encoder = nullptr;

void encoder_tick()
{
  encoder->tick(); // just call tick() to check the state.
}

void encoder_init()
{
    encoder = new RotaryEncoder(16, 17, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(16), encoder_tick, CHANGE);
    attachInterrupt(digitalPinToInterrupt(17), encoder_tick, CHANGE);
}

void encoder_run()
{
    static int pos = 0;
    int newPos = encoder->getPosition();
    if (pos != newPos) 
    {
        Serial.print("pos:");
        Serial.print(newPos);
        Serial.print(" dir:");
        Serial.println((int)(encoder->getDirection()));
        pos = newPos;
    }
}