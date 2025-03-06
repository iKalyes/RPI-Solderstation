#include <encoder.h>

RotaryEncoder *encoder = nullptr;

void encoder_tick()
{
  encoder->tick(); // just call tick() to check the state.
}

void encoder_init()
{
    encoder = new RotaryEncoder(14, 15, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(14), encoder_tick, CHANGE);
    attachInterrupt(digitalPinToInterrupt(15), encoder_tick, CHANGE);
}

void encoder_run()
{
    static int pos = 0;
    int newPos = encoder->getPosition();
    int Dir = (int)(encoder->getDirection());
    if (pos != newPos) 
    {
        Serial.print("pos:");
        Serial.print(newPos);
        Serial.print(" dir:");
        Serial.println((int)(encoder->getDirection()));
        pos = newPos;
    }
}