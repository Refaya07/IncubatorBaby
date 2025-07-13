#pragma once

#include <Arduino.h>

class FanControl {
   private:
    int pin;
    int channel;
    bool activeHigh = true;  // True: Active High, False: Active Low
    int speed = 0;           // Speed percentage (0-100)

   public:
    FanControl(int pin) : pin(pin) {}

    void init(int channel, bool activeHigh = true);
    void setSpeed(int speed);
    bool getSpeed() const { return speed; }
};