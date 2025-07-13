#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lib/FreqCountESP.h"

class RPMSensor {
   private:
    bool isSensorError = false;
    float currentRPM = 5000.0f;

   public:
    void init(uint8_t pin);
    float read();

    bool isSensorFailed() const { return isSensorError; }
};