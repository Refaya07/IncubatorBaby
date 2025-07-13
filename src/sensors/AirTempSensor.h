#pragma once

#include <DallasTemperature.h>
#include <OneWire.h>

class AirTempSensor {
   private:
    DallasTemperature* dev;
    OneWire* oneWire;
    bool isSensorError = false;

    float gradient = 1.0;

    float _correctTemperature(float temperature);

   public:
    ~AirTempSensor() {
        if (dev) delete dev;
    }

    void init(uint8_t pin);
    float read();
    bool isSensorFailed() const { return isSensorError; }
};