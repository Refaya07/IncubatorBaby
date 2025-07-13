#pragma once

#include <DHT.h>

class HumiditySensor {
   private:
    DHT* dev;
    bool isSensorError = false;

    float humidity = 0;
    float temperature = 0;

   public:
    ~HumiditySensor() {
        if (dev) delete dev;
    }

    void init(uint8_t pin, uint8_t type);
    float read();
    float readTemperature();
    bool isSensorFailed() const { return isSensorError; }
    void readTick();
};