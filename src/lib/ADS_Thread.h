#pragma once

#include <ADS1X15.h>
#include "freertos/semphr.h"

class ADS_Thread {
   private:
    ADS1015 dev;
    const char* devName;
    uint8_t chEnable[4];
    int16_t adcs[4] = {0, 0, 0, 0};
    float volts[4] = {0, 0, 0, 0};

    SemaphoreHandle_t mutex = NULL;

   public:
    ADS_Thread(const char* name, uint8_t addr) : devName(name), dev(addr) {}

    bool begin(bool ch0En, bool ch1En, bool ch2En, bool ch3En, uint8_t dataRate = 7, uint8_t gain = 1);
    int16_t readADC(uint8_t ch);
    float readVoltage(uint8_t ch);

    void requestADSData();
};