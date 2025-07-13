#pragma once

#include "lib/ADS_Peripherals.h"

class SkinTempSensor : public ADS_Peripherals {
   private:
    const float vref = 3.285;    // Reference voltage
    const float rknown = 10000;  // Known resistance in ohms

    float temperatureResistanceTable(float r);

   public:
    SkinTempSensor(ADS_Thread* ads, uint8_t ch) : ADS_Peripherals(ads, ch) {}
    float read();
};