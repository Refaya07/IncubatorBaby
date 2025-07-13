#pragma once

#include "lib/ADS_Peripherals.h"

class WaterLevelSensor : public ADS_Peripherals {
   public:
    WaterLevelSensor(ADS_Thread* ads, uint8_t ch) : ADS_Peripherals(ads, ch) {}
    float read();
};