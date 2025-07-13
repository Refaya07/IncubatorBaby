#pragma once

#include "lib/ADS_Peripherals.h"

class VoltageSensor : public ADS_Peripherals {
   public:
    VoltageSensor(ADS_Thread* ads, uint8_t ch) : ADS_Peripherals(ads, ch) {}
    float read();
};