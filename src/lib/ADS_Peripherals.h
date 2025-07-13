#pragma once

#include "ADS_Thread.h"

class ADS_Peripherals {
   protected:
    ADS_Thread* dev;
    uint8_t channel;
    bool isSensorError = false;

   public:
    ADS_Peripherals(ADS_Thread* ads, uint8_t ch = 0) : dev(ads), channel(ch) {}

    bool isSensorFailed() const { return isSensorError; }
};