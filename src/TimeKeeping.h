#pragma once

#include <RTClib.h>

class TimeKeeping {
   private:
    RTC_DS3231 rtc;

   public:
    TimeKeeping() {}

    bool init();

    void setTime(DateTime dt);
    DateTime getTime();

    void printTime(DateTime dt);
};

extern TimeKeeping timeKeeping;