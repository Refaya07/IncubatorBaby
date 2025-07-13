#include "TimeKeeping.h"

bool TimeKeeping::init() {
    if (!rtc.begin()) {
        Serial0.println("RTC not found!");
        return false;
    }

    if (rtc.lostPower()) {  // Jika RTC kehilangan daya atau belum diatur, set waktu
        Serial0.println("RTC kehilangan daya, mengatur ulang waktu!");
        // Atur RTC ke tanggal & waktu kompilasi sketsa
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    return true;
}

void TimeKeeping::setTime(DateTime dt) { rtc.adjust(dt); }

DateTime TimeKeeping::getTime() { return rtc.now(); }

void TimeKeeping::printTime(DateTime dt) {
    ESP_LOGI("TimeKeeping", "Current Time: %04d/%02d/%02d %02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(),
             dt.minute(), dt.second());
}