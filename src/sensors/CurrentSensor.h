#pragma once

#include <ADS1X15.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lib/ADS_Peripherals.h"

typedef enum {
    SENSE_CURRENT_NORMAL = 0,
    SENSE_UNDER_CURRENT,
    SENSE_OVERCURRENT,
} CurrentSenseState_t;

class CurrentSensor : public ADS_Peripherals {
   private:
    bool enabled = false;
    float currentReading_A = 0.0f;
    float burdenResistor_ohm;

    TaskHandle_t taskHandle = NULL;
    QueueHandle_t queue = NULL;

    CurrentSenseState_t state = SENSE_CURRENT_NORMAL;

    float lowThreshold_A;
    float highThreshold_A;

   public:
    CurrentSensor(ADS_Thread* ads, uint8_t ch) : ADS_Peripherals(ads, ch) {}

    void init(float r, float lowThres, float highThres);
    void enableReading(bool enable);

    void calcIrms();
    float read();
    CurrentSenseState_t getState() { return state; }
};