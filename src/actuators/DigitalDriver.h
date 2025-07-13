#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <functional>

typedef struct {
    uint32_t duration_ms;
    int32_t times;
} OneShot_t;

typedef std::function<void(bool)> CallbackOnActive;

class DigitalDriver {
   private:
    int pin;
    bool en = true;
    bool isOn = false;
    bool activeHigh = false;  // True: Active High, False: Active Low
    int onDuration_ms = 0;
    int offDuration_ms = 0;
    int dutyCycle = 0;

    TimerHandle_t blinkTimer = NULL;
    QueueHandle_t oneShotQueue = NULL;

    CallbackOnActive callbackOnActive = NULL;

    void _setDigitalPin(bool state);
    bool _stopPWM();

   public:
    DigitalDriver(int pin) : pin(pin) {}

    void init(bool defaultState = LOW, bool onState = LOW, CallbackOnActive cb = NULL);
    void toggle();
    void enable(bool en);
    bool setPWM(int dutyCycle, int period_ms = 1000);
    bool setOneShot(uint32_t duration_ms = 50, int32_t times = 1);
    void runOneShot();
    int getDutyCycle() const { return dutyCycle; }
};