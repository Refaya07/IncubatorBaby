#include "DigitalDriver.h"
#include "esp_log.h"
#include "freertos/task.h"

static void blinkTimerCallback(TimerHandle_t xTimer) {
    DigitalDriver* driver = (DigitalDriver*)pvTimerGetTimerID(xTimer);
    if (driver != NULL) driver->toggle();
}

static void oneShotActive(void* param) {
    DigitalDriver* driver = static_cast<DigitalDriver*>(param);
    while (true) {
        driver->runOneShot();
    }
}

void DigitalDriver::_setDigitalPin(bool state) {
    digitalWrite(pin, (activeHigh) ? state : !state);
    isOn = state;
    if (callbackOnActive != NULL) callbackOnActive(isOn);
}

bool DigitalDriver::_stopPWM() {
    if (blinkTimer == NULL) return false;

    xTimerStop(blinkTimer, portMAX_DELAY);
    _setDigitalPin(LOW);

    return true;
}

void DigitalDriver::init(bool defaultState, bool onState, CallbackOnActive cb) {
    callbackOnActive = cb;

    pinMode(pin, OUTPUT);
    activeHigh = onState;

    blinkTimer = xTimerCreate("Blink timer", pdMS_TO_TICKS(1000), pdTRUE, this, &blinkTimerCallback);
    oneShotQueue = xQueueCreate(1, sizeof(OneShot_t));
    xTaskCreatePinnedToCore(oneShotActive, "One shot digital output", (1024 * 3), this, 1, NULL, 0);

    _setDigitalPin(defaultState);
}

void DigitalDriver::toggle() {
    if (!en) return;
    int duration_ms = (isOn) ? offDuration_ms : onDuration_ms;
    xTimerChangePeriod(blinkTimer, pdMS_TO_TICKS(duration_ms), 0);
    _setDigitalPin(!isOn);
}

void DigitalDriver::enable(bool en) {
    this->en = en;
    if (en) return;
    _stopPWM();
}

bool DigitalDriver::setPWM(int dutyCycle, int period_ms) {
    if (!en) return false;
    if (blinkTimer == NULL) return false;
    if (dutyCycle < 0 || dutyCycle > 100) return false;

    // ESP_LOGI("out", "Setting duty cycle to %d%", dutyCycle);
    this->dutyCycle = dutyCycle;

    if (dutyCycle == 0 || dutyCycle == 100) {
        _stopPWM();
        if (dutyCycle == 100) _setDigitalPin(HIGH);
        return true;
    }

    onDuration_ms = (period_ms * dutyCycle) / 100;
    offDuration_ms = period_ms - onDuration_ms;

    xTimerChangePeriod(blinkTimer, pdMS_TO_TICKS(onDuration_ms), 0);
    xTimerStart(blinkTimer, 0);

    _setDigitalPin(HIGH);

    return true;
}

bool DigitalDriver::setOneShot(uint32_t duration_ms, int32_t times) {
    if (!en) return false;
    OneShot_t cmd = {duration_ms, times};
    xQueueSend(oneShotQueue, &cmd, 0);
    return true;
}

void DigitalDriver::runOneShot() {
    OneShot_t cmd;
    xQueueReceive(oneShotQueue, &cmd, portMAX_DELAY);
    for (int8_t i = 0; i < cmd.times; i++) {
        _setDigitalPin(HIGH);
        vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));
        _setDigitalPin(LOW);
        vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms / 2));
    }
}