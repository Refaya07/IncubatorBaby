#include "FanControl.h"
#include <esp_log.h>

static const char* TAG = "FanControl";

void FanControl::init(int channel, bool activeHigh) {
    ledcSetup(channel, 25000, 8);  // Set up PWM on pin with 8-bit resolution
    ledcAttachPin(pin, channel);   // Attach the pin to the PWM channel
    ledcWrite(pin, 0);             // Start with fan off
    this->channel = channel;
    this->activeHigh = activeHigh;
}

void FanControl::setSpeed(int speed) {
    if (speed < 0 || speed > 100) {
        ESP_LOGE(TAG, "Invalid speed value. Must be between 0 and 100.");
        return;
    }

    this->speed = speed;
    int pwmValue = (speed * 255) / 100;  // Convert percentage to PWM value (0-255)

    if (activeHigh) {
        ledcWrite(channel, pwmValue);  // Set the PWM value to control fan speed
    } else {
        ledcWrite(channel, 255 - pwmValue);  // Invert the PWM value for active low
    }
}