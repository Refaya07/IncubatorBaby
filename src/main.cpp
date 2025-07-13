#include <Arduino.h>
#include <Wire.h>
#include <esp_log.h>
#include "ASTRA_BabyIncubator.h"
#include "ASTRA_BabyIncubatorHMI.h"
#include "HardwareConfig.h"
#include "TimeKeeping.h"

#define FW_VERSION "1.0.0"  ///< Firmware version string

TimeKeeping timeKeeping;
ASTRA_BabyIncubator abi;
ASTRA_BabyIncubatorHMI hmi;

static const char* TAG = "Main";

static void checkCriticalError(const char* msg, bool isFailed) {
    if (!isFailed) return;
    ESP_LOGE(TAG, "%s, restarting...", msg);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

void setup() {
    /** Essentials setup **/
    Serial0.begin(115200);
    Serial0.setDebugOutput(true);

    // Give first pulse to external WDT
    pinMode(WDT_WDI_PIN, OUTPUT);
    digitalWrite(WDT_WDI_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(WDT_WDI_PIN, LOW);

    ESP_LOGI(TAG, "Initializing Astra Baby Incubator...");
    ESP_LOGI(TAG, "Firmware version: v %s", FW_VERSION);

    checkCriticalError("I2C bus initialization failed", !Wire.begin(SDA_PIN, SCL_PIN));
    checkCriticalError("Time keeping initialization failed", !timeKeeping.init());
    checkCriticalError("Failed to start Astra Baby Incubator program", !abi.begin());
    checkCriticalError("Failed to start Astra Baby Incubator HMI program", !hmi.begin());
}

static void printSensorStatus(const char* title, bool isFailed, float data) {
    if (isFailed) {
        ESP_LOGW(TAG, "%s: FAILED", title);
        return;
    }

    ESP_LOGI(TAG, "%s: %.3f", title, data);
}

static void pulseWDT() {
    digitalWrite(WDT_WDI_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(WDT_WDI_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void loop() { pulseWDT(); }