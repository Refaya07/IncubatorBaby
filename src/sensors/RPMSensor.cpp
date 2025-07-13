#include "RPMSensor.h"
#include "HardwareConfig.h"
#include "esp_log.h"

static void readRPMTask(void* param) {
    float* rpm = (float*)param;
    while (true) {
        if (xSemaphoreTake(critMutex, pdMS_TO_TICKS(50))) {
            if (FreqCountESP.available()) {
                uint32_t count = FreqCountESP.read();
                *rpm = (count * 60) / 2;
            }
            xSemaphoreGive(critMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void RPMSensor::init(uint8_t pin) {
    FreqCountESP.begin(pin, 1000);
    xTaskCreatePinnedToCore(readRPMTask, "Read RPM Task", 2048, &currentRPM, 1, NULL, 0);
}

float RPMSensor::read() { return currentRPM; }