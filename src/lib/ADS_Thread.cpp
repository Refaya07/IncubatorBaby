#include "ADS_Thread.h"
#include "esp_log.h"

#define AVERAGE_ALPHA 0.1

static void readADSTask(void* param) {
    ADS_Thread* thread = static_cast<ADS_Thread*>(param);
    while (true) {
        thread->requestADSData();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool ADS_Thread::begin(bool ch0En, bool ch1En, bool ch2En, bool ch3En, uint8_t dataRate, uint8_t gain) {
    if (!dev.begin()) {
        ESP_LOGE(devName, "ADS1015 initialization failed");
        return false;
    }

    // Set ADS1015/ADS1115 configuration
    dev.setDataRate(7);
    dev.setGain(1);  // 4.096V range

    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        ESP_LOGE(devName, "Failed to create mutex");
        return false;
    }

    uint8_t chEn[4] = {ch0En, ch1En, ch2En, ch3En};
    for (uint8_t i = 0; i < 4; i++) chEnable[i] = chEn[i];

    xTaskCreatePinnedToCore(readADSTask, devName, (1024 * 5), this, 5, NULL, 0);
    return true;
}

int16_t ADS_Thread::readADC(uint8_t ch) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int16_t data = adcs[ch];
    xSemaphoreGive(mutex);
    return data;
}

float ADS_Thread::readVoltage(uint8_t ch) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    float data = volts[ch];
    xSemaphoreGive(mutex);
    return data;
}

void ADS_Thread::requestADSData() {
    for (uint8_t i = 0; i < 4; i++) {
        if (!chEnable[i]) continue;
        xSemaphoreTake(mutex, portMAX_DELAY);
        adcs[i] = dev.readADC(i);
        float voltage = dev.toVoltage(adcs[i]);
        volts[i] = (AVERAGE_ALPHA * voltage) + ((1 - AVERAGE_ALPHA) * volts[i]);
        xSemaphoreGive(mutex);
    }
}