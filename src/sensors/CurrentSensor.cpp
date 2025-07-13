#include "CurrentSensor.h"

#define NUMBER_OF_SAMPLES 20
#define CT_RATIO 1000.0f

static const char* TAG = "CurrentSensor";

static void readCurrentTask(void* param) {
    CurrentSensor* sensor = (CurrentSensor*)param;
    while (true) {
        sensor->calcIrms();
    }
}

void CurrentSensor::init(float r, float lowThres, float highThres) {
    lowThreshold_A = lowThres;
    highThreshold_A = highThres;

    burdenResistor_ohm = r;

    queue = xQueueCreate(1, sizeof(bool));
    if (queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    xTaskCreatePinnedToCore(readCurrentTask, "Read Current Task", 2048 * 4, this, 1, &taskHandle, 0);
    if (taskHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create read current task");
        return;
    }
}

void CurrentSensor::enableReading(bool enable) {
    if (enabled == enable) return;

    enabled = enable;
    xQueueSend(queue, &enabled, portMAX_DELAY);
}

float CurrentSensor::read() {
    float data = currentReading_A;
    return data;
}

void CurrentSensor::calcIrms() {
    bool isEnabled = false;
    xQueueReceive(queue, &isEnabled, portMAX_DELAY);
    enabled = isEnabled;

    while (isEnabled) {
        double sumSq = 0;
        double sumV = 0;
        for (unsigned int n = 0; n < NUMBER_OF_SAMPLES; n++) {
            float voltage = dev->readVoltage(channel);
            sumV += voltage;
            if (xQueueReceive(queue, &isEnabled, pdMS_TO_TICKS(5))) return;
        }

        float offset = sumV / NUMBER_OF_SAMPLES;
        for (uint16_t i = 0; i < NUMBER_OF_SAMPLES; i++) {
            float voltage = dev->readVoltage(channel);
            float centered = voltage - offset;
            sumSq += (centered * centered);
            if (xQueueReceive(queue, &isEnabled, pdMS_TO_TICKS(5))) return;
        }

        float voltage_rms = sqrt(sumSq / NUMBER_OF_SAMPLES);
        float current_rms = (voltage_rms / burdenResistor_ohm) * CT_RATIO;

        currentReading_A = current_rms;

        // ESP_LOGW(TAG, "Current: %.2f", currentReading_A);s

        if (!enabled) return;

        if (currentReading_A < lowThreshold_A) {
            state = SENSE_UNDER_CURRENT;
            ESP_LOGW(TAG, "Under current detected: %.2f", currentReading_A);
        } else if (currentReading_A > highThreshold_A) {
            state = SENSE_OVERCURRENT;
            ESP_LOGW(TAG, "Over current detected: %.2f", currentReading_A);
        }

        if (state != SENSE_CURRENT_NORMAL) {
            enabled = false;
            return;
        }
    }
}