#include "WaterLevelSensor.h"
#include "esp_log.h"

static const char* TAG = "WaterLevelSensor";

float WaterLevelSensor::read() {
    float voltage = dev->readVoltage(channel);
    // ESP_LOGI(TAG, "Water Level Voltage: %f", voltage);
    return voltage;
}