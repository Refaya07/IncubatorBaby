#include "SkinTempSensor.h"

static const float r_table[] = {7356, 5721, 4484, 3540, 3380, 3227, 3083, 2945,  2815,  2691,  2573, 2461, 2354,
                                2253, 2157, 2065, 1978, 1894, 1815, 1740, 1668,  1599,  1534,  1472, 1412, 1355,
                                1301, 1249, 1200, 1153, 1108, 1065, 1023, 984.1, 811.5, 672.7, 560.5};

static const float t_table[] = {0,  5,  10, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                                31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 50, 55, 60};

static const char* TAG = "SkinTempSensor";

float SkinTempSensor::read() {
    int16_t adc = dev->readADC(channel);

    // Get skin sensor resistance value
    float r = ((vref * rknown) / dev->readVoltage(channel)) - rknown;

    // ESP_LOGI(TAG, "Voltage: %f  ", dev->toVoltage(adc));
    // ESP_LOGI(TAG, "ADC: %d", adc);
    // ESP_LOGI(TAG, "Resistance: %f", r);

    float temperature = temperatureResistanceTable(r);

    isSensorError = (temperature == -999);
    return (float)temperature;
}

float SkinTempSensor::temperatureResistanceTable(float r) {
    int tableSize = sizeof(r_table) / sizeof(r_table[0]);

    // Resistance is outside the table range, return error (-999)
    if (r > r_table[0] || r < r_table[tableSize - 1]) return -999;

    for (int i = 0; i < tableSize - 1; i++) {
        if (r > r_table[i]) {
            // ESP_LOGI(TAG, "%.0f-%.0f -> %.0f-%.0f", r_table[i - 1], r_table[i], t_table[i - 1], t_table[i]);
            // Linear interpolation
            float tempDiff = t_table[i - 1] - t_table[i];
            float resDiff = r_table[i - 1] - r_table[i];
            float slope = tempDiff / resDiff;
            return slope * (r - r_table[i]) + t_table[i];
        }
    }

    return -999;
}