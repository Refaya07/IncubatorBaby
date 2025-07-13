#include "AirTempSensor.h"

#define RETRY_GET_DATA 3

#define CALIB_POINT_32 32.31f
#define CALIB_POINT_36 35.25f

#define CALIB_POINT_32_ACTUAL 33.26f
#define CALIB_POINT_36_ACTUAL 36.48f

float AirTempSensor::_correctTemperature(float temperature) {
    return gradient * (temperature - CALIB_POINT_36) + CALIB_POINT_36_ACTUAL;
}

void AirTempSensor::init(uint8_t pin) {
    oneWire = new OneWire(pin);
    dev = new DallasTemperature(oneWire);
    dev->begin();
    dev->setWaitForConversion(false);  // Non-blocking mode

    gradient = (CALIB_POINT_36_ACTUAL - CALIB_POINT_32_ACTUAL) / (CALIB_POINT_36 - CALIB_POINT_32);
}

float AirTempSensor::read() {
    float data = DEVICE_DISCONNECTED_C;
    for (uint8_t i = 0; (i < RETRY_GET_DATA) && data == DEVICE_DISCONNECTED_C; i++) {
        dev->requestTemperatures();
        data = dev->getTempCByIndex(0);
    }

    isSensorError = (data == DEVICE_DISCONNECTED_C);
    return (isSensorError ? data : _correctTemperature(data));
}