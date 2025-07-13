#include "VoltageSensor.h"

float VoltageSensor::read() {
    isSensorError = false;  // TODO: Implement error handling
    return dev->readVoltage(channel);
}