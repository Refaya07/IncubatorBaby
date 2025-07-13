#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// I2C Communication Pins
// Used for communication with I2C devices like temperature sensors and ADCs
#define SDA_PIN 42
#define SCL_PIN 41

// Watchdog Timer Pin
// Used to reset the system if it becomes unresponsive
#define WDT_WDI_PIN 14

// HMI (Nextion Display) Serial Communication Pins
// Used for communication with the Nextion HMI display
#define HMI_SERIAL_RX 17
#define HMI_SERIAL_TX 18

// ADS1X15 ADC Device Addresses
// Two ADC devices for reading analog sensors
#define ADS1X15_DEV1_ADDRESS 0x48  // First ADC device
#define ADS1X15_DEV2_ADDRESS 0x49  // Second ADC device

// Output Control Pins
// PWM outputs for controlling heaters and buzzer
#define HEATER_CABIN_OUT_PIN 4  // Cabin heater control
#define HEATER_HUMID_OUT_PIN 5  // Humidity heater control
#define BUZZER_OUT_PIN 11       // Alarm buzzer

// Fan Control
// PWM channel and pin for fan speed control
#define FAN_OUT_PIN 12
#define FAN_PWM_CHANNEL 0

// Sensor Pins
// Digital and analog inputs for various sensors
#define HUMIDITY_SENSOR_PIN 9       // DHT21 humidity sensor
#define HUMIDITY_SENSOR_TYPE DHT21  // Humidity sensor type
#define AIR_TEMP_SENSOR_PIN 8       // Air temperature sensor
#define RPM_SENSOR_PIN 13           // Fan RPM sensor

// Current Sensor Burden Resistors
// Used for current measurement calculations
#define R_BURDEN_CT_HEATER_CABIN 1000.0  // Burden resistor for cabin heater current sensor
#define R_BURDEN_CT_HEATER_HUMID 1000.0  // Burden resistor for humidity heater current sensor

// ADS1X15 Device #1 Channel Assignments
// First ADC device channel mappings
#define SKIN_TEMP_SENSOR_CHANNEL 0  // Skin temperature sensor
#define I_HEATER_CABIN_CHANNEL 2    // Cabin heater current sensor

// ADS1X15 Device #2 Channel Assignments
// Second ADC device channel mappings
#define WATER_LEVEL_SENSOR_CHANNEL 0  // Water level sensor
#define BATTERY_VOLTAGE_CHANNEL 1     // Battery voltage measurement
#define I_HEATER_HUMID_CHANNEL 2      // Humidity heater current sensor

extern SemaphoreHandle_t critMutex;