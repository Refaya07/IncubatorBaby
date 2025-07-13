/**
 * @file ASTRA_BabyIncubator.h
 * @brief Header file for the ASTRA Baby Incubator main control system
 * @details This file contains the class definition and related structures for the core
 * functionality of the baby incubator, including temperature control, humidity control,
 * and safety monitoring.
 */

#pragma once

#include <RTClib.h>
#include <functional>
#include "HardwareConfig.h"
#include "PID_v1.h"
#include "Preferences.h"
#include "actuators/DigitalDriver.h"
#include "actuators/FanControl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lib/ADS_Thread.h"
#include "sensors/AirTempSensor.h"
#include "sensors/CurrentSensor.h"
#include "sensors/HumiditySensor.h"
#include "sensors/RPMSensor.h"
#include "sensors/SkinTempSensor.h"
#include "sensors/VoltageSensor.h"
#include "sensors/WaterLevelSensor.h"

/**
 * @brief Structure containing incubator configuration parameters
 */
typedef struct {
    float airTemperature;   ///< Target air temperature
    float skinTemperature;  ///< Target skin temperature
    float humidity;         ///< Target humidity level
    float airTempOffset;    ///< Air temperature sensor offset
    float skinTempOffset;   ///< Skin temperature sensor offset
    float humidityOffset;   ///< Humidity sensor offset
    int logo;               ///< Selected logo identifier
} ABI_config_t;

/**
 * @brief Structure containing peripheral error states
 */
struct ABI_errorflag_t {
    bool isSensorAirFailed = false;       ///< Air temperature sensor failure
    bool isSensorSkinFailed = false;      ///< Skin temperature sensor failure
    bool isSensorHumidityFailed = false;  ///< Humidity sensor failure
    bool isWaterLow = false;              ///< Low water level warning
    bool isHeaterCabinFailed = false;     ///< Cabin heater failure
    bool isHeaterHumidFailed = false;     ///< Humidity heater failure
    bool isFanFailed = false;             ///< Fan failure
    bool isRTCFailed = false;             ///< Real-time clock failure
    bool isStorageFailed = false;         ///< Storage system failure
    bool isBatteryLow = false;            ///< Low battery warning
    bool isTempTooHigh = false;           ///< Temperature too high warning
    bool isTempTooLow = false;            ///< Temperature too low warning
    bool isSystemFailed = false;
    bool isIncubatorOpen = false;

    bool isErrorExist() const {
        return isSensorAirFailed || isSensorSkinFailed || isSensorHumidityFailed || isWaterLow || isHeaterCabinFailed ||
               isHeaterHumidFailed || isFanFailed || isRTCFailed || isStorageFailed || isBatteryLow || isTempTooHigh ||
               isTempTooLow || isSystemFailed || isIncubatorOpen;
    }

    bool operator==(const ABI_errorflag_t& other) const {
        return isSensorAirFailed == other.isSensorAirFailed && isSensorSkinFailed == other.isSensorSkinFailed &&
               isSensorHumidityFailed == other.isSensorHumidityFailed && isWaterLow == other.isWaterLow &&
               isHeaterCabinFailed == other.isHeaterCabinFailed && isHeaterHumidFailed == other.isHeaterHumidFailed &&
               isFanFailed == other.isFanFailed && isRTCFailed == other.isRTCFailed &&
               isStorageFailed == other.isStorageFailed && isBatteryLow == other.isBatteryLow &&
               isTempTooHigh == other.isTempTooHigh && isTempTooLow == other.isTempTooLow;
    }

    ABI_errorflag_t& operator=(const ABI_errorflag_t& other) {
        if (this != &other) {
            isSensorAirFailed = other.isSensorAirFailed;
            isSensorSkinFailed = other.isSensorSkinFailed;
            isSensorHumidityFailed = other.isSensorHumidityFailed;
            isWaterLow = other.isWaterLow;
            isHeaterCabinFailed = other.isHeaterCabinFailed;
            isHeaterHumidFailed = other.isHeaterHumidFailed;
            isFanFailed = other.isFanFailed;
            isRTCFailed = other.isRTCFailed;
            isStorageFailed = other.isStorageFailed;
            isBatteryLow = other.isBatteryLow;
            isTempTooHigh = other.isTempTooHigh;
            isTempTooLow = other.isTempTooLow;
        }
        return *this;
    }
};

/**
 * @brief Structure containing control system parameters
 */

class ASTRA_BabyIncubator;
typedef struct {
    char name[20];                             ///< Control system name
    bool state = false;                        ///< Current control state
    QueueHandle_t queue = NULL;                ///< Control command queue
    PID* pid = NULL;                           ///< PID controller instance
    float bangThreshold;                       ///< Bang-bang control threshold
    double input = 0.0;                        ///< Current input value
    double output = 0.0;                       ///< Current output value
    double setpoint = 0.0;                     ///< Target setpoint
    double kp = 0.0;                           ///< Proportional gain
    double ki = 0.0;                           ///< Integral gain
    double kd = 0.0;                           ///< Derivative gain
    std::function<float()> readSensor;         ///< Sensor reading function
    std::function<bool(int, int)> drivePWM;    ///< PWM control function
    std::function<void(int)> driveFan = NULL;  ///< Fan control function
    unsigned long bangBangStartTime;           ///< Start time bang bang
    bool isBangBangMode;                       ///< Bang bang Mode
    bool alarmInkubatorTerbuka;                ///< Alarm Incubator Open
    ASTRA_BabyIncubator* parent = nullptr;
} ControlMode_t;

/**
 * @class ASTRA_BabyIncubator
 * @brief Main class for the baby incubator control system
 * @details Manages all aspects of the incubator including temperature control,
 * humidity control, safety monitoring, and peripheral management.
 */
class ASTRA_BabyIncubator {
   private:
    /* Peripherals */
    Preferences storage;  ///< Non-volatile storage system

    ADS_Thread analogDev_1;  ///< First analog device interface
    ADS_Thread analogDev_2;  ///< Second analog device interface

    AirTempSensor airTempSensor;        ///< Air temperature sensor
    HumiditySensor humiditySensor;      ///< Humidity sensor
    SkinTempSensor skinTempSensor;      ///< Skin temperature sensor
    CurrentSensor i_heaterCabin;        ///< Cabin heater current sensor
    CurrentSensor i_heaterHumid;        ///< Humidity heater current sensor
    WaterLevelSensor waterLevelSensor;  ///< Water level sensor
    VoltageSensor batteryVoltageLevel;  ///< Battery voltage sensor
    RPMSensor rpmSensor;                ///< Fan RPM sensor

    DigitalDriver heaterCabin;  ///< Cabin heater control
    DigitalDriver heaterHumid;  ///< Humidity heater control
    FanControl fan;             ///< Fan control

    /* Internal Data */
    ABI_config_t config;  ///< Current configuration
    // ABI_errorflag_t errorFlag;   ///< Current error flags
    bool criticalError = false;  ///< Critical error flag

    /* Control Objects */
    ControlMode_t airMode;            ///< Air temperature control system
    ControlMode_t babyMode;           ///< Skin temperature control system
    ControlMode_t humidityMode;       ///< Humidity control system
    SemaphoreHandle_t _mutex = NULL;  ///< Control system mutex

    bool _checkConfigKeys(const char* key);

    /**
     * @brief Initialize the storage system
     * @return true if initialization was successful
     */
    bool _initStorage();

    /**
     * @brief Initialize all peripherals. Sets up analog devices, sensors, and actuators with their respective
     * configurations.
     * @return true if all components initialized successfully, false otherwise
     */
    bool _initPeripherals();

    /**
     * @brief Set the critical error flag
     */
    void _setCriticalErrorFlag() { criticalError = true; }

    /**
     * @brief Start a control system
     * @param obj Pointer to the control object
     * @param bang Bang-bang control threshold
     * @param setpoint Target setpoint
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void _startControl(ControlMode_t* obj, float bang, float setpoint, float kp, float ki, float kd);
    bool _stopControl(ControlMode_t* obj);

    bool _checkAirTemperatureLimits();
    bool _checkSkinTemperatureLimits();
    bool _checkHeaterCabin();
    bool _checkHeaterHumidity();
    bool _checkFan();
    bool _checkBattery();
    bool _checkWater();

   public:
    /**
     * @brief Constructor for the incubator control system
     */
    ASTRA_BabyIncubator()
        : analogDev_1("ADS 1", ADS1X15_DEV1_ADDRESS),
          analogDev_2("ADS 2", ADS1X15_DEV2_ADDRESS),
          skinTempSensor(&analogDev_1, SKIN_TEMP_SENSOR_CHANNEL),
          i_heaterCabin(&analogDev_1, I_HEATER_CABIN_CHANNEL),
          i_heaterHumid(&analogDev_2, I_HEATER_HUMID_CHANNEL),
          waterLevelSensor(&analogDev_2, WATER_LEVEL_SENSOR_CHANNEL),
          batteryVoltageLevel(&analogDev_2, BATTERY_VOLTAGE_CHANNEL),
          heaterCabin(HEATER_CABIN_OUT_PIN),
          heaterHumid(HEATER_HUMID_OUT_PIN),
          fan(FAN_OUT_PIN) {}

    /**
     * @brief Initialize the incubator system. Sets up storage, peripherals, control systems, and safety monitoring.
     * Creates necessary FreeRTOS tasks and synchronization primitives.
     * @return true if all initialization steps successful, false otherwise
     */
    bool begin();

    /* Get internal data methods */
    /**
     * @brief Get the current error state
     * @return Current peripheral error state
     */
    ABI_errorflag_t getError() const { return errorFlag; }

    bool _checkOnlyAirTempHigh();
    bool _checkOnlySkinTempHigh();

    ABI_errorflag_t errorFlag;

    /**
     * @brief Get the current operating mode
     * @return Current incubator mode
     */
    bool isCriticalErrorDetected() const { return criticalError; }

    /**
     * @brief Get the current configuration
     * @return Current incubator configuration
     */
    ABI_config_t getConfig() const { return config; }

    /**
     * @brief Get air temperature control state
     * @return true if air temperature control is enabled
     */
    bool getAirTempControlState() const { return airMode.state; }

    /**
     * @brief Get skin temperature control state
     * @return true if skin temperature control is enabled
     */
    bool getSkinTempControlState() const { return babyMode.state; }

    /**
     * @brief Get humidity control state
     * @return true if humidity control is enabled
     */
    bool getHumidityControlState() const { return humidityMode.state; }

    bool checkInkubatorClosed(ControlMode_t* control, float errorAbs);

    /**
     * @brief Get the current logo setting
     * @return Current logo identifier (1-4)
     */
    int getLogo() const { return config.logo; }

    /**
     * @brief Get the current date and time
     * @return Current date and time
     */
    DateTime getDateTime() const;

    /* Get sensor data methods */
    /**
     * @brief Get current air temperature
     * @return Air temperature in degrees Celsius
     */
    float getAirTemperature();

    /**
     * @brief Get current skin temperature
     * @return Skin temperature in degrees Celsius
     */
    float getSkinTemperature();

    /**
     * @brief Get current humidity level
     * @return Humidity percentage
     */
    float getHumidity();

    /**
     * @brief Get current water level
     * @return The current water level in arbitrary units
     */
    float getWaterLevel();

    /**
     * @brief Get cabin heater current
     * @return Current in amperes
     */
    float getCabinHeaterCurrent();

    /**
     * @brief Get humidity heater current
     * @return Current in amperes
     */
    float getHumidityHeaterCurrent();

    /**
     * @brief Get fan RPM
     * @return Fan speed in RPM
     */
    float getFanRPM();

    /**
     * @brief Get fan speed
     * @return Fan speed percentage (0-100)
     */
    int getFanSpeed();

    /**
     * @brief Get heater power
     * @return Heater power percentage (0-100)
     */
    int getHeaterPower() const;

    /* Set configuration methods */
    /**
     * @brief Set the logo
     * @param logo Logo identifier (1-4)
     */
    void setLogo(int logo);

    /**
     * @brief Set the date and time
     * @param dateTime New date and time
     * @return true if successful
     */
    bool setDateTime(DateTime dateTime);

    /**
     * @brief Set target air temperature and start air mode
     * @param setpoint Target temperature in degrees Celsius
     * @return true if successful, otherwise false
     */
    bool startAirMode(float setpoint);

    /**
     * @brief Set target skin temperature and start baby mode
     * @param setpoint Target temperature in degrees Celsius
     * @return true if successful, otherwise false
     */
    bool startBabyMode(float setpoint);

    /**
     * @brief Set target humidity and start humidity mode
     * @param setpoint Target humidity percentage
     * @return true if successful, otherwise false
     */
    bool startHumidityMode(float setpoint);

    /**
     * @brief Set air temperature sensor offset
     * @param offset Temperature offset in degrees Celsius
     * @return true if successful
     */
    bool setAirTempOffset(float offset);

    /**
     * @brief Set skin temperature sensor offset
     * @param offset Temperature offset in degrees Celsius
     * @return true if successful
     */
    bool setSkinTempOffset(float offset);

    /**
     * @brief Set humidity sensor offset
     * @param offset Humidity offset percentage
     * @return true if successful
     */
    bool setHumidityOffset(float offset);

    /**
     * @brief Check system safety conditions
     * @note This function is called periodically to ensure system safety.
     * @return true if all safety conditions are met
     */
    bool checkSafety();

    /**
     * @brief Stop air temperature control
     */
    void stopAirMode();

    /**
     * @brief Stop skin temperature control
     */
    void stopBabyMode();

    /**
     * @brief Stop humidity control
     */
    void stopHumidityMode();
};

extern ASTRA_BabyIncubator abi;  ///< Global incubator instance