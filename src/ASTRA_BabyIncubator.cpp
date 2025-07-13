#include "ASTRA_BabyIncubator.h"
#include "ControlSystemConstants.h"
#include "TimeKeeping.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ASTRA_BabyIncubator";

/**
 * @brief FreeRTOS task for PID control system
 * @details Implements a hybrid control system that combines bang-bang control for large errors
 * and PID control for fine-tuning. The control output is quantized to 10% steps to reduce
 * switching frequency and improve system stability.
 *
 * Control Strategy:
 * 1. For large errors (> bangThreshold): Uses bang-bang control for faster response
 * 2. For small errors: Uses PID control for precise regulation
 * 3. Output is quantized to 10% steps to reduce relay switching frequency
 *
 * @param param Pointer to ControlMode_t containing control parameters and state
 */

bool checkInkubatorClosed(ASTRA_BabyIncubator* incubator, ControlMode_t* control, float errorAbs) {
    const float bangThreshold = control->bangThreshold;
    const unsigned long timeout = 35 * 60 * 1000;  // 25 menit dalam ms

    // Deteksi apakah masuk bang-bang
    if (errorAbs > bangThreshold) {
        if (!control->isBangBangMode) {
            control->bangBangStartTime = millis();
            control->isBangBangMode = true;
        }
    } else {
        control->isBangBangMode = false;
        control->bangBangStartTime = 0;
    }

    if (control->isBangBangMode && (millis() - control->bangBangStartTime > timeout)) {
        control->alarmInkubatorTerbuka = true;
        incubator->errorFlag.isSystemFailed = true;
        ESP_LOGW(control->name, "ALARM SEMENTARA: Inkubator Terbuka (kontrol bang-bang > 15 menit)");
        ESP_LOGW(TAG, "CLOSE INCUBATOR");
    }

    if (!control->isBangBangMode && errorAbs < bangThreshold && control->pid != NULL) {
        control->alarmInkubatorTerbuka = false;
        incubator->errorFlag.isSystemFailed = false;
    }

    return !control->alarmInkubatorTerbuka;
}

static void pidTask(void* param) {
    ControlMode_t* control = (ControlMode_t*)param;

    while (true) {
        // Skip control if system is not in normal state (e.g., during startup or error conditions)
        while (!control->state) xQueueReceive(control->queue, control, portMAX_DELAY);

        // Read current sensor value
        control->input = control->readSensor();

        // Calculate error and its absolute value
        float error = (control->input - control->setpoint);
        float errorAbs = std::fabs(error);

        // check incubator closed
        checkInkubatorClosed(control->parent, control, errorAbs);

        // Use PID control only when error is within threshold to prevent integral wind-up
        // This helps maintain stability and prevents overshooting
        if (errorAbs < (control->bangThreshold + 0.4)) {
            if (control->pid != NULL) control->pid->Compute();
        }

        // Use bang-bang control for large errors to achieve faster response
        // This helps the system reach the setpoint more
        if (errorAbs > control->bangThreshold + 2.0) {
            control->output = (error < 0) ? (MAX_OUTPUT_DUTY_CYCLE) : MIN_OUTPUT_DUTY_CYCLE;
        } else if (errorAbs > control->bangThreshold) {
            control->output = (error < 0) ? (MAX_OUTPUT_DUTY_CYCLE - 30) : MIN_OUTPUT_DUTY_CYCLE;
        }

        // Stop control if error is larger than 0.3 to prevent overshooting
        if (control->input > control->setpoint + 0.3) {
            control->output = 0;
        }

        // Check for new control commands before processing output
        if (xQueueReceive(control->queue, control, pdMS_TO_TICKS(10)) == pdTRUE) continue;

        // Quantize output to 10% steps to reduce switching frequency
        // This helps extend the life of mechanical components like relays
        uint8_t dutycycle = uint8_t(std::floor(control->output / 10) * 10);
        control->drivePWM(dutycycle, int(CONTROL_UPDATE_TICK_mS / 2));

        // Control fan speed based on duty cycle
        uint8_t fanDutyCycle = FAN_DUTY_CYCLE_DEFAULT;
        if (control->driveFan != NULL) {
            float duty = ((6 * control->output) / 10) + 30;
            fanDutyCycle = static_cast<uint8_t>(duty);
            control->driveFan(fanDutyCycle);
        }

        ESP_LOGI(control->name, "{heater: %d, fan: %u, input: %.2f, error: %.2f}", dutycycle, fanDutyCycle,
                 control->input, error);

        // Wait for next control cycle
        xQueueReceive(control->queue, control, pdMS_TO_TICKS(SAFETY_CHECK_TICK_mS));
    }
}

void monitorAirTemperatureTask(void* parameter) {
    ASTRA_BabyIncubator* incubator = static_cast<ASTRA_BabyIncubator*>(parameter);

    while (true) {
        incubator->_checkOnlyAirTempHigh();  // Ini kunci utamanya
        vTaskDelay(pdMS_TO_TICKS(1000));     // Cek tiap detik
    }
}

void monitorSkinTemperatureTask(void* parameter) {
    ASTRA_BabyIncubator* incubator = static_cast<ASTRA_BabyIncubator*>(parameter);

    while (true) {
        incubator->_checkOnlySkinTempHigh();  // Ini kunci utamanya
        vTaskDelay(pdMS_TO_TICKS(1000));      // Cek tiap detik
    }
}

/**
 * @brief FreeRTOS task for safety monitoring
 * @details Continuously monitors system safety conditions and stops operation if any
 * safety check fails. The task runs at a fixed interval defined by SAFETY_CHECK_TICK_mS.
 * @param param Pointer to ASTRA_BabyIncubator instance
 */
static void checkSafetyTask(void* param) {
    ASTRA_BabyIncubator* self = static_cast<ASTRA_BabyIncubator*>(param);
    while (true) {
        // Stop operation if any safety check fails
        if (!self->checkSafety()) {
            ESP_LOGE(TAG, "Failure detected! Stopping operation...");
            self->stopAirMode();
            self->stopBabyMode();
            self->stopHumidityMode();
            vTaskDelay(portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(SAFETY_CHECK_TICK_mS));
    }
}

bool ASTRA_BabyIncubator::_checkConfigKeys(const char* key) {
    bool keyExist = storage.isKey(key);
    if (keyExist) return true;
    ESP_LOGI(TAG, "Storage not initialized, setting default values");
    return false;
}

/**
 * @brief Initialize non-volatile storage system
 * @details Sets up the storage system and loads or initializes default configuration values.
 * Uses single-character keys for efficient storage usage.
 *
 * Storage Keys:
 * - 'a': Air temperature setpoint
 * - 's': Skin temperature setpoint
 * - 'h': Humidity setpoint
 * - 'A': Air temperature offset
 * - 'S': Skin temperature offset
 * - 'H': Humidity offset
 * - 'h_en': Humidity control enabled flag
 * - 'logo': Selected logo ID
 *
 * @return true if initialization successful, false otherwise
 */
bool ASTRA_BabyIncubator::_initStorage() {
    ESP_LOGI(TAG, "Initializing NVS storage...");

    // Initialize storage with namespace "astra_bi"
    if (!storage.begin("astra_bi", false)) {
        ESP_LOGE(TAG, "Failed to initialize storage, restarting...");
        return false;
    }

    ESP_LOGI(TAG, "Reading configuration from NVS...");

    // Initialize default values if storage is empty
    if (!_checkConfigKeys("a")) storage.putFloat("a", DEFAULT_AIRTEMP_SETPOINT);
    if (!_checkConfigKeys("s")) storage.putFloat("s", DEFAULT_SKINTEMP_SETPOINT);
    if (!_checkConfigKeys("h")) storage.putFloat("h", DEFAULT_HUMIDITY_SETPOINT);
    if (!_checkConfigKeys("A")) storage.putFloat("A", 0.0);
    if (!_checkConfigKeys("S")) storage.putFloat("S", 0.0);
    if (!_checkConfigKeys("H")) storage.putFloat("H", 0.0);
    if (!_checkConfigKeys("h_en")) storage.putBool("h_en", false);
    if (!_checkConfigKeys("logo")) storage.putInt("logo", 0);

    // Load configuration from storage
    config.airTemperature = storage.getFloat("a", DEFAULT_AIRTEMP_SETPOINT);
    config.skinTemperature = storage.getFloat("s", DEFAULT_SKINTEMP_SETPOINT);
    config.humidity = storage.getFloat("h", DEFAULT_HUMIDITY_SETPOINT);
    config.airTempOffset = storage.getFloat("A", 0.0);
    config.skinTempOffset = storage.getFloat("S", 0.0);
    config.humidityOffset = storage.getFloat("H", 0.0);
    config.logo = storage.getInt("logo", 0);

    // Log loaded configuration for debugging
    ESP_LOGI(TAG, "Air temperature setpoint: %.2f", config.airTemperature);
    ESP_LOGI(TAG, "Skin temperature setpoint: %.2f", config.skinTemperature);
    ESP_LOGI(TAG, "Humidity setpoint: %.2f", config.humidity);
    ESP_LOGI(TAG, "Air temperature offset: %.2f", config.airTempOffset);
    ESP_LOGI(TAG, "Skin temperature offset: %.2f", config.skinTempOffset);
    ESP_LOGI(TAG, "Humidity offset: %.2f", config.humidityOffset);
    ESP_LOGI(TAG, "Logo(asset ID): %d", config.logo);
    ESP_LOGI(TAG, "Reading configuration from NVS done.");

    return true;
}

bool ASTRA_BabyIncubator::_initPeripherals() {
    ESP_LOGI(TAG, "Initializing peripherals...");

    /** Sensors Initialization **/
    // Initialize analog-to-digital converters for sensor readings
    if (!analogDev_1.begin(1, 0, 1, 0) || !analogDev_2.begin(1, 1, 1, 0)) {
        ESP_LOGE(TAG, "ADSS1X15 device initialization failed");
        return false;
    }

    // Initialize various sensors with their respective pins and configurations
    humiditySensor.init(HUMIDITY_SENSOR_PIN, HUMIDITY_SENSOR_TYPE);
    airTempSensor.init(AIR_TEMP_SENSOR_PIN);
    i_heaterCabin.init(R_BURDEN_CT_HEATER_CABIN, UNDERCURRENT_THRESHOLD, OVERCURRENT_CABIN_HEATER_THRESHOLD);
    i_heaterHumid.init(R_BURDEN_CT_HEATER_HUMID, UNDERCURRENT_THRESHOLD, OVERCURRENT_HUMID_HEATER_THRESHOLD);
    rpmSensor.init(RPM_SENSOR_PIN);

    /** Setup actuators **/
    // Initialize heaters with current monitoring callbacks
    // Bind enableReading to callback - Creates an automatic safety monitoring system
    // When the heater is turned on/off, this callback automatically enables/disables current monitoring
    // This ensures we can't forget to monitor current when the heater is running
    // The function pointer approach means the heater doesn't need to know how current monitoring works
    heaterCabin.init(LOW, HIGH, std::bind(&CurrentSensor::enableReading, &i_heaterCabin, std::placeholders::_1));
    heaterHumid.init(LOW, HIGH, std::bind(&CurrentSensor::enableReading, &i_heaterHumid, std::placeholders::_1));

    // Initialize fan control with active low configuration
    fan.init(FAN_PWM_CHANNEL, LOW);

    return true;
}

bool ASTRA_BabyIncubator::begin() {
    // Create mutex for timing sensitive tasks such as DHT (humidity), HMI serial, and RPM.
    critMutex = xSemaphoreCreateMutex();
    if (critMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create critical mutex, exiting...");
        return false;
    }

    // Create mutex for protecting sensor data access
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex for sensor data, exiting...");
        return false;
    }

    // Initialize storage and peripherals
    if (!_initStorage()) return false;
    if (!_initPeripherals()) return false;

    /** Setup PID control program **/
    // Initialize air temperature control
    sprintf(airMode.name, "Air Mode");
    airMode.queue = xQueueCreate(1, sizeof(ControlMode_t));
    if (airMode.queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control system queue for air temperature, exiting...");
        return false;
    }
    // Bind getAirTemperature to readSensor function pointer - PID control system uses function pointers to make the
    // control loop generic This allows the same PID control code to work with different sensors by just changing the
    // function pointer The 'this' parameter ensures the member function is called on the correct object instance
    airMode.readSensor = std::bind(&ASTRA_BabyIncubator::getAirTemperature, this);
    // Bind setPWM to drivePWM function pointer - PID control system needs a way to control the output device
    // Function pointers allow the control system to be device-agnostic - it doesn't need to know if it's controlling
    // a heater, fan, or any other device, it just calls the provided function
    airMode.drivePWM = std::bind(&DigitalDriver::setPWM, &heaterCabin, std::placeholders::_1, std::placeholders::_2);
    airMode.driveFan = std::bind(&FanControl::setSpeed, &fan, std::placeholders::_1);
    airMode.parent = this;
    xTaskCreatePinnedToCore(pidTask, airMode.name, CTRL_PID_TASK_SIZE, (void*)&airMode, CTRL_PID_TASK_PRIORITY, NULL,
                            0);
    xTaskCreatePinnedToCore(monitorSkinTemperatureTask, "MonitorSkinTemp", 2048, this, 2, NULL,
                            1  // Core 1
    );

    // Initialize skin temperature control
    sprintf(babyMode.name, "Baby Mode");
    babyMode.queue = xQueueCreate(1, sizeof(ControlMode_t));
    if (babyMode.queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control system queue for skin temperature, exiting...");
        return false;
    }
    // Bind getSkinTemperature to readSensor function pointer - Same PID control system design pattern as air
    // temperature The control system is reusable because it works through function pointers rather than direct sensor
    // access This makes the code more modular and easier to test (can swap in mock sensor functions)
    babyMode.readSensor = std::bind(&ASTRA_BabyIncubator::getSkinTemperature, this);
    // Bind setPWM to drivePWM function pointer - Reuses the same PID control output mechanism
    // The placeholders (_1, _2) allow the PID system to pass its calculated values to the heater control
    babyMode.drivePWM = std::bind(&DigitalDriver::setPWM, &heaterCabin, std::placeholders::_1, std::placeholders::_2);
    babyMode.driveFan = std::bind(&FanControl::setSpeed, &fan, std::placeholders::_1);
    babyMode.parent = this;
    xTaskCreatePinnedToCore(pidTask, babyMode.name, CTRL_PID_TASK_SIZE, (void*)&babyMode, CTRL_PID_TASK_PRIORITY, NULL,
                            0);
    xTaskCreatePinnedToCore(monitorAirTemperatureTask, "MonitorAirTemp", 2048, this, 2, NULL,
                            1  // Core 1
    );

    // Initialize humidity control
    sprintf(humidityMode.name, "Humidity Mode");
    humidityMode.queue = xQueueCreate(1, sizeof(ControlMode_t));
    if (humidityMode.queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control system queue for humidity, exiting...");
        return false;
    }
    // Bind getHumidity to readSensor function pointer - Completes the pattern of using function pointers for sensor
    // access This design allows each control system to be independent while sharing the same control logic The PID
    // system doesn't need to know which sensor it's reading from, it just calls the provided function
    humidityMode.readSensor = std::bind(&ASTRA_BabyIncubator::getHumidity, this);
    // Bind setPWM to drivePWM function pointer - Uses the same pattern but with the humidity heater
    // The function pointer approach means we can easily change how we control the heater without modifying the PID code
    humidityMode.drivePWM =
        std::bind(&DigitalDriver::setPWM, &heaterHumid, std::placeholders::_1, std::placeholders::_2);
    xTaskCreatePinnedToCore(pidTask, humidityMode.name, CTRL_PID_TASK_SIZE, (void*)&humidityMode,
                            CTRL_PID_TASK_PRIORITY, NULL, 0);

    // Set initial operating conditions
    fan.setSpeed(FAN_DUTY_CYCLE_DEFAULT);
    startAirMode(config.airTemperature);
    humidityMode.parent = this;

    xTaskCreatePinnedToCore(checkSafetyTask, "Check safety", CHECK_SAFETY_TASK_STACK_SIZE, this,
                            CHECK_SAFETY_TASK_PRIORITY, NULL, 0);

    return true;
}

void ASTRA_BabyIncubator::setLogo(int logo) {
    config.logo = logo;
    storage.putInt("logo", logo);
}

float ASTRA_BabyIncubator::getAirTemperature() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = airTempSensor.read() + config.airTempOffset;
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getSkinTemperature() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = skinTempSensor.read() + config.skinTempOffset;
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getHumidity() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = humiditySensor.read() + config.humidityOffset;
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getWaterLevel() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = waterLevelSensor.read();
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getCabinHeaterCurrent() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = i_heaterCabin.read();
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getHumidityHeaterCurrent() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = i_heaterHumid.read();
    xSemaphoreGive(_mutex);
    return data;
}

float ASTRA_BabyIncubator::getFanRPM() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = rpmSensor.read();
    xSemaphoreGive(_mutex);
    return data;
}

int ASTRA_BabyIncubator::getFanSpeed() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float data = fan.getSpeed();
    xSemaphoreGive(_mutex);
    return data;
}

int ASTRA_BabyIncubator::getHeaterPower() const { return heaterCabin.getDutyCycle(); }

DateTime ASTRA_BabyIncubator::getDateTime() const { return timeKeeping.getTime(); }

bool ASTRA_BabyIncubator::setDateTime(DateTime dateTime) {
    if (dateTime.year() < 2024) return false;  // Invalid date
    timeKeeping.setTime(dateTime);
    return true;
}

void ASTRA_BabyIncubator::_startControl(ControlMode_t* obj, float bang, float setpoint, float kp, float ki, float kd) {
    obj->state = true;
    obj->bangThreshold = bang;
    obj->setpoint = setpoint;
    obj->kp = kp;
    obj->ki = ki;
    obj->kd = kd;

    if (obj->pid != NULL) {
        delete obj->pid;
        obj->pid = NULL;
    }
    obj->pid = new PID(&obj->input, &obj->output, &obj->setpoint, obj->kp, obj->ki, obj->kd, DIRECT);

    obj->pid->SetMode(AUTOMATIC);
    obj->pid->SetOutputLimits(MIN_OUTPUT_DUTY_CYCLE, MAX_OUTPUT_DUTY_CYCLE);
    xQueueSend(obj->queue, obj, portMAX_DELAY);
}

bool ASTRA_BabyIncubator::startAirMode(float setpoint) {
    if (isCriticalErrorDetected()) {
        ESP_LOGE(TAG, "Cannot start air mode. Critical error detected.");
        return false;
    } else if (setpoint < UNDER_TEMPERATURE_CABIN_THRESHOLD || setpoint > OVER_TEMPERATURE_CABIN_THRESHOLD) {
        ESP_LOGW(TAG, "Cannot start air mode. Setpoint out of range: %.2f", setpoint);
        return false;
    }

    config.airTemperature = std::floor(setpoint / 0.5) * 0.5;
    if (setpoint <= MAX_NORMAL_TEMPERATURE_CABIN_THRESHOLD) {
        storage.putFloat("a", setpoint);
        ESP_LOGI(TAG, "Starting air mode. Setpoint: %.2f", setpoint);
    } else {
        ESP_LOGI(TAG, "Starting air mode. Setpoint(override): %.2f", setpoint);
    }

    stopBabyMode();
    _startControl(&airMode, AIR_TEMP_CTRL_BANG_WINDOW, setpoint, AIR_TEMP_PID_KP, AIR_TEMP_PID_KI, AIR_TEMP_PID_KD);
    return true;
}

bool ASTRA_BabyIncubator::startBabyMode(float setpoint) {
    if (isCriticalErrorDetected()) {
        ESP_LOGE(TAG, "Cannot start baby mode. Critical error detected.");
        return false;
    } else if (setpoint < UNDER_TEMPERATURE_SKIN_THRESHOLD || setpoint > OVER_TEMPERATURE_SKIN_THRESHOLD) {
        ESP_LOGW(TAG, "Cannot start baby mode. Setpoint out of range: %.2f", setpoint);
        return false;
    }

    config.skinTemperature = std::floor(setpoint / 0.5) * 0.5;
    if (setpoint <= MAX_NORMAL_SKIN_THRESHOLD) {
        storage.putFloat("s", setpoint);
        ESP_LOGI(TAG, "Starting baby mode. Setpoint: %.2f", setpoint);
    } else {
        ESP_LOGI(TAG, "Starting baby mode. Setpoint(override): %.2f", setpoint);
    }

    stopAirMode();
    _startControl(&babyMode, SKIN_TEMP_CTRL_BANG_WINDOW, setpoint, SKIN_TEMP_PID_KP, SKIN_TEMP_PID_KI,
                  SKIN_TEMP_PID_KD);
    return true;
}

bool ASTRA_BabyIncubator::startHumidityMode(float setpoint) {
    if (isCriticalErrorDetected()) {
        ESP_LOGE(TAG, "Cannot start humidity mode. Critical error detected.");
        return false;
    } else if (setpoint < UNDER_HUMIDITY_THRESHOLD || setpoint > OVER_HUMIDITY_THRESHOLD) {
        ESP_LOGW(TAG, "Cannot start humidity mode. Setpoint out of range: %.2f", setpoint);
        return false;
    }

    config.humidity = setpoint;
    ESP_LOGI(TAG, "Starting humidity mode. Setpoint: %.2f", setpoint);
    storage.putFloat("h", setpoint);

    _startControl(&humidityMode, HUMIDITY_CTRL_BANG_WINDOW, setpoint, HUMIDITY_PID_KP, HUMIDITY_PID_KI,
                  HUMIDITY_PID_KD);
    return true;
}

bool ASTRA_BabyIncubator::setAirTempOffset(float offset) {
    config.airTempOffset = offset;
    ESP_LOGI(TAG, "Air temperature offset: %.2f", offset);
    storage.putFloat("A", offset);
    return true;
}

bool ASTRA_BabyIncubator::setSkinTempOffset(float offset) {
    config.skinTempOffset = offset;
    ESP_LOGI(TAG, "Skin temperature offset: %.2f", offset);
    storage.putFloat("S", offset);
    return true;
}

bool ASTRA_BabyIncubator::setHumidityOffset(float offset) {
    config.humidityOffset = offset;
    ESP_LOGI(TAG, "Humidity offset: %.2f", offset);
    storage.putFloat("H", offset);
    return true;
}

bool ASTRA_BabyIncubator::_stopControl(ControlMode_t* obj) {
    if (obj->state == false) {
        ESP_LOGW(TAG, "Stopping failed. %s is already stopped", obj->name);
        return false;
    }

    obj->state = false;
    xQueueSend(obj->queue, obj, portMAX_DELAY);

    return true;
}

void ASTRA_BabyIncubator::stopAirMode() {
    if (!_stopControl(&airMode)) return;
    heaterCabin.setPWM(0);
}

void ASTRA_BabyIncubator::stopBabyMode() {
    if (!_stopControl(&babyMode)) return;
    heaterCabin.setPWM(0);
}

void ASTRA_BabyIncubator::stopHumidityMode() {
    if (!_stopControl(&humidityMode)) return;
    heaterHumid.setPWM(0);
}

bool ASTRA_BabyIncubator::checkSafety() {
    if (!_checkFan()) return false;

    errorFlag.isSensorAirFailed = airTempSensor.isSensorFailed();
    errorFlag.isSensorSkinFailed = skinTempSensor.isSensorFailed();

    if (airMode.state) {
        if (!_checkAirTemperatureLimits()) return false;
        if (!_checkHeaterCabin()) return false;
        if (errorFlag.isSensorAirFailed) {
            ESP_LOGW(TAG, "AIR TEMPERATURE SENSOR FAILED!");
            _setCriticalErrorFlag();
            return false;
        }
    } else if (babyMode.state) {
        if (!_checkSkinTemperatureLimits()) return false;
        if (!_checkHeaterCabin()) return false;
        if (errorFlag.isSensorSkinFailed) {
            ESP_LOGW(TAG, "SKIN TEMPERATURE SENSOR FAILED!");
            _setCriticalErrorFlag();
            return false;
        }
    } else {
        errorFlag.isTempTooLow = false;
    }

    // if (humidityMode.state) {
    //     if (!_checkHeaterHumidity()) return false;

    //     errorFlag.isWaterLow = getWaterLevel() > 3.2;
    //     if (errorFlag.isWaterLow) {
    //         stopHumidityMode();
    //     }
    // }

    // Check non-critical conditions
    errorFlag.isSensorHumidityFailed = humiditySensor.isSensorFailed();

    (void)_checkWater();
    (void)_checkBattery();

    // TODO: Implement these checks
    errorFlag.isRTCFailed = false;      // TODO: Implement RTC check
    errorFlag.isStorageFailed = false;  // TODO: Implement storage check

    return true;
}

bool ASTRA_BabyIncubator::_checkAirTemperatureLimits() {
    float temperature = getAirTemperature();
    errorFlag.isTempTooLow = (temperature < UNDER_TEMPERATURE_CABIN_THRESHOLD);
    errorFlag.isTempTooHigh = (temperature > OVER_TEMPERATURE_CABIN_THRESHOLD);
    errorFlag.isTempTooHigh = (getSkinTemperature() > OVER_TEMPERATURE_SYSTEM_THRESHOLD);
    if (!errorFlag.isTempTooHigh) return true;

    ESP_LOGW(TAG, "AIR TEMPERATURE TOO HIGH! (%.2f)", temperature);
    _setCriticalErrorFlag();
    return false;
}

bool ASTRA_BabyIncubator::_checkSkinTemperatureLimits() {
    float temperature = getSkinTemperature();
    errorFlag.isTempTooLow = (temperature < UNDER_TEMPERATURE_SKIN_THRESHOLD);
    errorFlag.isTempTooHigh = (temperature > OVER_TEMPERATURE_SKIN_THRESHOLD);
    errorFlag.isTempTooHigh = (getAirTemperature() > OVER_TEMPERATURE_SYSTEM_THRESHOLD);
    if (!errorFlag.isTempTooHigh) return true;

    ESP_LOGW(TAG, "SKIN TEMPERATURE TOO HIGH! (%.2f)", temperature);
    _setCriticalErrorFlag();
    return false;
}

// BUAT SAFETY
bool ASTRA_BabyIncubator::_checkOnlyAirTempHigh() {
    float airTemp = getAirTemperature();

    bool overAir = (airTemp > 42);

    errorFlag.isTempTooHigh = overAir;

    if (!errorFlag.isTempTooHigh) return true;

    ESP_LOGW(TAG, "AIR TEMP TOO HIGH! (%.2f)", airTemp);
    return false;
}

bool ASTRA_BabyIncubator::_checkOnlySkinTempHigh() {
    float skinTemp = getSkinTemperature();

    bool overSkin = (skinTemp > 42);

    errorFlag.isTempTooHigh = overSkin;

    if (!errorFlag.isTempTooHigh) return true;

    ESP_LOGW(TAG, "SKIN TEMP TOO HIGH! (%.2f)", skinTemp);
    return false;
}

bool ASTRA_BabyIncubator::_checkHeaterCabin() {
    CurrentSenseState_t heaterState = i_heaterCabin.getState();
    errorFlag.isHeaterCabinFailed = (heaterState != SENSE_CURRENT_NORMAL);
    if (!errorFlag.isHeaterCabinFailed) return true;

    ESP_LOGW(TAG, "HEATER CABIN %s!", (heaterState == SENSE_UNDER_CURRENT) ? "DISCONNECTED" : "OVERCURRENT");
    _setCriticalErrorFlag();
    return false;
}

bool ASTRA_BabyIncubator::_checkHeaterHumidity() {
    CurrentSenseState_t heaterState = i_heaterHumid.getState();
    errorFlag.isHeaterHumidFailed = (heaterState != SENSE_CURRENT_NORMAL);
    if (!errorFlag.isHeaterHumidFailed) return true;

    ESP_LOGW(TAG, "HEATER HUMIDITY %s!", (heaterState == SENSE_UNDER_CURRENT) ? "DISCONNECTED" : "OVERCURRENT");
    _setCriticalErrorFlag();
    return false;
}

bool ASTRA_BabyIncubator::_checkFan() {
    float fanRPM = getFanRPM();
    errorFlag.isFanFailed = (fanRPM < FAN_RPM_CONSIDERED_STALL);
    if (!errorFlag.isFanFailed) return true;

    ESP_LOGW(TAG, "FAN STALL! (RPM: %.2f)", fanRPM);
    _setCriticalErrorFlag();
    return false;
}

bool ASTRA_BabyIncubator::_checkBattery() {
    // ! This is not actually measuring a battery voltage
    // ! These lines detect battery connection only
    float batteryVoltage = (batteryVoltageLevel.read() * (10000 + 3000)) / 3000;
    errorFlag.isBatteryLow = (batteryVoltage >= BATTERY_VOLTAGE_CONSIDERED_LOW);
    if (!errorFlag.isBatteryLow) return true;

    ESP_LOGW(TAG, "BATTERY VOLTAGE LOW! (%.2f V)", batteryVoltage);
    return false;
}

bool ASTRA_BabyIncubator::_checkWater() {
    float waterLevel = (waterLevelSensor.read());
    errorFlag.isWaterLow = getWaterLevel() > 3.2;
    if (!errorFlag.isWaterLow) return true;

    ESP_LOGW(TAG, "WATER TANK LOW! (%.2f V)", waterLevel);
    return false;
}