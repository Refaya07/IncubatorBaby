#include "ASTRA_BabyIncubatorHMI.h"
#include "ASTRA_BabyIncubator.h"
#include "HMI_commands.h"
#include "HardwareConfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Serial communication configuration for Nextion display
#define NXT_Serial Serial1
#define NXT_SERIAL_BAUDRATE 115200
#define WELCOME_TO_HOME_SCREEN_DELAY_MS 5000

SemaphoreHandle_t critMutex = NULL;

static const char* TAG = "HMI";

/**
 * @brief Timer callback for screen timeout
 * @details Locks the screen after the specified timeout period to prevent
 * accidental changes and save power. The screen can be unlocked by user input.
 *
 * @param xTimer Handle to the timer that triggered this callback
 */
static void screenTimeoutCallback(TimerHandle_t xTimer) {
    bool* locked = (bool*)pvTimerGetTimerID(xTimer);
    *locked = true;
}

/**
 * @brief FreeRTOS task for reading HMI commands
 * @details Continuously monitors the serial port for commands from the Nextion display.
 *
 * @param param Pointer to ASTRA_BabyIncubatorHMI instance
 */
static void readHMICommandTask(void* param) {
    ASTRA_BabyIncubatorHMI* hmi = (ASTRA_BabyIncubatorHMI*)param;
    while (true) {
        hmi->listenCommand();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief FreeRTOS task for streaming data to HMI
 * @details Updates the display with current sensor readings and system status
 * at regular intervals (1 second).
 *
 * @param param Pointer to ASTRA_BabyIncubatorHMI instance
 */
static void streamHMIDataTask(void* param) {
    ASTRA_BabyIncubatorHMI* hmi = (ASTRA_BabyIncubatorHMI*)param;
    while (true) {
        hmi->streamData();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ASTRA_BabyIncubatorHMI::_NXT_write(String command) {
    if (xSemaphoreTake(critMutex, pdMS_TO_TICKS(50)) == pdFAIL) return;
    NXT_Serial.print(command);
    NXT_Serial.write(0xFF);
    NXT_Serial.write(0xFF);
    NXT_Serial.write(0xFF);  // Send three 0xFF bytes as end of command
    xSemaphoreGive(critMutex);
}

void ASTRA_BabyIncubatorHMI::_NXT_writeString(const char* assetID, const char* value) {
    String cmd = String(assetID) + String(".txt=\"") + String(value) + "\"";
    _NXT_write(cmd);
}

void ASTRA_BabyIncubatorHMI::_NXT_writeFloat(const char* assetID, float value) {
    String cmd = String(assetID) + String(".val=") + String(int(value * 10));
    _NXT_write(cmd);
}

void ASTRA_BabyIncubatorHMI::_NXT_writeInt(const char* assetID, int value) {
    String cmd = String(assetID) + String(".val=") + String(value);
    _NXT_write(cmd);
}

void ASTRA_BabyIncubatorHMI::_PAGE_welcome() { _NXT_write("p100.pic=" + String(logoWelcome[currentLogo])); }

void ASTRA_BabyIncubatorHMI::_PAGE_home(bool firstTime) {
    if (firstTime) {
        _NXT_write("p101.pic=" + String(logoHome[currentLogo]));
        // Initialize control states and setpoints
        _NXT_writeInt(HMI_ASSET_AIRTEMP_CONTROL_EN, abi.getAirTempControlState());
        _NXT_writeInt(HMI_ASSET_SKINTEMP_CONTROL_EN, abi.getSkinTempControlState());
        _NXT_writeInt(HMI_ASSET_HUMIDITY_CONTROL_EN, abi.getHumidityControlState());

        ABI_config_t config = abi.getConfig();
        _NXT_writeFloat(HMI_ASSET_HOME_AIRTEMP_SETPOINT_PREV, config.airTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_SKINTEMP_SETPOINT_PREV, config.skinTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_HUMIDITY_SETPOINT_PREV, config.humidity);
        _NXT_writeFloat(HMI_ASSET_HOME_AIRTEMP_SETPOINT, config.airTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_SKINTEMP_SETPOINT, config.skinTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_HUMIDITY_SETPOINT, config.humidity);
    }

    // Update sensor readings
    _NXT_writeFloat(HMI_ASSET_HOME_AIRTEMP_READING, abi.getAirTemperature());
    _NXT_writeFloat(HMI_ASSET_HOME_SKINTEMP_READING, abi.getSkinTemperature());
    _NXT_writeFloat(HMI_ASSET_HOME_HUMIDITY_READING, abi.getHumidity());

    // Update date and time display
    DateTime now = abi.getDateTime();
    char date[24];
    char time[24];
    sprintf(date, "%02d-%02d-%04d", now.day(), now.month(), now.year());
    sprintf(time, "%02d:%02d", now.hour(), now.minute());
    _NXT_writeString(HMI_ASSET_HOME_DATE, date);
    _NXT_writeString(HMI_ASSET_HOME_TIME, time);

    // Check and display error states
    ABI_errorflag_t err = abi.getError();

    // Update error flags only if they have changed to avoid unnecessary updates
    if (!(currentErrorFlag == err)) {
        // Activate buzzer if any error exists
        buzzer.setPWM(err.isErrorExist() ? 60 : 0);

        // Update sensor display opacity based on error state
        // Failed sensors are shown with reduced opacity
        char cmdAir[100];
        sprintf(cmdAir, "%s.aph=%u", HMI_ASSET_HOME_AIRTEMP_READING, err.isSensorAirFailed ? 0 : 127);
        _NXT_write(cmdAir);

        char cmdSkin[100];
        sprintf(cmdSkin, "%s.aph=%u", HMI_ASSET_HOME_SKINTEMP_READING, err.isSensorSkinFailed ? 0 : 127);
        _NXT_write(cmdSkin);

        char cmdHumidity[100];
        sprintf(cmdHumidity, "%s.aph=%u", HMI_ASSET_HOME_HUMIDITY_READING, err.isSensorHumidityFailed ? 0 : 127);
        _NXT_write(cmdHumidity);

        // Update error indicators
        bool isSensorFailed = err.isSensorAirFailed || err.isSensorSkinFailed || err.isSensorHumidityFailed;
        bool isSystemFailed = err.isRTCFailed || err.isStorageFailed ;
        isSystemFailed = isSensorFailed && err.isFanFailed && err.isHeaterCabinFailed && err.isHeaterHumidFailed;

        _NXT_writeInt(HMI_ASSET_SENSOR_IS_FAILED, isSensorFailed);
        _NXT_writeInt(HMI_ASSET_FAN_IS_FAILED, err.isFanFailed);
        _NXT_writeInt(HMI_ASSET_HEATER_IS_FAILED, err.isHeaterCabinFailed || err.isHeaterHumidFailed);
        _NXT_writeInt(HMI_ASSET_TOO_HIGH_TEMP, err.isTempTooHigh);
        _NXT_writeInt(HMI_ASSET_TOO_LOW_TEMP, err.isTempTooLow);
        _NXT_writeInt(HMI_ASSET_BATTERY_LOW, err.isBatteryLow);
        _NXT_writeInt(HMI_ASSET_WATER_LOW, err.isWaterLow);
        _NXT_writeInt(HMI_ASSET_SYSTEM_ERROR, isSystemFailed);  // TODO: Implement system error check

        currentErrorFlag = err;
    }

    // ! Update current and power readings (Development only)
    // _NXT_writeString(HMI_ASSET_HOME_HEATER_CURRENT, String(abi.getCabinHeaterCurrent()).c_str());
    // _NXT_writeString(HMI_ASSET_HOME_HUMIDITY_CURRENT, String(abi.getHumidityHeaterCurrent()).c_str());
    // _NXT_writeString(HMI_ASSET_HOME_FAN_RPM_READING, String(abi.getFanRPM()).c_str());

    // Update heater power display (quantized to 10% steps)
    _NXT_writeInt(HMI_ASSET_HOME_HEATER_POWER, int(abi.getHeaterPower() / 10.0));

    // Update screen lock state
    if (locked) _NXT_writeInt(HMI_ASSET_LOCK_STATE, 1);

    if (abi.isCriticalErrorDetected()) _NXT_writeInt(HMI_ASSET_STOP_ALL_MODE, true);
}

void ASTRA_BabyIncubatorHMI::_PAGE_date() {
    DateTime now = abi.getDateTime();
    ABI_config_t config = abi.getConfig();

    _NXT_writeInt(HMI_ASSET_SETTING_DAY, now.day());
    _NXT_writeInt(HMI_ASSET_SETTING_MONTH, now.month());
    _NXT_writeInt(HMI_ASSET_SETTING_YEAR, now.year());
    _NXT_writeInt(HMI_ASSET_SETTING_HOUR, now.hour());
    _NXT_writeInt(HMI_ASSET_SETTING_MINUTE, now.minute());
}

void ASTRA_BabyIncubatorHMI::_PAGE_setting() {
    DateTime now = abi.getDateTime();
    ABI_config_t config = abi.getConfig();

    _NXT_writeInt(HMI_ASSET_SETTING_DAY, now.day());
    _NXT_writeInt(HMI_ASSET_SETTING_MONTH, now.month());
    _NXT_writeInt(HMI_ASSET_SETTING_YEAR, now.year());
    _NXT_writeInt(HMI_ASSET_SETTING_HOUR, now.hour());
    _NXT_writeInt(HMI_ASSET_SETTING_MINUTE, now.minute());
    _NXT_writeFloat(HMI_ASSET_SETTING_AIRTEMP_OFFSET, config.airTempOffset);
    _NXT_writeFloat(HMI_ASSET_SETTING_SKINTEMP_OFFSET, config.skinTempOffset);
    _NXT_writeInt(HMI_ASSET_SETTING_HUMIDITY_OFFSET, config.humidityOffset);
}

void ASTRA_BabyIncubatorHMI::_PAGE_password() {}

void ASTRA_BabyIncubatorHMI::_PAGE_logoSetting() {}

bool ASTRA_BabyIncubatorHMI::begin() {
    ESP_LOGI(TAG, "Initializing Astra Baby Incubator HMI...");
    buzzer.init(LOW, HIGH);
    buzzer.setPWM(0);

    NXT_Serial.begin(NXT_SERIAL_BAUDRATE, SERIAL_8N1, HMI_SERIAL_RX, HMI_SERIAL_TX);

    currentLogo = abi.getLogo();
    switchPage(HMI_Page::HMI_PAGE_WELCOME);

    vTaskDelay(pdMS_TO_TICKS(WELCOME_TO_HOME_SCREEN_DELAY_MS));
    switchPage(HMI_Page::HMI_PAGE_HOME);

    // Create tasks for HMI operation
    xTaskCreatePinnedToCore(readHMICommandTask, "Read HMI Command Task", 2048 * 5, this, 1, NULL, 0);
    xTaskCreatePinnedToCore(streamHMIDataTask, "Stream HMI Data Task", 2048 * 5, this, 1, NULL, 0);

    // Create screen timeout timer
    screenTimeoutTimerHandle = xTimerCreate("Screen Timer Timeout", pdMS_TO_TICKS(screenTimeout_ms), pdFALSE,
                                            (void*)&locked, screenTimeoutCallback);
    if (screenTimeoutTimerHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create screen timeout timer");
        return false;
    }

    return true;
}

void ASTRA_BabyIncubatorHMI::switchPage(const HMI_Page page) {
    currentScreen = page;
    switch (page) {
        case HMI_PAGE_WELCOME:
            _NXT_write(HMI_PAGE_ID_WELCOME);
            _PAGE_welcome();
            break;
        case HMI_PAGE_HOME:
            _NXT_write(HMI_PAGE_ID_HOME);
            _PAGE_home(true);
            break;
        case HMI_PAGE_SETTING:
            _NXT_write(HMI_PAGE_ID_SETTING);
            _PAGE_setting();
            break;
        case HMI_PAGE_PASSWORD:
            _NXT_write(HMI_PAGE_ID_PASSWORD);
            _PAGE_password();
            break;
        case HMI_PAGE_LOGO:
            _NXT_write(HMI_PAGE_ID_LOGO_SETTING);
            _PAGE_logoSetting();
            break;
        case HMI_PAGE_DATE:
            _NXT_write(HMI_PAGE_ID_DATE);
            _PAGE_date();
            break;

        default:
            break;
    }
}

/**
 * @brief Stream data to the current page
 * @details Updates the display with current sensor readings and system status.
 * Only the home page requires continuous updates, other pages are static.
 */
void ASTRA_BabyIncubatorHMI::streamData() {
    switch (currentScreen) {
        case HMI_PAGE_HOME:
            _PAGE_home();
            break;
        default:
            break;
    }
}

/**
 * @brief Process incoming HMI commands
 * @details Reads commands from the serial port and handles screen timeout.
 * Commands are terminated with a semicolon (;) and can include:
 *
 * The function also manages screen timeout and provides audio feedback
 * for user interactions.
 */
void ASTRA_BabyIncubatorHMI::listenCommand() {
    const size_t MAX_COMMAND_LENGTH = 64;    // Maximum command length
    char commandBuffer[MAX_COMMAND_LENGTH];  // Static buffer for command
    size_t bufferIndex = 0;                  // Current position in buffer

    while (true) {
        char c;

        if (xSemaphoreTake(critMutex, pdMS_TO_TICKS(50)) == pdFAIL) return;
        if (NXT_Serial.available()) {
            c = NXT_Serial.read();
            xSemaphoreGive(critMutex);
        } else {
            xSemaphoreGive(critMutex);
            break;  // Nothing more to read
        }

        // Check for buffer overflow
        if (bufferIndex >= MAX_COMMAND_LENGTH - 1) {
            bufferIndex = 0;  // Reset buffer if overflow
            continue;
        }

        commandBuffer[bufferIndex++] = c;

        if (c == ';') {
            commandBuffer[bufferIndex - 1] = '\0';  // Remove semicolon and null terminate
            String command(commandBuffer);          // Create String object from buffer

            ESP_LOGI(TAG, "Received command: %s", command.c_str());
            _processCommand(command);

            // Provide audio feedback for user interaction
            buzzer.setOneShot();

            // Reset buffer after processing command
            bufferIndex = 0;
        }
    }
}

void ASTRA_BabyIncubatorHMI::_processCommand(String command) {
    // Reset screen timeout timer if there is screen activity and screen is not locked
    if (xTimerIsTimerActive(screenTimeoutTimerHandle) == pdTRUE && !locked)
        xTimerReset(screenTimeoutTimerHandle, portMAX_DELAY);

    // Handle screen lock/unlock commands
    if (command == HMI_CMD_UNLOCK_SCREEN) {
        locked = false;
        xTimerStart(screenTimeoutTimerHandle, portMAX_DELAY);
    } else if (command == HMI_CMD_LOCK_SCREEN) {
        xTimerStop(screenTimeoutTimerHandle, portMAX_DELAY);
        locked = true;
        ABI_config_t config = abi.getConfig();
        _NXT_writeFloat(HMI_ASSET_HOME_AIRTEMP_SETPOINT, config.airTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_SKINTEMP_SETPOINT, config.skinTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_HUMIDITY_SETPOINT, config.humidity);
        _NXT_writeFloat(HMI_ASSET_HOME_AIRTEMP_SETPOINT_PREV, config.airTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_SKINTEMP_SETPOINT_PREV, config.skinTemperature);
        _NXT_writeFloat(HMI_ASSET_HOME_HUMIDITY_SETPOINT_PREV, config.humidity);
    }
    // Handle mute/unmute commands
    else if (command == HMI_CMD_MUTE) {
        mute = true;
        buzzer.enable(!mute);
        buzzer.setPWM(0); // Pastikan buzzer mati saat mute
    } else if (command == HMI_CMD_UNMUTE) {
        mute = false;
        buzzer.enable(!mute);
        ABI_errorflag_t err = abi.getError();
        buzzer.setPWM(err.isErrorExist() ? 60 : 0); // Update status buzzer sesuai error
    }
    // Handle page navigation commands
    else if (command == HMI_CMD_PAGE_LOGO) {
        switchPage(HMI_Page::HMI_PAGE_LOGO);
    } else if (command == HMI_CMD_PAGE_HOME) {
        switchPage(HMI_Page::HMI_PAGE_HOME);
    } else if (command == HMI_CMD_PAGE_SETT) {
        switchPage(HMI_Page::HMI_PAGE_SETTING);
    } else if (command == HMI_CMD_PAGE_DATE) {
        switchPage(HMI_Page::HMI_PAGE_DATE);
    } else if (command == HMI_CMD_PAGE_PASSWORD) {
        switchPage(HMI_Page::HMI_PAGE_PASSWORD);
    }
    // Handle mode control commands
    else if (command == HMI_CMD_STOP_AIR_MODE) {
        abi.stopAirMode();
    } else if (command == HMI_CMD_STOP_BABY_MODE) {
        abi.stopBabyMode();
    } else if (command == HMI_CMD_STOP_HUMIDITY_MODE) {
        abi.stopHumidityMode();
    }

    // Handle logo selection commands
    int setLogo = 0;
    if (command == HMI_CMD_SET_LOGO1) {
        setLogo = 1;
    } else if (command == HMI_CMD_SET_LOGO2) {
        setLogo = 2;
    } else if (command == HMI_CMD_SET_LOGO3) {
        setLogo = 3;
    } else if (command == HMI_CMD_SET_LOGO4) {
        setLogo = 4;
    }

    if (setLogo != 0) {
        currentLogo = setLogo;
        abi.setLogo(currentLogo);
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }

    /** Setting commands **/
    if (command[1] == HMI_CMD_SET_VAR) {
        String text = command;
        int separatorIndex = text.indexOf(HMI_CMD_SET_VAR);

        String varName = text.substring(0, separatorIndex);
        String nilai = text.substring(separatorIndex + 1);

        // Convert string to float and divide by 10.0
        float valToStore = nilai.toFloat() / 10.0;

        // Handle various setting commands
        if (varName == HMI_CMD_SET_VAR_AIRTEMP_SETPOINT) {
            abi.startAirMode(valToStore);
        } else if (varName == HMI_CMD_SET_VAR_SKINTEMP_SETPOINT) {
            abi.startBabyMode(valToStore);
        } else if (varName == HMI_CMD_SET_VAR_HUMIDITY_SETPOINT) {
            abi.startHumidityMode(valToStore);
        } else if (varName == HMI_CMD_SET_VAR_AIRTEMP_OFFSET) {
            abi.setAirTempOffset(valToStore);
        } else if (varName == HMI_CMD_SET_VAR_SKINTEMP_OFFSET) {
            abi.setSkinTempOffset(valToStore);
        } else if (varName == HMI_CMD_SET_VAR_HUMIDITY_OFFSET) {
            abi.setHumidityOffset(valToStore * 10);
        } else if (varName == HMI_CMD_SET_DATE_TIME) {
            int y, m, d, h, mi;
            int nInput = sscanf(command.c_str(), "t:%d %d %d %d %d;", &d, &m, &y, &h, &mi);
            if (nInput == 5) {
                DateTime dateTime(y, m, d, h, mi, 0);
                abi.setDateTime(dateTime);
                ESP_LOGI(TAG, "Date and time set to: %04d-%02d-%02d %02d:%02d:00", y, m, d, h, mi);
            } else {
                ESP_LOGW(TAG, "Failed to parse date and time from command(%d): %s", nInput, command.c_str());
            }
        }
    }
}