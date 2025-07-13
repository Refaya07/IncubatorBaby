/**
 * @file ASTRA_BabyIncubatorHMI.h
 * @brief Header file for the ASTRA Baby Incubator Human Machine Interface (HMI)
 * @details This file contains the class definition and related enums for the HMI system
 * that controls the user interface of the baby incubator.
 */

#pragma once

#include <Arduino.h>
#include "ASTRA_BabyIncubator.h"
#include "actuators/DigitalDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

/**
 * @brief Enumeration of available HMI pages
 */
typedef enum {
    HMI_PAGE_WELCOME,   ///< Welcome/startup page
    HMI_PAGE_HOME,      ///< Main control page
    HMI_PAGE_SETTING,   ///< Settings configuration page
    HMI_PAGE_PASSWORD,  ///< Password entry page
    HMI_PAGE_LOGO,      ///< Logo selection page
    HMI_PAGE_DATE       ///< Date and time setting page
} HMI_Page;

/**
 * @class ASTRA_BabyIncubatorHMI
 * @brief Class managing the Human Machine Interface of the baby incubator
 * @details Handles all user interface interactions, screen management, and command processing
 */
class ASTRA_BabyIncubatorHMI {
   private:
    DigitalDriver buzzer;                           ///< Buzzer driver for audio feedback
    TimerHandle_t screenTimeoutTimerHandle = NULL;  ///< Timer for screen timeout

    bool locked = true;                         ///< Interface lock state
    int screenTimeout_ms = 30000;               ///< Screen timeout duration in milliseconds
    bool mute = false;                          ///< Audio mute state
    HMI_Page currentScreen = HMI_PAGE_WELCOME;  ///< Current active screen
    int currentLogo = 0;                        ///< Currently selected logo

    ABI_errorflag_t currentErrorFlag;  ///< Current error flags

    /**
     * @brief Write a command to the Nextion display
     * @param command The command string to write
     */
    void _NXT_write(String command);

    /**
     * @brief Write a string value to a Nextion display asset
     * @param assetID The ID of the display asset
     * @param value The string value to write
     */
    void _NXT_writeString(const char* assetID, const char* value);

    /**
     * @brief Write a float value to a Nextion display asset
     * @param assetID The ID of the display asset
     * @param value The float value to write
     */
    void _NXT_writeFloat(const char* assetID, float value);

    /**
     * @brief Write an integer value to a Nextion display asset
     * @param assetID The ID of the display asset
     * @param value The integer value to write
     */
    void _NXT_writeInt(const char* assetID, int value);

    /**
     * @brief Display the welcome page
     */
    void _PAGE_welcome();

    /**
     * @brief Display the home page
     */
    void _PAGE_home(bool firstTime = false);

    /**
     * @brief Display the settings page
     */
    void _PAGE_setting();

    /**
     * @brief Display the password page
     */
    void _PAGE_password();

    /**
     * @brief Display the logo settings page
     */
    void _PAGE_logoSetting();

    /**
     * @brief general date settings page
     */
    void _PAGE_date();

    /**
     * @brief Process incoming command from UART
     */
    void _processCommand(String command);

   public:
    /**
     * @brief Constructor for the HMI class
     */
    ASTRA_BabyIncubatorHMI() : buzzer(BUZZER_OUT_PIN) {}

    /**
     * @brief Initialize the HMI system. Sets up serial communication, creates FreeRTOS tasks for command processing
     * and data streaming, and initializes the screen timeout timer.
     * @return true if initialization was successful, false otherwise
     */
    bool begin();

    /**
     * @brief Switch to a different HMI page
     * @param page The page to switch to
     */
    void switchPage(const HMI_Page page);

    /**
     * @brief Stream data to the display
     */
    void streamData();

    /**
     * @brief Listen for and process HMI commands
     */
    void listenCommand();

    /**
     * @brief Get the current active page
     * @return The current HMI page
     */
    HMI_Page getCurrentPage() const { return currentScreen; }

    /**
     * @brief Get the current lock state
     * @return true if the interface is locked, false otherwise
     */
    bool getLockedState() const { return locked; }
};

extern ASTRA_BabyIncubatorHMI hmi;  ///< Global HMI instance