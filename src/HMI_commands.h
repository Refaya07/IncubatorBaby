#pragma once

/* Pages */
// Page identifiers for the Nextion HMI display
// Each page has a unique ID used for navigation
#define HMI_PAGE_ID_WELCOME "page 0"       // Welcome/startup page
#define HMI_PAGE_ID_HOME "page 1"          // Main control page
#define HMI_PAGE_ID_SETTING "page 2"       // Settings configuration page
#define HMI_PAGE_ID_PASSWORD "page 3"      // Password protection page
#define HMI_PAGE_ID_LOGO_SETTING "page 4"  // Logo customization page
#define HMI_PAGE_ID_DATE "page 5"  // General date settings page

/* Commands */
// Control commands for the HMI display
// These commands are sent to the display to control various functions
#define HMI_CMD_STOP_AIR_MODE "disAir"       // Disable air temperature control
#define HMI_CMD_STOP_BABY_MODE "disBaby"     // Disable skin temperature control
#define HMI_CMD_STOP_HUMIDITY_MODE "disHum"  // Disable humidity control
#define HMI_CMD_LOCK_SCREEN "lock"           // Lock the display
#define HMI_CMD_UNLOCK_SCREEN "unlock"       // Unlock the display
#define HMI_CMD_MUTE "mute"                  // Mute alarms
#define HMI_CMD_UNMUTE "unmute"              // Unmute alarms
#define HMI_CMD_PAGE_LOGO "p logo"           // Navigate to logo page
#define HMI_CMD_PAGE_HOME "p home"           // Navigate to home page
#define HMI_CMD_PAGE_SETT "p setting"        // Navigate to settings page
#define HMI_CMD_PAGE_PASSWORD "p password"   // Navigate to password page
#define HMI_CMD_PAGE_DATE "p date"   // Navigate to password page
#define HMI_CMD_SET_LOGO1 "logo1"            // Set logo 1
#define HMI_CMD_SET_LOGO2 "logo2"            // Set logo 2
#define HMI_CMD_SET_LOGO3 "logo3"            // Set logo 3
#define HMI_CMD_SET_LOGO4 "logo4"            // Set logo 4

// Variable setting commands
// These commands are used to update display variables
#define HMI_CMD_SET_VAR ':'                    // Variable set command prefix
#define HMI_CMD_SET_VAR_AIRTEMP_SETPOINT "a"   // Set air temperature setpoint
#define HMI_CMD_SET_VAR_SKINTEMP_SETPOINT "s"  // Set skin temperature setpoint
#define HMI_CMD_SET_VAR_HUMIDITY_SETPOINT "h"  // Set humidity setpoint
#define HMI_CMD_SET_VAR_AIRTEMP_OFFSET "A"     // Set air temperature offset
#define HMI_CMD_SET_VAR_SKINTEMP_OFFSET "S"    // Set skin temperature offset
#define HMI_CMD_SET_VAR_HUMIDITY_OFFSET "H"    // Set humidity offset
#define HMI_CMD_SET_DATE_TIME "t"              // Set date and time

/* Assets */
/** HOME **/
// Date and time display elements
#define HMI_ASSET_HOME_DATE "t900"  // Date display
#define HMI_ASSET_HOME_TIME "t901"  // Time display

// Control enable/disable indicators
#define HMI_ASSET_AIRTEMP_CONTROL_EN "airEn"     // Air temperature control status
#define HMI_ASSET_SKINTEMP_CONTROL_EN "babyEn"   // Skin temperature control status
#define HMI_ASSET_HUMIDITY_CONTROL_EN "humidEn"  // Humidity control status
#define HMI_ASSET_STOP_ALL_MODE "isCritFail"     // Critical failure status

// Setpoint display elements
#define HMI_ASSET_HOME_AIRTEMP_SETPOINT_PREV "airSet"     // Previous air temperature setpoint
#define HMI_ASSET_HOME_SKINTEMP_SETPOINT_PREV "skinSet"   // Previous skin temperature setpoint
#define HMI_ASSET_HOME_HUMIDITY_SETPOINT_PREV "humidSet"  // Previous humidity setpoint

// Current readings and setpoints
#define HMI_ASSET_HOME_AIRTEMP_READING "x800"    // Current air temperature
#define HMI_ASSET_HOME_AIRTEMP_SETPOINT "x801"   // Air temperature setpoint
#define HMI_ASSET_HOME_SKINTEMP_READING "x802"   // Current skin temperature
#define HMI_ASSET_HOME_SKINTEMP_SETPOINT "x803"  // Skin temperature setpoint
#define HMI_ASSET_HOME_HUMIDITY_READING "x804"   // Current humidity
#define HMI_ASSET_HOME_HUMIDITY_SETPOINT "x805"  // Humidity setpoint

// System status displays
#define HMI_ASSET_HOME_FAN_RPM_READING "t7"      // Fan RPM display
#define HMI_ASSET_HOME_HEATER_CURRENT "t9"       // Heater current display
#define HMI_ASSET_HOME_HUMIDITY_CURRENT "t11"    // Humidity heater current display
#define HMI_ASSET_HOME_HEATER_POWER "heaterPow"  // Heater power display
#define HMI_ASSET_LOCK_STATE "lock"              // Screen lock state

// Alarm and error indicators
#define HMI_ASSET_SENSOR_IS_FAILED "isSensorFailed"  // Sensor failure indicator
#define HMI_ASSET_FAN_IS_FAILED "isFanFailed"        // Fan failure indicator
#define HMI_ASSET_HEATER_IS_FAILED "isHeaterFailed"  // Heater failure indicator
#define HMI_ASSET_TOO_HIGH_TEMP "isTempHigh"         // High temperature alarm
#define HMI_ASSET_TOO_LOW_TEMP "isTempLow"           // Low temperature alarm
#define HMI_ASSET_BATTERY_LOW "isBattLow"            // Low battery alarm
#define HMI_ASSET_SYSTEM_ERROR "isSystemFailed"      // System error indicator
#define HMI_ASSET_WATER_LOW "isWaterLow"             // Low water level alarm

/** SETTING **/
// Date and time setting elements
#define HMI_ASSET_SETTING_DAY "n100"     // Day setting
#define HMI_ASSET_SETTING_MONTH "n101"   // Month setting
#define HMI_ASSET_SETTING_YEAR "n102"    // Year setting
#define HMI_ASSET_SETTING_HOUR "n103"    // Hour setting
#define HMI_ASSET_SETTING_MINUTE "n104"  // Minute setting

// Sensor offset settings
#define HMI_ASSET_SETTING_AIRTEMP_OFFSET "x806"   // Air temperature offset
#define HMI_ASSET_SETTING_SKINTEMP_OFFSET "x807"  // Skin temperature offset
#define HMI_ASSET_SETTING_HUMIDITY_OFFSET "x808"  // Humidity offset

/* LOGO ASSET CODE */
// Logo image IDs for different pages
const int logoWelcome[5] = {9, 5, 6, 7, 8};    // Welcome page logos
const int logoHome[5] = {14, 10, 11, 12, 13};  // Home page logos