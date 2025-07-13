#pragma once

/* Control constants */
// Control system timing parameters: These values determine how often the control system updates
#define CONTROL_UPDATE_TICK_mS 5000  // Control loop runs every 5 seconds
#define SAFETY_CHECK_TICK_mS 1000    // Safety checks run every 1 second

// Control output limits for all control systems
// These define the range of possible output values (duty cycle)
#define MIN_OUTPUT_DUTY_CYCLE 0    // Minimum control output (0% duty cycle)
#define MAX_OUTPUT_DUTY_CYCLE 100  // Maximum control output (100% duty cycle)

// Default setpoints for the control system
#define DEFAULT_AIRTEMP_SETPOINT 36.0   // Default air temperature setpoint (36°C)
#define DEFAULT_SKINTEMP_SETPOINT 36.0  // Default skin temperature setpoint (36°C)
#define DEFAULT_HUMIDITY_SETPOINT 45.0  // Default humidity setpoint (45%)

// Current monitoring thresholds for heaters
// These values are used to detect heater malfunctions and ensure safe operation
#define UNDERCURRENT_THRESHOLD 0.02             // Minimum current to detect heater connection (50mA)
#define OVERCURRENT_CABIN_HEATER_THRESHOLD 4.0  // Maximum safe current for cabin heater (4A)
#define OVERCURRENT_HUMID_HEATER_THRESHOLD 3.0  // Maximum safe current for humidity heater (3A)

// Safety thresholds for temperature monitoring
// These values define the safe operating range for the incubator
#define UNDER_TEMPERATURE_CABIN_THRESHOLD 30.0       // Minimum safe air temperature (30°C)
#define OVER_TEMPERATURE_CABIN_THRESHOLD 39.5        // Maximum safe air temperature (39.5°C)
#define MAX_NORMAL_TEMPERATURE_CABIN_THRESHOLD 37.0  // Maximum normal air temperature (39.5°C)

#define UNDER_TEMPERATURE_SKIN_THRESHOLD 30.0  // Minimum safe skin temperature (30°C)
#define OVER_TEMPERATURE_SKIN_THRESHOLD 39.5   // Maximum safe skin temperature (39.5°C)
#define MAX_NORMAL_SKIN_THRESHOLD 37.0         // Maximum normal skin temperature (39.5°C)

#define OVER_TEMPERATURE_SYSTEM_THRESHOLD 45.0  // Maximum safe air temperature (39.5°C)

// Safety thresholds for humidity monitoring
// These define the acceptable humidity range for the incubator
#define UNDER_HUMIDITY_THRESHOLD 30.0  // Minimum safe humidity level (30%)
#define OVER_HUMIDITY_THRESHOLD 99.0   // Maximum safe humidity level (99%)

// Fan control parameters
// These values control fan operation and monitoring
#define FAN_DUTY_CYCLE_DEFAULT 20       // Default fan speed (20% duty cycle)
#define FAN_RPM_CONSIDERED_STALL 100.0  // RPM below which fan is considered stalled (100 RPM)

// Battery monitoring threshold
// This value determines when the system should indicate low battery
#define BATTERY_VOLTAGE_CONSIDERED_LOW 10.4  // Default 10.6

/* PID constants */
// Kp: Proportional gain - determines how aggressively the system responds to error
// Ki: Integral gain - helps eliminate steady-state error
// Kd: Derivative gain - helps reduce overshooting and improve stability

// PID control parameters for air temperature control
#define AIR_TEMP_CTRL_BANG_WINDOW 2.5  // Use bang-bang control for errors > 2.5°C
#define AIR_TEMP_PID_KP (3.450 * 10)
#define AIR_TEMP_PID_KI (0.380 * 10)
#define AIR_TEMP_PID_KD (2.120 * 10)

// PID control parameters for skin temperature control
#define SKIN_TEMP_CTRL_BANG_WINDOW 2.5  // Use bang-bang control for errors > 2.5°C
#define SKIN_TEMP_PID_KP (3.450 * 10)
#define SKIN_TEMP_PID_KI (0.380 * 10)
#define SKIN_TEMP_PID_KD (2.120 * 10)

// PID control parameters for humidity control
#define HUMIDITY_CTRL_BANG_WINDOW 10.0  // Use bang-bang control for errors > 10%
#define HUMIDITY_PID_KP 8.070
#define HUMIDITY_PID_KI 1.905
#define HUMIDITY_PID_KD 2.861
/* Control systems task */
// Task priorities for FreeRTOS tasks
#define CHECK_SAFETY_TASK_PRIORITY 20            // Highest priority for safety monitoring
#define CHECK_SAFETY_TASK_STACK_SIZE (1024 * 5)  // 5KB stack for safety task

// PID control task configuration
#define CTRL_PID_TASK_PRIORITY 10      // Normal priority for control tasks
#define CTRL_PID_TASK_SIZE (1024 * 5)  // 5KB stack for PID control task