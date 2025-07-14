# ASTRA Baby Incubator Controller

A control system application for baby incubators. The system provides precise temperature and humidity control with safety features following MK 011-18 Kementrian Kesehatan Republik Indonesia.

## Features

- **Temperature Control**
  - Air temperature control for incubator environment
  - Skin temperature monitoring for direct baby temperature control
  - Hybrid control system (Bang-Bang + PID) for optimal response

- **Humidity Control**
  - Precise humidity regulation
  - Water level monitoring
  - Automatic humidity heater control

- **Safety Features**
  - Temperature limits monitoring
  - Current monitoring for heaters
  - Fan stall detection
  - Battery voltage monitoring
  - Automatic shutdown on critical failures
  - Power loss alert
  - Sensor failure detection

- **User Interface**
  - Nextion HMI display integration
  - Real-time sensor data display
  - System status monitoring and alarms
  - Configuration interface
  - Alarm management system

## Technical Specifications

### Control System
- **Temperature Control**
  - Control Range: 30.0°C - 39.5°C
  - Accuracy: ±0.1°C
  - Response Time: < 2 minutes

- **Humidity Control**
  - Control Range: 20% - 99% RH
  - Accuracy: ±2% RH
  - Response Time: < 5 minutes

- **System Performance**
  - Control Update Rate: 5 seconds
  - Safety Check Rate: 1 second
  - Maximum Response Time: 100ms

### Hardware Requirements

**Controllers**: Astra Baby Incubator Controller Rev.1

### Software Requirements

- **Development Environment**
  - PlatformIO IDE or VSCode with PlatformIO extension
  - Platform Espressif 32 (Check PIO Home)
  - Git for version control

- **Required Libraries**: See `platformio.ini`

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your-username/astra-baby-incubator-controller.git
cd astra-baby-incubator-controller
```

2. Install PlatformIO:
   - For VSCode: Install the PlatformIO extension
   - For standalone: Follow instructions at [PlatformIO Installation](https://platformio.org/install)

3. Configure your development environment:
   - Install platform Espressif 32 on the PIO Home menu
   - Select serial COM port
   - Build and flash!

## Building the Project

1. Open the project in PlatformIO:
   - VSCode: Open the project folder
   - PlatformIO IDE: Import the project

2. Configure the project:
   - Select your ESP32 board in `platformio.ini`
   - Verify the upload port settings

3. Build the project:
```bash
pio run
```

4. Upload to your device:
```bash
pio run --target upload
```

5. Monitor serial output:
```bash
pio device monitor
```

OR, use the PIO GUI on the vscode!

## Configuration

The system can be configured through the following files:

- `platformio.ini`: Build and upload settings
  - Board configuration (ESP32)
  - Framework settings (Arduino)
  - Library dependencies
  - Build flags and optimization settings

- `HardwareConfig.h`: Hardware pin definitions and configurations
  - GPIO pin assignments for sensors and actuators
  - I2C and UART configurations
  - Hardware-specific timing parameters
  - Sensor calibration values

- `ControlSystemConstants.h`: System parameters and thresholds
  - PID control parameters
  - Safety thresholds
  - Timing parameters
  - Task priorities

- `HMI_commands.h`: Nextion HMI display commands and interface
  - Display page definitions
  - Command codes for HMI communication
  - UI element IDs and names
  - Display update protocols

## Safety Considerations

The system implements multiple safety features:

1. Temperature limits:
   - Air temperature: 30.0°C - 39.5°C
   - Skin temperature: 30.0°C - 39.5°C

2. Current monitoring:
   - Cabin heater: 0.05A - 4.0A
   - Humidity heater: 0.05A - 3.0A

3. Humidity limits:
   - Operating range: 20% - 99%

4. Fan monitoring:
   - Stall detection at 500 RPM
   - Default duty cycle: 60%

## Possible Future Works
  - Self-diagnostic program
  - Data logging and export