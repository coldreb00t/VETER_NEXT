/**
 * @file config.h
 * @brief Configuration file for VETER_NEXT ESP32 Sensor Hub
 *
 * This file contains all hardware pin definitions, sensor parameters,
 * and configuration constants for the sensor hub firmware.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// CAN Bus (MCP2515 SPI interface)
#define CAN_CS_PIN      5       // Chip Select
#define CAN_INT_PIN     4       // Interrupt pin
#define SPI_MOSI_PIN    11      // SPI MOSI
#define SPI_MISO_PIN    13      // SPI MISO
#define SPI_SCK_PIN     12      // SPI SCK

// HC-SR04 Ultrasonic Sensors (collision avoidance)
// Front sensor
#define SONAR_FRONT_TRIG_PIN    14
#define SONAR_FRONT_ECHO_PIN    15
// Rear sensor
#define SONAR_REAR_TRIG_PIN     16
#define SONAR_REAR_ECHO_PIN     17
// Left sensor
#define SONAR_LEFT_TRIG_PIN     18
#define SONAR_LEFT_ECHO_PIN     21
// Right sensor
#define SONAR_RIGHT_TRIG_PIN    47
#define SONAR_RIGHT_ECHO_PIN    48

// BME280 Sensor (I2C - temperature, humidity, pressure)
#define BME280_SDA_PIN          8       // I2C Data
#define BME280_SCL_PIN          9       // I2C Clock
#define BME280_I2C_ADDR         0x76    // Default I2C address (or 0x77)

// Camera Servo Control (PWM)
#define SERVO_PAN_PIN           38      // Horizontal rotation
#define SERVO_TILT_PIN          39      // Vertical tilt

// LED Lighting (PWM)
#define LED_FRONT_LEFT_PIN      40      // Front left LED
#define LED_FRONT_RIGHT_PIN     41      // Front right LED
#define LED_REAR_LEFT_PIN       42      // Rear left LED
#define LED_REAR_RIGHT_PIN      45      // Rear right LED

// Emergency Stop (shared with motor controller via CAN)
#define EMERGENCY_STOP_PIN      23      // Active LOW hardware emergency stop
#define ESTOP_DEBOUNCE_MS       50      // Debounce time in milliseconds

// Status LED (WS2812B)
#define LED_PIN                 19      // Data pin for WS2812B LED strip
#define LED_COUNT               1       // Number of LEDs

// ============================================================================
// DRONECAN CONFIGURATION
// ============================================================================

#define DRONECAN_NODE_ID        11      // Unique node ID on CAN bus
#define DRONECAN_NODE_NAME      "ESP32_SensorHub"
#define CAN_BITRATE             CAN_1000KBPS  // 1 Mbps for DroneCAN

// DroneCAN timing
#define HEARTBEAT_INTERVAL_MS   100     // Send heartbeat every 100ms
#define SENSOR_PUBLISH_RATE_MS  100     // Publish sensor data every 100ms

// ============================================================================
// ULTRASONIC SENSOR CONFIGURATION
// ============================================================================

#define SONAR_COUNT             4       // Number of ultrasonic sensors
#define SONAR_MAX_DISTANCE_CM   400     // Maximum distance to measure (cm)
#define SONAR_MIN_DISTANCE_CM   2       // Minimum reliable distance (cm)
#define SONAR_TIMEOUT_US        25000   // Timeout for echo (25ms for 400cm)

// Collision thresholds
#define SONAR_COLLISION_WARNING_CM  50  // Warning distance
#define SONAR_COLLISION_STOP_CM     20  // Emergency stop distance

// Sensor indices for DroneCAN messages
enum SonarIndex {
    SONAR_FRONT = 0,
    SONAR_REAR = 1,
    SONAR_LEFT = 2,
    SONAR_RIGHT = 3
};

// ============================================================================
// BME280 SENSOR CONFIGURATION
// ============================================================================

// Sampling rates (see BME280 datasheet)
#define BME280_SAMPLING_TEMP    BME280::SAMPLING_X16
#define BME280_SAMPLING_PRES    BME280::SAMPLING_X16
#define BME280_SAMPLING_HUM     BME280::SAMPLING_X16

// Sensor mode and filter
#define BME280_MODE             BME280::MODE_NORMAL
#define BME280_FILTER           BME280::FILTER_X16
#define BME280_STANDBY          BME280::STANDBY_MS_500

// Reading intervals
#define BME280_READ_INTERVAL_MS 1000    // Read every 1 second

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// Servo PWM parameters
#define SERVO_FREQ_HZ           50      // Standard servo frequency
#define SERVO_MIN_US            500     // Minimum pulse width (microseconds)
#define SERVO_MAX_US            2500    // Maximum pulse width (microseconds)

// Servo position limits (degrees)
#define SERVO_PAN_MIN_DEG       0
#define SERVO_PAN_MAX_DEG       180
#define SERVO_PAN_CENTER_DEG    90

#define SERVO_TILT_MIN_DEG      30      // Don't point camera too low
#define SERVO_TILT_MAX_DEG      150     // Don't point camera too high
#define SERVO_TILT_CENTER_DEG   90

// Servo control via DroneCAN or RC passthrough
#define SERVO_CONTROL_TIMEOUT_MS    1000    // Revert to center if no command

// ============================================================================
// LED LIGHTING CONFIGURATION
// ============================================================================

// LED PWM parameters
#define LED_PWM_FREQ_HZ         5000    // 5 kHz PWM frequency
#define LED_PWM_RESOLUTION      8       // 8-bit resolution (0-255)
#define LED_PWM_CHANNEL_FL      0       // PWM channel for front left
#define LED_PWM_CHANNEL_FR      1       // PWM channel for front right
#define LED_PWM_CHANNEL_RL      2       // PWM channel for rear left
#define LED_PWM_CHANNEL_RR      3       // PWM channel for rear right

// LED brightness levels (0-255)
#define LED_BRIGHTNESS_OFF      0
#define LED_BRIGHTNESS_LOW      64
#define LED_BRIGHTNESS_MED      128
#define LED_BRIGHTNESS_HIGH     255

// Default LED modes
enum LEDMode {
    LED_MODE_OFF = 0,
    LED_MODE_AUTO,          // Automatic based on ambient light
    LED_MODE_ON,            // Always on
    LED_MODE_BLINK,         // Blinking pattern
    LED_MODE_RUNNING        // Running light pattern
};

#define LED_DEFAULT_MODE        LED_MODE_AUTO
#define LED_DEFAULT_BRIGHTNESS  LED_BRIGHTNESS_MED

// ============================================================================
// FAILSAFE CONFIGURATION
// ============================================================================

// Failsafe triggers
#define FAILSAFE_TIMEOUT_MS     1000    // No DroneCAN command timeout
#define FAILSAFE_SENSOR_ERROR_MAX   3   // Max consecutive sensor errors

// Failsafe actions
enum FailsafeMode {
    FAILSAFE_NONE = 0,          // Normal operation
    FAILSAFE_NO_SIGNAL,         // No CAN signal
    FAILSAFE_EMERGENCY_STOP,    // Hardware E-stop triggered
    FAILSAFE_SENSOR_ERROR,      // Critical sensor failure
    FAILSAFE_COLLISION          // Imminent collision detected
};

// ============================================================================
// LED STATUS INDICATORS (WS2812B)
// ============================================================================

// LED colors (RGB)
#define LED_OFF             0x000000
#define LED_RED             0xFF0000
#define LED_GREEN           0x00FF00
#define LED_BLUE            0x0000FF
#define LED_YELLOW          0xFFFF00
#define LED_CYAN            0x00FFFF
#define LED_MAGENTA         0xFF00FF
#define LED_WHITE           0xFFFFFF
#define LED_ORANGE          0xFF8000
#define LED_PURPLE          0x8000FF

// Status indication
#define LED_BOOT            LED_BLUE        // Booting up
#define LED_NORMAL          LED_GREEN       // Normal operation
#define LED_NO_SIGNAL       LED_YELLOW      // No CAN signal
#define LED_ESTOP           LED_RED         // Emergency stop
#define LED_CAN_ERROR       LED_MAGENTA     // CAN error
#define LED_SENSOR_ERROR    LED_ORANGE      // Sensor error
#define LED_COLLISION       LED_PURPLE      // Collision warning

// LED blink patterns (milliseconds)
#define LED_BLINK_FAST      100
#define LED_BLINK_NORMAL    250
#define LED_BLINK_SLOW      500

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

#define MAIN_LOOP_RATE_HZ       100     // Main loop rate (100 Hz = 10ms)
#define MAIN_LOOP_PERIOD_MS     (1000 / MAIN_LOOP_RATE_HZ)

#define SONAR_UPDATE_RATE_HZ    10      // Ultrasonic sensor update rate
#define SONAR_UPDATE_PERIOD_MS  (1000 / SONAR_UPDATE_RATE_HZ)

#define STATUS_UPDATE_RATE_HZ   10      // Status LED update rate
#define STATUS_UPDATE_PERIOD_MS (1000 / STATUS_UPDATE_RATE_HZ)

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Enable/disable debug output
#define DEBUG_ENABLED           1
#define DEBUG_SONAR             0       // Print ultrasonic readings
#define DEBUG_BME280            0       // Print BME280 readings
#define DEBUG_SERVO             0       // Print servo positions
#define DEBUG_LED               0       // Print LED states
#define DEBUG_DRONECAN          0       // Print DroneCAN messages
#define DEBUG_TIMING            0       // Print timing information

// Debug macros
#if DEBUG_ENABLED
    #define DEBUG_PRINT(x)      Serial.print(x)
    #define DEBUG_PRINTLN(x)    Serial.println(x)
    #define DEBUG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

// ============================================================================
// VERSION INFORMATION
// ============================================================================

#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
#define FIRMWARE_BUILD_DATE     __DATE__
#define FIRMWARE_BUILD_TIME     __TIME__

#endif // CONFIG_H
