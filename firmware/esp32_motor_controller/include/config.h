/**
 * @file config.h
 * @brief Configuration file for VETER_NEXT ESP32 Motor Controller
 *
 * This file contains all hardware pin definitions, timing parameters,
 * and configuration constants for the motor controller firmware.
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

// CRSF/ExpressLRS Receiver
#define CRSF_RX_PIN     16      // UART RX from ExpressLRS
#define CRSF_TX_PIN     17      // UART TX to ExpressLRS (optional)
#define CRSF_BAUDRATE   420000  // ExpressLRS standard baudrate

// Emergency Stop
#define EMERGENCY_STOP_PIN  23  // Active LOW hardware emergency stop
#define ESTOP_DEBOUNCE_MS   50  // Debounce time in milliseconds

// Status LED (WS2812B)
#define LED_PIN         48      // Data pin for onboard RGB LED (GPIO48 on ESP32-S3-DevKitC-1 v1.0)
#define LED_COUNT       1       // Number of LEDs

// ============================================================================
// DRONECAN CONFIGURATION
// ============================================================================

#define DRONECAN_NODE_ID        10      // Unique node ID on CAN bus
#define DRONECAN_NODE_NAME      "ESP32_MotorController"
#define CAN_BITRATE             CAN_1000KBPS  // 1 Mbps for DroneCAN

// DroneCAN timing
#define HEARTBEAT_INTERVAL_MS   100     // Send heartbeat every 100ms
#define ESC_COMMAND_TIMEOUT_MS  1000    // Stop motors if no command for 1s

// ESC indices (matches VESC UAVCAN ESC Index)
#define ESC_LEFT_INDEX          0       // Left track motor
#define ESC_RIGHT_INDEX         1       // Right track motor

// ============================================================================
// CRSF CHANNEL MAPPING
// ============================================================================

// CRSF has 16 channels (0-15), typically:
// Ch 0: Roll (steering/differential)
// Ch 1: Pitch (throttle)
// Ch 2: Throttle (not used for ground vehicle)
// Ch 3: Yaw (not used)
// Ch 4: AUX1 (mode switch)
// Ch 5: AUX2 (emergency stop toggle)

#define CRSF_CHANNEL_THROTTLE   1   // Forward/backward
#define CRSF_CHANNEL_STEERING   0   // Left/right (differential)
#define CRSF_CHANNEL_MODE       4   // Control mode switch
#define CRSF_CHANNEL_ESTOP      5   // Emergency stop toggle

// CRSF value ranges (standard)
#define CRSF_MIN_VALUE          172     // Minimum stick value
#define CRSF_MID_VALUE          992     // Center stick value
#define CRSF_MAX_VALUE          1811    // Maximum stick value
#define CRSF_DEADBAND           50      // Deadband around center

// ============================================================================
// MOTOR CONTROL PARAMETERS
// ============================================================================

// ESC command range (DroneCAN uses 14-bit signed values)
#define ESC_COMMAND_MIN         -8191   // Full reverse
#define ESC_COMMAND_MAX         8191    // Full forward
#define ESC_COMMAND_NEUTRAL     0       // Stop

// Mixing parameters for differential steering
#define THROTTLE_SCALE          1.0f    // Throttle sensitivity (0.0-1.0)
#define STEERING_SCALE          0.7f    // Steering sensitivity (0.0-1.0)
#define MAX_DIFFERENTIAL        0.8f    // Max differential for turning (0.0-1.0)

// Safety limits
#define MAX_ACCELERATION        4000    // Max change per update (slew rate)
#define MAX_SPEED_PERCENT       100     // Maximum speed limit (0-100%)

// ============================================================================
// FAILSAFE CONFIGURATION
// ============================================================================

// Failsafe triggers
#define FAILSAFE_TIMEOUT_MS     1000    // No CRSF signal timeout
#define FAILSAFE_MIN_PACKETS    3       // Min packets before accepting input

// Failsafe actions
enum FailsafeMode {
    FAILSAFE_NONE = 0,          // Normal operation
    FAILSAFE_NO_SIGNAL,         // No RC signal
    FAILSAFE_EMERGENCY_STOP,    // Hardware E-stop triggered
    FAILSAFE_LOW_BATTERY,       // Battery critically low
    FAILSAFE_CAN_ERROR          // CAN bus communication error
};

// ============================================================================
// LED STATUS INDICATORS
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

// Status indication
#define LED_BOOT            LED_BLUE        // Booting up
#define LED_NORMAL          LED_GREEN       // Normal operation
#define LED_NO_RC           LED_YELLOW      // No RC signal
#define LED_ESTOP           LED_RED         // Emergency stop
#define LED_CAN_ERROR       LED_MAGENTA     // CAN error
#define LED_FAILSAFE        LED_ORANGE      // Other failsafe

// LED blink patterns (milliseconds)
#define LED_BLINK_FAST      100
#define LED_BLINK_NORMAL    250
#define LED_BLINK_SLOW      500

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

#define MAIN_LOOP_RATE_HZ       100     // Main loop rate (100 Hz = 10ms)
#define MAIN_LOOP_PERIOD_MS     (1000 / MAIN_LOOP_RATE_HZ)

#define CRSF_UPDATE_RATE_HZ     50      // CRSF processing rate
#define CRSF_UPDATE_PERIOD_MS   (1000 / CRSF_UPDATE_RATE_HZ)

#define STATUS_UPDATE_RATE_HZ   10      // Status LED update rate
#define STATUS_UPDATE_PERIOD_MS (1000 / STATUS_UPDATE_RATE_HZ)

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Enable/disable debug output
#define DEBUG_ENABLED           1
#define DEBUG_CRSF              0       // Print CRSF channel values
#define DEBUG_ESC_COMMANDS      1       // Print ESC commands
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
