/**
 * @file main.cpp
 * @brief VETER_NEXT ESP32 Motor Controller Main Application
 *
 * This firmware receives CRSF commands from ExpressLRS receiver
 * and converts them to DroneCAN ESC commands for VESC motor controllers.
 *
 * Features:
 * - CRSF/ExpressLRS receiver input
 * - DroneCAN motor command output
 * - Hardware emergency stop
 * - Failsafe handling
 * - Status LED indication
 */

#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "dronecan_interface.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// CRSF channel values (16 channels)
static uint16_t crsf_channels[16] = {CRSF_MID_VALUE};
static uint32_t last_crsf_packet_ms = 0;
static uint32_t crsf_packet_count = 0;

// Emergency stop state
static bool emergency_stop_active = false;
static uint32_t last_estop_check_ms = 0;

// Failsafe state
static FailsafeMode failsafe_mode = FAILSAFE_NONE;

// Status LED
CRGB leds[LED_COUNT];
static uint32_t last_led_update_ms = 0;
static bool led_state = false;

// Motor commands (after mixing)
static int16_t motor_left_cmd = 0;
static int16_t motor_right_cmd = 0;

// Timing
static uint32_t last_loop_time_ms = 0;

// ============================================================================
// CRSF PROTOCOL FUNCTIONS
// ============================================================================

/**
 * @brief Parse CRSF packet from serial
 * @return true if valid packet received
 */
bool CRSF_ParsePacket(void) {
    // Simple CRSF parser (simplified version)
    // Full implementation would use CRServoF library

    // Check if data available
    if (CRSF_SERIAL.available() < 25) {
        return false;
    }

    // Look for sync byte (0xC8)
    uint8_t sync = CRSF_SERIAL.read();
    if (sync != 0xC8) {
        return false;
    }

    // Read packet length
    uint8_t len = CRSF_SERIAL.read();
    if (len != 24) {  // RC channels packet
        // Flush remaining bytes
        while (len-- > 0 && CRSF_SERIAL.available()) {
            CRSF_SERIAL.read();
        }
        return false;
    }

    // Read packet type
    uint8_t type = CRSF_SERIAL.read();
    if (type != 0x16) {  // RC channels frame
        // Flush remaining bytes
        len -= 1;
        while (len-- > 0 && CRSF_SERIAL.available()) {
            CRSF_SERIAL.read();
        }
        return false;
    }

    // Read 16 channels (11 bits each, packed)
    uint8_t payload[22];
    if (CRSF_SERIAL.readBytes(payload, 22) != 22) {
        return false;
    }

    // Read CRC
    uint8_t crc = CRSF_SERIAL.read();
    // TODO: Verify CRC

    // Unpack 11-bit channels
    crsf_channels[0] = ((payload[0] | payload[1] << 8) & 0x07FF);
    crsf_channels[1] = ((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
    crsf_channels[2] = ((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
    // ... (continue for all 16 channels as needed)

    // Update timestamp and counter
    last_crsf_packet_ms = millis();
    crsf_packet_count++;

    return true;
}

/**
 * @brief Convert CRSF channel value to normalized float (-1.0 to 1.0)
 */
float CRSF_ChannelToFloat(uint16_t channel_value) {
    // Map from CRSF range to -1.0 to 1.0
    float normalized = (float)(channel_value - CRSF_MIN_VALUE) /
                      (float)(CRSF_MAX_VALUE - CRSF_MIN_VALUE);
    normalized = (normalized - 0.5f) * 2.0f;  // Convert 0-1 to -1 to 1

    // Apply deadband around center
    if (abs(normalized) < (float)CRSF_DEADBAND / (float)(CRSF_MAX_VALUE - CRSF_MIN_VALUE)) {
        normalized = 0.0f;
    }

    return constrain(normalized, -1.0f, 1.0f);
}

// ============================================================================
// MOTOR MIXING AND CONTROL
// ============================================================================

/**
 * @brief Mix throttle and steering into differential motor commands
 */
void MixMotorCommands(float throttle, float steering) {
    // Apply scaling
    throttle *= THROTTLE_SCALE;
    steering *= STEERING_SCALE;

    // Differential mixing
    float left = throttle + (steering * MAX_DIFFERENTIAL);
    float right = throttle - (steering * MAX_DIFFERENTIAL);

    // Constrain to -1.0 to 1.0
    left = constrain(left, -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    // Convert to ESC command range
    motor_left_cmd = (int16_t)(left * ESC_COMMAND_MAX);
    motor_right_cmd = (int16_t)(right * ESC_COMMAND_MAX);

    // Apply speed limit
    if (MAX_SPEED_PERCENT < 100) {
        motor_left_cmd = (motor_left_cmd * MAX_SPEED_PERCENT) / 100;
        motor_right_cmd = (motor_right_cmd * MAX_SPEED_PERCENT) / 100;
    }
}

// ============================================================================
// EMERGENCY STOP AND FAILSAFE
// ============================================================================

/**
 * @brief Check hardware emergency stop button
 */
void CheckEmergencyStop(void) {
    uint32_t now = millis();

    if (now - last_estop_check_ms < ESTOP_DEBOUNCE_MS) {
        return;
    }
    last_estop_check_ms = now;

    // Read E-stop pin (active LOW)
    bool estop_pressed = (digitalRead(EMERGENCY_STOP_PIN) == LOW);

    if (estop_pressed && !emergency_stop_active) {
        emergency_stop_active = true;
        failsafe_mode = FAILSAFE_EMERGENCY_STOP;
        DEBUG_PRINTLN("[ESTOP] Emergency stop activated!");
        DroneCAN_SetHealth(3);  // CRITICAL
    } else if (!estop_pressed && emergency_stop_active) {
        emergency_stop_active = false;
        if (failsafe_mode == FAILSAFE_EMERGENCY_STOP) {
            failsafe_mode = FAILSAFE_NONE;
            DroneCAN_SetHealth(0);  // OK
        }
        DEBUG_PRINTLN("[ESTOP] Emergency stop released");
    }
}

/**
 * @brief Check for RC signal loss
 */
void CheckRCFailsafe(void) {
    uint32_t now = millis();
    uint32_t time_since_packet = now - last_crsf_packet_ms;

    // Check for signal loss
    if (time_since_packet > FAILSAFE_TIMEOUT_MS) {
        if (failsafe_mode == FAILSAFE_NONE) {
            failsafe_mode = FAILSAFE_NO_SIGNAL;
            DEBUG_PRINTLN("[FAILSAFE] RC signal lost!");
            DroneCAN_SetHealth(2);  // ERROR
        }
    } else {
        if (failsafe_mode == FAILSAFE_NO_SIGNAL && crsf_packet_count >= FAILSAFE_MIN_PACKETS) {
            failsafe_mode = FAILSAFE_NONE;
            DEBUG_PRINTLN("[FAILSAFE] RC signal restored");
            DroneCAN_SetHealth(0);  // OK
        }
    }
}

// ============================================================================
// STATUS LED
// ============================================================================

/**
 * @brief Update status LED based on system state
 */
void UpdateStatusLED(void) {
    uint32_t now = millis();

    if (now - last_led_update_ms < STATUS_UPDATE_PERIOD_MS) {
        return;
    }
    last_led_update_ms = now;

    uint32_t color = LED_OFF;
    uint32_t blink_period = LED_BLINK_NORMAL;

    // Determine LED color and pattern based on state
    if (failsafe_mode == FAILSAFE_EMERGENCY_STOP) {
        color = LED_ESTOP;
        blink_period = LED_BLINK_FAST;
    } else if (failsafe_mode == FAILSAFE_NO_SIGNAL) {
        color = LED_NO_RC;
        blink_period = LED_BLINK_NORMAL;
    } else if (failsafe_mode != FAILSAFE_NONE) {
        color = LED_FAILSAFE;
        blink_period = LED_BLINK_SLOW;
    } else {
        color = LED_NORMAL;
        blink_period = 0;  // Solid on
    }

    // Apply blink pattern
    if (blink_period > 0) {
        led_state = ((now / blink_period) % 2) == 0;
    } else {
        led_state = true;  // Solid on
    }

    // Set LED color
    if (led_state) {
        leds[0] = color;
    } else {
        leds[0] = CRGB::Black;
    }

    FastLED.show();
}

// ============================================================================
// SETUP AND MAIN LOOP
// ============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(100);

    DEBUG_PRINTLN("\n\n==========================================");
    DEBUG_PRINTLN("VETER_NEXT ESP32 Motor Controller");
    DEBUG_PRINTF("Firmware v%d.%d.%d\n", FIRMWARE_VERSION_MAJOR,
                 FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    DEBUG_PRINTF("Build: %s %s\n", FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME);
    DEBUG_PRINTLN("==========================================\n");

    // Initialize pins
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // Initialize LED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(50);
    leds[0] = LED_BOOT;
    FastLED.show();

    // Initialize CRSF serial
    CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    DEBUG_PRINTF("[CRSF] Initialized on pins RX=%d TX=%d at %d baud\n",
                 CRSF_RX_PIN, CRSF_TX_PIN, CRSF_BAUDRATE);

    // Initialize DroneCAN
    if (!DroneCAN_Init()) {
        DEBUG_PRINTLN("[ERROR] DroneCAN initialization failed!");
        leds[0] = LED_CAN_ERROR;
        FastLED.show();
        while (1) { delay(1000); }  // Halt
    }

    DEBUG_PRINTLN("[INIT] System initialized successfully");
    DEBUG_PRINTLN("[INIT] Entering main loop...\n");

    last_loop_time_ms = millis();
}

void loop() {
    uint32_t now = millis();

    // Main loop timing control
    if (now - last_loop_time_ms < MAIN_LOOP_PERIOD_MS) {
        return;
    }
    last_loop_time_ms = now;

    // Check emergency stop
    CheckEmergencyStop();

    // Process CRSF input
    while (CRSF_SERIAL.available()) {
        if (CRSF_ParsePacket()) {
            #if DEBUG_CRSF
            DEBUG_PRINTF("[CRSF] Throttle=%d Steering=%d\n",
                        crsf_channels[CRSF_CHANNEL_THROTTLE],
                        crsf_channels[CRSF_CHANNEL_STEERING]);
            #endif
        }
    }

    // Check failsafe conditions
    CheckRCFailsafe();

    // Process motor commands if not in failsafe
    if (failsafe_mode == FAILSAFE_NONE) {
        // Get normalized inputs
        float throttle = CRSF_ChannelToFloat(crsf_channels[CRSF_CHANNEL_THROTTLE]);
        float steering = CRSF_ChannelToFloat(crsf_channels[CRSF_CHANNEL_STEERING]);

        // Mix and send motor commands
        MixMotorCommands(throttle, steering);
        DroneCAN_SendESCCommand(motor_left_cmd, motor_right_cmd);
    } else {
        // Failsafe: stop motors
        DroneCAN_StopMotors();
    }

    // Send DroneCAN heartbeat
    DroneCAN_SendHeartbeat();

    // Process incoming DroneCAN messages
    DroneCAN_Process();

    // Update status LED
    UpdateStatusLED();
}
