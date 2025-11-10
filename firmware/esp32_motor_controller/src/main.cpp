/**
 * @file main.cpp
 * @brief ESP32-S3 Motor Controller - VETER_NEXT
 * Board: ESP32-S3-DevKitC-1 v1.0 (RGB LED on GPIO48)
 */

#include <Arduino.h>
#include <FastLED.h>
#include "CRSFforArduino.hpp"
#include "config.h"
#include "dronecan_interface.h"
#include "motor_mixing.h"

// Status LED
CRGB leds[LED_COUNT];

// CRSF (ExpressLRS) receiver
CRSFforArduino *crsf = nullptr;

// Emergency stop state
volatile bool emergency_stop_active = false;

// CRSF data (12 channels as specified by user)
#define CRSF_NUM_CHANNELS 12
uint16_t crsf_channels[CRSF_NUM_CHANNELS] = {0};
bool crsf_signal_valid = false;
uint32_t last_crsf_packet = 0;

// Failsafe state
enum FailsafeMode current_failsafe = FAILSAFE_NONE;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==========================================");
    Serial.println("VETER_NEXT ESP32 Motor Controller");
    Serial.printf("Firmware v%d.%d.%d\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    Serial.printf("Build: %s %s\n", FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME);
    Serial.println("==========================================");

    Serial.printf("LED_PIN: %d\n", LED_PIN);
    Serial.printf("Emergency Stop PIN: %d\n", EMERGENCY_STOP_PIN);
    Serial.printf("DroneCAN Node ID: %d\n", DRONECAN_NODE_ID);
    Serial.println();

    // Initialize FastLED (GPIO48 for v1.0 board)
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Blue;
    FastLED.show();
    Serial.println("[OK] FastLED initialized");

    // Initialize Emergency Stop pin
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    emergency_stop_active = (digitalRead(EMERGENCY_STOP_PIN) == LOW);
    Serial.printf("[OK] Emergency Stop Pin initialized (State: %s)\n",
                  emergency_stop_active ? "ACTIVE" : "INACTIVE");

    // Initialize CRSF (ExpressLRS)
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    crsf = new CRSFforArduino(&Serial1);
    if (crsf != nullptr && crsf->begin()) {
        Serial.printf("[OK] CRSF initialized (RX: GPIO%d, 420000 baud, %d channels)\n",
                     CRSF_RX_PIN, CRSF_NUM_CHANNELS);
    } else {
        Serial.println("[WARN] CRSF initialization issue");
    }

    // Initialize DroneCAN
    if (DroneCAN_Init()) {
        Serial.println("[OK] DroneCAN initialized");
    } else {
        Serial.println("[ERROR] DroneCAN init failed");
    }

    // Initialize motor mixing
    MotorMixing_Init();
    Serial.println("[OK] Motor mixing initialized");

    Serial.println("\n[READY] All systems initialized\n");
    Serial.println("===========================================");
    Serial.println("Motor Controller Active - Waiting for RC");
    Serial.println("===========================================\n");
}

void loop() {
    static uint32_t count = 0;
    static uint32_t last_print = 0;
    static uint32_t last_motor_update = 0;

    // ========================================
    // 1. Check emergency stop
    // ========================================
    emergency_stop_active = (digitalRead(EMERGENCY_STOP_PIN) == LOW);

    // ========================================
    // 2. Update CRSF and read all 12 channels
    // ========================================
    if (crsf != nullptr) {
        crsf->update();

        // Read all 12 channels
        for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
            crsf_channels[i] = crsf->getChannel(i + 1);  // CRSF channels are 1-indexed
        }

        // Check if we have valid CRSF data (channel 1 should be non-zero)
        if (crsf_channels[0] > 0) {
            crsf_signal_valid = true;
            last_crsf_packet = millis();
        }
    }

    // ========================================
    // 3. Check failsafe conditions
    // ========================================
    if (emergency_stop_active) {
        current_failsafe = FAILSAFE_EMERGENCY_STOP;
    } else if (millis() - last_crsf_packet > FAILSAFE_TIMEOUT_MS) {
        current_failsafe = FAILSAFE_NO_SIGNAL;
        crsf_signal_valid = false;
    } else {
        current_failsafe = FAILSAFE_NONE;
    }

    // ========================================
    // 4. Calculate motor commands
    // ========================================
    MotorCommands motor_cmd = {0, 0};

    if (current_failsafe == FAILSAFE_NONE && crsf_signal_valid) {
        // Normal operation: calculate motor commands from CRSF
        uint16_t throttle = crsf_channels[CRSF_CHANNEL_THROTTLE];
        uint16_t steering = crsf_channels[CRSF_CHANNEL_STEERING];

        motor_cmd = MotorMixing_Calculate(throttle, steering, false);
    } else {
        // Failsafe: stop motors
        motor_cmd = MotorMixing_Calculate(0, 0, true);
    }

    // ========================================
    // 5. Send motor commands via DroneCAN (10ms rate = 100Hz)
    // ========================================
    if (millis() - last_motor_update >= MAIN_LOOP_PERIOD_MS) {
        last_motor_update = millis();

        DroneCAN_SendESCCommand(motor_cmd.left, motor_cmd.right);
    }

    // ========================================
    // 6. Process DroneCAN (heartbeat, etc.)
    // ========================================
    DroneCAN_Process();

    // Update DroneCAN health status
    if (current_failsafe == FAILSAFE_EMERGENCY_STOP) {
        DroneCAN_SetHealth(3);  // CRITICAL
    } else if (current_failsafe == FAILSAFE_NO_SIGNAL) {
        DroneCAN_SetHealth(2);  // ERROR
    } else {
        DroneCAN_SetHealth(0);  // OK
    }

    // ========================================
    // 7. Update LED status based on movement direction
    // ========================================
    // TEST MODE: Show direction even with E-Stop for debugging
    if (current_failsafe == FAILSAFE_NO_SIGNAL) {
        // No signal: Blink yellow
        leds[0] = ((millis() / LED_BLINK_NORMAL) % 2) ? CRGB::Yellow : CRGB::Black;
    } else if (crsf_signal_valid) {
        // Show direction based on CRSF channels (even if E-Stop active)
        uint16_t throttle = crsf_channels[CRSF_CHANNEL_THROTTLE];
        uint16_t steering = crsf_channels[CRSF_CHANNEL_STEERING];

        // Calculate normalized values to detect direction
        int16_t throttle_delta = throttle - CRSF_MID_VALUE;
        int16_t steering_delta = steering - CRSF_MID_VALUE;

        // Priority: Steering (left/right) > Throttle forward > Throttle backward > Neutral
        if (abs(steering_delta) > (int16_t)CRSF_DEADBAND) {
            // Turning left or right: Blue
            leds[0] = CRGB::Blue;
        } else if (throttle_delta > (int16_t)CRSF_DEADBAND) {
            // Moving forward: Green
            leds[0] = CRGB::Green;
        } else if (throttle_delta < -(int16_t)CRSF_DEADBAND) {
            // Moving backward: Red
            leds[0] = CRGB::Red;
        } else {
            // Neutral (all sticks centered): White
            leds[0] = CRGB::White;
        }

        // Dim LED if E-Stop active (but still show direction)
        if (emergency_stop_active) {
            leds[0].fadeToBlackBy(128);  // 50% brightness when E-Stop
        }
    } else {
        // Initializing: Blink blue
        leds[0] = ((millis() / LED_BLINK_SLOW) % 2) ? CRGB::Blue : CRGB::Black;
    }
    FastLED.show();

    // ========================================
    // 8. Debug output (every second)
    // ========================================
    if (millis() - last_print >= 1000) {
        last_print = millis();

        Serial.printf("Loop #%lu | E-Stop: %s | RC: %s | FS: %d | Motors: L=%d R=%d",
                     count++,
                     emergency_stop_active ? "ACTIVE" : "OK",
                     crsf_signal_valid ? "OK" : "NO_SIG",
                     current_failsafe,
                     motor_cmd.left,
                     motor_cmd.right);

        // Print all 12 CRSF channels
        #if DEBUG_CRSF
        Serial.print(" | Ch: ");
        for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
            Serial.printf("%d ", crsf_channels[i]);
        }
        #else
        // Print only throttle and steering
        Serial.printf(" | Thr=%d Str=%d",
                     crsf_channels[CRSF_CHANNEL_THROTTLE],
                     crsf_channels[CRSF_CHANNEL_STEERING]);
        #endif

        Serial.println();
    }

    delay(5);  // Small delay to prevent watchdog timeout
}
