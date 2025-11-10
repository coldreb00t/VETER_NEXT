/**
 * @file main.cpp
 * @brief ESP32-S3 Motor Controller - VETER_NEXT
 * Board: ESP32-S3-DevKitC-1 v1.0 (RGB LED on GPIO48)
 */

#include <Arduino.h>
#include <FastLED.h>
#include <SPI.h>
#include <mcp2515.h>
#include "CRSFforArduino.hpp"
#include "config.h"

// Status LED
CRGB leds[LED_COUNT];

// MCP2515 CAN controller
MCP2515 mcp2515(CAN_CS_PIN);

// CRSF (ExpressLRS) receiver
CRSFforArduino *crsf = nullptr;

// Emergency stop state
volatile bool emergency_stop_active = false;

// CRSF data
bool crsf_signal_valid = false;
uint32_t last_crsf_packet = 0;

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

    // Initialize SPI for MCP2515
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);
    Serial.println("[OK] SPI initialized");

    // Initialize MCP2515
    mcp2515.reset();
    MCP2515::ERROR can_error = mcp2515.setBitrate(CAN_BITRATE, MCP_8MHZ);
    if (can_error == MCP2515::ERROR_OK) {
        mcp2515.setNormalMode();
        Serial.println("[OK] MCP2515 CAN initialized (1 Mbps)");
    } else {
        Serial.printf("[ERROR] MCP2515 init failed: %d\n", can_error);
        leds[0] = CRGB::Red;
        FastLED.show();
    }

    // Initialize CRSF (ExpressLRS)
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    crsf = new CRSFforArduino(&Serial1);
    if (crsf != nullptr && crsf->begin()) {
        Serial.printf("[OK] CRSF initialized (RX: GPIO%d, 420000 baud)\n", CRSF_RX_PIN);
    } else {
        Serial.println("[WARN] CRSF initialization issue");
    }

    Serial.println("\n[READY] All systems initialized\n");
}

void loop() {
    static uint32_t count = 0;
    static uint32_t last_print = 0;

    // Check emergency stop
    emergency_stop_active = (digitalRead(EMERGENCY_STOP_PIN) == LOW);

    // Update CRSF
    if (crsf != nullptr) {
        crsf->update();
        if (crsf->getChannel(1) > 0) {
            crsf_signal_valid = true;
            last_crsf_packet = millis();
        }
    }

    // Check CRSF timeout
    if (millis() - last_crsf_packet > FAILSAFE_TIMEOUT_MS) {
        crsf_signal_valid = false;
    }

    // Update LED status based on system state
    if (emergency_stop_active) {
        leds[0] = CRGB::Red;  // E-Stop active
    } else if (!crsf_signal_valid && last_crsf_packet > 0) {
        leds[0] = CRGB::Yellow;  // No RC signal
    } else {
        // Normal operation - blink green
        leds[0] = (millis() % 1000 < 500) ? CRGB::Green : CRGB::Black;
    }
    FastLED.show();

    // Print status every second
    if (millis() - last_print >= 1000) {
        last_print = millis();

        Serial.printf("Loop #%d | E-Stop: %s | RC: %s",
                     count++,
                     emergency_stop_active ? "ACTIVE" : "OK",
                     crsf_signal_valid ? "OK" : "NO SIGNAL");

        if (crsf != nullptr) {
            // Read all available channels for diagnostics
            uint16_t ch1 = crsf->getChannel(1);  // Throttle (usually channel 1)
            uint16_t ch2 = crsf->getChannel(2);  // Steering (usually channel 2)
            uint16_t ch3 = crsf->getChannel(3);
            uint16_t ch4 = crsf->getChannel(4);

            Serial.printf(" | Ch[1-4]: %d %d %d %d", ch1, ch2, ch3, ch4);

            // Check if values are changing (indicates real signal)
            static uint16_t last_ch1 = 992;
            if (ch1 != last_ch1) {
                Serial.print(" [MOVING!]");
                last_ch1 = ch1;
            }
        }
        Serial.println();
    }

    delay(10);
}
