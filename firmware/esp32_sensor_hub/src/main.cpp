/**
 * @file main.cpp
 * @brief VETER_NEXT ESP32 Sensor Hub Main Application
 *
 * This firmware manages all sensors and actuators for the robot:
 * - 4x HC-SR04 ultrasonic sensors (collision avoidance)
 * - BME280 temperature/humidity/pressure sensor
 * - Camera servo control (pan/tilt)
 * - LED lighting control
 * - DroneCAN sensor data publishing
 *
 * Features:
 * - Real-time collision detection
 * - Environmental monitoring
 * - Remote camera control
 * - Adaptive lighting
 * - Status indication
 */

#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "config.h"
#include "dronecan_interface.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Ultrasonic sensors (HC-SR04)
NewPing sonar[SONAR_COUNT] = {
    NewPing(SONAR_FRONT_TRIG_PIN, SONAR_FRONT_ECHO_PIN, SONAR_MAX_DISTANCE_CM),
    NewPing(SONAR_REAR_TRIG_PIN, SONAR_REAR_ECHO_PIN, SONAR_MAX_DISTANCE_CM),
    NewPing(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_MAX_DISTANCE_CM),
    NewPing(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_MAX_DISTANCE_CM)
};

static SonarData sonar_data;
static uint32_t last_sonar_update_ms = 0;
static uint8_t sonar_error_count = 0;

// BME280 sensor
Adafruit_BME280 bme;
static BME280Data bme_data;
static uint32_t last_bme_update_ms = 0;
static bool bme_initialized = false;

// Camera servos
Servo servo_pan;
Servo servo_tilt;
static uint16_t servo_pan_pos = SERVO_PAN_CENTER_DEG;
static uint16_t servo_tilt_pos = SERVO_TILT_CENTER_DEG;

// LED lighting (PWM)
static LEDCommand led_command;
static uint8_t led_brightness[4] = {0, 0, 0, 0};  // FL, FR, RL, RR

// Emergency stop state
static bool emergency_stop_active = false;
static uint32_t last_estop_check_ms = 0;

// Failsafe state
static FailsafeMode failsafe_mode = FAILSAFE_NONE;

// Status LED
CRGB leds[LED_COUNT];
static uint32_t last_led_update_ms = 0;
static bool led_state = false;

// Timing
static uint32_t last_loop_time_ms = 0;
static uint32_t last_publish_ms = 0;

// ============================================================================
// ULTRASONIC SENSOR FUNCTIONS
// ============================================================================

/**
 * @brief Read all ultrasonic sensors
 */
void UpdateUltrasonicSensors(void) {
    uint32_t now = millis();

    if (now - last_sonar_update_ms < SONAR_UPDATE_PERIOD_MS) {
        return;
    }
    last_sonar_update_ms = now;

    bool any_error = false;

    // Read each sensor
    for (int i = 0; i < SONAR_COUNT; i++) {
        unsigned int distance_cm = sonar[i].ping_cm();

        if (distance_cm == 0) {
            // Sensor error or out of range
            sonar_data.valid[i] = false;
            sonar_data.distance_cm[i] = SONAR_MAX_DISTANCE_CM;
            any_error = true;
        } else {
            sonar_data.valid[i] = true;
            sonar_data.distance_cm[i] = (float)distance_cm;
        }

        #if DEBUG_SONAR
        DEBUG_PRINTF("[SONAR %d] Distance: %.1f cm (valid: %d)\\n",
                    i, sonar_data.distance_cm[i], sonar_data.valid[i]);
        #endif

        // Check for collision warning
        if (sonar_data.valid[i] && sonar_data.distance_cm[i] < SONAR_COLLISION_WARNING_CM) {
            DroneCAN_PublishCollisionWarning(i, sonar_data.distance_cm[i]);

            // Critical collision distance
            if (sonar_data.distance_cm[i] < SONAR_COLLISION_STOP_CM) {
                failsafe_mode = FAILSAFE_COLLISION;
                DEBUG_PRINTF("[COLLISION] Critical! Sensor %d: %.1fcm\\n", i, sonar_data.distance_cm[i]);
            }
        }
    }

    sonar_data.timestamp_ms = now;

    // Update error counter
    if (any_error) {
        sonar_error_count++;
        if (sonar_error_count > FAILSAFE_SENSOR_ERROR_MAX) {
            failsafe_mode = FAILSAFE_SENSOR_ERROR;
            DEBUG_PRINTLN("[SONAR] Too many sensor errors!");
        }
    } else {
        sonar_error_count = 0;
        if (failsafe_mode == FAILSAFE_SENSOR_ERROR) {
            failsafe_mode = FAILSAFE_NONE;
        }
    }
}

// ============================================================================
// BME280 SENSOR FUNCTIONS
// ============================================================================

/**
 * @brief Initialize BME280 sensor
 */
bool InitBME280(void) {
    Wire.begin(BME280_SDA_PIN, BME280_SCL_PIN);

    if (!bme.begin(BME280_I2C_ADDR, &Wire)) {
        DEBUG_PRINTLN("[BME280] ERROR: Sensor not found!");
        return false;
    }

    // Configure sensor
    bme.setSampling(BME280_MODE,
                   BME280_SAMPLING_TEMP,
                   BME280_SAMPLING_PRES,
                   BME280_SAMPLING_HUM,
                   BME280_FILTER,
                   BME280_STANDBY);

    DEBUG_PRINTLN("[BME280] Initialized successfully");
    return true;
}

/**
 * @brief Read BME280 sensor
 */
void UpdateBME280Sensor(void) {
    if (!bme_initialized) return;

    uint32_t now = millis();

    if (now - last_bme_update_ms < BME280_READ_INTERVAL_MS) {
        return;
    }
    last_bme_update_ms = now;

    // Read sensor
    bme_data.temperature_c = bme.readTemperature();
    bme_data.humidity_percent = bme.readHumidity();
    bme_data.pressure_hpa = bme.readPressure() / 100.0f;
    bme_data.timestamp_ms = now;

    // Validate readings
    if (isnan(bme_data.temperature_c) || isnan(bme_data.humidity_percent) ||
        isnan(bme_data.pressure_hpa)) {
        bme_data.valid = false;
        DEBUG_PRINTLN("[BME280] ERROR: Invalid reading");
    } else {
        bme_data.valid = true;

        #if DEBUG_BME280
        DEBUG_PRINTF("[BME280] T=%.1f°C H=%.1f%% P=%.1fhPa\\n",
                    bme_data.temperature_c, bme_data.humidity_percent,
                    bme_data.pressure_hpa);
        #endif
    }
}

// ============================================================================
// SERVO CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Initialize camera servos
 */
void InitServos(void) {
    // Attach servos
    servo_pan.attach(SERVO_PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo_tilt.attach(SERVO_TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);

    // Set to center position
    servo_pan.write(SERVO_PAN_CENTER_DEG);
    servo_tilt.write(SERVO_TILT_CENTER_DEG);

    DEBUG_PRINTLN("[SERVO] Camera servos initialized");
}

/**
 * @brief Update servo positions from DroneCAN commands
 */
void UpdateServos(void) {
    ServoCommand cmd;

    // Get command from DroneCAN (or use default/center position)
    if (DroneCAN_GetServoCommand(&cmd)) {
        // Constrain to safe ranges
        servo_pan_pos = constrain(cmd.pan_deg, SERVO_PAN_MIN_DEG, SERVO_PAN_MAX_DEG);
        servo_tilt_pos = constrain(cmd.tilt_deg, SERVO_TILT_MIN_DEG, SERVO_TILT_MAX_DEG);

        #if DEBUG_SERVO
        DEBUG_PRINTF("[SERVO] Pan=%d Tilt=%d\\n", servo_pan_pos, servo_tilt_pos);
        #endif
    } else {
        // No command, return to center
        servo_pan_pos = SERVO_PAN_CENTER_DEG;
        servo_tilt_pos = SERVO_TILT_CENTER_DEG;
    }

    // Update servo positions
    servo_pan.write(servo_pan_pos);
    servo_tilt.write(servo_tilt_pos);
}

// ============================================================================
// LED LIGHTING FUNCTIONS
// ============================================================================

/**
 * @brief Initialize LED lighting
 */
void InitLEDLighting(void) {
    // Configure PWM channels
    ledcSetup(LED_PWM_CHANNEL_FL, LED_PWM_FREQ_HZ, LED_PWM_RESOLUTION);
    ledcSetup(LED_PWM_CHANNEL_FR, LED_PWM_FREQ_HZ, LED_PWM_RESOLUTION);
    ledcSetup(LED_PWM_CHANNEL_RL, LED_PWM_FREQ_HZ, LED_PWM_RESOLUTION);
    ledcSetup(LED_PWM_CHANNEL_RR, LED_PWM_FREQ_HZ, LED_PWM_RESOLUTION);

    // Attach pins to channels
    ledcAttachPin(LED_FRONT_LEFT_PIN, LED_PWM_CHANNEL_FL);
    ledcAttachPin(LED_FRONT_RIGHT_PIN, LED_PWM_CHANNEL_FR);
    ledcAttachPin(LED_REAR_LEFT_PIN, LED_PWM_CHANNEL_RL);
    ledcAttachPin(LED_REAR_RIGHT_PIN, LED_PWM_CHANNEL_RR);

    // Set default brightness
    led_command.mode = LED_DEFAULT_MODE;
    led_command.brightness = LED_DEFAULT_BRIGHTNESS;

    DEBUG_PRINTLN("[LED] Lighting initialized");
}

/**
 * @brief Update LED lighting from DroneCAN commands
 */
void UpdateLEDLighting(void) {
    // Get LED command from DroneCAN
    DroneCAN_GetLEDCommand(&led_command);

    // Apply mode-specific behavior
    switch (led_command.mode) {
        case LED_MODE_OFF:
            led_brightness[0] = 0;
            led_brightness[1] = 0;
            led_brightness[2] = 0;
            led_brightness[3] = 0;
            break;

        case LED_MODE_AUTO:
            // Auto mode: use ambient light sensor or default
            // For now, use default brightness
            led_brightness[0] = led_command.brightness;
            led_brightness[1] = led_command.brightness;
            led_brightness[2] = led_command.brightness;
            led_brightness[3] = led_command.brightness;
            break;

        case LED_MODE_ON:
            // Always on at specified brightness
            led_brightness[0] = led_command.front_left ? led_command.front_left : led_command.brightness;
            led_brightness[1] = led_command.front_right ? led_command.front_right : led_command.brightness;
            led_brightness[2] = led_command.rear_left ? led_command.rear_left : led_command.brightness;
            led_brightness[3] = led_command.rear_right ? led_command.rear_right : led_command.brightness;
            break;

        case LED_MODE_BLINK:
            // Blink pattern
            if ((millis() / 500) % 2 == 0) {
                led_brightness[0] = led_command.brightness;
                led_brightness[1] = led_command.brightness;
                led_brightness[2] = led_command.brightness;
                led_brightness[3] = led_command.brightness;
            } else {
                led_brightness[0] = 0;
                led_brightness[1] = 0;
                led_brightness[2] = 0;
                led_brightness[3] = 0;
            }
            break;

        default:
            break;
    }

    // Update PWM outputs
    ledcWrite(LED_PWM_CHANNEL_FL, led_brightness[0]);
    ledcWrite(LED_PWM_CHANNEL_FR, led_brightness[1]);
    ledcWrite(LED_PWM_CHANNEL_RL, led_brightness[2]);
    ledcWrite(LED_PWM_CHANNEL_RR, led_brightness[3]);

    #if DEBUG_LED
    DEBUG_PRINTF("[LED] Mode=%d Brightness: FL=%d FR=%d RL=%d RR=%d\\n",
                led_command.mode, led_brightness[0], led_brightness[1],
                led_brightness[2], led_brightness[3]);
    #endif
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
    uint32_t blink_period = 0;

    // Determine LED color and pattern based on state
    if (failsafe_mode == FAILSAFE_EMERGENCY_STOP) {
        color = LED_ESTOP;
        blink_period = LED_BLINK_FAST;
    } else if (failsafe_mode == FAILSAFE_COLLISION) {
        color = LED_COLLISION;
        blink_period = LED_BLINK_FAST;
    } else if (failsafe_mode == FAILSAFE_SENSOR_ERROR) {
        color = LED_SENSOR_ERROR;
        blink_period = LED_BLINK_NORMAL;
    } else if (failsafe_mode == FAILSAFE_NO_SIGNAL) {
        color = LED_NO_SIGNAL;
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

    DEBUG_PRINTLN("\\n\\n==========================================");
    DEBUG_PRINTLN("VETER_NEXT ESP32 Sensor Hub");
    DEBUG_PRINTF("Firmware v%d.%d.%d\\n", FIRMWARE_VERSION_MAJOR,
                 FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    DEBUG_PRINTF("Build: %s %s\\n", FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME);
    DEBUG_PRINTLN("==========================================\\n");

    // Initialize pins
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // Initialize status LED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(50);
    leds[0] = LED_BOOT;
    FastLED.show();

    // Initialize DroneCAN
    if (!DroneCAN_Init()) {
        DEBUG_PRINTLN("[ERROR] DroneCAN initialization failed!");
        leds[0] = LED_CAN_ERROR;
        FastLED.show();
        while (1) { delay(1000); }  // Halt
    }

    // Initialize BME280
    bme_initialized = InitBME280();
    if (!bme_initialized) {
        DEBUG_PRINTLN("[WARNING] BME280 not available");
    }

    // Initialize servos
    InitServos();

    // Initialize LED lighting
    InitLEDLighting();

    // Initialize sonar data
    for (int i = 0; i < SONAR_COUNT; i++) {
        sonar_data.distance_cm[i] = SONAR_MAX_DISTANCE_CM;
        sonar_data.valid[i] = false;
    }

    DEBUG_PRINTLN("[INIT] System initialized successfully");
    DEBUG_PRINTLN("[INIT] Entering main loop...\\n");

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

    // Update sensors
    UpdateUltrasonicSensors();
    UpdateBME280Sensor();

    // Update actuators
    UpdateServos();
    UpdateLEDLighting();

    // Publish sensor data via DroneCAN
    if (now - last_publish_ms >= SENSOR_PUBLISH_RATE_MS) {
        last_publish_ms = now;

        // Publish sonar data
        DroneCAN_PublishSonarData(&sonar_data);

        // Publish BME280 data
        if (bme_initialized && bme_data.valid) {
            DroneCAN_PublishBME280Data(&bme_data);
        }
    }

    // Send DroneCAN heartbeat
    DroneCAN_SendHeartbeat();

    // Process incoming DroneCAN messages
    DroneCAN_Process();

    // Update status LED
    UpdateStatusLED();
}
