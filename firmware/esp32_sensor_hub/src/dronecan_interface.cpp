/**
 * @file dronecan_interface.cpp
 * @brief DroneCAN interface implementation for ESP32 Sensor Hub
 *
 * Handles all DroneCAN communication including:
 * - Node initialization and heartbeat
 * - Sensor data publication (ultrasonic, BME280)
 * - Command reception (servo, LED control)
 */

#include "dronecan_interface.h"
#include "config.h"
#include <SPI.h>

// MCP2515 CAN controller instance
MCP2515 can0(CAN_CS_PIN);

// DroneCAN node state
static uint8_t node_health = 0;  // 0=OK, 1=WARNING, 2=ERROR, 3=CRITICAL
static uint8_t node_mode = 0;    // 0=OPERATIONAL, 1=INITIALIZATION, etc.
static uint32_t uptime_sec = 0;
static uint32_t last_heartbeat_ms = 0;

// Command buffers
static ServoCommand last_servo_cmd = {90, 90, 0};
static LEDCommand last_led_cmd = {LED_MODE_AUTO, LED_BRIGHTNESS_MED, 0, 0, 0, 0, 0};
static uint32_t last_command_ms = 0;

// Connection state
static bool can_connected = false;

/**
 * @brief Initialize DroneCAN interface
 * @return true if successful, false otherwise
 */
bool DroneCAN_Init(void) {
    DEBUG_PRINTLN("[DroneCAN] Initializing...");

    // Initialize SPI for MCP2515
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    // Reset MCP2515
    can0.reset();
    delay(10);

    // Set CAN bitrate to 1 Mbps
    if (can0.setBitrate(CAN_BITRATE, MCP_8MHZ) != MCP2515::ERROR_OK) {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to set bitrate");
        return false;
    }

    // Set to normal mode
    if (can0.setNormalMode() != MCP2515::ERROR_OK) {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to set normal mode");
        return false;
    }

    can_connected = true;

    DEBUG_PRINTF("[DroneCAN] Initialized as Node ID %d\\n", DRONECAN_NODE_ID);
    DEBUG_PRINTF("[DroneCAN] Node name: %s\\n", DRONECAN_NODE_NAME);

    return true;
}

/**
 * @brief Send DroneCAN heartbeat message
 */
void DroneCAN_SendHeartbeat(void) {
    uint32_t now = millis();

    // Send heartbeat at configured interval
    if (now - last_heartbeat_ms < HEARTBEAT_INTERVAL_MS) {
        return;
    }
    last_heartbeat_ms = now;

    // Update uptime (seconds)
    uptime_sec = now / 1000;

    // Prepare heartbeat message
    // DroneCAN NodeStatus message ID: 341 (0x155)
    // Data: [uptime(32bit), health(2bit), mode(3bit), vendor_specific(19bit)]

    struct can_frame frame;
    frame.can_id = 0x155;  // NodeStatus message type
    frame.can_dlc = 7;

    // Uptime (4 bytes, little-endian)
    frame.data[0] = (uptime_sec >> 0) & 0xFF;
    frame.data[1] = (uptime_sec >> 8) & 0xFF;
    frame.data[2] = (uptime_sec >> 16) & 0xFF;
    frame.data[3] = (uptime_sec >> 24) & 0xFF;

    // Health and mode
    frame.data[4] = (node_health & 0x03) | ((node_mode & 0x07) << 2);

    // Vendor-specific (unused for now)
    frame.data[5] = 0;
    frame.data[6] = 0;

    // Add source node ID to CAN ID (bits 0-6)
    frame.can_id |= (DRONECAN_NODE_ID & 0x7F);

    // Send CAN frame
    if (can0.sendMessage(&frame) == MCP2515::ERROR_OK) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Heartbeat sent (uptime: %lu s)\\n", uptime_sec);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send heartbeat");
        can_connected = false;
    }
}

/**
 * @brief Publish ultrasonic sensor data
 * @param data Pointer to sonar data structure
 */
void DroneCAN_PublishSonarData(const SonarData* data) {
    if (!data) return;

    // DroneCAN RangeSensor message (custom or standard)
    // Message ID: 1050 (example, use appropriate ID)
    struct can_frame frame;
    frame.can_id = 0x41A;  // RangeSensor message type
    frame.can_dlc = 8;

    // Pack 4 distances (16-bit each, in mm)
    for (int i = 0; i < 4; i++) {
        uint16_t distance_mm = data->valid[i] ? (uint16_t)(data->distance_cm[i] * 10) : 0xFFFF;
        frame.data[i * 2] = distance_mm & 0xFF;
        frame.data[i * 2 + 1] = (distance_mm >> 8) & 0xFF;
    }

    // Add source node ID
    frame.can_id |= (DRONECAN_NODE_ID & 0x7F);

    // Send CAN frame
    if (can0.sendMessage(&frame) == MCP2515::ERROR_OK) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Sonar data sent: F=%.1f R=%.1f L=%.1f R=%.1f cm\\n",
                    data->distance_cm[0], data->distance_cm[1],
                    data->distance_cm[2], data->distance_cm[3]);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send sonar data");
    }
}

/**
 * @brief Publish BME280 sensor data
 * @param data Pointer to BME280 data structure
 */
void DroneCAN_PublishBME280Data(const BME280Data* data) {
    if (!data || !data->valid) return;

    // DroneCAN AirData message (temperature, humidity, pressure)
    // Message ID: 1060 (example)
    struct can_frame frame;
    frame.can_id = 0x424;  // AirData message type
    frame.can_dlc = 8;

    // Temperature (16-bit, 0.01 degC resolution)
    int16_t temp_raw = (int16_t)(data->temperature_c * 100);
    frame.data[0] = temp_raw & 0xFF;
    frame.data[1] = (temp_raw >> 8) & 0xFF;

    // Humidity (16-bit, 0.01% resolution)
    uint16_t hum_raw = (uint16_t)(data->humidity_percent * 100);
    frame.data[2] = hum_raw & 0xFF;
    frame.data[3] = (hum_raw >> 8) & 0xFF;

    // Pressure (32-bit, 0.01 hPa resolution)
    uint32_t pres_raw = (uint32_t)(data->pressure_hpa * 100);
    frame.data[4] = (pres_raw >> 0) & 0xFF;
    frame.data[5] = (pres_raw >> 8) & 0xFF;
    frame.data[6] = (pres_raw >> 16) & 0xFF;
    frame.data[7] = (pres_raw >> 24) & 0xFF;

    // Add source node ID
    frame.can_id |= (DRONECAN_NODE_ID & 0x7F);

    // Send CAN frame
    if (can0.sendMessage(&frame) == MCP2515::ERROR_OK) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] BME280 sent: T=%.1f°C H=%.1f%% P=%.1fhPa\\n",
                    data->temperature_c, data->humidity_percent, data->pressure_hpa);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send BME280 data");
    }
}

/**
 * @brief Publish collision warning
 * @param direction Sensor direction (0=front, 1=rear, 2=left, 3=right)
 * @param distance_cm Distance to obstacle in cm
 */
void DroneCAN_PublishCollisionWarning(uint8_t direction, float distance_cm) {
    // DroneCAN CollisionWarning message
    // Message ID: 1070 (example)
    struct can_frame frame;
    frame.can_id = 0x42E;  // CollisionWarning message type
    frame.can_dlc = 4;

    // Direction (1 byte)
    frame.data[0] = direction & 0x03;

    // Distance (16-bit, mm)
    uint16_t distance_mm = (uint16_t)(distance_cm * 10);
    frame.data[1] = distance_mm & 0xFF;
    frame.data[2] = (distance_mm >> 8) & 0xFF;

    // Severity (1 byte: 0=info, 1=warning, 2=critical)
    uint8_t severity = 0;
    if (distance_cm < SONAR_COLLISION_STOP_CM) {
        severity = 2;  // Critical
    } else if (distance_cm < SONAR_COLLISION_WARNING_CM) {
        severity = 1;  // Warning
    }
    frame.data[3] = severity;

    // Add source node ID
    frame.can_id |= (DRONECAN_NODE_ID & 0x7F);

    // Send CAN frame
    if (can0.sendMessage(&frame) == MCP2515::ERROR_OK) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Collision warning: dir=%d dist=%.1fcm sev=%d\\n",
                    direction, distance_cm, severity);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send collision warning");
    }
}

/**
 * @brief Process incoming DroneCAN messages
 */
void DroneCAN_Process(void) {
    struct can_frame frame;

    // Check for received messages
    while (can0.readMessage(&frame) == MCP2515::ERROR_OK) {
        can_connected = true;
        last_command_ms = millis();

        // Extract message type and source node ID
        uint32_t msg_type = frame.can_id & 0xFFFFFF80;  // Remove node ID
        uint8_t src_node = frame.can_id & 0x7F;

        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] RX: ID=0x%lX Type=0x%lX Node=%d DLC=%d\\n",
                    frame.can_id, msg_type, src_node, frame.can_dlc);
        #endif

        // Parse different message types
        switch (msg_type) {
            case 0x480:  // Servo command (example ID)
                if (frame.can_dlc >= 4) {
                    last_servo_cmd.pan_deg = frame.data[0] | ((uint16_t)frame.data[1] << 8);
                    last_servo_cmd.tilt_deg = frame.data[2] | ((uint16_t)frame.data[3] << 8);
                    last_servo_cmd.timestamp_ms = millis();
                }
                break;

            case 0x490:  // LED command (example ID)
                if (frame.can_dlc >= 6) {
                    last_led_cmd.mode = frame.data[0];
                    last_led_cmd.brightness = frame.data[1];
                    last_led_cmd.front_left = frame.data[2];
                    last_led_cmd.front_right = frame.data[3];
                    last_led_cmd.rear_left = frame.data[4];
                    last_led_cmd.rear_right = frame.data[5];
                    last_led_cmd.timestamp_ms = millis();
                }
                break;

            default:
                // Unknown message type
                break;
        }
    }
}

/**
 * @brief Get latest servo command
 * @param cmd Pointer to servo command structure
 * @return true if valid command available
 */
bool DroneCAN_GetServoCommand(ServoCommand* cmd) {
    if (!cmd) return false;

    // Check if command is recent
    if (millis() - last_servo_cmd.timestamp_ms > SERVO_CONTROL_TIMEOUT_MS) {
        return false;
    }

    *cmd = last_servo_cmd;
    return true;
}

/**
 * @brief Get latest LED command
 * @param cmd Pointer to LED command structure
 * @return true if valid command available
 */
bool DroneCAN_GetLEDCommand(LEDCommand* cmd) {
    if (!cmd) return false;

    // LED commands don't timeout, use default if no recent command
    *cmd = last_led_cmd;
    return true;
}

/**
 * @brief Set node health status
 * @param health 0=OK, 1=WARNING, 2=ERROR, 3=CRITICAL
 */
void DroneCAN_SetHealth(uint8_t health) {
    node_health = health & 0x03;
}

/**
 * @brief Set node mode
 * @param mode 0=OPERATIONAL, 1=INITIALIZATION, etc.
 */
void DroneCAN_SetMode(uint8_t mode) {
    node_mode = mode & 0x07;
}

/**
 * @brief Get milliseconds since last command
 * @return Time in milliseconds
 */
uint32_t DroneCAN_GetTimeSinceLastCommand(void) {
    return millis() - last_command_ms;
}

/**
 * @brief Check if CAN bus is connected
 * @return true if connected
 */
bool DroneCAN_IsConnected(void) {
    return can_connected;
}
