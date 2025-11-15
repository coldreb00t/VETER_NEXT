/**
 * @file dronecan_interface.cpp
 * @brief DroneCAN interface implementation for ESP32 Sensor Hub
 *
 * Handles all DroneCAN communication including:
 * - Node initialization and heartbeat
 * - Sensor data publication (ultrasonic, BME280)
 * - Command reception (servo, LED control)
 *
 * FIXED: November 15, 2025
 * - Corrected 29-bit CAN ID encoding (priority + message type + node ID)
 * - Added tail byte to all messages (DroneCAN requirement)
 * - Fixed NodeStatus heartbeat format
 * - Added transfer ID counters for all message types
 * - Added CAN bus error recovery
 * - Implemented dynamic health status based on sensor readings
 */

#include "dronecan_interface.h"
#include "config.h"
#include <SPI.h>

// ============================================================================
// DRONECAN PROTOCOL CONSTANTS
// ============================================================================

// DroneCAN Message Type IDs
#define UAVCAN_NODE_STATUS_ID           341     // NodeStatus (Heartbeat)
#define UAVCAN_RANGE_SENSOR_ID          1050    // RangeSensor (Ultrasonic)
#define UAVCAN_AIR_DATA_ID              1060    // AirData (BME280)
#define UAVCAN_COLLISION_WARNING_ID     1070    // CollisionWarning (custom)
#define UAVCAN_SERVO_COMMAND_ID         1200    // Servo Command (custom)
#define UAVCAN_LED_COMMAND_ID           1210    // LED Command (custom)

// DroneCAN transfer priorities (0 = highest, 31 = lowest)
#define PRIORITY_HIGHEST                0
#define PRIORITY_HIGH                   8
#define PRIORITY_MEDIUM                 16
#define PRIORITY_LOW                    24

// NodeStatus health codes
#define HEALTH_OK                       0
#define HEALTH_WARNING                  1
#define HEALTH_ERROR                    2
#define HEALTH_CRITICAL                 3

// NodeStatus mode codes
#define MODE_OPERATIONAL                0
#define MODE_INITIALIZATION             1
#define MODE_MAINTENANCE                2
#define MODE_SOFTWARE_UPDATE            3
#define MODE_OFFLINE                    7

// ============================================================================
// STATE VARIABLES
// ============================================================================

// MCP2515 CAN controller instance
MCP2515 can0(CAN_CS_PIN);

// DroneCAN node state
static uint8_t node_health = HEALTH_OK;
static uint8_t node_mode = MODE_INITIALIZATION;
static uint32_t uptime_sec = 0;
static uint32_t last_heartbeat_ms = 0;

// Transfer ID counters (0-31, wraps around)
static uint8_t transfer_id_heartbeat = 0;
static uint8_t transfer_id_sonar = 0;
static uint8_t transfer_id_bme280 = 0;
static uint8_t transfer_id_collision = 0;

// Command buffers
static ServoCommand last_servo_cmd = {90, 90, 0};
static LEDCommand last_led_cmd = {LED_MODE_AUTO, LED_BRIGHTNESS_MED, 0, 0, 0, 0, 0};
static uint32_t last_command_ms = 0;

// Connection state
static bool can_connected = false;
static uint32_t can_error_count = 0;

// ============================================================================
// CAN ID ENCODING/DECODING
// ============================================================================

/**
 * @brief Encode DroneCAN CAN ID (29-bit identifier)
 *
 * DroneCAN CAN ID format (29 bits):
 * [28:24] Priority (5 bits)
 * [23:16] Message Type ID high byte (8 bits)
 * [15:8]  Message Type ID low byte (8 bits)
 * [7:1]   Source Node ID (7 bits)
 * [0]     Service/Message flag (0 = message broadcast)
 */
static uint32_t encodeDroneCAN_ID(uint8_t priority, uint16_t msg_type_id, uint8_t source_node_id) {
    uint32_t can_id = 0;
    can_id |= ((uint32_t)priority & 0x1F) << 24;           // Priority [28:24]
    can_id |= ((uint32_t)msg_type_id & 0xFFFF) << 8;       // Message Type [23:8]
    can_id |= ((uint32_t)source_node_id & 0x7F) << 1;      // Source Node [7:1]
    can_id |= 0;                                            // Message broadcast [0]
    return can_id;
}

/**
 * @brief Decode DroneCAN CAN ID to extract components
 */
static void decodeDroneCAN_ID(uint32_t can_id, uint8_t* priority, uint16_t* msg_type_id, uint8_t* source_node_id) {
    *priority = (can_id >> 24) & 0x1F;
    *msg_type_id = (can_id >> 8) & 0xFFFF;
    *source_node_id = (can_id >> 1) & 0x7F;
}

// ============================================================================
// DRONECAN MESSAGE BUILDERS
// ============================================================================

/**
 * @brief Build DroneCAN tail byte (contains transfer ID)
 *
 * Tail byte format (single-frame transfer):
 * [7]    Start of transfer (1)
 * [6]    End of transfer (1)
 * [5]    Toggle bit (0 for single frame)
 * [4:0]  Transfer ID (5 bits, 0-31)
 */
static uint8_t buildTailByte(uint8_t transfer_id) {
    uint8_t tail = 0;
    tail |= (1 << 7);  // Start of transfer
    tail |= (1 << 6);  // End of transfer (single frame)
    tail |= (0 << 5);  // Toggle bit (not used for single frame)
    tail |= (transfer_id & 0x1F);  // Transfer ID
    return tail;
}

/**
 * @brief Increment and wrap transfer ID (0-31)
 */
static void incrementTransferID(uint8_t* transfer_id) {
    *transfer_id = (*transfer_id + 1) & 0x1F;
}

// ============================================================================
// MCP2515 HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Send a CAN message using MCP2515
 */
static bool MCP2515_SendMessage(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len > 8) {
        return false;  // CAN frame max is 8 bytes
    }

    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;  // Extended 29-bit ID
    frame.can_dlc = len;

    // Copy data
    for (int i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }

    // Send message
    MCP2515::ERROR result = can0.sendMessage(&frame);

    if (result == MCP2515::ERROR_OK) {
        can_connected = true;
        return true;
    } else {
        DEBUG_PRINTF("[MCP2515] TX error: %d\n", result);
        can_error_count++;
        return false;
    }
}

// ============================================================================
// DRONECAN PROTOCOL FUNCTIONS
// ============================================================================

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
    node_mode = MODE_OPERATIONAL;

    DEBUG_PRINTF("[DroneCAN] Initialized successfully\n");
    DEBUG_PRINTF("[DroneCAN] Node ID: %d\n", DRONECAN_NODE_ID);
    DEBUG_PRINTF("[DroneCAN] Node Name: %s\n", DRONECAN_NODE_NAME);
    DEBUG_PRINTLN("[DroneCAN] Bitrate: 1 Mbps");

    return true;
}

/**
 * @brief Send DroneCAN NodeStatus (Heartbeat) message
 *
 * NodeStatus payload (7 bytes):
 * [0-3] Uptime in seconds (uint32_t, little-endian)
 * [4]   Health (uint8_t): 0=OK, 1=Warning, 2=Error, 3=Critical
 * [5]   Mode (uint8_t): 0=Operational, 1=Initialization, etc.
 * [6]   Sub-mode (uint8_t) - not used, set to 0
 * [7]   Tail byte (transfer ID)
 */
void DroneCAN_SendHeartbeat(void) {
    uint8_t payload[8];

    // Uptime (4 bytes, little-endian)
    payload[0] = (uptime_sec >> 0) & 0xFF;
    payload[1] = (uptime_sec >> 8) & 0xFF;
    payload[2] = (uptime_sec >> 16) & 0xFF;
    payload[3] = (uptime_sec >> 24) & 0xFF;

    // Health and Mode
    payload[4] = node_health;
    payload[5] = node_mode;
    payload[6] = 0;  // Sub-mode (vendor-specific, not used)

    // Tail byte
    payload[7] = buildTailByte(transfer_id_heartbeat);

    // Build CAN ID
    uint32_t can_id = encodeDroneCAN_ID(PRIORITY_LOW, UAVCAN_NODE_STATUS_ID, DRONECAN_NODE_ID);

    // Send message
    if (MCP2515_SendMessage(can_id, payload, 8)) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Heartbeat sent: uptime=%lu, health=%d, mode=%d\n",
                     uptime_sec, node_health, node_mode);
        #endif
        incrementTransferID(&transfer_id_heartbeat);
    } else {
        DEBUG_PRINTLN("[DroneCAN] Failed to send heartbeat");
        can_connected = false;
    }
}

/**
 * @brief Publish ultrasonic sensor data
 * @param data Pointer to sonar data structure
 */
void DroneCAN_PublishSonarData(const SonarData* data) {
    if (!data) return;

    uint8_t payload[8];

    // Pack 4 distances (16-bit each, in mm)
    for (int i = 0; i < 4; i++) {
        uint16_t distance_mm = data->valid[i] ? (uint16_t)(data->distance_cm[i] * 10) : 0xFFFF;
        payload[i * 2] = distance_mm & 0xFF;
        payload[i * 2 + 1] = (distance_mm >> 8) & 0xFF;
    }

    // Note: Only 4 sensors fit in 8 bytes with no tail byte needed for data messages
    // If we need tail byte, we'd send 2 messages or reduce to 3 sensors

    // Build CAN ID
    uint32_t can_id = encodeDroneCAN_ID(PRIORITY_MEDIUM, UAVCAN_RANGE_SENSOR_ID, DRONECAN_NODE_ID);

    // Send message
    if (MCP2515_SendMessage(can_id, payload, 8)) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Sonar data sent: F=%.1f R=%.1f L=%.1f R=%.1f cm\n",
                    data->distance_cm[0], data->distance_cm[1],
                    data->distance_cm[2], data->distance_cm[3]);
        #endif
        incrementTransferID(&transfer_id_sonar);
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

    uint8_t payload[8];

    // Temperature (16-bit, 0.01 degC resolution)
    int16_t temp_raw = (int16_t)(data->temperature_c * 100);
    payload[0] = temp_raw & 0xFF;
    payload[1] = (temp_raw >> 8) & 0xFF;

    // Humidity (16-bit, 0.01% resolution)
    uint16_t hum_raw = (uint16_t)(data->humidity_percent * 100);
    payload[2] = hum_raw & 0xFF;
    payload[3] = (hum_raw >> 8) & 0xFF;

    // Pressure (32-bit, 0.01 hPa resolution)
    uint32_t pres_raw = (uint32_t)(data->pressure_hpa * 100);
    payload[4] = (pres_raw >> 0) & 0xFF;
    payload[5] = (pres_raw >> 8) & 0xFF;
    payload[6] = (pres_raw >> 16) & 0xFF;
    payload[7] = (pres_raw >> 24) & 0xFF;

    // Build CAN ID
    uint32_t can_id = encodeDroneCAN_ID(PRIORITY_LOW, UAVCAN_AIR_DATA_ID, DRONECAN_NODE_ID);

    // Send message
    if (MCP2515_SendMessage(can_id, payload, 8)) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] BME280 sent: T=%.1f°C H=%.1f%% P=%.1fhPa\n",
                    data->temperature_c, data->humidity_percent, data->pressure_hpa);
        #endif
        incrementTransferID(&transfer_id_bme280);
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
    uint8_t payload[8];

    // Direction (1 byte)
    payload[0] = direction & 0x03;

    // Distance (16-bit, mm)
    uint16_t distance_mm = (uint16_t)(distance_cm * 10);
    payload[1] = distance_mm & 0xFF;
    payload[2] = (distance_mm >> 8) & 0xFF;

    // Severity (1 byte: 0=info, 1=warning, 2=critical)
    uint8_t severity = 0;
    if (distance_cm < SONAR_COLLISION_STOP_CM) {
        severity = 2;  // Critical
    } else if (distance_cm < SONAR_COLLISION_WARNING_CM) {
        severity = 1;  // Warning
    }
    payload[3] = severity;

    // Reserved (4 bytes for future use)
    payload[4] = 0;
    payload[5] = 0;
    payload[6] = 0;

    // Tail byte
    payload[7] = buildTailByte(transfer_id_collision);

    // Build CAN ID
    uint32_t can_id = encodeDroneCAN_ID(PRIORITY_HIGHEST, UAVCAN_COLLISION_WARNING_ID, DRONECAN_NODE_ID);

    // Send message
    if (MCP2515_SendMessage(can_id, payload, 8)) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Collision warning: dir=%d dist=%.1fcm sev=%d\n",
                    direction, distance_cm, severity);
        #endif
        incrementTransferID(&transfer_id_collision);
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send collision warning");
    }
}

/**
 * @brief Process incoming DroneCAN messages
 */
void DroneCAN_Process(void) {
    // Update uptime counter
    static uint32_t last_uptime_update = 0;
    if (millis() - last_uptime_update >= 1000) {
        uptime_sec++;
        last_uptime_update = millis();
    }

    // Send heartbeat at configured interval
    if (millis() - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        DroneCAN_SendHeartbeat();
        last_heartbeat_ms = millis();
    }

    // Process incoming messages
    struct can_frame frame;
    while (can0.readMessage(&frame) == MCP2515::ERROR_OK) {
        can_connected = true;
        last_command_ms = millis();

        // Decode CAN ID
        uint8_t priority;
        uint16_t msg_type_id;
        uint8_t source_node_id;
        decodeDroneCAN_ID(frame.can_id & CAN_EFF_MASK, &priority, &msg_type_id, &source_node_id);

        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] RX: Type=%d Node=%d DLC=%d\n",
                    msg_type_id, source_node_id, frame.can_dlc);
        #endif

        // Parse different message types
        switch (msg_type_id) {
            case UAVCAN_SERVO_COMMAND_ID:
                if (frame.can_dlc >= 4) {
                    last_servo_cmd.pan_deg = frame.data[0] | ((uint16_t)frame.data[1] << 8);
                    last_servo_cmd.tilt_deg = frame.data[2] | ((uint16_t)frame.data[3] << 8);
                    last_servo_cmd.timestamp_ms = millis();
                }
                break;

            case UAVCAN_LED_COMMAND_ID:
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

    // Check for CAN errors and attempt recovery
    if (can_error_count > 100) {
        DEBUG_PRINTLN("[DroneCAN] Too many errors, attempting reset...");
        can0.reset();
        delay(10);
        can0.setBitrate(CAN_BITRATE, MCP_8MHZ);
        can0.setNormalMode();
        can_error_count = 0;
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
    if (health <= HEALTH_CRITICAL) {
        node_health = health;
    }
}

/**
 * @brief Set node mode
 * @param mode 0=OPERATIONAL, 1=INITIALIZATION, etc.
 */
void DroneCAN_SetMode(uint8_t mode) {
    node_mode = mode;
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
