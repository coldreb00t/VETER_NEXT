/**
 * @file dronecan_interface.cpp
 * @brief DroneCAN protocol implementation for ESP32 Motor Controller
 *
 * Implements minimal DroneCAN protocol for:
 * - NodeStatus heartbeat (ID: 341)
 * - ESC RawCommand (ID: 1030)
 */

#include "dronecan_interface.h"
#include "config.h"
#include <Arduino.h>

// External MCP2515 instance (defined in main.cpp)
extern MCP2515 mcp2515;

// DroneCAN state
static uint32_t uptime_sec = 0;
static uint32_t last_heartbeat_ms = 0;
static uint32_t last_esc_command_ms = 0;
static uint8_t node_health = 0;  // 0=OK, 1=WARNING, 2=ERROR, 3=CRITICAL
static uint8_t node_mode = 0;    // 0=OPERATIONAL, 1=INITIALIZATION, 2=MAINTENANCE, 3=SOFTWARE_UPDATE, 7=OFFLINE

// DroneCAN Message IDs
#define DRONECAN_MSG_NODESTATUS     341
#define DRONECAN_MSG_ESC_RAWCOMMAND 1030

// Transfer ID counters (0-31, wraps around)
static uint8_t transfer_id_nodestatus = 0;
static uint8_t transfer_id_esc_command = 0;

/**
 * @brief Build DroneCAN CAN ID
 * @param priority Priority (0-31, lower = higher priority)
 * @param message_type_id Message type ID
 * @param source_node_id Source node ID
 * @param is_service Is this a service request/response?
 * @return 29-bit CAN ID
 */
static uint32_t buildDroneCANID(uint8_t priority, uint16_t message_type_id, uint8_t source_node_id, bool is_service = false) {
    uint32_t can_id = 0;

    if (is_service) {
        // Service frame (not implemented yet)
        can_id = 0x80000000;  // Extended frame
    } else {
        // Message frame
        can_id = ((uint32_t)priority << 24) |
                 ((uint32_t)message_type_id << 8) |
                 source_node_id;
        can_id |= 0x80000000;  // Extended frame flag
    }

    return can_id;
}

/**
 * @brief Initialize DroneCAN interface
 * @return true if successful
 */
bool DroneCAN_Init(void) {
    // MCP2515 should already be initialized in main.cpp
    // Just reset state variables
    uptime_sec = 0;
    last_heartbeat_ms = 0;
    last_esc_command_ms = 0;
    node_health = 0;  // OK
    node_mode = 0;    // OPERATIONAL

    DEBUG_PRINTLN("[DroneCAN] Initialized");
    DEBUG_PRINTF("[DroneCAN] Node ID: %d\n", DRONECAN_NODE_ID);

    return true;
}

/**
 * @brief Send NodeStatus heartbeat
 *
 * Message format (7 bytes):
 * - uptime_sec (uint32_t, 4 bytes)
 * - health (uint2_t, 2 bits)
 * - mode (uint3_t, 3 bits)
 * - sub_mode (uint3_t, 3 bits) - not used, set to 0
 * - vendor_specific_status_code (uint16_t, 2 bytes)
 */
void DroneCAN_SendHeartbeat(void) {
    struct can_frame frame;

    // Build CAN ID: priority=0 (highest), message_type=341, source=DRONECAN_NODE_ID
    frame.can_id = buildDroneCANID(0, DRONECAN_MSG_NODESTATUS, DRONECAN_NODE_ID);

    // Build payload
    frame.can_dlc = 7;

    // uptime_sec (4 bytes, little-endian)
    frame.data[0] = (uptime_sec >> 0) & 0xFF;
    frame.data[1] = (uptime_sec >> 8) & 0xFF;
    frame.data[2] = (uptime_sec >> 16) & 0xFF;
    frame.data[3] = (uptime_sec >> 24) & 0xFF;

    // health (2 bits) + mode (3 bits) + sub_mode (3 bits) = 1 byte
    frame.data[4] = (node_health & 0x03) | ((node_mode & 0x07) << 2) | ((0 & 0x07) << 5);

    // vendor_specific_status_code (2 bytes, little-endian)
    uint16_t vendor_code = transfer_id_nodestatus;  // Use transfer ID as vendor code
    frame.data[5] = (vendor_code >> 0) & 0xFF;
    frame.data[6] = (vendor_code >> 8) & 0xFF;

    // Send via MCP2515
    MCP2515::ERROR result = mcp2515.sendMessage(&frame);

    if (result == MCP2515::ERROR_OK) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] NodeStatus sent (uptime=%d, health=%d, mode=%d)\n",
                     uptime_sec, node_health, node_mode);
        #endif
    } else {
        DEBUG_PRINTF("[DroneCAN ERROR] Failed to send NodeStatus: %d\n", result);
    }

    // Increment transfer ID (0-31)
    transfer_id_nodestatus = (transfer_id_nodestatus + 1) & 0x1F;
    last_heartbeat_ms = millis();
}

/**
 * @brief Send ESC RawCommand to both motors
 * @param left_cmd Left motor command (-8191 to +8191)
 * @param right_cmd Right motor command (-8191 to +8191)
 *
 * Message format:
 * - Array of int14_t values (14-bit signed integers)
 * - For 2 motors: 4 bytes total (2 bytes per motor)
 */
void DroneCAN_SendESCCommand(int16_t left_cmd, int16_t right_cmd) {
    struct can_frame frame;

    // Clamp values to valid range
    if (left_cmd < ESC_COMMAND_MIN) left_cmd = ESC_COMMAND_MIN;
    if (left_cmd > ESC_COMMAND_MAX) left_cmd = ESC_COMMAND_MAX;
    if (right_cmd < ESC_COMMAND_MIN) right_cmd = ESC_COMMAND_MIN;
    if (right_cmd > ESC_COMMAND_MAX) right_cmd = ESC_COMMAND_MAX;

    // Build CAN ID: priority=16 (normal), message_type=1030, source=DRONECAN_NODE_ID
    frame.can_id = buildDroneCANID(16, DRONECAN_MSG_ESC_RAWCOMMAND, DRONECAN_NODE_ID);

    // Build payload
    // DroneCAN ESC RawCommand uses 14-bit signed integers
    // We pack 2 motors into 4 bytes:
    // Motor 0 (left): bits 0-13
    // Motor 1 (right): bits 14-27
    // Tail byte: transfer ID (bits 28-31)

    uint32_t packed = 0;

    // Pack left motor (14 bits)
    packed |= ((uint32_t)(left_cmd & 0x3FFF) << 0);

    // Pack right motor (14 bits)
    packed |= ((uint32_t)(right_cmd & 0x3FFF) << 14);

    frame.can_dlc = 4;
    frame.data[0] = (packed >> 0) & 0xFF;
    frame.data[1] = (packed >> 8) & 0xFF;
    frame.data[2] = (packed >> 16) & 0xFF;
    frame.data[3] = (packed >> 24) & 0xFF;

    // Send via MCP2515
    MCP2515::ERROR result = mcp2515.sendMessage(&frame);

    if (result == MCP2515::ERROR_OK) {
        #if DEBUG_ESC_COMMANDS
        DEBUG_PRINTF("[DroneCAN] ESC Command sent: L=%d R=%d\n", left_cmd, right_cmd);
        #endif
    } else {
        DEBUG_PRINTF("[DroneCAN ERROR] Failed to send ESC command: %d\n", result);
    }

    // Increment transfer ID (0-31)
    transfer_id_esc_command = (transfer_id_esc_command + 1) & 0x1F;
    last_esc_command_ms = millis();
}

/**
 * @brief Stop all motors (send zero command)
 */
void DroneCAN_StopMotors(void) {
    DroneCAN_SendESCCommand(0, 0);
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
 * @param mode 0=OPERATIONAL, 1=INITIALIZATION, 2=MAINTENANCE, 3=SOFTWARE_UPDATE, 7=OFFLINE
 */
void DroneCAN_SetMode(uint8_t mode) {
    node_mode = mode & 0x07;
}

/**
 * @brief Get time since last ESC command
 * @return Milliseconds since last command
 */
uint32_t DroneCAN_GetTimeSinceLastCommand(void) {
    return millis() - last_esc_command_ms;
}

/**
 * @brief Process DroneCAN tasks (call regularly)
 *
 * Handles:
 * - Heartbeat transmission (every 100ms)
 * - Uptime counter
 */
void DroneCAN_Process(void) {
    // Update uptime (once per second)
    static uint32_t last_uptime_update = 0;
    if (millis() - last_uptime_update >= 1000) {
        uptime_sec++;
        last_uptime_update = millis();
    }

    // Send heartbeat (every 100ms)
    if (millis() - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        DroneCAN_SendHeartbeat();
    }
}
