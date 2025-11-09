/**
 * @file dronecan_interface.cpp
 * @brief DroneCAN interface implementation for ESP32 Motor Controller
 *
 * Handles all DroneCAN communication including:
 * - Node initialization and heartbeat
 * - ESC command transmission
 * - Node status reporting
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

// ESC command state
static int16_t esc_commands[2] = {0, 0};  // Left and right motor commands
static uint32_t last_esc_command_ms = 0;

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

    DEBUG_PRINTF("[DroneCAN] Initialized as Node ID %d\n", DRONECAN_NODE_ID);
    DEBUG_PRINTF("[DroneCAN] Node name: %s\n", DRONECAN_NODE_NAME);

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
        DEBUG_PRINTF("[DroneCAN] Heartbeat sent (uptime: %lu s)\n", uptime_sec);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send heartbeat");
    }
}

/**
 * @brief Send ESC RawCommand to motor controllers
 * @param left_cmd Left motor command (-8191 to 8191)
 * @param right_cmd Right motor command (-8191 to 8191)
 */
void DroneCAN_SendESCCommand(int16_t left_cmd, int16_t right_cmd) {
    // Store commands
    esc_commands[ESC_LEFT_INDEX] = constrain(left_cmd, ESC_COMMAND_MIN, ESC_COMMAND_MAX);
    esc_commands[ESC_RIGHT_INDEX] = constrain(right_cmd, ESC_COMMAND_MIN, ESC_COMMAND_MAX);

    // Update timestamp
    last_esc_command_ms = millis();

    // Prepare ESC RawCommand message
    // DroneCAN message ID: 1030 (0x406)
    // Data: Array of int14 values (14-bit signed integers)

    struct can_frame frame;
    frame.can_id = 0x406;  // ESC RawCommand message type
    frame.can_dlc = 4;     // 2 motors * 2 bytes each

    // Pack two 14-bit values into 3.5 bytes (we'll use 4 bytes)
    // Format: [cmd0_low, cmd0_high, cmd1_low, cmd1_high]
    frame.data[0] = esc_commands[0] & 0xFF;
    frame.data[1] = (esc_commands[0] >> 8) & 0x3F;  // Only 6 bits
    frame.data[2] = esc_commands[1] & 0xFF;
    frame.data[3] = (esc_commands[1] >> 8) & 0x3F;  // Only 6 bits

    // Add source node ID
    frame.can_id |= (DRONECAN_NODE_ID & 0x7F);

    // Send CAN frame
    if (can0.sendMessage(&frame) == MCP2515::ERROR_OK) {
        #if DEBUG_ESC_COMMANDS
        DEBUG_PRINTF("[DroneCAN] ESC Command: L=%d R=%d\n", left_cmd, right_cmd);
        #endif
    } else {
        DEBUG_PRINTLN("[DroneCAN] ERROR: Failed to send ESC command");
    }
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
 * @param mode 0=OPERATIONAL, 1=INITIALIZATION, etc.
 */
void DroneCAN_SetMode(uint8_t mode) {
    node_mode = mode & 0x07;
}

/**
 * @brief Get milliseconds since last ESC command
 * @return Time in milliseconds
 */
uint32_t DroneCAN_GetTimeSinceLastCommand(void) {
    return millis() - last_esc_command_ms;
}

/**
 * @brief Process incoming DroneCAN messages (if any)
 */
void DroneCAN_Process(void) {
    struct can_frame frame;

    // Check for received messages
    if (can0.readMessage(&frame) == MCP2515::ERROR_OK) {
        // Process received message (currently not implemented)
        // Future: Handle configuration messages, parameter updates, etc.
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] RX: ID=0x%lX DLC=%d\n", frame.can_id, frame.can_dlc);
        #endif
    }
}
