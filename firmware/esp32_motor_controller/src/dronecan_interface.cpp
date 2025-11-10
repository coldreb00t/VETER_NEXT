/**
 * @file dronecan_interface.cpp
 * @brief DroneCAN interface implementation using ESP32 TWAI (CAN controller)
 *
 * This implementation uses ESP32-S3's built-in TWAI controller with WCMCU-230
 * transceiver (SN65HVD230) for CAN bus communication.
 *
 * Hardware: ESP32 TWAI (TX=GPIO4, RX=GPIO5) → WCMCU-230 → CAN Bus
 */

#include "dronecan_interface.h"
#include "config.h"

// ============================================================================
// DRONECAN PROTOCOL CONSTANTS
// ============================================================================

// DroneCAN Message Type IDs
#define UAVCAN_NODE_STATUS_ID           341     // NodeStatus (Heartbeat)
#define UAVCAN_ESC_RAW_COMMAND_ID       1030    // ESC RawCommand
#define UAVCAN_ESC_STATUS_ID            1034    // ESC Status

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

static uint8_t node_health = HEALTH_OK;
static uint8_t node_mode = MODE_INITIALIZATION;
static uint32_t uptime_sec = 0;
static uint32_t last_heartbeat_ms = 0;
static uint32_t last_esc_command_ms = 0;

// Transfer ID counters (0-31, wraps around)
static uint8_t transfer_id_heartbeat = 0;
static uint8_t transfer_id_esc_cmd = 0;

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

// ============================================================================
// TWAI HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Send a CAN message using TWAI
 */
static bool TWAI_SendMessage(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len > 8) {
        return false;  // CAN frame max is 8 bytes
    }

    twai_message_t message;
    message.identifier = can_id;
    message.extd = 1;  // Extended 29-bit ID
    message.rtr = 0;   // Data frame (not remote request)
    message.ss = 0;    // Not single shot
    message.self = 0;  // Not self-reception
    message.dlc_non_comp = 0;
    message.data_length_code = len;

    // Copy data
    for (int i = 0; i < len; i++) {
        message.data[i] = data[i];
    }

    // Send message (non-blocking with 10ms timeout)
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));

    if (result == ESP_OK) {
        return true;
    } else if (result == ESP_ERR_TIMEOUT) {
        DEBUG_PRINTLN("[TWAI] TX timeout");
        return false;
    } else {
        DEBUG_PRINTF("[TWAI] TX error: %d\n", result);
        return false;
    }
}

/**
 * @brief Receive CAN messages (non-blocking)
 */
static bool TWAI_ReceiveMessage(twai_message_t* message) {
    esp_err_t result = twai_receive(message, pdMS_TO_TICKS(0));  // Non-blocking
    return (result == ESP_OK);
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
// DRONECAN PROTOCOL FUNCTIONS
// ============================================================================

/**
 * @brief Initialize DroneCAN (TWAI controller)
 */
bool DroneCAN_Init(void) {
    DEBUG_PRINTLN("[DroneCAN] Initializing TWAI controller...");

    // TWAI general configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );

    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;

    // TWAI timing configuration for 1 Mbps
    // ESP32-S3 @ 80MHz APB clock
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

    // TWAI filter configuration (accept all messages)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        DEBUG_PRINTF("[DroneCAN] TWAI driver install failed: %d\n", result);
        return false;
    }

    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        DEBUG_PRINTF("[DroneCAN] TWAI start failed: %d\n", result);
        twai_driver_uninstall();
        return false;
    }

    DEBUG_PRINTLN("[DroneCAN] TWAI initialized successfully");
    DEBUG_PRINTF("[DroneCAN] Node ID: %d\n", DRONECAN_NODE_ID);
    DEBUG_PRINTF("[DroneCAN] TX Pin: GPIO%d, RX Pin: GPIO%d\n", CAN_TX_PIN, CAN_RX_PIN);
    DEBUG_PRINTLN("[DroneCAN] Bitrate: 1 Mbps");

    node_mode = MODE_OPERATIONAL;
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
    if (TWAI_SendMessage(can_id, payload, 8)) {
        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] Heartbeat sent: uptime=%lu, health=%d, mode=%d\n",
                     uptime_sec, node_health, node_mode);
        #endif
        incrementTransferID(&transfer_id_heartbeat);
    } else {
        DEBUG_PRINTLN("[DroneCAN] Failed to send heartbeat");
    }
}

/**
 * @brief Send ESC RawCommand message
 *
 * ESC RawCommand payload (variable length, 2 bytes per ESC):
 * For each ESC:
 *   [0-1] Command value (int16_t, little-endian, range: -8191 to +8191)
 * [N] Tail byte (transfer ID)
 *
 * We send 2 ESC commands: Left (index 0) and Right (index 1)
 */
void DroneCAN_SendESCCommand(int16_t left_cmd, int16_t right_cmd) {
    // Clamp commands to valid range
    if (left_cmd < ESC_COMMAND_MIN) left_cmd = ESC_COMMAND_MIN;
    if (left_cmd > ESC_COMMAND_MAX) left_cmd = ESC_COMMAND_MAX;
    if (right_cmd < ESC_COMMAND_MIN) right_cmd = ESC_COMMAND_MIN;
    if (right_cmd > ESC_COMMAND_MAX) right_cmd = ESC_COMMAND_MAX;

    uint8_t payload[5];  // 2 ESCs × 2 bytes + 1 tail byte = 5 bytes

    // Left motor command (ESC index 0)
    payload[0] = (left_cmd >> 0) & 0xFF;   // Low byte
    payload[1] = (left_cmd >> 8) & 0xFF;   // High byte

    // Right motor command (ESC index 1)
    payload[2] = (right_cmd >> 0) & 0xFF;  // Low byte
    payload[3] = (right_cmd >> 8) & 0xFF;  // High byte

    // Tail byte
    payload[4] = buildTailByte(transfer_id_esc_cmd);

    // Build CAN ID
    uint32_t can_id = encodeDroneCAN_ID(PRIORITY_HIGH, UAVCAN_ESC_RAW_COMMAND_ID, DRONECAN_NODE_ID);

    // Send message
    if (TWAI_SendMessage(can_id, payload, 5)) {
        #if DEBUG_ESC_COMMANDS
        DEBUG_PRINTF("[ESC] L=%d R=%d\n", left_cmd, right_cmd);
        #endif
        incrementTransferID(&transfer_id_esc_cmd);
        last_esc_command_ms = millis();
    } else {
        DEBUG_PRINTLN("[DroneCAN] Failed to send ESC command");
    }
}

/**
 * @brief Stop motors immediately
 */
void DroneCAN_StopMotors(void) {
    DroneCAN_SendESCCommand(0, 0);
}

/**
 * @brief Set node health status
 */
void DroneCAN_SetHealth(uint8_t health) {
    if (health <= HEALTH_CRITICAL) {
        node_health = health;
    }
}

/**
 * @brief Set node mode
 */
void DroneCAN_SetMode(uint8_t mode) {
    node_mode = mode;
}

/**
 * @brief Get time since last ESC command was sent
 */
uint32_t DroneCAN_GetTimeSinceLastCommand(void) {
    return millis() - last_esc_command_ms;
}

/**
 * @brief Process DroneCAN (heartbeat, receive messages)
 *
 * This function should be called frequently (e.g., every loop iteration)
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

    // Process incoming messages (non-blocking)
    twai_message_t rx_message;
    while (TWAI_ReceiveMessage(&rx_message)) {
        // Handle received messages here (future expansion)
        // For now, we only transmit (motor controller role)

        #if DEBUG_DRONECAN
        DEBUG_PRINTF("[DroneCAN] RX: ID=0x%08lX, DLC=%d\n",
                     rx_message.identifier,
                     rx_message.data_length_code);
        #endif
    }

    // Check for bus errors and recover if needed
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state == TWAI_STATE_BUS_OFF) {
            DEBUG_PRINTLN("[TWAI] Bus-off detected, attempting recovery...");
            twai_initiate_recovery();
            delay(100);
            twai_start();
        }
    }
}
