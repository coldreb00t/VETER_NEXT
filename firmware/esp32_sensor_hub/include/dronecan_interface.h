/**
 * @file dronecan_interface.h
 * @brief DroneCAN interface header for Sensor Hub
 */

#ifndef DRONECAN_INTERFACE_H
#define DRONECAN_INTERFACE_H

#include <mcp2515_can.h>
#include <stdint.h>

// Sensor data structures
struct SonarData {
    float distance_cm[4];       // Front, Rear, Left, Right
    bool valid[4];              // Validity flags
    uint32_t timestamp_ms;
};

struct BME280Data {
    float temperature_c;        // Temperature in Celsius
    float humidity_percent;     // Relative humidity
    float pressure_hpa;         // Atmospheric pressure in hPa
    bool valid;
    uint32_t timestamp_ms;
};

struct ServoCommand {
    uint16_t pan_deg;           // Pan position (0-180 degrees)
    uint16_t tilt_deg;          // Tilt position (0-180 degrees)
    uint32_t timestamp_ms;
};

struct LEDCommand {
    uint8_t mode;               // LED mode (off, auto, on, blink, etc.)
    uint8_t brightness;         // Brightness (0-255)
    uint8_t front_left;         // Front left LED (0-255)
    uint8_t front_right;        // Front right LED (0-255)
    uint8_t rear_left;          // Rear left LED (0-255)
    uint8_t rear_right;         // Rear right LED (0-255)
    uint32_t timestamp_ms;
};

// Function prototypes

// Initialization
bool DroneCAN_Init(void);

// Node status
void DroneCAN_SendHeartbeat(void);
void DroneCAN_SetHealth(uint8_t health);
void DroneCAN_SetMode(uint8_t mode);

// Sensor data publishing
void DroneCAN_PublishSonarData(const SonarData* data);
void DroneCAN_PublishBME280Data(const BME280Data* data);
void DroneCAN_PublishCollisionWarning(uint8_t direction, float distance_cm);

// Command reception
void DroneCAN_Process(void);
bool DroneCAN_GetServoCommand(ServoCommand* cmd);
bool DroneCAN_GetLEDCommand(LEDCommand* cmd);

// Utility functions
uint32_t DroneCAN_GetTimeSinceLastCommand(void);
bool DroneCAN_IsConnected(void);

#endif // DRONECAN_INTERFACE_H
