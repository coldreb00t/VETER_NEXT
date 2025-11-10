/**
 * @file dronecan_interface.h
 * @brief DroneCAN interface header using ESP32 TWAI
 */

#ifndef DRONECAN_INTERFACE_H
#define DRONECAN_INTERFACE_H

#include <Arduino.h>
#include "driver/twai.h"

// Function prototypes
bool DroneCAN_Init(void);
void DroneCAN_SendHeartbeat(void);
void DroneCAN_SendESCCommand(int16_t left_cmd, int16_t right_cmd);
void DroneCAN_StopMotors(void);
void DroneCAN_SetHealth(uint8_t health);
void DroneCAN_SetMode(uint8_t mode);
uint32_t DroneCAN_GetTimeSinceLastCommand(void);
void DroneCAN_Process(void);

#endif // DRONECAN_INTERFACE_H
