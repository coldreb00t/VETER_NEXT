/**
 * @file motor_mixing.h
 * @brief Motor mixing and differential steering logic
 */

#ifndef MOTOR_MIXING_H
#define MOTOR_MIXING_H

#include <Arduino.h>

/**
 * @brief Motor command structure
 */
struct MotorCommands {
    int16_t left;   // Left motor command (-8191 to +8191)
    int16_t right;  // Right motor command (-8191 to +8191)
};

/**
 * @brief Initialize motor mixing
 */
void MotorMixing_Init(void);

/**
 * @brief Calculate motor commands from CRSF channels
 * @param throttle_raw Raw CRSF throttle value (172-1811, center=992)
 * @param steering_raw Raw CRSF steering value (172-1811, center=992)
 * @param failsafe If true, return zero commands
 * @return Motor commands for left and right motors
 */
MotorCommands MotorMixing_Calculate(uint16_t throttle_raw, uint16_t steering_raw, bool failsafe = false);

/**
 * @brief Get last calculated motor commands
 * @return Last motor commands
 */
MotorCommands MotorMixing_GetLastCommands(void);

/**
 * @brief Apply slew rate limiting to motor commands
 * @param target Target motor commands
 * @param max_delta Maximum change per update
 * @return Slew rate limited commands
 */
MotorCommands MotorMixing_ApplySlewRate(MotorCommands target, int16_t max_delta);

#endif // MOTOR_MIXING_H
