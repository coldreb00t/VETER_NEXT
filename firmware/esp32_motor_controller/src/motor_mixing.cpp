/**
 * @file motor_mixing.cpp
 * @brief Motor mixing and differential steering implementation
 */

#include "motor_mixing.h"
#include "config.h"

// Last calculated motor commands (for slew rate limiting)
static MotorCommands last_commands = {0, 0};

/**
 * @brief Initialize motor mixing
 */
void MotorMixing_Init(void) {
    last_commands.left = 0;
    last_commands.right = 0;
    DEBUG_PRINTLN("[MotorMixing] Initialized");
}

/**
 * @brief Map CRSF value to normalized range [-1.0, +1.0]
 * @param raw_value Raw CRSF value (172-1811)
 * @param deadband Deadband around center
 * @return Normalized value [-1.0, +1.0]
 */
static float mapCRSFtoNormalized(uint16_t raw_value, uint16_t deadband = CRSF_DEADBAND) {
    // CRSF range: 172 (min) to 1811 (max), center at 992

    int16_t value = (int16_t)raw_value - CRSF_MID_VALUE;  // Convert to signed, centered at 0

    // Apply deadband
    if (abs(value) < (int16_t)deadband) {
        return 0.0f;
    }

    // Map to [-1.0, +1.0]
    float normalized;
    if (value > 0) {
        // Positive: map (deadband, max-center) to (0.0, 1.0)
        normalized = (float)(value - deadband) / (float)(CRSF_MAX_VALUE - CRSF_MID_VALUE - deadband);
    } else {
        // Negative: map (center-min, -deadband) to (-1.0, 0.0)
        normalized = (float)(value + deadband) / (float)(CRSF_MID_VALUE - CRSF_MIN_VALUE - deadband);
    }

    // Clamp to [-1.0, +1.0]
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    return normalized;
}

/**
 * @brief Map normalized value to ESC command
 * @param normalized Normalized value [-1.0, +1.0]
 * @param scale Scale factor (0.0-1.0)
 * @return ESC command (-8191 to +8191)
 */
static int16_t mapNormalizedToESC(float normalized, float scale = 1.0f) {
    float scaled = normalized * scale;

    // Clamp to [-1.0, +1.0]
    if (scaled > 1.0f) scaled = 1.0f;
    if (scaled < -1.0f) scaled = -1.0f;

    // Map to ESC command range
    int16_t command = (int16_t)(scaled * (float)ESC_COMMAND_MAX);

    return command;
}

/**
 * @brief Calculate motor commands from CRSF channels
 * @param throttle_raw Raw CRSF throttle value (172-1811, center=992)
 * @param steering_raw Raw CRSF steering value (172-1811, center=992)
 * @param failsafe If true, return zero commands
 * @return Motor commands for left and right motors
 */
MotorCommands MotorMixing_Calculate(uint16_t throttle_raw, uint16_t steering_raw, bool failsafe) {
    MotorCommands commands = {0, 0};

    // If failsafe, return zero commands
    if (failsafe) {
        last_commands = commands;
        return commands;
    }

    // Convert CRSF values to normalized [-1.0, +1.0]
    float throttle = mapCRSFtoNormalized(throttle_raw);
    float steering = mapCRSFtoNormalized(steering_raw);

    // Apply scaling
    throttle *= THROTTLE_SCALE;
    steering *= STEERING_SCALE;

    // Differential steering mixing
    // Formula:
    //   left = throttle + steering
    //   right = throttle - steering
    //
    // This creates arcade-style controls:
    // - Throttle forward + steering right: left motor faster, right motor slower (turn right)
    // - Throttle forward + steering left: right motor faster, left motor slower (turn left)
    // - Zero throttle + steering: tank turn (one motor forward, one reverse)

    float left_normalized = throttle + steering;
    float right_normalized = throttle - steering;

    // Apply max differential limit (prevent one motor from being too much faster than the other)
    float differential = abs(left_normalized - right_normalized) / 2.0f;
    if (differential > MAX_DIFFERENTIAL) {
        float scale_factor = MAX_DIFFERENTIAL / differential;
        left_normalized *= scale_factor;
        right_normalized *= scale_factor;
    }

    // Clamp to [-1.0, +1.0]
    if (left_normalized > 1.0f) left_normalized = 1.0f;
    if (left_normalized < -1.0f) left_normalized = -1.0f;
    if (right_normalized > 1.0f) right_normalized = 1.0f;
    if (right_normalized < -1.0f) right_normalized = -1.0f;

    // Apply max speed limit
    float speed_limit = (float)MAX_SPEED_PERCENT / 100.0f;
    left_normalized *= speed_limit;
    right_normalized *= speed_limit;

    // Convert to ESC commands
    commands.left = mapNormalizedToESC(left_normalized);
    commands.right = mapNormalizedToESC(right_normalized);

    // Apply slew rate limiting
    commands = MotorMixing_ApplySlewRate(commands, MAX_ACCELERATION);

    // Store last commands
    last_commands = commands;

    return commands;
}

/**
 * @brief Get last calculated motor commands
 * @return Last motor commands
 */
MotorCommands MotorMixing_GetLastCommands(void) {
    return last_commands;
}

/**
 * @brief Apply slew rate limiting to motor commands
 * @param target Target motor commands
 * @param max_delta Maximum change per update
 * @return Slew rate limited commands
 */
MotorCommands MotorMixing_ApplySlewRate(MotorCommands target, int16_t max_delta) {
    MotorCommands limited = target;

    // Limit left motor
    int16_t left_delta = target.left - last_commands.left;
    if (abs(left_delta) > max_delta) {
        limited.left = last_commands.left + (left_delta > 0 ? max_delta : -max_delta);
    }

    // Limit right motor
    int16_t right_delta = target.right - last_commands.right;
    if (abs(right_delta) > max_delta) {
        limited.right = last_commands.right + (right_delta > 0 ? max_delta : -max_delta);
    }

    return limited;
}
