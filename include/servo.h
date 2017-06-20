/**
 * Move32 - Naze32 for robotics
 * Control RC Servos via PWM
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_SERVO_H
#define MOVE32_SERVO_H


#include <stdint.h>
#include <stdbool.h>


// **************************************************************************
// Controlling servos via PWM
// **************************************************************************

/// Number of servos to control
#define SERVOS_MAX 6

/**
 * Initialize all servo outputs, including GPIO pins and timers.
 */
extern void servosInit();


/**
 * Get the fail save pulse length.
 * @param index Channel/Servo index from [0, SERVOS_MAX[
 */
extern uint16_t servoGetFailsafe(uint8_t index);

/**
 * Set the fail save pulse length.
 * @param index Channel/Servo index from [0, SERVOS_MAX[
 * @param pulseLengthUs pulse length in us
 */
extern void servoSetFailsafe(uint8_t index, uint16_t pulseLengthUs);

/**
 * Output pulses of the given length to the servo/channel or
 * the fail save pulse if the given pulse length is zero.
 * @param index Channel/Servo index from [0, SERVOS_MAX[
 * @param pulseLengthUs pulse length in us
 */
extern void servoWrite(uint8_t index,
                uint16_t pulseLengthUs);



#endif //MOVE32_SERVO_H
