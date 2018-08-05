/**
 * Move32 - Naze32 for robotics
 * Control RC Servos via PWM
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_SERVOS_H
#define MOVE32_SERVOS_H


#include <stdint.h>
#include <stdbool.h>

#include "board.h"


// **************************************************************************

/// Number of servos to control
#define SERVOS_MAX 6

// **************************************************************************

/**
 * Abstraction for a single servo
 */
class Servo {
public:

    Servo():
        mCcrPtr(nullptr),
        mTimerInstancePtr(nullptr),
        mFailSafePulseUs(1500) {}

    /**
     * Initialize a servo.
     * @param ccrPtr Pointer to the Capture Compare Register (CCR) of the timer channel
     *        controlling the servo output
     * @param timerInstancePtr Pointer to the timer's register block, e.g. TIM1
     * @param failSafePulseUs Default pulse length for the servo in us, e.g. 1500
     */
    void init(volatile uint32_t *ccrPtr, TIM_TypeDef *timerInstancePtr, uint16_t failSafePulseUs) {
        mCcrPtr = ccrPtr;
        mTimerInstancePtr = timerInstancePtr;
        mFailSafePulseUs = failSafePulseUs;
    }

    /**
     * Get the fail save pulse length.
     */
    uint16_t getFailsafe() { return mFailSafePulseUs; }

    /**
     * Set the fail save pulse length.
     * @param pulseLengthUs pulse length in us
     */
    void setFailsafe(uint16_t pulseLengthUs) { mFailSafePulseUs = pulseLengthUs; }

    /**
     * Output pulses of the given length to the servo/channel or
     * the fail save pulse if the given pulse length is zero.
     * @param pulseLengthUs pulse length in us
     */
    void set(uint16_t pulseLengthUs) {
        if (mCcrPtr) {
            if (pulseLengthUs == 0) {
                *(mCcrPtr) = mFailSafePulseUs;
            } else {
                *(mCcrPtr) = pulseLengthUs;
            }
        }
    }

private:
    volatile uint32_t *mCcrPtr;     //< pointer to Capture Compare Register
    TIM_TypeDef *mTimerInstancePtr; //< pointer to timer peripheral's registers
    uint16_t mFailSafePulseUs;      //< pulse length (us) for the fail safe position
};


// **************************************************************************

/**
 * Hardware abstraction to control servos via PWM
 */
class Servos {
public:

    Servos() { }

    /**
     * Initialize all servo outputs, including GPIO pins and timers.
     */
    void init() {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_TIM4_CLK_ENABLE();

        // initialize timer output pins
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_11;
        gpioInitStruct.Mode = GPIO_MODE_AF_PP;
        gpioInitStruct.Pull = GPIO_NOPULL;
        gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpioInitStruct);

        gpioInitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP; // TODO GPIO_MODE_AF_PP;
        gpioInitStruct.Pull = GPIO_NOPULL;
        gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &gpioInitStruct);

        timerInit(&htim1, TIM1, 20000);
        pwmInit(&htim1, TIM_CHANNEL_1, 1500);
        pwmInit(&htim1, TIM_CHANNEL_4, 1500);

        timerInit(&htim4, TIM4, 20000);
        pwmInit(&htim4, TIM_CHANNEL_1, 1500);
        pwmInit(&htim4, TIM_CHANNEL_2, 1500);
        pwmInit(&htim4, TIM_CHANNEL_3, 1500);
        pwmInit(&htim4, TIM_CHANNEL_4, 1500);

        mServos[0].init(&(TIM1->CCR1), TIM1, 1500);
        mServos[1].init(&(TIM1->CCR4), TIM1, 1500);
        mServos[2].init(&(TIM4->CCR1), TIM4, 1500);
        mServos[3].init(&(TIM4->CCR2), TIM4, 1500);
        mServos[4].init(&(TIM4->CCR3), TIM4, 1500);
        mServos[5].init(&(TIM4->CCR4), TIM4, 1500);
    }

    /**
     * Get a reference to a servo.
     * @param index Channel/Servo index from [0, SERVOS_MAX[
     */
    Servo &operator[](uint8_t index) {
        if (index >= SERVOS_MAX) {
            board.fatal(__FILE__, __LINE__, "Servo index out of bounds");
        }
        return mServos[index];
    }

private:
    Servo mServos[SERVOS_MAX];

};


// **************************************************************************

extern Servos servos;

extern void servosInit();

// **************************************************************************


#endif //MOVE32_SERVOS_H
