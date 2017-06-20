/**
 * Move32 - Naze32 for robotics
 * Control RC Servos via PWM
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include "board.h"
#include "servo.h"


// **************************************************************************
// Controlling servos via PWM
// **************************************************************************

/// type to describe a servo (pwm) output
typedef struct {
    volatile uint32_t *ccrPtr;     //< pointer to Capture Compare Register
    TIM_TypeDef *timerInstancePtr; //< pointer to timer peripheral's registers
    uint16_t failSafePulseUs;      //< pulse length (us) for the fail safe position
} servo_t;

static servo_t servos[SERVOS_MAX];


/**
 * Initialize @see servos array for one servo.
 * @param index Index of the servo to initialize from [0, SERVOS_MAX[
 * @param ccrPtr Pointer to the Capture Compare Register (CCR) of the timer channel
 *        controlling the servo output
 * @param timerInstancePtr Pointer to the timer's register block, e.g. TIM1
 * @param failSafePulseUs Default pulse length for the servo in us, e.g. 1500
 */
static void servoInit(uint8_t index,
                      volatile uint32_t *ccrPtr,
                      TIM_TypeDef *timerInstancePtr,
                      uint16_t failSafePulseUs) {
    servos[index].ccrPtr = ccrPtr;
    servos[index].timerInstancePtr = timerInstancePtr;
    servos[index].failSafePulseUs = failSafePulseUs;
}

void servosInit() {
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

    servoInit(0, &(TIM1->CCR1), TIM1, 1500);
    servoInit(1, &(TIM1->CCR4), TIM1, 1500);
    servoInit(2, &(TIM4->CCR1), TIM4, 1500);
    servoInit(3, &(TIM4->CCR2), TIM4, 1500);
    servoInit(4, &(TIM4->CCR3), TIM4, 1500);
    servoInit(5, &(TIM4->CCR4), TIM4, 1500);
}

uint16_t servoGetFailsafe(uint8_t index) {
    uint16_t ret = 0;
    if (index < SERVOS_MAX) {
        ret = servos[index].failSafePulseUs;
    }
    return ret;
}

void servoSetFailsafe(uint8_t index, uint16_t pulseLengthUs) {
    if (index < SERVOS_MAX) {
        servos[index].failSafePulseUs = pulseLengthUs;
    }
}

void servoWrite(uint8_t index,
                uint16_t pulseLengthUs) {
    if (index < SERVOS_MAX && servos[index].ccrPtr) {
        if (pulseLengthUs == 0) {
            *(servos[index].ccrPtr) = servos[index].failSafePulseUs;
        } else {
            *(servos[index].ccrPtr) = pulseLengthUs;

        }
    }
}

