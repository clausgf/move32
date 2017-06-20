/**
 * Move32 - Naze32 for robotics
 * Decode signals from a rc receiver
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include "board.h"
#include "rx.h"


// **************************************************************************
// Decode input signals (channels) from the rc receiver (rx)
// via input capture
// **************************************************************************

/// structure to control decoding a channel of a receiver
typedef struct {
    TIM_HandleTypeDef *hTimer;  //< timer for this rx input
    uint32_t timerChannel;      //< timer channel for this rx input
    uint32_t timerActiveChannel;//< timer active channel for this rx input
    bool wasRisingEdgeDetected; //< flag to remember whether the last ic event
    uint16_t pulseStartTime;    //< timer/counter value of last rising edge
    uint16_t pulseEndTime;      //< timer/counter value of last falling edge
    uint16_t pulseLength;       //< length of pulse (us)
    uint32_t lastUpdateTime;    //< system ticker (ms) of last update
} rxChannel_t;

rxChannel_t rxChannels[RX_MAX_CHANNEL];

uint32_t rxTimeoutMs;

/**
 *
 * @param rxChannel Index of the rx channel to initialize from [0, RX_MAX_CHANNEL[
 * @param hTimer STHAL handle for the timer, e.g. htim1
 * @param timerChannel STHAL constant for the timer channel
 *        (which is *not* the channel number),
 *        e.g. TIM_CHANNEL_1
 * @param timerActiveChannel STHAL constant for the timer "active channel",
 *        which to the author's understanding is a different constant
 *        referencing the same timer channel as timerChannel. Constants are
 *        HAL_TIM_ACTIVE_CHANNEL_1 etc. from the HAL_TIM_ActiveChannel enum.
 */
static void rxChannelInit(uint8_t rxChannel,
                          TIM_HandleTypeDef *hTimer,
                          uint32_t timerChannel,
                          uint32_t timerActiveChannel) {
    rxChannels[rxChannel].hTimer = hTimer;
    rxChannels[rxChannel].timerChannel = timerChannel;
    rxChannels[rxChannel].timerActiveChannel = timerActiveChannel;
    rxChannels[rxChannel].wasRisingEdgeDetected = false;
    rxChannels[rxChannel].pulseStartTime = 0;
    rxChannels[rxChannel].pulseEndTime = 0;
    rxChannels[rxChannel].pulseLength = 0;
    icInit(hTimer, timerChannel, TIM_ICPOLARITY_RISING);
}

void rxInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // initialize timer ic input pins
    GPIO_InitTypeDef gpioInitStruct;
    gpioInitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                         | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
    gpioInitStruct.Mode = GPIO_MODE_INPUT;
    gpioInitStruct.Pull = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpioInitStruct);

    gpioInitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpioInitStruct.Mode = GPIO_MODE_INPUT;
    gpioInitStruct.Pull = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpioInitStruct);

    timerInit(&htim2, TIM2, 0);
    rxChannelInit(0, &htim2, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1);
    rxChannelInit(1, &htim2, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2);
    rxChannelInit(2, &htim2, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3);
    rxChannelInit(3, &htim2, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn); // enable IRQ for input capture

    timerInit(&htim3, TIM3, 0);
    rxChannelInit(4, &htim3, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1);
    rxChannelInit(5, &htim3, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2);
    rxChannelInit(6, &htim3, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3);
    rxChannelInit(7, &htim3, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM3_IRQn); // enable IRQ for input capture

    rxTimeoutMs = 50;
}

uint16_t rxRead(uint8_t rxChannel) {
    uint16_t ret = 0;
    if (rxChannel < RX_MAX_CHANNEL) {
        if ((HAL_GetTick() - rxChannels[rxChannel].lastUpdateTime) < rxTimeoutMs) {
            ret = rxChannels[rxChannel].pulseLength;
        } else {
            // we're in a timeout situation, let's make sure that this is conserved
            // even in timer wraparounds
            rxChannels[rxChannel].lastUpdateTime = HAL_GetTick() - rxTimeoutMs;
        }
    }
    return ret;
}

void rxSetTimeout(uint32_t timeoutMs) {
    rxTimeoutMs = timeoutMs;

    // preserve timeout condition for channels already timed out
    for (uint8_t index=0; index<RX_MAX_CHANNEL; index++) {
        if ((HAL_GetTick() - rxChannels[index].lastUpdateTime) >= rxTimeoutMs) {
            rxChannels[index].lastUpdateTime = HAL_GetTick() - rxTimeoutMs;
        }
    }
}


// **************************************************************************
// Input Capture interrupt handling for rx decoding
// **************************************************************************

/**
 * Callback called within the HAL's timer interrupt handler
 * Beware of the poor HAL design (again):
 * - hTimer->Channel indicates the timer channel as HAL_TIM_ACTIVE_CHANNEL_x
 * - HAL_TIM_ReadCaputredValue needs a TIM_CHANNEL_x
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hTimer) {
    for (uint8_t rxChannelIndex = 0; rxChannelIndex < RX_MAX_CHANNEL; rxChannelIndex++) {
        rxChannel_t *channelPtr = &rxChannels[rxChannelIndex];
        // check whether the IRQ was generated for this timer and channel
        if (hTimer == channelPtr->hTimer && hTimer->Channel == channelPtr->timerActiveChannel) {
            uint32_t nextPolarity;
            if (channelPtr->wasRisingEdgeDetected) {
                // rising edge already detected, this is the falling edge
                channelPtr->pulseEndTime = HAL_TIM_ReadCapturedValue(hTimer, channelPtr->timerChannel);
                channelPtr->pulseLength = channelPtr->pulseEndTime - channelPtr->pulseStartTime;
                channelPtr->lastUpdateTime = HAL_GetTick();
                channelPtr->wasRisingEdgeDetected = false;
                nextPolarity = TIM_ICPOLARITY_RISING;
            } else {
                // this is the rising edge
                channelPtr->pulseStartTime = HAL_TIM_ReadCapturedValue(hTimer, channelPtr->timerChannel);
                channelPtr->wasRisingEdgeDetected = true;
                nextPolarity = TIM_ICPOLARITY_FALLING;
            }
            // reconfigure for next edge:
            // another unexpected, annoying HAL "feature": reconfiguration
            // using HAL_TIM_IC_ConfigChannel requires HAL_TIM_IC_Start_IT;
            // thus just call icInit for reconfiguration
            icInit(channelPtr->hTimer, channelPtr->timerChannel, nextPolarity);

            rxUpdateCallback(rxChannelIndex, channelPtr->pulseLength);
        }
    }
}

/**
 * On Timer 2 IRQ, call the HAL timer interrupt handler, which determines
 * the kind of IRQ and calls @see HAL_TIM_IC_CaptureCallback on IC IRQs.
 */
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}


/**
 * On Timer 3 IRQ, call the HAL timer interrupt handler, which determines
 * the kind of IRQ and calls @see HAL_TIM_IC_CaptureCallback on IC IRQs.
 */
void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
}


