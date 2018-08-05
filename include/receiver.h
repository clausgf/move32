/**
 * Move32 - Naze32 for robotics
 * Decode signals from a rc receiver via input capture
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_RECEIVER_H
#define MOVE32_RECEIVER_H


#include <stdint.h>
#include <stdbool.h>

#include "board.h"


/// Number of channels to decode
#define RX_MAX_CHANNEL 8


extern void rxUpdateCallback(uint8_t rxChannel, uint16_t pulseLenUs);


/**
 * Hardware abstraction to decode input signals (channels) from the
 * rc receiver (rx) via input capture
 */
class Receiver {
public:

    Receiver() { }

    /**
     * Initialize all rx channel inputs, including GPIO pins and timers.
     */
    void init() {
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

        mTimeoutMs = 50;
    }

    uint32_t getTimeout(void) { return mTimeoutMs; }

    /**
     * Timeout (in HAL Ticks or ms) for rx decoding - after the timeout
     * has expired without a signal, the the channel's state is considered unknown
     * @param timeoutMs timeout in ms after which a received pulse
     *        is considered outdated
     */
    void setTimeout(uint32_t timeoutMs) {
        mTimeoutMs = timeoutMs;

        // preserve timeout condition for channels already timed out
        for (uint8_t index=0; index<RX_MAX_CHANNEL; index++) {
            if ((HAL_GetTick() - mChannels[index].lastUpdateTime) >= mTimeoutMs) {
                mChannels[index].lastUpdateTime = HAL_GetTick() - mTimeoutMs;
            }
        }
    }

    /**
     * Return pulse length from the rc receiver in ms or zero for the given channel.
     * @param rxChannel Rx channel index from the interval [0, RX_MAX_CHANNEL[
     * @return pulse length in us or zero if pulse length unknown or too old
     */
    uint16_t getChannel(uint8_t rxChannel) {
        uint16_t ret = 0;
        if (rxChannel < RX_MAX_CHANNEL) {
            if ((HAL_GetTick() - mChannels[rxChannel].lastUpdateTime) < mTimeoutMs) {
                ret = mChannels[rxChannel].pulseLength;
            } else {
                // we're in a timeout situation, let's make sure that this is conserved
                // even in timer wraparounds
                mChannels[rxChannel].lastUpdateTime = HAL_GetTick() - mTimeoutMs;
            }
        }
        return ret;
    }

    /**
     * This routine is called from the input capture ISR when a new pulse length
     * is available. It has to be implemented outside this module by the user.
     * @param rxChannel Rx channel index from [0, RX_MAX_CHANNEL[
     * @param pulseLenUs Pulse length in us
     */
    void updateChannelFromIsr(uint8_t rxChannel, uint16_t pulseLenUs) {}


    /**
     * Called from the input capture ISR when an edge has beed detected.
     * @param hTimer
     */
    void onTimerEventFromIsr(TIM_HandleTypeDef *hTimer) {
        for (uint8_t rxChannelIndex = 0; rxChannelIndex < RX_MAX_CHANNEL; rxChannelIndex++) {
            rxChannel_t *channelPtr = &mChannels[rxChannelIndex];
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

                updateChannelFromIsr(rxChannelIndex, channelPtr->pulseLength);
            }
        }
    }


private:

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

    rxChannel_t mChannels[RX_MAX_CHANNEL];

    uint32_t mTimeoutMs;



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
    void rxChannelInit(uint8_t rxChannel,
            TIM_HandleTypeDef *hTimer,
            uint32_t timerChannel,
            uint32_t timerActiveChannel) {
        mChannels[rxChannel].hTimer = hTimer;
        mChannels[rxChannel].timerChannel = timerChannel;
        mChannels[rxChannel].timerActiveChannel = timerActiveChannel;
        mChannels[rxChannel].wasRisingEdgeDetected = false;
        mChannels[rxChannel].pulseStartTime = 0;
        mChannels[rxChannel].pulseEndTime = 0;
        mChannels[rxChannel].pulseLength = 0;
        icInit(hTimer, timerChannel, TIM_ICPOLARITY_RISING);
    }

};


// **************************************************************************

extern Receiver receiver;

extern void receiverInit();

// **************************************************************************

#endif //MOVE32_RECEIVER_H
