/**
 * Move32 - Naze32 for robotics
 * Decode signals from a rc receiver via input capture
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_RX_H
#define MOVE32_RX_H


#include <stdint.h>
#include <stdbool.h>


// **************************************************************************
// Decode input signals (channels) from the rc receiver (rx)
// via input capture
// **************************************************************************

/// Number of channels to decode
#define RX_MAX_CHANNEL 8

/**
 * Initialize all rx channel inputs, including GPIO pins and timers.
 */
extern void rxInit();

/**
 * Timeout (in HAL Ticks or ms) for rx decoding - after the timeout
 * has expired without a signal, the the channel's state is considered unknown
 * @param timeoutMs timeout in ms after which a received pulse
 *        is considered outdated
 */
extern void rxSetTimeout(uint32_t timeoutMs);

/**
 * Return pulse length from the rc receiver in ms or zero for the given channel.
 * @param rxChannel Rx channel index from [0, RX_MAX_CHANNEL[
 * @return pulse length in us or zero if pulse length unknown or too old
 */
extern uint16_t rxRead(uint8_t rxChannel);

/**
 * This routine is called from the input capture ISR when a new pulse length
 * is available. It has to be implemented outside this module by the user.
 * @param rxChannel Rx channel index from [0, RX_MAX_CHANNEL[
 * @param pulseLenUs Pulse length in us
 */
extern void rxUpdateCallback(uint8_t rxChannel, uint16_t pulseLenUs);



#endif //MOVE32_RX_H
