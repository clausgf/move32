/**
 * Move32 - Naze32 for robotics
 * Decode signals from a rc receiver
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include "board.h"
#include "receiver.h"


// **************************************************************************

Receiver receiver;

void receiverInit() {
    receiver.init();
}


// **************************************************************************
// Input Capture interrupt handling for rx decoding
// **************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback called within the HAL's timer interrupt handler
 * Beware of the poor HAL design (again):
 * - hTimer->Channel indicates the timer channel as HAL_TIM_ACTIVE_CHANNEL_x
 * - HAL_TIM_ReadCaputredValue needs a TIM_CHANNEL_x
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hTimer) {
    receiver.onTimerEventFromIsr(hTimer);
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

#ifdef __cplusplus
}
#endif
