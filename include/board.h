/**
 * Move32 - Naze32 for robotics
 * High-Level HAL for board support
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_BOARD_H
#define MOVE32_BOARD_H


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f1xx_hal.h"

// **************************************************************************
// Handles for accessing the peripherals via STHAL
// **************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern IWDG_HandleTypeDef hiwdg;

extern UART_HandleTypeDef *syscall_UART_handle_ptr;

#ifdef __cplusplus
}
#endif


// **************************************************************************
// Timer initialization
// **************************************************************************

/**
 * Basic initialization for a timer running at 1 MHz from the system clock.
 * @param hTimer STHAL handle for the timer, e.g. htim1
 * @param timerInstancePtr Pointer to the timer's register block, e.g. TIM1
 * @param period Period (in us) for timer resets, e.g. 20000 for 20 ms
 */
extern void timerInit(TIM_HandleTypeDef *hTimer,
                      TIM_TypeDef *timerInstancePtr,
                      uint16_t period);

/**
 * Configure a timer channel for PWM output. GPIO pins and the basic
 * timer must be initialized separately (@see timerInit).
 * @param hTimer STHAL handle for the timer, e.g. htim1
 * @param timerChannel STHAL constant for the timer channel
 *        (which is *not* the channel number),
 *        e.g. TIM_CHANNEL_1
 * @param value Pulse length in timer ticks (here: us),
 *        e.g. 1500 for 1.5 ms pulse length
 */
extern void pwmInit(TIM_HandleTypeDef *hTimer,
                    uint32_t timerChannel,
                    uint16_t value);

/**
 * Configure a timer channel for input capture. GPIO pin and the basic
 * timer must be initialized separately (@see imterInit).
 * @param handle STHAL handle for the timer, e.g. htim1
 * @param timerChannel STHAL constant for the timer channel
 *        (which is *not* the channel number),
 *        e.g. TIM_CHANNEL_1
 * @param polarity Edge to trigger an input capture event,
 *        i.e. TIM_ICPOLARITY_RISING, TIM_ICPOLARITY_FALLING or TIM_ICPOLARITY_BOTHEDGE
 */
extern void icInit(TIM_HandleTypeDef *handle,
                   uint32_t timerChannel,
                   uint32_t polarity);


// **************************************************************************
// Handling of the watchdog timer
// **************************************************************************

/**
 * Abstraction for the watchdog timer which resets the system if not reset
 * after WATCHDOG_INTERVAL ms.
 * The timer is automatically initialized and started after
 * construction the object.
 *
 * WATCHDOG_INTERVAL = 1023 * 8 / 40 kHz ~ 1600 ms
 */
class Watchdog {
public:
    Watchdog() {
        mWasResetByWatchdog = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0;
    }

    // TODO watchdog timer for some reason does not reset the system
    void init() {/*
        // Configure and start IWDG
        hiwdg.Instance = IWDG;
        hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
        hiwdg.Init.Reload = 1023;
        if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
    */}

    bool getWasResetByWatchdog() { return mWasResetByWatchdog; }

    /**
     * Reset the watchdog timer, call this function periodically if you're
     * still alive!
     */
    void reset() {
        // TODO HAL_IWDG_Refresh(&hiwdg);
    }

private:
    bool mWasResetByWatchdog;
};


// **************************************************************************
// Handling of the GPIO digital I/O
// **************************************************************************

/**
 * Abstraction for a digital i/o pin on a given port; can switched to high/low.
 */
class PortPin {
public:

    PortPin(GPIO_TypeDef * gpioPort, const uint16_t gpioPin) :
        mGpioPort(gpioPort), mGpioPin(gpioPin) { }

    void init() {
        // reset pin
        HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_RESET);
        // Configure GPIO pin
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = mGpioPin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(mGpioPort, &GPIO_InitStruct);
    }

    /**
     * Set state of port pin
     * @param state (GPIO_PIN_SET or GPIO_PIN_RESET)
     */
    void set(GPIO_PinState state) {
        HAL_GPIO_WritePin(mGpioPort, mGpioPin, state);
    }

    /**
     * Toggle state of port pin
     */
    void toggle() {
        HAL_GPIO_TogglePin(mGpioPort, mGpioPin);
    }

protected:
    GPIO_TypeDef * mGpioPort;
    const uint16_t mGpioPin;
};


// **************************************************************************
// Handling of the on-board LEDs
// **************************************************************************

/**
 * Abstraction for a LED connected to a digital i/o pin; can be switched on/off.
 */
class Led : public PortPin {
public:

    Led(GPIO_TypeDef * gpioPort, const uint16_t gpioPin) : PortPin(gpioPort, gpioPin) {}

    /**
     * Switch LED on (true) or off (false)
     * @param state
     */
    void set(bool state) {
        if (state) {
            HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_RESET); // this switches the led on
        } else {
            HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_SET); // this switches the led off
        }
    }

};


// **************************************************************************
// Serial communication via UART
// **************************************************************************

#define SERIAL_RX_MAX 40


/**
 * Abstraction for a serial interface.
 */
class Serial {
public:
    Serial() { }

    void init() {
        // configure I/O pins
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.Pin = TX_Pin;
        gpioInitStruct.Mode = GPIO_MODE_AF_PP;
        gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TX_GPIO_Port, &gpioInitStruct);
        gpioInitStruct.Pin = RX_Pin;
        gpioInitStruct.Mode = GPIO_MODE_INPUT;
        gpioInitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(RX_GPIO_Port, &gpioInitStruct);

        // initialize UART
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200; // 9600;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_UART_Init(&huart1);

        // set up & link DMA 1 channel 5 for rx
        hdma_usart1_rx.Instance = DMA1_Channel5;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_DISABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
        __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

        // set up & link DMA 1 channel 4 for tx
        hdma_usart1_tx.Instance = DMA1_Channel4;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
        __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

        // enable IRQs
        HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
        // enabling USART1_IRQn necessary to catch the TX complete
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART1_IRQn);

        // Start reception
        memset((void *)mSerialRxReceived, 0, SERIAL_RX_MAX);
        mSerialRxIndex = 0;
        __HAL_UART_FLUSH_DRREGISTER(&huart1);
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)&mSerialRxDmaBuffer, 1);
    }

    char *serialReceive() {
        static char *localBuffer[SERIAL_RX_MAX];
        char *ret = NULL;
        if (mSerialRxIndex < 0) {
            memcpy(localBuffer, (const void *)mSerialRxReceived, SERIAL_RX_MAX);
            memset((void *)mSerialRxReceived, 0, SERIAL_RX_MAX);
            mSerialRxIndex = 0;
            ret = (char *)localBuffer;
        }
        return ret;
    }

    void rxCompleteFromIsr() {
        if (mSerialRxIndex < 0) {
            // ignore characters received before the last line was accepted
        } else {
            if (mSerialRxDmaBuffer == '\r') {
                // ignore
            } else if (mSerialRxDmaBuffer == '\n') {
                mSerialRxReceived[mSerialRxIndex] = 0;
                mSerialRxIndex = -1;
            } else {
                mSerialRxReceived[mSerialRxIndex] = mSerialRxDmaBuffer;
                // prepare for reception of next character, handle buffer overflows
                if (mSerialRxIndex < SERIAL_RX_MAX - 1) {
                    mSerialRxIndex++;
                }
            }
        }
    }

private:
    volatile uint8_t mSerialRxDmaBuffer = 0;
    volatile uint8_t mSerialRxReceived[SERIAL_RX_MAX];
    volatile int32_t mSerialRxIndex = 0;
};


// **************************************************************************
// TODO I2C Communication
// **************************************************************************

class I2c {
public:

    I2c() { }

    void init() {
        // PB10  I2C2_SCL
        // PB11  I2C2_SDA
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpioInitStruct.Mode = GPIO_MODE_AF_OD;
        gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gpioInitStruct);

        hi2c2.Instance = I2C2;
        hi2c2.Init.ClockSpeed = 100000;
        hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c2.Init.OwnAddress1 = 0;
        hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c2.Init.OwnAddress2 = 0;
        hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        if (HAL_I2C_Init(&hi2c2) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
    }
};


// **************************************************************************
// TODO Spi Communication
// **************************************************************************

class Spi {
public:

    Spi() { }

    void init() {
        hspi2.Instance = SPI2;
        hspi2.Init.Mode = SPI_MODE_MASTER;
        hspi2.Init.Direction = SPI_DIRECTION_2LINES;
        hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi2.Init.NSS = SPI_NSS_SOFT;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi2.Init.CRCPolynomial = 10;
        if (HAL_SPI_Init(&hspi2) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
    }
};


// **************************************************************************
// TODO Adc
// **************************************************************************

class Adc {
public:

    Adc() { }

    void init() {
        ADC_ChannelConfTypeDef sConfig;

        // Common config
        hadc1.Instance = ADC1;
        hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
        hadc1.Init.ContinuousConvMode = DISABLE;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.NbrOfConversion = 1;
        if (HAL_ADC_Init(&hadc1) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }

        // Configure Regular Channel
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            _Error_Handler((char *)__FILE__, __LINE__);
        }
    }
};


// **************************************************************************
// Board
// **************************************************************************


extern Watchdog watchdog;
extern Led led0;
extern Led led1;
extern PortPin beeper;
extern Serial serial;
extern I2c i2c;


/**
 * Abstraction of some general functions of the microcontroller and the board;
 * please note that the other classes/objects defined in this module (plus the
 * receiver and servo modules) are also board specific.
 *
 * Design decision: Every piece of hardware provides an init() function; these
 * functions are called in boardInit. Compared to the RAII pattern, this
 * allows better control of the initialization order over several modules.
 */
class Naze32 {
public:

    Naze32() { }

    void init() { }

    uint32_t getTicks() const {
        return HAL_GetTick();
    }

    void reset() {
        NVIC_SystemReset();
    }

    /**
     * Fatal error: output a message, blink the leds for a few seconds,
     * and reboot.
     */
    void fatal(const char *filename, int lineNo, const char *msg) {
        printf("\n*** Fatal error in file %s line %d: %s\n", filename, lineNo, msg);
        for (int i=0; i<6; i++) {
            // blink both LEDs and wait 250 MS
            led0.set(false);
            led1.set(true);
            HAL_Delay(250);
            // blink both LEDs and wait 250 MS
            led0.set(true);
            led1.set(false);
            HAL_Delay(250);
        }
        reset();
    }

private:

};


// **************************************************************************

extern Naze32 board;

extern void _Error_Handler(char *file, int lineNo);
extern void boardInit();

// **************************************************************************


#endif //MOVE32_BOARD_H
