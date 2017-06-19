
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stm32f1xx_hal.h"



// **************************************************************************
// Handles for accessing the peripherals via STHAL
// **************************************************************************

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef *syscall_UART_handle_ptr = &huart1;

// **************************************************************************
// Timer initialization
// **************************************************************************

/**
 * Basic initialization for a timer running at 1 MHz from the system clock.
 * @param hTimer STHAL handle for the timer, e.g. htim1
 * @param timerInstancePtr Pointer to the timer's register block, e.g. TIM1
 * @param period Period (in us) for timer resets, e.g. 20000 for 20 ms
 */
static void timerInit(TIM_HandleTypeDef *hTimer,
                      TIM_TypeDef *timerInstancePtr,
                      uint16_t period) {
    hTimer->Instance = timerInstancePtr;
    hTimer->Init.Prescaler = (SystemCoreClock / 1000000) - 1;
    hTimer->Init.Period = (period - 1) & 0xffff;
    hTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
    hTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hTimer->Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(hTimer);

    TIM_ClockConfigTypeDef sClockSourceConfig;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(hTimer, &sClockSourceConfig);

    TIM_MasterConfigTypeDef sMasterConfig;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    HAL_TIM_Base_Start(hTimer);
}

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
static void pwmInit(TIM_HandleTypeDef *hTimer,
                    uint32_t timerChannel,
                    uint16_t value) {
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.Pulse = value;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(hTimer, &sConfigOC, timerChannel);

    HAL_TIM_PWM_Start(hTimer, timerChannel);
}

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
static void icInit(TIM_HandleTypeDef *handle,
                   uint32_t timerChannel,
                   uint32_t polarity) {
    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = polarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x00;
    HAL_TIM_IC_ConfigChannel(handle, &sConfigIC, timerChannel);

    HAL_TIM_IC_Start_IT(handle, timerChannel);
}


// **************************************************************************
// Controlling servos via PWM
// **************************************************************************

/// Number of servos to control
#define SERVOS_MAX 6

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

/**
 * Initialize all servo outputs, including GPIO pins and timers.
 */
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

/**
 * Output pulses of the given length to the servo/channel or
 * the fail save pulse if the given pulse length is zero.
 * @param index Channel/Servo index from [0, SERVOS_MAX[
 * @param pulseLengthUs pulse length in us
 */
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

// **************************************************************************
// Decode input signals (channels) from the rc receiver (rx)
// via input capture
// **************************************************************************

/// Number of channels to decode
#define RX_MAX_CHANNEL 8

// Timeout (in HAL Ticks or ms) for rx decoding - after the timeout
// has expired without a signal, the the channel's state is considered unknown
#define RX_CHANNEL_TIMEOUT_MS 50

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

/**
 * Initialize all rx channel inputs, including GPIO pins and timers.
 */
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
}

/**
 * Return pulse length from the rc receiver in ms or zero for the given channel.
 * @param rxChannel Rx channel index from [0, RX_MAX_CHANNEL[
 * @return pulse length in us or zero if pulse length unknown or too old
 */
uint16_t rxRead(uint8_t rxChannel) {
    uint16_t ret = 0;
    if (rxChannel < RX_MAX_CHANNEL) {
        if ((HAL_GetTick() - rxChannels[rxChannel].lastUpdateTime) < RX_CHANNEL_TIMEOUT_MS) {
            ret = rxChannels[rxChannel].pulseLength;
        }
    }
    return ret;
}

/**
 * This routine is called from the input capture ISR when a new pulse length
 * is available.
 * @param rxChannel Rx channel index from [0, RX_MAX_CHANNEL[
 * @param pulseLenUs Pulse length in us
 */
void rxUpdateCallback(uint8_t rxChannel, uint16_t pulseLenUs);


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


// **************************************************************************
// Serial communication via UART
// **************************************************************************

#define SERIAL_BUFLEN 80
static char serial_buf[SERIAL_BUFLEN];

static void serialInit(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpioInitStruct;
    // TX Pin
    gpioInitStruct.Pin = GPIO_PIN_9;
    gpioInitStruct.Mode = GPIO_MODE_AF_PP;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInitStruct);
    // RX Pin
    gpioInitStruct.Pin = GPIO_PIN_10;
    gpioInitStruct.Mode = GPIO_MODE_INPUT;
    gpioInitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInitStruct);

    //HAL_NVIC_SetPriority(USART1_IRQn, 0, 5);
    //HAL_NVIC_EnableIRQ(USART1_IRQn);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void serialOutStr(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t *) str, strlen(str), 100);
}

void serialOut(char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsniprintf(serial_buf, SERIAL_BUFLEN, fmt, args);
    serialOutStr(serial_buf);
    va_end(args);
}


// **************************************************************************
// TODO I2C Communication using your protocol!
// **************************************************************************

void i2cInit() {
    __HAL_RCC_I2C2_CLK_ENABLE();

    // PB10  I2C2_SCL
    // PB11  I2C2_SDA
    GPIO_InitTypeDef gpioInitStruct;
    gpioInitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInitStruct.Mode = GPIO_MODE_AF_OD;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInitStruct);
}


// **************************************************************************
// Handling of the on-board LEDs
// **************************************************************************


/**
 * Set state of LED0 (pin PB4)
 * @param state GPIO_PIN_SET or GPIO_PIN_RESET
 */
void led0Set(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state);
}

/**
 * Set state of LED0 (pin PB3
 * @param state GPIO_PIN_SET or GPIO_PIN_RESET)
 */
void led1Set(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, state);
}

/**
 * Set state of external beeper output (pin PA12)
 * @param state GPIO_PIN_SET or GPIO_PIN_RESET
 */
void beeperSet(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, state);
}

/**
 * Initialize the two on-board LEDs and the BEEP output, including GPIOs.
 */
void ledInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // beeper
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET); // leds

    // Configure GPIO pins for beeper and leds
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_12; // beeper
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4; // leds
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// **************************************************************************
// System initialization and error handling generated by CubeMX
// **************************************************************************

void SystemClock_Config(void) {
    // Initializes the CPU, AHB and APB bus clocks
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    // Why? RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initialize the preripheral clock
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void _Error_Handler(char *file, int line) {
    printf("\nFatal error in file %s line %d!\n", file, line);
    while (1) {
        // blink both LEDs and wait 250 MS
        led0Set(GPIO_PIN_SET);
        led1Set(GPIO_PIN_RESET);
        HAL_Delay(250);
        // blink both LEDs and wait 250 MS
        led0Set(GPIO_PIN_RESET);
        led1Set(GPIO_PIN_SET);
        HAL_Delay(250);
    }
}



// **************************************************************************
// User portion of the code
// **************************************************************************

void rxUpdateCallback(uint8_t rxChannel, uint16_t pulseLenUs) {
    if (rxChannel < SERVOS_MAX) {
        servoWrite(rxChannel, pulseLenUs);
    }
//    for (int i = 0; i < SERVOS_MAX && i < RX_MAX_CHANNEL; i++) {
//        servoWrite(i, rxRead(i));
//    }
}


int main(void) {
    // Reset of all peripherals, Initializes the Flash interface and Systick.
    // This function calls HAL_MspInit() generated into *_hal_msp.c for
    // board-specific initialization.
    HAL_Init();

    // Configure system clock system including PLL, AHB and APB clocks
    SystemClock_Config();

    // **********************************************************************

    ledInit();
    serialInit();
    servosInit();
    rxInit();

    printf("Hello World, it's %lu local time!\n", SystemCoreClock);

    // **********************************************************************

    while (1) {
        for (uint16_t pos = 1000; pos <= 2000; pos += 100) {
            serialOut("%ld  0:%04d 1:%04d 2:%04d 3:%04d 4:%04d 5:%04d 6:%04d 7:%04d\n",
                      HAL_GetTick(),
                      rxRead(0), rxRead(1), rxRead(2), rxRead(3),
                      rxRead(4), rxRead(5), rxRead(6), rxRead(7));
            //serialOut("%8x %8x %8x %8x  %8x %d\n", &htim1, &htim2, &htim3, &htim4, forTimer, forChannel);
            //serialOut("%4x %4x %4x\n", rxChannels[0].pulseStartTime, rxChannels[0].pulseEndTime, rxChannels[0].pulseLength);
            //serialOut("%4x %4x %4x\n", rxChannels[4].pulseStartTime, rxChannels[4].pulseEndTime, rxChannels[4].pulseLength);
            for (int servo = 0; servo < 6; servo++)
                servoWrite(servo, 1000);

            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            for (int i = 0; i < 200e3; i++);
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
            HAL_Delay(500);
        }
    }

}

// **************************************************************************
