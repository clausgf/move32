/**
 * Move32 - Naze32 for robotics
 * High-Level HAL for board support
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include "board.h"


// **************************************************************************
// Handles for accessing the peripherals via STHAL
// **************************************************************************

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef *syscall_UART_handle_ptr = &huart1;

// **************************************************************************
// Timer initialization
// **************************************************************************

void timerInit(TIM_HandleTypeDef *hTimer,
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

void pwmInit(TIM_HandleTypeDef *hTimer,
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

void icInit(TIM_HandleTypeDef *handle,
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
// Serial communication via UART
// **************************************************************************


uint8_t serialRxBuffer = 0;
uint8_t serialRxReceived[SERIAL_RX_MAX];
int32_t serialRxIndex = 0;

void serialInit(void) {
    // enable peripheral clocks
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // configure I/O pins
    GPIO_InitTypeDef gpioInitStruct;
    gpioInitStruct.Pin = GPIO_PIN_9; // TX Pin
    gpioInitStruct.Mode = GPIO_MODE_AF_PP;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInitStruct);
    gpioInitStruct.Pin = GPIO_PIN_10; // RX Pin
    gpioInitStruct.Mode = GPIO_MODE_INPUT;
    gpioInitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInitStruct);

    // initialize UART
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
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
    HAL_DMA_Init(&hdma_usart1_rx);
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
    HAL_DMA_Init(&hdma_usart1_tx);
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
    memset(serialRxReceived, 0, SERIAL_RX_MAX);
    serialRxIndex = 0;
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    HAL_UART_Receive_DMA(&huart1, &serialRxBuffer, 1);
}

char *serialReceive() {
    static char *localBuffer[SERIAL_RX_MAX];
    char *ret = NULL;
    if (serialRxIndex < 0) {
        memcpy(localBuffer, serialRxReceived, SERIAL_RX_MAX);
        memset(serialRxReceived, 0, SERIAL_RX_MAX);
        serialRxIndex = 0;
        ret = (char *)localBuffer;
    }
    return ret;
}

// **************************************************************************
// UART IRQ, DMA and STHAL callback handling
// **************************************************************************

/**
 * Called within the STHAL UART/DMA interrupt handlers on rx complete.
 * @param huart The UART to process
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun

    if (serialRxIndex < 0) {
        // ignore characters received before the last line was accepted
    } else {
        if (serialRxBuffer == '\r') {
            // ignore
        } else if (serialRxBuffer == '\n') {
            serialRxReceived[serialRxIndex] = 0;
            serialRxIndex = -1;
        } else {
            serialRxReceived[serialRxIndex] = serialRxBuffer;
            // prepare for reception of next character, handle buffer overflows
            if (serialRxIndex <= SERIAL_RX_MAX - 1) {
                serialRxIndex++;
            }
        }
    }
}

/**
 * USART1 IRQ handler to catch the TX complete
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * On DMA1 Channel 5 IRQ, call the HAL DMA interrupt handler for USART1 RX.
 * That handler calls the respective HAL UART handlers, which can activate
 * user callbacks.
 */
void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
 * On DMA1 Channel 4 IRQ, call the HAL DMA interrupt handler for USART1 TX.
 * That handler calls the respective HAL UART handlers, which can activate
 * user callbacks.
 */
void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}


// **************************************************************************
// TODO I2C Communication
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


void led0Set(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state);
}

void led0Toggle() {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
}

void led1Set(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, state);
}

void led1Toggle() {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

void beeperSet(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, state);
}

void beeperToggle() {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
}

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

void boardInit() {
    // Configure system clock system including PLL, AHB and APB clocks
    SystemClock_Config();

    ledInit();
    serialInit();
}
