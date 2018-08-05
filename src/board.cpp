/**
 * Move32 - Naze32 for robotics
 * High-Level HAL for board support
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include <string.h>

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
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef *syscall_UART_handle_ptr = &huart1;


// **************************************************************************

Naze32 board;
Watchdog watchdog;
Led led0(LED0_GPIO_Port, LED0_Pin);
Led led1(LED1_GPIO_Port, LED1_Pin);
PortPin beeper(BEEP_GPIO_Port, BEEP_Pin);
Serial serial;
I2c i2c;

void SystemClock_Config(void);

void boardInit() {
    // Configure system clock system including PLL, AHB and APB clocks
    SystemClock_Config();
    board.init();
    //watchdog.init();
    led0.init();
    led1.init();
    beeper.init();
    serial.init();
    i2c.init();
}

void _Error_Handler(char *file, int line) {
    printf("\nFatal error in file %s line %d!\n", file, line);
    while (1) {
        // blink both LEDs and wait 250 MS
        led0.set(GPIO_PIN_SET);
        led1.set(GPIO_PIN_RESET);
        HAL_Delay(250);
        // blink both LEDs and wait 250 MS
        led0.set(GPIO_PIN_RESET);
        led1.set(GPIO_PIN_SET);
        HAL_Delay(250);
    }
}


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
// UART IRQ, DMA and STHAL callback handling
// **************************************************************************

/**
 * Called within the STHAL UART/DMA interrupt handlers on rx complete.
 * @param huart The UART to process
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun
    serial.rxCompleteFromIsr();
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
        _Error_Handler((char *)__FILE__, __LINE__);
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
        _Error_Handler((char *)__FILE__, __LINE__);
    }

    // Initialize the preripheral clock
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        _Error_Handler((char *)__FILE__, __LINE__);
    }

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
