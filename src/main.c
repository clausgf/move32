
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stm32f1xx_hal.h"

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

// **************************************************************************
// copied and modified from:
// https://sites.google.com/site/stm32discovery/open-source-development-with-the-stm32-discovery/getting-newlib-to-work-with-stm32-and-code-sourcery-lite-eabi

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#undef errno
extern int errno;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status) {
    _write(1, "exit", 4);
    while (1) {
        ;
    }
}

int _close(int file) {
    return -1;
}

/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid() {
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {
    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

		char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */
int _read(int file, char *ptr, int len) {
    int n;
    int num = 0;
    switch (file) {
    case STDIN_FILENO:
		    HAL_UART_Receive(&huart1, (uint8_t *)ptr, len, 100);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {
    int n;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
		case STDERR_FILENO: /* stderr */
				HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}

// **************************************************************************

/**
 * Basic timer initialization (1 MHz)
 */
static void timerInit(TIM_HandleTypeDef *handle, TIM_TypeDef *timerInstance, uint16_t period)
{
	handle->Instance = timerInstance;
	handle->Init.Prescaler = (SystemCoreClock / 1000000) - 1;
	handle->Init.Period = (period - 1) & 0xffff;
	handle->Init.CounterMode = TIM_COUNTERMODE_UP;
	handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	handle->Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(handle);

	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(handle, &sClockSourceConfig);

	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	HAL_TIM_Base_Start(handle);
}

static void pwmInit(TIM_HandleTypeDef *handle, uint32_t channel, uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.Pulse = value;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(handle, &sConfigOC, channel);

	HAL_TIM_PWM_Start(handle, channel);
}

/**
 * @param polarity TIM_ICPOLARITY_RISING, TIM_ICPOLARITY_FALLING or TIM_ICPOLARITY_BOTHEDGE
 * @param channel TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
 */
static void icInit(TIM_HandleTypeDef *handle, uint32_t channel, uint32_t polarity)
{
    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = polarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x00;
    HAL_TIM_IC_ConfigChannel(handle, &sConfigIC, channel);

    HAL_TIM_IC_Start_IT(handle, channel);
}

// **************************************************************************

#define SERVOS_MAX 6

typedef struct {
	volatile uint32_t *ccrPtr; //< pointer to Capture Compare Register
	TIM_TypeDef *timerPtr;     //< pointer to timer block with registers
	uint16_t failSafePulseMs;  //< pulse length (ms) for the fail safe position
} servo_t;                     //< type to describe a servo (pwm) output

static servo_t servos[SERVOS_MAX];

static void servoInit(uint8_t index, volatile uint32_t *ccr, TIM_TypeDef *timer, uint16_t failSafePulseMs)
{
	servos[index].ccrPtr = ccr;
	servos[index].timerPtr = timer;
	servos[index].failSafePulseMs = failSafePulseMs;
}

void servosInit()
{
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

//    timerInit(&htim4, TIM4, 20000);
//    pwmInit(&htim4, TIM_CHANNEL_1, 1500);
//    pwmInit(&htim4, TIM_CHANNEL_2, 1500);
//    pwmInit(&htim4, TIM_CHANNEL_3, 1500);
//    pwmInit(&htim4, TIM_CHANNEL_4, 1500);

    servoInit(0, &(TIM1->CCR1), TIM1, 1500);
	servoInit(1, &(TIM1->CCR4), TIM1, 1500);
//	servoInit(2, &(TIM4->CCR1), TIM4, 1500);
//	servoInit(3, &(TIM4->CCR2), TIM4, 1500);
//	servoInit(4, &(TIM4->CCR3), TIM4, 1500);
//	servoInit(5, &(TIM4->CCR4), TIM4, 1500);
}

/**
 * Output pulses of the given length to the servo/channel or
 * the fail save pulse if the given pulse length is zero.
 *
 * @param index Channel/Servo index from [0, SERVOS_MAX[
 * @param pulseLengthMs pulse length in ms
 */
void servoWrite(uint8_t index, uint16_t pulseLengthMs)
{
	if (index < SERVOS_MAX && servos[index].ccrPtr)
	{
		if (pulseLengthMs == 0)
		{
			*(servos[index].ccrPtr) = servos[index].failSafePulseMs;
		} else {
			*(servos[index].ccrPtr) = pulseLengthMs;

		}
	}
}

// **************************************************************************

typedef struct {
	TIM_HandleTypeDef *hTimer;  //< timer for this rx input
	uint32_t timerChannel;		//< timer channel for this rx input
	uint32_t timerActiveChannel;//< timer active channel for this rx input
	bool wasRisingEdgeDetected; //< flag to remember whether the last ic event
	uint16_t pulseStartTime;	//< timer/counter value of last rising edge
	uint16_t pulseEndTime;		//< timer/counter value of last falling edge
	uint16_t pulseLen; 			//< length of pulse (ms)
	uint32_t lastUpdateTime; 	//< system ticker (ms) of last update
} rxChannel_t;					//< type to describe and input capture for a rx channel

#define RX_MAX_CHANNEL 8
#define RX_CHANNEL_TIMEOUT_MS 50
rxChannel_t rxChannels[RX_MAX_CHANNEL];

static void rxChannelInit(uint8_t rxChannel, TIM_HandleTypeDef *hTimer, uint32_t timerChannel, uint32_t timerActiveChannel)
{
	rxChannels[rxChannel].hTimer = hTimer;
	rxChannels[rxChannel].timerChannel = timerChannel;
	rxChannels[rxChannel].timerActiveChannel = timerActiveChannel;
	rxChannels[rxChannel].wasRisingEdgeDetected = false;
	rxChannels[rxChannel].pulseStartTime = 0;
	rxChannels[rxChannel].pulseEndTime = 0;
	rxChannels[rxChannel].pulseLen = 0;
	icInit(hTimer, timerChannel, TIM_ICPOLARITY_RISING);
}

void rxInit()
{
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
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	timerInit(&htim3, TIM3, 0);
	rxChannelInit(4, &htim3, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1);
	rxChannelInit(5, &htim3, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2);
	rxChannelInit(6, &htim3, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3);
	rxChannelInit(7, &htim3, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4);
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * Return pulse length from the rc receiver in ms or zero for the given channel.
 * @param channel Rx channel index from [0, RX_MAX_CHANNEL[
 * @return pulse length in ms or zero if pulse length unknown or too old
 */
uint16_t rxRead(uint32_t channel)
{
	uint16_t ret = 0;
	if (channel < RX_MAX_CHANNEL)
	{
		if ( (HAL_GetTick()-rxChannels[channel].lastUpdateTime) < RX_CHANNEL_TIMEOUT_MS )
		{
			ret = rxChannels[channel].pulseLen;
		}
	}
	return ret;
}

void rxUpdateCallback(void);

// **************************************************************************

/**
 * Callback called within the HAL's timer interrupt handler
 * Beware of the poor HAL design (again):
 * - hTimer->Channel indicates the timer channel as HAL_TIM_ACTIVE_CHANNEL_x
 * - HAL_TIM_ReadCaputredValue needs a TIM_CHANNEL_x
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hTimer)
{
	for (int rxChannelIndex=0; rxChannelIndex<RX_MAX_CHANNEL; rxChannelIndex++)
	{
		rxChannel_t *channel = &rxChannels[rxChannelIndex];
		if (hTimer == channel->hTimer && hTimer->Channel == channel->timerActiveChannel)
		{
			uint32_t nextPolarity;
			if (channel->wasRisingEdgeDetected)
			{
				// rising edge already detected, this is the falling edge
				channel->pulseEndTime = HAL_TIM_ReadCapturedValue(hTimer, channel->timerChannel);
				channel->pulseLen = channel->pulseEndTime - channel->pulseStartTime;
				channel->lastUpdateTime = HAL_GetTick();
				channel->wasRisingEdgeDetected = false;
				nextPolarity = TIM_ICPOLARITY_RISING;
			}
			else
			{
				// this is the rising edge
				channel->pulseStartTime = HAL_TIM_ReadCapturedValue(hTimer, channel->timerChannel);
				channel->wasRisingEdgeDetected = true;
				nextPolarity = TIM_ICPOLARITY_FALLING;
			}
			// reconfigure for next edge
			// another unexpected, annoying HAL "feature": reconfiguration
			// using HAL_TIM_IC_ConfigChannel requires HAL_TIM_IC_Start_IT;
			// thus just call icInit for reconfiguration
			icInit(channel->hTimer, channel->timerChannel, nextPolarity);

			rxUpdateCallback();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *hTimer)
{
	if (hTimer == &htim2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6); // Channel 3/2
	}
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *hTimer)
{
	if (hTimer == &htim2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // Channel 4/3
	}
}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *hTimer)
{
	if (hTimer == &htim2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8); // Channel 5/4
	}
}

void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *hTimer)
{
	if (hTimer == &htim2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9); // Channel 6/5
	}
}

/**
 * On Timer 2 IRQ, call the HAL timer interrupt handler
 */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
}


/**
 * On Timer 3 IRQ, call the HAL timer interrupt handler
 */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);
}


// **************************************************************************

#define SERIAL_BUFLEN 80
static char serial_buf[SERIAL_BUFLEN];

static void serialInit(void)
{
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

static void serialOutStr(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void serialOut(char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsniprintf(serial_buf, SERIAL_BUFLEN, fmt, args);
    serialOutStr(serial_buf);
    va_end(args);
}

// **************************************************************************

void i2cInit()
{
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

void SystemClock_Config(void)
{
  // Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  // Wozu? RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Initialize the preripheral clock
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Configure the Systick interrupt time
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  // Configure the Systick
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 MAG_DRDY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|MAG_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED0_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}



// **************************************************************************

void rxUpdateCallback(void)
{
	for (int i=0; i<SERVOS_MAX && i<RX_MAX_CHANNEL; i++)
	{
		servoWrite(i, rxRead(i));
	}
}

// **************************************************************************

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // ************************************************************************

  serialInit();
  servosInit();
  rxInit();

  printf("Hallo Welt %lu!\n", SystemCoreClock);

  // ************************************************************************

  while (1)
  {
	  for (uint16_t pos=1000; pos<=2000; pos+=100)
	  {
		  serialOut("%ld  0:%04d 1:%04d 2:%04d 3:%04d 4:%04d 5:%04d 6:%04d 7:%04d\n",
				  HAL_GetTick(),
				  rxRead(0), rxRead(1), rxRead(2), rxRead(3),
				  rxRead(4), rxRead(5), rxRead(6), rxRead(7));
		  //serialOut("%8x %8x %8x %8x  %8x %d\n", &htim1, &htim2, &htim3, &htim4, forTimer, forChannel);
		  //serialOut("%4x %4x %4x\n", rxChannels[0].pulseStartTime, rxChannels[0].pulseEndTime, rxChannels[0].pulseLen);
		  //serialOut("%4x %4x %4x\n", rxChannels[4].pulseStartTime, rxChannels[4].pulseEndTime, rxChannels[4].pulseLen);
		  for (int servo=0; servo<6; servo++)
			  servoWrite(servo, 1000);

		  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		  for (int i=0; i<200e3; i++)
			  ;
		  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		  HAL_Delay(500);
	  }
  }

}

// **************************************************************************
