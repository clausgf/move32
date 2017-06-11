/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MPU_INT_Pin GPIO_PIN_13
#define MPU_INT_GPIO_Port GPIOC
#define RC_CH1_Pin GPIO_PIN_0
#define RC_CH1_GPIO_Port GPIOA
#define RC_CH2_Pin GPIO_PIN_1
#define RC_CH2_GPIO_Port GPIOA
#define RC_CH3_Pin GPIO_PIN_2
#define RC_CH3_GPIO_Port GPIOA
#define RC_CH4_Pin GPIO_PIN_3
#define RC_CH4_GPIO_Port GPIOA
#define POWER_ADC_Pin GPIO_PIN_4
#define POWER_ADC_GPIO_Port GPIOA
#define ACC_INT2_Pin GPIO_PIN_5
#define ACC_INT2_GPIO_Port GPIOA
#define RC_CH5_Pin GPIO_PIN_6
#define RC_CH5_GPIO_Port GPIOA
#define RC_CH6_Pin GPIO_PIN_7
#define RC_CH6_GPIO_Port GPIOA
#define RC_CH7_Pin GPIO_PIN_0
#define RC_CH7_GPIO_Port GPIOB
#define RC_CH8_Pin GPIO_PIN_1
#define RC_CH8_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define MAG_DRDY_Pin GPIO_PIN_12
#define MAG_DRDY_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_12
#define BEEP_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_4
#define LED0_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_6
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_7
#define PWM4_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_8
#define PWM5_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_9
#define PWM6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
