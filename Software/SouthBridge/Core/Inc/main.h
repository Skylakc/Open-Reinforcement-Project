/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BT_HALT_Pin GPIO_PIN_4
#define BT_HALT_GPIO_Port GPIOE
#define ARM_RESET_Pin GPIO_PIN_5
#define ARM_RESET_GPIO_Port GPIOE
#define FPGA_RESET1_Pin GPIO_PIN_6
#define FPGA_RESET1_GPIO_Port GPIOE
#define LED_OS_Pin GPIO_PIN_13
#define LED_OS_GPIO_Port GPIOC
#define LED_ARM_Pin GPIO_PIN_14
#define LED_ARM_GPIO_Port GPIOC
#define LED_SB_Pin GPIO_PIN_15
#define LED_SB_GPIO_Port GPIOC
#define LED_CPU_Pin GPIO_PIN_0
#define LED_CPU_GPIO_Port GPIOF
#define LED_PWM_ON_Pin GPIO_PIN_1
#define LED_PWM_ON_GPIO_Port GPIOF
#define LED_SAFE_Pin GPIO_PIN_2
#define LED_SAFE_GPIO_Port GPIOF
#define LED_FPGA_Pin GPIO_PIN_3
#define LED_FPGA_GPIO_Port GPIOF
#define LED_DANGER_Pin GPIO_PIN_4
#define LED_DANGER_GPIO_Port GPIOF
#define BT_RUN_Pin GPIO_PIN_5
#define BT_RUN_GPIO_Port GPIOF
#define BT_LEFT_Pin GPIO_PIN_6
#define BT_LEFT_GPIO_Port GPIOF
#define BT_RIGHT_Pin GPIO_PIN_7
#define BT_RIGHT_GPIO_Port GPIOF
#define BT_UP_Pin GPIO_PIN_8
#define BT_UP_GPIO_Port GPIOF
#define BT_DOWN_Pin GPIO_PIN_9
#define BT_DOWN_GPIO_Port GPIOF
#define BT_OK_Pin GPIO_PIN_10
#define BT_OK_GPIO_Port GPIOF
#define Voltage_12V_Pin GPIO_PIN_0
#define Voltage_12V_GPIO_Port GPIOC
#define Voltage_5V_Pin GPIO_PIN_1
#define Voltage_5V_GPIO_Port GPIOC
#define Voltage_3V3_Pin GPIO_PIN_2
#define Voltage_3V3_GPIO_Port GPIOC
#define Voltage_2V5_Pin GPIO_PIN_3
#define Voltage_2V5_GPIO_Port GPIOC
#define Voltage_1V35_Pin GPIO_PIN_1
#define Voltage_1V35_GPIO_Port GPIOA
#define Voltage_1V2_Pin GPIO_PIN_2
#define Voltage_1V2_GPIO_Port GPIOA
#define Voltage_1V_Pin GPIO_PIN_3
#define Voltage_1V_GPIO_Port GPIOA
#define Voltage_0V75_Pin GPIO_PIN_4
#define Voltage_0V75_GPIO_Port GPIOA
#define Voltage_DRAM_Pin GPIO_PIN_5
#define Voltage_DRAM_GPIO_Port GPIOA
#define Temp_AUX1_Pin GPIO_PIN_6
#define Temp_AUX1_GPIO_Port GPIOA
#define Temp_AUX2_Pin GPIO_PIN_7
#define Temp_AUX2_GPIO_Port GPIOA
#define Voltage_1V8_Pin GPIO_PIN_4
#define Voltage_1V8_GPIO_Port GPIOC
#define Voltage_1V5_Pin GPIO_PIN_5
#define Voltage_1V5_GPIO_Port GPIOC
#define Temp_AUX3_Pin GPIO_PIN_0
#define Temp_AUX3_GPIO_Port GPIOB
#define Temp_AUX4_Pin GPIO_PIN_1
#define Temp_AUX4_GPIO_Port GPIOB
#define FPGA_RESET2_Pin GPIO_PIN_14
#define FPGA_RESET2_GPIO_Port GPIOD
#define DSP_RESET2_Pin GPIO_PIN_8
#define DSP_RESET2_GPIO_Port GPIOG
#define DSP_RESET1_Pin GPIO_PIN_1
#define DSP_RESET1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
