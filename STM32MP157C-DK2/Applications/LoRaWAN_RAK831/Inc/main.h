/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/OpenAMP_TTY_echo/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32mp1xx_hal.h"
#include "openamp.h"
#include "lock_resource.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "log.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define BT_HOST_WAKE_Pin GPIO_PIN_5
#define BT_HOST_WAKE_GPIO_Port GPIOH
#define INT_Pin GPIO_PIN_2
#define INT_GPIO_Port GPIOF
#define WL_REG_ON_Pin GPIO_PIN_4
#define WL_REG_ON_GPIO_Port GPIOH
#define WL_HOST_WAKE_Pin GPIO_PIN_0
#define WL_HOST_WAKE_GPIO_Port GPIOD
#define TE_Pin GPIO_PIN_6
#define TE_GPIO_Port GPIOC
#define BL_CTRL_Pin GPIO_PIN_15
#define BL_CTRL_GPIO_Port GPIOA
#define uSD_DETECT_Pin GPIO_PIN_7
#define uSD_DETECT_GPIO_Port GPIOB
#define RSTN_Pin GPIO_PIN_4
#define RSTN_GPIO_Port GPIOE
#define BT_REG_ON_Pin GPIO_PIN_6
#define BT_REG_ON_GPIO_Port GPIOZ
#define BT_DEV_WAKE_Pin GPIO_PIN_7
#define BT_DEV_WAKE_GPIO_Port GPIOZ
#define PMIC_WAKEUP_Pin GPIO_PIN_13
#define PMIC_WAKEUP_GPIO_Port GPIOC
#define PA13_Pin GPIO_PIN_13
#define PA13_GPIO_Port GPIOA
#define STUSB1600_IRQOUTn_Pin GPIO_PIN_11
#define STUSB1600_IRQOUTn_GPIO_Port GPIOI
#define PA14_Pin GPIO_PIN_14
#define PA14_GPIO_Port GPIOA
#define HDMI_INT_Pin GPIO_PIN_1
#define HDMI_INT_GPIO_Port GPIOG
#define LED_Y_Pin GPIO_PIN_7
#define LED_Y_GPIO_Port GPIOH
#define AUDIO_RST_Pin GPIO_PIN_9
#define AUDIO_RST_GPIO_Port GPIOG
#define HDMI_NRST_Pin GPIO_PIN_10
#define HDMI_NRST_GPIO_Port GPIOA
#define ETH_MDINT_Pin GPIO_PIN_6
#define ETH_MDINT_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_11
#define LED_B_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
