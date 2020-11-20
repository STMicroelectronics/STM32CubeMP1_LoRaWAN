/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    OpenAMP/OpenAMP_TTY_echo/Src/stm32mp1xx_hal_msp.c
 * @author  MCD Application Team
 * @brief   This file provides code for the MSP Initialization
 *          and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

//extern DMA_HandleTypeDef hdma_spi5_rx;

//extern DMA_HandleTypeDef hdma_spi5_tx;
//extern void Error_Handler(void);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	/* System interrupt init*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief IPCC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hipcc: IPCC handle pointer
 * @retval None
 */
void HAL_IPCC_MspInit(IPCC_HandleTypeDef* hipcc) {

	if (hipcc->Instance == IPCC) {
		/* USER CODE BEGIN IPCC_MspInit 0 */

		/* USER CODE END IPCC_MspInit 0 */
		/* Peripheral clock enable */

		__HAL_RCC_IPCC_CLK_ENABLE();
		/* IPCC interrupt Init */
		HAL_NVIC_SetPriority(IPCC_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(IPCC_RX1_IRQn);
		/* USER CODE BEGIN IPCC_MspInit 1 */

		/* USER CODE END IPCC_MspInit 1 */
	}

}

/**
 * @brief IPCC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hipcc: IPCC handle pointer
 * @retval None
 */

void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* hipcc) {

	if (hipcc->Instance == IPCC) {
		/* USER CODE BEGIN IPCC_MspDeInit 0 */

		/* USER CODE END IPCC_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_IPCC_CLK_DISABLE();

		/* IPCC interrupt DeInit */
		HAL_NVIC_DisableIRQ(IPCC_RX1_IRQn);
		/* USER CODE BEGIN IPCC_MspDeInit 1 */

		/* USER CODE END IPCC_MspDeInit 1 */
	}

}

/* USER CODE BEGIN 1*/

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitTypeDef GPIO_Reset = { 0 };
	int timer = 0;
	if (hspi->Instance == SPI5) {
		/* USER CODE BEGIN SPI5_MspInit 0 */

		/* USER CODE END SPI5_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI5_CLK_ENABLE();

		/**SPI5 GPIO Configuration
		 PF6     ------> SPI5_NSS
		 PF9     ------> SPI5_MOSI
		 PF7     ------> SPI5_SCK
		 PF8     ------> SPI5_MISO
		 */
	    GPIO_InitStruct.Pin = GPIO_PIN_9| GPIO_PIN_8 | GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
	    PERIPH_LOCK(GPIOF);
	    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	    PERIPH_UNLOCK(GPIOF);

	    GPIO_InitStruct.Pin = GPIO_PIN_6 ;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
		PERIPH_LOCK(GPIOF);
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		PERIPH_UNLOCK(GPIOF);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);


		/*RESET the RAK831*/
		GPIO_Reset.Pin = GPIO_PIN_8;
		GPIO_Reset.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Reset.Pull = GPIO_PULLDOWN;
		GPIO_Reset.Speed = GPIO_SPEED_FREQ_LOW;
		PERIPH_LOCK(GPIOG);
		HAL_GPIO_Init(GPIOG, &GPIO_Reset);
		PERIPH_UNLOCK(GPIOG);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
		while(timer < 600)
		{
			timer = timer + 1;
		}
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

		/* SPI5 DMA Init */
		/* SPI5_RX Init */
		/*	    hdma_spi5_rx.Instance = DMA2_Stream0;
		 hdma_spi5_rx.Init.Request = DMA_REQUEST_SPI5_RX;
		 hdma_spi5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		 hdma_spi5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		 hdma_spi5_rx.Init.MemInc = DMA_MINC_ENABLE;
		 hdma_spi5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		 hdma_spi5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		 hdma_spi5_rx.Init.Mode = DMA_NORMAL;
		 hdma_spi5_rx.Init.Priority = DMA_PRIORITY_LOW;
		 hdma_spi5_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		 hdma_spi5_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		 hdma_spi5_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		 hdma_spi5_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		 if (HAL_DMA_Init(&hdma_spi5_rx) != HAL_OK)
		 {
		 Error_Handler();
		 }

		 __HAL_LINKDMA(hspi,hdmarx,hdma_spi5_rx);
		 */
		/* SPI5_TX Init */
		/*	    hdma_spi5_tx.Instance = DMA2_Stream1;
		 hdma_spi5_tx.Init.Request = DMA_REQUEST_SPI5_TX;
		 hdma_spi5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		 hdma_spi5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		 hdma_spi5_tx.Init.MemInc = DMA_MINC_ENABLE;
		 hdma_spi5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		 hdma_spi5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		 hdma_spi5_tx.Init.Mode = DMA_NORMAL;
		 hdma_spi5_tx.Init.Priority = DMA_PRIORITY_LOW;
		 hdma_spi5_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		 hdma_spi5_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		 hdma_spi5_tx.Init.MemBurst = DMA_MBURST_SINGLE;
		 hdma_spi5_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		 if (HAL_DMA_Init(&hdma_spi5_tx) != HAL_OK)
		 {
		 Error_Handler();
		 }

		 __HAL_LINKDMA(hspi,hdmatx,hdma_spi5_tx);
		 */
		/* SPI5 interrupt Init */
		HAL_NVIC_SetPriority(SPI5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI5_IRQn);
		/* USER CODE BEGIN SPI5_MspInit 1 */

		/* USER CODE END SPI5_MspInit 1 */
	}
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
	if (hspi->Instance == SPI5) {
		/* USER CODE BEGIN SPI5_MspDeInit 0 */

		/* USER CODE END SPI5_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI5_CLK_DISABLE();

		/**SPI5 GPIO Configuration
		 PF6     ------> SPI5_NSS
		 PF9     ------> SPI5_MOSI
		 PF7     ------> SPI5_SCK
		 PF8     ------> SPI5_MISO
		 */
		HAL_GPIO_DeInit(GPIOF,
		GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_7 | GPIO_PIN_8);

		/* SPI5 DMA DeInit */
		/*		    HAL_DMA_DeInit(hspi->hdmarx);
		 HAL_DMA_DeInit(hspi->hdmatx);
		 */
		/* SPI5 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI5_IRQn);
		/* USER CODE BEGIN SPI5_MspDeInit 1 */

		/* USER CODE END SPI5_MspDeInit 1 */
	}

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
