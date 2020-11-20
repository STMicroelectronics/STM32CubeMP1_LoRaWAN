/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Host specific functions to address the LoRa concentrator registers through
    a SPI interface.
    Single-byte read/write and burst read/write.
    Does not handle pagination.
    Could be used with multiple SPI ports in parallel (explicit file descriptor)

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont

Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
*/
/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>        /* C99 types */
#include <stdio.h>        /* printf fprintf */
#include <stdlib.h>        /* malloc free */
#include <unistd.h>        /* lseek, close */
#include <fcntl.h>        /* open */
#include <string.h>        /* memset */

#include "loragw_spi.h"
#include "loragw_hal.h"

#include "main.h"
#include "stm32mp1xx_hal_spi.h"
#include <inttypes.h>

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SPI == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

enum {
	TRANSFER_WAIT,
	RECEPTION_WAIT,
	TRANSFER_COMPLETE,
	RECEPTION_COMPLETE,
	TRANSFER_RECEPTION_COMPLETE,
	TRANSFER_ERROR
};

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80
#define READ_BUFFER_SIZE 8

extern SPI_HandleTypeDef hspi5;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* SPI initialization and configuration */
int lgw_spi_open(void **spi_target_ptr) {
	if (HAL_SPI_Init(&hspi5) != HAL_OK) {
		return LGW_SPI_ERROR;
		log_info("Failed to init SPI\n");
	}

	return LGW_SPI_SUCCESS;
}

/* SPI release */
int lgw_spi_close(void *spi_target) {

	if (HAL_SPI_DeInit(&hspi5) != HAL_OK) {
		return LGW_SPI_ERROR;
		log_info("Failed to close SPI\n");
	}

	return LGW_SPI_SUCCESS;
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_spi_w(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target,
		uint8_t address, uint8_t data) {
	int size = 1, size_buffer = 0;
	uint8_t * out_buf;

	/*low the chip select*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

	/* check input variables */
	CHECK_NULL(spi_target);
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}

	/* prepare the frame to be sent depending on the mux mode */
	if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
		size_buffer = size + 2;
		out_buf = malloc(size_buffer * sizeof(uint8_t));
		out_buf[0] = spi_mux_target;
		out_buf[1] = WRITE_ACCESS | (address & 0x7F);
		out_buf[2] = data;

	} else {
		size_buffer = size + 1;
		out_buf = malloc(size_buffer * sizeof(uint8_t));
		out_buf[0] = WRITE_ACCESS | (address & 0x7F);
		out_buf[1] = data;
	}

	/*transmit to the concentrator*/
	if (HAL_SPI_Transmit(&hspi5, (uint8_t*) out_buf, size_buffer, 10)
			!= HAL_OK) {
		Error_Handler();
	}

	free(out_buf);
	/*Delays to let the concentrator managing the end of the writing*/
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	return LGW_SPI_SUCCESS;
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target,
		uint8_t address, uint8_t *data) {
	int size = 1, size_buffer = 0, i = 0;
	uint8_t *out_buf, *in_data;

	/*low the chip select*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

	/* check input variables */
	CHECK_NULL(spi_target);
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);

	/* prepare the frame to be sent depending on the mux mode */
	if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
		size_buffer = size + 2;
		out_buf = malloc(size_buffer * sizeof(uint8_t));
		in_data = malloc(size_buffer * sizeof(uint8_t));

		out_buf[0] = spi_mux_target;
		out_buf[1] = READ_ACCESS | (address & 0x7F);
		for (i = 2; i < size_buffer; i++) {
			out_buf[i] = 0x0;
		}
		/*transmit the command and receiving*/
		if (HAL_SPI_TransmitReceive(&hspi5, (uint8_t*) out_buf,
				(uint8_t*) in_data, size_buffer, 10) != HAL_OK) {
			Error_Handler();
		}
		/*stock of the datas*/
		for (i = 2; i < size_buffer; i++) {
			data[i - 2] = in_data[i];
		}

	} else {
		size_buffer = size + 1;
		out_buf = malloc(size_buffer * sizeof(uint8_t));
		in_data = malloc(size_buffer * sizeof(uint8_t));

		out_buf[0] = READ_ACCESS | (address & 0x7F);
		for (i = 1; i < size_buffer; i++) {
			out_buf[i] = 0x0;
		}
		/*transmit the command and receiving*/
		if (HAL_SPI_TransmitReceive(&hspi5, (uint8_t*) out_buf,
				(uint8_t*) in_data, size_buffer, 10) != HAL_OK) {
			Error_Handler();
		}
		/*stock of the datas*/
		for (i = 1; i < size_buffer; i++) {
			data[i - 1] = in_data[i];
		}
	}

	free(out_buf);
	free(in_data);
	/*Delays to let the concentrator managing the end of the reading*/
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	return LGW_SPI_SUCCESS;
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target,
		uint8_t address, uint8_t *data, uint16_t size) {
	uint8_t * out_buf;
	/*low the chip select*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

	/* check input variables */
	CHECK_NULL(spi_target);
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_SPI_ERROR;
	}

	/* prepare the frame to be sent depending on the mux mode */
	if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
		out_buf = malloc(2 * sizeof(uint8_t));
		out_buf[0] = spi_mux_target;
		out_buf[1] = WRITE_ACCESS | (address & 0x7F);
		/*transmit the command to the concentrator*/
		if (HAL_SPI_Transmit(&hspi5, (uint8_t*) out_buf, 2, 10) != HAL_OK) {
			Error_Handler();
		}
	} else {
		out_buf = malloc(1 * sizeof(uint8_t));
		out_buf[0] = WRITE_ACCESS | (address & 0x7F);
		/*transmit the command to the concentrator*/
		if (HAL_SPI_Transmit(&hspi5, (uint8_t*) out_buf, 1, 10) != HAL_OK) {
			Error_Handler();
		}
	}
	/*transmit the datas to the concentrator*/
	if (HAL_SPI_Transmit(&hspi5, (uint8_t*) data, size, 10) != HAL_OK) {
		Error_Handler();
	}

	free(out_buf);
	/*Delays to let the concentrator managing the end of the writing*/
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	return LGW_SPI_SUCCESS;
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target,
		uint8_t address, uint8_t *data, uint16_t size) {
	uint8_t * out_buf;
	uint16_t size_to_do = 0, buffer_size = 0;
	int index = 0;

	/*low the chip select*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

	/* check input variables */
	CHECK_NULL(spi_target);
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_SPI_ERROR;
	}
	size_to_do = size;
	index = 0;
	/* prepare the frame to be sent depending on the mux mode */
	if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
		out_buf = malloc(2 * sizeof(uint8_t));
		out_buf[0] = spi_mux_target;
		out_buf[1] = READ_ACCESS | (address & 0x7F);
		/*transmit the command to the concentrator*/
		if (HAL_SPI_Transmit(&hspi5, (uint8_t*) out_buf, 2, 10) != HAL_OK) {
			Error_Handler();
		}
	} else {
		out_buf = malloc(1 * sizeof(uint8_t));
		out_buf[0] = READ_ACCESS | (address & 0x7F);
		/*transmit the command to the concentrator*/
		if (HAL_SPI_Transmit(&hspi5, (uint8_t*) out_buf, 1, 10) != HAL_OK) {
			Error_Handler();
		}
	}
	HAL_Delay(1); /*delay needed by the concentrator to prepare the sending*/
	while (size_to_do > 0) {
		/*manage the size of the packet to receive*/
		if (size_to_do < READ_BUFFER_SIZE) {
			buffer_size = size_to_do;
		} else {
			buffer_size = READ_BUFFER_SIZE;
		}
		/*receive datas, 8 bytes maximum*/
		if (HAL_SPI_Receive(&hspi5, (uint8_t*) data + index, buffer_size, 10)
				!= HAL_OK) {
			Error_Handler();
		}
		size_to_do -= buffer_size;
		index += buffer_size;
	}

	free(out_buf);
	/*Delays to let the concentrator managing the end of the reading*/
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	return LGW_SPI_SUCCESS;
}
/* --- EOF ------------------------------------------------------------------ */
