/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    lorawan/lorawan-m4/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stm32mp15xx_disco.h"
#include "stm32mp1xx_hal_spi.h"
#include "stm32mp1xx_hal_exti.h"
#include "stm32mp1xx_hal_gpio.h"
#include "stm32mp157cxx_cm4.h"
#include "stm32mp1xx_hal.h"
#include "openamp.h"
#include "lock_resource.h"
#include <inttypes.h>
#include "string.h"
#include "loragw_spi.h"
#include "loragw_hal.h"
#include "loragw_reg.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define BUFFERSIZE                       (COUNTOF(aTxBuffer)-1)

/*RAK831*/
/*in loragw.h*/
#define LBT_CHANNEL_FREQ_NB 8 /* Number of LBT channels */
#define TX_GAIN_LUT_SIZE_MAX 16
#define LGW_RF_CHAIN_NB 2
#define NB_PKT_MAX 8
#define MAX_BUFFER_SIZE 490
#define PAYLOAD_LENGTH	256
#define OK_BUFFER_LENGTH	10
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

enum {
	TRANSFER_WAIT,
	RECEPTION_WAIT,
	TRANSFER_COMPLETE,
	RECEPTION_COMPLETE,
	TRANSFER_RECEPTION_COMPLETE,
	TRANSFER_ERROR
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef hipcc;

SPI_HandleTypeDef hspi5;
//DMA_HandleTypeDef hdma_spi5_rx;
//DMA_HandleTypeDef hdma_spi5_tx;

/* transfer state */
__IO uint32_t wTransferState;
__IO uint32_t wReceiveState;
/* USER CODE END PM */

/* USER CODE BEGIN PV */
VIRT_UART_HandleTypeDef huart0;
VIRT_UART_HandleTypeDef huart1;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

__IO FlagStatus VirtUart1RxMsg = RESET;
uint8_t VirtUart1ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart1ChannelRxSize = 0;

uint8_t From_RAK[MAX_BUFFER_SIZE] = { 0 };
uint8_t To_RAK[MAX_BUFFER_SIZE] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETZPC_Init(void);
//static void MX_DMA_Init(void);
static void MX_SPI5_Init(void);
static void MX_IPCC_Init(void);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initialize the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* Configure the system clock */
	if (IS_ENGINEERING_BOOT_MODE()) {
		/* Configure the system clock */
		SystemClock_Config();
	}

	log_info(
			"Cortex-M4 boot successful with STM32Cube FW version: v%ld.%ld.%ld \r\n",
			((HAL_GetHalVersion() >> 24) & 0x000000FF),
			((HAL_GetHalVersion() >> 16) & 0x000000FF),
			((HAL_GetHalVersion() >> 8) & 0x000000FF));
	/* USER CODE END Init */

	/* IPCC initialisation */
	MX_IPCC_Init();


	/* USER CODE BEGIN SysInit */
	/* OpenAmp initialisation ---------------------------------*/
		MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ETZPC_Init();
	//MX_DMA_Init();
	MX_SPI5_Init();
	/* USER CODE BEGIN 2 */
	/*
	 * Create Virtual UART device
	 * defined by a rpmsg channel attached to the remote device
	 */
	log_info("Virtual UART0 OpenAMP-rpmsg channel creation\r\n");
	if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
		log_err("VIRT_UART_Init UART0 failed.\r\n");
		Error_Handler();
	}

	log_info("Virtual UART1 OpenAMP-rpmsg channel creation\r\n");
	if (VIRT_UART_Init(&huart1) != VIRT_UART_OK) {
		log_err("VIRT_UART_Init UART1 failed.\r\n");
		Error_Handler();
	}

	/*Need to register callback for message reception by channels*/
	if (VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID,
			VIRT_UART0_RxCpltCallback) != VIRT_UART_OK) {
		Error_Handler();
	}
	if (VIRT_UART_RegisterCallback(&huart1, VIRT_UART_RXCPLT_CB_ID,
			VIRT_UART1_RxCpltCallback) != VIRT_UART_OK) {
		Error_Handler();
	}


	BSP_LED_Init(LED6);
	BSP_LED_Init(LED5);

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int configration_done = 0, index_done = 0, transfert_to_do = 0,
			receive_to_do = 0;
	int timer = 0, delay = 20000, receive_enable = 0;
	int nb_pkt = 0;
	/* allocate memory for packet fetching and processing */
	struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX]; /* array containing inbound packets + metadata */
	struct lgw_pkt_tx_s pkt;

	/*array to communicate with the A7*/
	char msg_to_transmit[MAX_BUFFER_SIZE];

	int start_done = 0;

	/*float to array uint8_t*/
	char tmpSign;
	float tempVal = 0, tempFraction;
	int Integer = 0, Fraction = 0;
	float f_signe = 0;
	int delta_size = 0, msg_size = 0; /*messages size*/
	int y = 0, i = 0, result = LGW_HAL_SUCCESS, ipayload = 0;

	struct lgw_conf_board_s boardconf;
	struct lgw_conf_lbt_s lbtconf;
	struct lgw_conf_rxrf_s rfconf;
	struct lgw_conf_rxif_s ifconf;
	struct lgw_tx_gain_lut_s txlut;
	uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
	uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */
	uint8_t code_status;
	uint32_t trig_tstamp;
	uint32_t sx1301_timecount;

	/*used for sscanf*/
	uint8_t *v_array;

	/* initialize memory*/
	memset(&boardconf, 0, sizeof boardconf);
	memset(&lbtconf, 0, sizeof lbtconf);
	memset(&txlut, 0, sizeof txlut);
	memset(&rfconf, 0, sizeof rfconf);
	memset(&ifconf, 0, sizeof ifconf);
	memset(VirtUart0ChannelBuffRx, 0, MAX_BUFFER_SIZE);
	memset(VirtUart1ChannelBuffRx, 0, MAX_BUFFER_SIZE); /*cleaning the buffer*/
	memset(msg_to_transmit, 0, MAX_BUFFER_SIZE);
	memset(&pkt, 0, sizeof(pkt));

	configration_done = 1;
	/*receiving the json variables configuration from the A7*/
	while (configration_done) {
		OPENAMP_check_for_message();
		if (VirtUart0RxMsg) {
			VirtUart0RxMsg = RESET;

			/*lgw_conf_board_s*/
			/* bool lorawan_public*/
			boardconf.lorawan_public = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*uint8_t clksrc*/
			boardconf.clksrc = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;

			if (lgw_board_setconf(boardconf) != LGW_HAL_SUCCESS) {
				return -1;
			}

			/* set configuration for Tx gain LUT */
			/*uint8_t size*/
			txlut.size = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
				/*uint8_t pa_gain*/
				txlut.lut[i].pa_gain = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*int8_t  rf_power*/
				txlut.lut[i].rf_power = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint8_t mix_gain*/
				txlut.lut[i].mix_gain = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint8_t dig_gain*/
				txlut.lut[i].dig_gain = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint8_t dac_gain*/
				txlut.lut[i].dac_gain = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
			}
			if (txlut.size > 0) {
				if (lgw_txgain_setconf(&txlut) != LGW_HAL_SUCCESS) {
					return -1;
				}
			}

			/* set configuration for RF chains */
			for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
				/*bool enable*/
				rfconf.enable = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint32_t freq_hz*/
				rfconf.freq_hz = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				rfconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				rfconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				rfconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;

				/*float rssi_offset*/
				if (VirtUart0ChannelBuffRx[delta_size] == '-') {
					f_signe = -1;
				} else {
					f_signe = 1;
				}
				delta_size += 1;
				Integer = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				Integer |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				Integer |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				Integer |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;
				Fraction = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				Fraction |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				Fraction |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				Fraction |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;
				rfconf.rssi_offset = f_signe
						* (Integer + ((float) Fraction / 100));

				/*enum lgw_radio_type_e   type*/
				rfconf.type = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*bool tx_enable*/
				rfconf.tx_enable = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint32_t tx_notch_freq*/
				rfconf.tx_notch_freq = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				rfconf.tx_notch_freq |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				rfconf.tx_notch_freq |= VirtUart0ChannelBuffRx[delta_size]
						<< 16;
				delta_size += 1;
				rfconf.tx_notch_freq |= VirtUart0ChannelBuffRx[delta_size]
						<< 24;
				delta_size += 1;
				/*uint32_t tx_freq_min*/
				tx_freq_min[i] = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				tx_freq_min[i] |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				tx_freq_min[i] |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				tx_freq_min[i] |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;
				/*uint32_t tx_freq_max*/
				tx_freq_max[i] = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				tx_freq_max[i] |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				tx_freq_max[i] |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				tx_freq_max[i] |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;

				if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
					return -1;
				}
				memset(&rfconf, 0, sizeof rfconf);
			}
			/* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
			for (i = 0; i < LGW_MULTI_NB; ++i) {
				/*bool enable*/
				ifconf.enable = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*uint8_t rf_chain*/
				ifconf.rf_chain = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				/*int32_t freq_hz*/
				ifconf.freq_hz = VirtUart0ChannelBuffRx[delta_size];
				delta_size += 1;
				ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 8;
				delta_size += 1;
				ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 16;
				delta_size += 1;
				ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 24;
				delta_size += 1;

				if (lgw_rxif_setconf(i, ifconf) != LGW_HAL_SUCCESS) {
					return -1;
				}
				memset(&ifconf, 0, sizeof ifconf);
			}

			/* set configuration for Lora standard channel */
			/*bool enable*/
			ifconf.enable = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*uint8_t rf_chain*/
			ifconf.rf_chain = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*int32_t freq_hz*/
			ifconf.freq_hz = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 8;
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 16;
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 24;
			delta_size += 1;
			/*uint8_t bandwidth*/
			ifconf.bandwidth = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*uint32_t datarate*/
			ifconf.datarate = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 8;
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 16;
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 24;
			delta_size += 1;

			if (lgw_rxif_setconf(8, ifconf) != LGW_HAL_SUCCESS) {
				return -1;
			}
			memset(&ifconf, 0, sizeof ifconf);

			/* set configuration for FSK channel */
			/*bool enable*/
			ifconf.enable = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*uint8_t rf_chain*/
			ifconf.rf_chain = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*int32_t freq_hz*/
			ifconf.freq_hz = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 8;
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 16;
			delta_size += 1;
			ifconf.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 24;
			delta_size += 1;
			/*uint8_t bandwidth*/
			ifconf.bandwidth = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			/*uint32_t datarate*/
			ifconf.datarate = VirtUart0ChannelBuffRx[delta_size];
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 8;
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 16;
			delta_size += 1;
			ifconf.datarate |= VirtUart0ChannelBuffRx[delta_size] << 24;
			delta_size += 1;

			if (lgw_rxif_setconf(9, ifconf) != LGW_HAL_SUCCESS) {
				return -1;
			}
			memset(&ifconf, 0, sizeof ifconf);
			/*synchronize with the A7 that the configuration is received and set on the M4 side*/
			memset(VirtUart0ChannelBuffRx, 0, MAX_BUFFER_SIZE);
			msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "Conf_Set\n");
			VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit, msg_size);
			configration_done = 0;
		}
	}

	/*Start the concentrator*/
	configration_done = 1;
	while (configration_done) {
		OPENAMP_check_for_message();
		if (VirtUart0RxMsg) {
			VirtUart0RxMsg = RESET;

			if (memcmp(VirtUart0ChannelBuffRx, "lgw_start\n",
					strlen("lgw_start\n")) == 0) {
				memset(VirtUart0ChannelBuffRx, 0, MAX_BUFFER_SIZE);

				/*attempt multiple start in case of miss writing/reading*/
				/*usually succeeds on the second attempt*/
				while (y < 5) {
					log_info("start gateway\n");
					start_done = lgw_start();
					if (start_done == LGW_HAL_SUCCESS) {
						/*start of the concentrator done*/
						msg_size =
								snprintf(msg_to_transmit, MAX_BUFFER_SIZE,
										"LGW_HAL_SUCCESS from M4: concentrator started, packet can now be received\n");
						VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
								msg_size);
						/*configuration ends*/
						configration_done = 0;
						break;
					}
					y += 1;
				}
				/*configuration failed, stop of the process*/
				if (start_done != LGW_HAL_SUCCESS) {
					msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE,
							"ERROR from M4: fail to start concentrator\n");
					VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
							msg_size);
					lgw_spi_close((void *) &hspi5);
					exit(EXIT_FAILURE);
				}
			}
		}
	}

	/*Super loop*/
	/*Manage to receive packets from the LoRaWAN nodes and transmit them with RPMSG to the A7*/
	/*Manage to receive messages from the A7 if it needs informations from the concentrator*/
	while (1) {
		OPENAMP_check_for_message();
		if (VirtUart0RxMsg) {
			/*LED 5 to indicate there is an activity from the A7*/
			BSP_LED_On(LED5);
			/*A7 needs the tx status from the concentrator*/
			if (memcmp(VirtUart0ChannelBuffRx, "request_status\n",
					strlen("request_status\n")) == 0) {
				VirtUart0RxMsg = RESET;
				log_info("request_status_____\n");
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);
				result = lgw_status(TX_STATUS, &code_status);
				memset(From_RAK, 0, MAX_BUFFER_SIZE);
				delta_size = 0;
				From_RAK[delta_size] = result;
				delta_size += 1;
				From_RAK[delta_size] = code_status;
				delta_size += 1;
				From_RAK[delta_size] = '\n';
				delta_size += 1;
				if (VIRT_UART_Transmit(&huart0, From_RAK, delta_size) != 0) {
					return 0;
				}
				VirtUart0RxMsg = RESET;
			} /*A7 needs the timestamp captured on PPM pulse from the concentrator*/
			else if (memcmp(VirtUart0ChannelBuffRx, "request_trigcnt\n",
					strlen("request_trigcnt\n")) == 0) {
				VirtUart0RxMsg = RESET;
				log_info("request_trigcnt_____\n");
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);
				memset(From_RAK, 0, MAX_BUFFER_SIZE);

				result = lgw_get_trigcnt(&trig_tstamp);
				delta_size = 0;
				From_RAK[delta_size] = result;
				delta_size += 1;
				From_RAK[delta_size] = trig_tstamp; /*LSB*/
				delta_size += 1;
				From_RAK[delta_size] = trig_tstamp >> 8;
				delta_size += 1;
				From_RAK[delta_size] = trig_tstamp >> 16;
				delta_size += 1;
				From_RAK[delta_size] = trig_tstamp >> 24; /*MSB*/
				delta_size += 1;
				From_RAK[delta_size] = '\n';
				delta_size += 1;
				if (VIRT_UART_Transmit(&huart0, From_RAK, delta_size) != 0) {
					return 0;
				}
				VirtUart0RxMsg = RESET;

			}
			/*Manage the counter from the timersync thread*/
			else if (memcmp(VirtUart0ChannelBuffRx, "disable_gps\n",
					strlen("disable_gps\n")) == 0) {
				VirtUart0RxMsg = RESET;
				log_info("disable_gps_________\n");
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);

				lgw_reg_w(LGW_GPS_EN, 0);

				msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "ok\n");
				VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
						msg_size);
				VirtUart0RxMsg = RESET;

			} else if (memcmp(VirtUart0ChannelBuffRx, "enable_gps\n",
					strlen("enable_gps\n")) == 0) {
				VirtUart0RxMsg = RESET;
				log_info("enable_gps\n");
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);

				lgw_reg_w(LGW_GPS_EN, 1);

				msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "ok\n");
				VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
						msg_size);
				VirtUart0RxMsg = RESET;

			} else if (memcmp(VirtUart0ChannelBuffRx, "sx1301_timecount\n",
					strlen("sx1301_timecount\n")) == 0) {
				VirtUart0RxMsg = RESET;
				log_info("sx1301_timecount\n");
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);
				memset(From_RAK, 0, MAX_BUFFER_SIZE);

				lgw_get_trigcnt(&sx1301_timecount);
				delta_size = 0;
				From_RAK[delta_size] = sx1301_timecount; /*LSB*/
				delta_size += 1;
				From_RAK[delta_size] = sx1301_timecount >> 8;
				delta_size += 1;
				From_RAK[delta_size] = sx1301_timecount >> 16;
				delta_size += 1;
				From_RAK[delta_size] = sx1301_timecount >> 24; /*MSB*/
				delta_size += 1;
				From_RAK[delta_size] = '\n';
				delta_size += 1;
				if (VIRT_UART_Transmit(&huart0, From_RAK, delta_size) != 0) {
					return 0;
				}
				VirtUart0RxMsg = RESET;
			} else if (memcmp(VirtUart0ChannelBuffRx, "data to send\n",
					strlen("data to send\n")) == 0) {
				VirtUart0RxMsg = RESET;
				memset(VirtUart0ChannelBuffRx, 0,
				MAX_BUFFER_SIZE);
				/*acknoledge to continue the process*/
				msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "ack\n");
				VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
						msg_size);
				VirtUart0RxMsg = RESET;
				/*the A7 is sending a packet to the concentrator*/
				receive_to_do = 1;
				while (receive_to_do) {
					OPENAMP_check_for_message();
					if (VirtUart0RxMsg) {
						VirtUart0RxMsg = RESET;
						log_info("data to send\n");
						delta_size = 0;
						/*freq_hz*/
						pkt.freq_hz = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;
						pkt.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 8;
						delta_size += 1;
						pkt.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 16;
						delta_size += 1;
						pkt.freq_hz |= VirtUart0ChannelBuffRx[delta_size] << 24;
						delta_size += 1;

						/*tx_mode*/
						pkt.tx_mode = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*count_us*/
						pkt.count_us = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;
						pkt.count_us |= VirtUart0ChannelBuffRx[delta_size] << 8;
						delta_size += 1;
						pkt.count_us |= VirtUart0ChannelBuffRx[delta_size]
								<< 16;
						delta_size += 1;
						pkt.count_us |= VirtUart0ChannelBuffRx[delta_size]
								<< 24;
						delta_size += 1;

						/*rf_chain*/
						pkt.rf_chain = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*rf_power*/
						pkt.rf_power =
								(int8_t) VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*modulation*/
						pkt.modulation = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*bandwidth*/
						pkt.bandwidth = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*datarate*/
						pkt.datarate = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;
						pkt.datarate |= VirtUart0ChannelBuffRx[delta_size] << 8;
						delta_size += 1;
						pkt.datarate |= VirtUart0ChannelBuffRx[delta_size]
								<< 16;
						delta_size += 1;
						pkt.datarate |= VirtUart0ChannelBuffRx[delta_size]
								<< 24;
						delta_size += 1;

						/*coderate*/
						pkt.coderate = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*invert_pol*/
						pkt.invert_pol =
								(bool) VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*f_dev*/
						pkt.f_dev = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*preamble*/
						pkt.preamble = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;
						pkt.preamble |= VirtUart0ChannelBuffRx[delta_size] << 8;
						delta_size += 1;

						/*no_crc*/
						pkt.no_crc = (bool) VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*no_header*/
						pkt.no_header =
								(bool) VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;

						/*size*/
						pkt.size = VirtUart0ChannelBuffRx[delta_size];
						delta_size += 1;
						pkt.size |= VirtUart0ChannelBuffRx[delta_size] << 8;
						delta_size += 1;

						/*payload*/
						for (ipayload = 0; ipayload < PAYLOAD_LENGTH;
								ipayload++) {
							pkt.payload[ipayload] =
									VirtUart0ChannelBuffRx[delta_size];
							delta_size += 1;
						}
						/*sending the parsed buffer*/
						result = lgw_send(pkt);
						memset(VirtUart0ChannelBuffRx, 0, MAX_BUFFER_SIZE);
						/*acknoledge to continue the process*/
						msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE,
								"packet sent\n");
						VIRT_UART_Transmit(&huart0, (uint8_t*) msg_to_transmit,
								msg_size);
						VirtUart0RxMsg = RESET;
						receive_to_do = 0;

					}
				}
			}
			BSP_LED_Off(LED5);
		}

		/*Delay of 7ms between two attemps of reading but not blocking the loop*/
		if (timer < delay) {
			timer += 1;
		} else {
			receive_enable = 1;
			timer = 0;
		}

		if (receive_enable == 1) {
			memset(&rxpkt, 0, sizeof rxpkt);
			memset(From_RAK, 0, MAX_BUFFER_SIZE);
			/*try to read from the concentrator register*/
			nb_pkt = lgw_receive(NB_PKT_MAX, rxpkt);
			receive_enable = 0;
		}
		/*if there is an error during the reading*/
		if (nb_pkt == LGW_HAL_ERROR) {
			exit(EXIT_FAILURE);
		}
		/*if a packet is available*/
		if (nb_pkt > 0) {
			log_info("sending data\n");
			/*LED 6 to indicate there is an activity to the A7*/
			BSP_LED_On(LED6);
			/*sending to the A7 the number of packets it will have to manage*/
			msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "%d\n",
					nb_pkt);
			VIRT_UART_Transmit(&huart1, (uint8_t*) msg_to_transmit, msg_size);
			VirtUart1RxMsg = RESET;

			transfert_to_do = 1;
			/*waiting that the A7's acknoledge after receiving the number of packets*/
			while (transfert_to_do) {
				OPENAMP_check_for_message();
				if (VirtUart1RxMsg) {
					VirtUart1RxMsg = RESET;
					memset(VirtUart1ChannelBuffRx, 0, MAX_BUFFER_SIZE); /*cleaning the buffer*/

					/*parsing the datas into a buffer*/
					for (i = 0; i < nb_pkt; i++) {
						delta_size = 0;
						/*freq_hz*/
						From_RAK[delta_size] = rxpkt[i].freq_hz; /*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].freq_hz >> 8;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].freq_hz >> 16;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].freq_hz >> 24; /*MSB*/
						/*if_chain*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].if_chain;
						/*status*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].status;
						/*count_us*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].count_us; /*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].count_us >> 8;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].count_us >> 16;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].count_us >> 24; /*MSB*/
						/*rf_chain*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].rf_chain;
						/*modulation*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].modulation;
						/*bandwidth*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].bandwidth;
						/*datarate*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].datarate; /*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].datarate >> 8;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].datarate >> 16;
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].datarate >> 24; /*MSB*/
						/*coderate*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].coderate;
						/*RSSI*/
						delta_size += 1;
						if (rxpkt[i].rssi < 0) {
							sscanf("-", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						} else {
							sscanf("+", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						}
						tempVal =
								(rxpkt[i].rssi < 0) ?
										-rxpkt[i].rssi : rxpkt[i].rssi;
						Integer = tempVal;
						tempFraction = tempVal - Integer;
						Fraction = trunc(tempFraction * 100); /*2 fractionnal number maximum*/

						v_array = (uint8_t*) (&Integer);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						v_array = (uint8_t*) (&Fraction);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						/*snr*/
						delta_size += 1;
						if (rxpkt[i].snr < 0) {
							sscanf("-", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						} else {
							sscanf("0", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						}
						tempVal =
								(rxpkt[i].snr < 0) ?
										-rxpkt[i].snr : rxpkt[i].snr;
						Integer = tempVal;
						tempFraction = tempVal - Integer;
						Fraction = trunc(tempFraction * 100); /*2 fractionnal number maximum*/

						v_array = (uint8_t*) (&Integer);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						v_array = (uint8_t*) (&Fraction);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						/*snr_min*/
						delta_size += 1;
						if (rxpkt[i].snr_min < 0) {
							sscanf("-", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						} else {
							sscanf("0", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						}
						tempVal =
								(rxpkt[i].snr_min < 0) ?
										-rxpkt[i].snr_min : rxpkt[i].snr_min;
						Integer = tempVal;
						tempFraction = tempVal - Integer;
						Fraction = trunc(tempFraction * 100); /*2 fractionnal number maximum*/

						v_array = (uint8_t*) (&Integer);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						v_array = (uint8_t*) (&Fraction);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						/*snr_max*/
						delta_size += 1;
						if (rxpkt[i].snr_max < 0) {
							sscanf("-", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						} else {
							sscanf("0", "%s", &tmpSign);
							From_RAK[delta_size] = tmpSign;
						}
						tempVal =
								(rxpkt[i].snr_max < 0) ?
										-rxpkt[i].snr_max : rxpkt[i].snr_max;
						Integer = tempVal;
						tempFraction = tempVal - Integer;
						Fraction = trunc(tempFraction * 100); /*2 fractionnal number maximum*/

						v_array = (uint8_t*) (&Integer);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						v_array = (uint8_t*) (&Fraction);
						delta_size += 1;
						From_RAK[delta_size] = v_array[0];/*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = v_array[1];
						delta_size += 1;
						From_RAK[delta_size] = v_array[2];
						delta_size += 1;
						From_RAK[delta_size] = v_array[3];/*MSB*/

						/*crc*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].crc; /*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].crc >> 8; /*MSB*/
						/*size*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].size; /*LSB*/
						delta_size += 1;
						From_RAK[delta_size] = rxpkt[i].size >> 8; /*MSB*/
						delta_size += 1;
						/*payload*/
						for (ipayload = 0; ipayload < PAYLOAD_LENGTH;
								ipayload++) {
							From_RAK[delta_size] = rxpkt[i].payload[ipayload];
							delta_size += 1;
						}
						/*closing the array*/
						delta_size += 1;
						From_RAK[delta_size] = '\n';
						/*transmitting to the M4 the buffer*/
						if (VIRT_UART_Transmit(&huart1, From_RAK, delta_size)
								!= 0) {
							return 0;
						}
						VirtUart1RxMsg = RESET;

						/*waiting for the A7 to finish the management of the buffer*/
						index_done = 1;
						while (index_done) {
							OPENAMP_check_for_message();
							if (VirtUart1RxMsg) {
								VirtUart1RxMsg = RESET;
								index_done = 0;
							}
						}
					}
					transfert_to_do = 0;
				}
			}
			/*wait for a new packet to send*/
			BSP_LED_Off(LED6);
			nb_pkt = 0;
		}
	}
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

	/**PLL1 Config
	 */
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 3;
	RCC_OscInitStruct.PLL.PLLN = 81;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 1;
	RCC_OscInitStruct.PLL.PLLR = 1;
	RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
	RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

	/**PLL2 Config
	 */
	RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
	RCC_OscInitStruct.PLL2.PLLM = 3;
	RCC_OscInitStruct.PLL2.PLLN = 66;
	RCC_OscInitStruct.PLL2.PLLP = 2;
	RCC_OscInitStruct.PLL2.PLLQ = 1;
	RCC_OscInitStruct.PLL2.PLLR = 1;
	RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
	RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

	/**PLL3 Config
	 */
	RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
	RCC_OscInitStruct.PLL3.PLLM = 2;
	RCC_OscInitStruct.PLL3.PLLN = 34;
	RCC_OscInitStruct.PLL3.PLLP = 2;
	RCC_OscInitStruct.PLL3.PLLQ = 17;
	RCC_OscInitStruct.PLL3.PLLR = 37;
	RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
	RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
	RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
	RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

	/**PLL4 Config
	 */
	RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
	RCC_OscInitStruct.PLL4.PLLM = 4;
	RCC_OscInitStruct.PLL4.PLLN = 99;
	RCC_OscInitStruct.PLL4.PLLP = 6;
	RCC_OscInitStruct.PLL4.PLLQ = 8;
	RCC_OscInitStruct.PLL4.PLLR = 8;
	RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
	RCC_OscInitStruct.PLL4.PLLFRACV = 0;
	RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
	RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
	RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**RCC Clock Config
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_ACLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3
			| RCC_CLOCKTYPE_PCLK4 | RCC_CLOCKTYPE_PCLK5 | RCC_CLOCKTYPE_MPU;
	RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
	RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
	RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
	RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
	RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
	RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
	RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
	RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
	RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Set the HSE division factor for RTC clock
	 */
	__HAL_RCC_RTC_HSEDIV(24);
}

/**
 * @brief IPPC Initialization Function
 * @param None
 * @retval None
 */
static void MX_IPCC_Init(void) {

	hipcc.Instance = IPCC;
	if (HAL_IPCC_Init(&hipcc) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ETZPC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETZPC_Init(void) {

	/* USER CODE BEGIN ETZPC_Init 0 */

	/* USER CODE END ETZPC_Init 0 */

	/* USER CODE BEGIN ETZPC_Init 1 */

	/* USER CODE END ETZPC_Init 1 */
	/* USER CODE BEGIN ETZPC_Init 2 */

	/* USER CODE END ETZPC_Init 2 */

}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void) {

	/* USER CODE BEGIN SPI5_Init 0 */

	/* USER CODE END SPI5_Init 0 */

	/* USER CODE BEGIN SPI5_Init 1 */

	/* USER CODE END SPI5_Init 1 */
	/* SPI5 parameter configuration*/
	hspi5.Instance = SPI5;
	hspi5.Init.Mode = SPI_MODE_MASTER;
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi5.Init.NSS = SPI_NSS_SOFT;
	hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi5.Init.CRCPolynomial = 0x0;
	hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi5.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi5.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_01CYCLE;
	hspi5.Init.MasterInterDataIdleness =
	SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
	hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	/*
	 if (HAL_SPI_Init(&hspi5) != HAL_OK) {
	 Error_Handler();
	 }
	 */
	/* USER CODE BEGIN SPI5_Init 2 */
	/* USER CODE END SPI5_Init 2 */

}

/**
 * Enable DMA controller clock
 */
#if 0
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMAMUX_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}
#endif
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOZ_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart) {

	log_info("Msg received on VIRTUAL UART0 channel:  %s \n\r",
			(char * ) huart->pRxBuffPtr);

	/* copy received msg in a variable to sent it back to master processor in main infinite loop*/
	VirtUart0ChannelRxSize =
			huart->RxXferSize < MAX_BUFFER_SIZE ?
					huart->RxXferSize : MAX_BUFFER_SIZE - 1;
	memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
	VirtUart0RxMsg = SET;
}

void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart) {

	log_info("Msg received on VIRTUAL UART1 channel:  %s \n\r",
			(char * ) huart->pRxBuffPtr);

	/* copy received msg in a variable to sent it back to master processor in main infinite loop*/
	VirtUart1ChannelRxSize =
			huart->RxXferSize < MAX_BUFFER_SIZE ?
					huart->RxXferSize : MAX_BUFFER_SIZE - 1;
	memcpy(VirtUart1ChannelBuffRx, huart->pRxBuffPtr, VirtUart1ChannelRxSize);
	VirtUart1RxMsg = SET;
}

/**
 * @brief  TxRx Transfer completed callback.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	/* Turn LED5 on: Transfer in transmission/reception process is complete */
	wTransferState = TRANSFER_RECEPTION_COMPLETE;
	wReceiveState = TRANSFER_RECEPTION_COMPLETE;
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_RECEPTION_COMPLETE;
	wReceiveState = TRANSFER_RECEPTION_COMPLETE;
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	wReceiveState = RECEPTION_COMPLETE;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	wReceiveState = RECEPTION_COMPLETE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_ERROR;
	wReceiveState = TRANSFER_ERROR;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	log_err("Error_Handler");
	while (1) {
		/* Toggle LED7 for error */
		BSP_LED_Toggle(LED7);
		HAL_Delay(1000);
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	log_err("OOOps: file %s, line %d\r\n", __FILE__, __LINE__);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
