/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#define VERSION 0x00000102

typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t buf[8];
} rxMessage_t;

rxMessage_t recMsgs[128];
uint8_t usbBuf[128];
char idString[16];
char errorString[128];
uint8_t error = 0;
uint8_t usbPtr = 0;
uint8_t usbMesLen = 0;
uint8_t gotUsbMessage = 0;

uint8_t recCnt = 0;
uint8_t canState = 0;
uint32_t canMode = CAN_MODE_NORMAL;
uint32_t lastRx = 0;
uint32_t lastTx = 0;
uint32_t totalRXCnt = 0;
uint32_t totalTXCnt = 0;
CAN_FilterTypeDef filter;
uint32_t mailBox = 0;

uint8_t listen = 1;

uint8_t len;
uint8_t txData[8] = { 0, };
char outString[32] = { 0, };
CAN_TxHeaderTypeDef header = { 0, };
uint16_t bitrate = 250;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void handleError(char *str) {
	char finalString[128];
	sprintf(finalString, "<E%s>\r\n", str);
	CDC_Transmit_FS((uint8_t*) finalString, strlen(finalString));

	error = 1;

}

void printLog(char *str) {
	char finalString[128];
	sprintf(finalString, "<P%s>\r\n", str);
	CDC_Transmit_FS((uint8_t*) finalString, strlen(finalString));
}
uint32_t pow10_(uint8_t value) {
	if (value > 0)
		return 10 * pow10_(value - 1);
	else
		return 1;
}

uint32_t DecToInt(uint8_t *string, uint8_t len) {
	uint32_t value = 0;
	uint8_t i = 0;
	for (i = 0; i < len; i++)
		if (string[i] >= '0' && string[i] <= '9')
			value += (string[i] - '0') * pow10_(len - i - 1);
		else {
			handleError("Can't convert into int, not a number!");
			return 0xFFFFFFFF;
		}
	return value;
}

uint32_t pow16(uint8_t value) {
	return 1 << (value * 4);
}

uint32_t HexToInt(uint8_t *string, uint8_t len) {
	uint32_t value = 0;
	uint32_t tmp = 0;
	uint8_t i = 0;

	for (i = 0; i < len; i++) {
		switch (string[i]) {
		case '0':
			tmp = 0;
			break;
		case '1':
			tmp = 1;
			break;
		case '2':
			tmp = 2;
			break;
		case '3':
			tmp = 3;
			break;
		case '4':
			tmp = 4;
			break;
		case '5':
			tmp = 5;
			break;
		case '6':
			tmp = 6;
			break;
		case '7':
			tmp = 7;
			break;
		case '8':
			tmp = 8;
			break;
		case '9':
			tmp = 9;
			break;
		case 'A':
		case 'a':
			tmp = 10;
			break;
		case 'B':
		case 'b':
			tmp = 11;
			break;
		case 'C':
		case 'c':
			tmp = 12;
			break;
		case 'D':
		case 'd':
			tmp = 13;
			break;
		case 'E':
		case 'e':
			tmp = 14;
			break;
		case 'F':
		case 'f':
			tmp = 15;
			break;
		default:
			handleError("Can't parse hex value! Invalid string!");
			return 0xFFFFFFFF;
		}
		value += tmp * pow16(len - i - 1);
	}
	return value;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &recMsgs[recCnt].header, recMsgs[recCnt].buf) != HAL_OK)
		Error_Handler();
	if (recCnt < 128)
		recCnt++;

	lastRx = HAL_GetTick();
	totalRXCnt++;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	handleError("CAN BUS error acquired. Restarting CAN");
}

void CDC_ReceiveCallback(uint8_t *Buf, uint32_t len) {
	uint8_t *ptr = Buf;
	while (len--) {
		if (*ptr == '<')
			usbPtr = 0;
		else if (*ptr == '>') {
			usbMesLen = usbPtr;
			gotUsbMessage = 1;
			usbBuf[usbPtr] = 0;
			usbPtr = 0;
		} else {
			usbBuf[usbPtr] = *ptr;
			usbPtr++;
		}
		ptr++;
	}

}

void CanCustomInit() {
	HAL_CAN_Stop(&hcan);
	HAL_CAN_DeInit(&hcan);
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 3000 / bitrate;
	hcan.Init.Mode = canMode;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
		handleError("Cant' init CAN!");
	if (canState)
		if (HAL_CAN_Start(&hcan) != HAL_OK)
			handleError("Can't start CAN!");
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_USB_DEVICE_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk;
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
		handleError("Cant' config CAN filter!");
		goto loopStart;
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		handleError("Cant' start CAN!");
		goto loopStart;
	}
	canState = 1;
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		handleError("Cant' attach CAN RXD PENDING IRQ!");
		goto loopStart;
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR) != HAL_OK) {
		handleError("Cant' attach CAN ERROR IRQ!");
		goto loopStart;
	}
	printLog("Reset");

	while (1) {
		loopStart:
		HAL_IWDG_Refresh(&hiwdg);
		if (error) {
			error = 0;
			CanCustomInit();
			memset(usbBuf,0,sizeof(usbBuf));
			usbPtr=0;
		}

		if (recCnt > 0) {
			uint8_t cur = recCnt - 1;
			CAN_RxHeaderTypeDef *hdr = &recMsgs[cur].header;
			sprintf(outString, "<R%1d%1d%1d_", (int) hdr->DLC, (int) hdr->IDE, (int) hdr->RTR);
			if (!hdr->IDE) {
				sprintf(idString, "%03lX_", hdr->StdId);
			} else {
				sprintf(idString, "%08lX_", hdr->ExtId);
			}
			strcat(outString, idString);
			for (int i = 0; i < hdr->DLC; i++) {
				char byteString[3];
				sprintf(byteString, "%02X", recMsgs[cur].buf[i]);
				strcat(outString, byteString);
			}
			strcat(outString, ">\r\n");
			CDC_Transmit_FS((uint8_t*) outString, strlen(outString));
			recCnt--;
		}

		if (HAL_GetTick() - lastRx < 30 || recCnt==128)
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
		else
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);

		if (HAL_GetTick() - lastTx < 30)
			HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
		else
			HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 1);

		if (gotUsbMessage) {
			uint8_t len = usbMesLen;
			gotUsbMessage = 0;

			switch (usbBuf[0]) {
			case '1':
				usbBuf[0]=0;
				if (canState) {
					printLog("CAN Adapter is already started");
					break;
				}
				if (HAL_CAN_Start(&hcan) != HAL_OK) {
					handleError("Cant' start CAN!");
					goto loopStart;
				}
				if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
					handleError("Cant' attach CAN IRQ!");
					goto loopStart;
				}

				if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR) != HAL_OK) {
					handleError("Cant' attach CAN IRQ!");
					goto loopStart;
				}
				canState = 1;
				printLog("CAN Adapter turned ON");
				break;
			case '2':
				usbBuf[0]=0;
				if (!canState) {
					printLog("CAN Adapter is already stopped");
					break;
				}
				if (HAL_CAN_Stop(&hcan) != HAL_OK) {
					handleError("Cant' stop CAN!");
					goto loopStart;
				}
				canState = 0;
				printLog("CAN Adapter turned OFF");
				break;
			case '3':
				usbBuf[0]=0;
				if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
					handleError("Cant' config CAN filter!");
					goto loopStart;
				}
				printLog("Filter set");
				break;
			case '4':
				usbBuf[0]=0;
				if (usbBuf[1] == '0') {
					filter.FilterMode = CAN_FILTERMODE_IDMASK;
					printLog("Mask filter selected");
				} else if (usbBuf[1] == '1') {
					filter.FilterMode = CAN_FILTERMODE_IDLIST;
					printLog("List filter selected");
				} else {
					sprintf(errorString, "Filter code must be 0 or 1, not %c", usbBuf[1]);
					handleError(errorString);
					goto loopStart;
				}

				break;
			case '5':
				usbBuf[0]=0;
				filter.FilterMaskIdLow = HexToInt(usbBuf + 1, 8);
				if (error) goto loopStart;
				break;
			case '6':
				usbBuf[0]=0;
				filter.FilterMaskIdHigh = HexToInt(usbBuf + 1, 8);
				if (error) goto loopStart;
				break;
			case '7':
				usbBuf[0]=0;
				filter.FilterIdLow = HexToInt(usbBuf + 1, 8);
				if (error) goto loopStart;
				break;
			case '8':
				usbBuf[0]=0;
				filter.FilterIdHigh = HexToInt(usbBuf + 1, 8);
				if (error) goto loopStart;
				break;
			case '9':
				usbBuf[0]=0;
				bitrate = DecToInt(usbBuf + 1, len - 1);
				if (error) goto loopStart;
				if (bitrate == 0 || bitrate > 1000) {
					sprintf(errorString, "%d is not valid bitrate, must be 1..1000 kb/s", bitrate);
					handleError(errorString);
					goto loopStart;
				}
				CanCustomInit();
				sprintf(outString, "Bitrate set to %d kB/s", bitrate);
				printLog(outString);
				break;
			case 'T':
				usbBuf[0]=0;
				if ((usbBuf[1] - '0') > 8) {
					handleError("Message length can't be more than 8!");
					goto loopStart;
					break;
				}
				if ((usbBuf[2] - '0') > 9) {
					handleError("IDE symbol must be a digit!");
					goto loopStart;
					break;
				}
				if ((usbBuf[3] - '0') > 9) {
					handleError("RTR symbol must be a digit!");
					goto loopStart;
					break;
				}
				uint8_t txData[8] = { 0, };
				CAN_TxHeaderTypeDef header = { 0, };
				header.DLC = usbBuf[1] - '0';
				header.IDE = (usbBuf[2] == '0') ? 0 : 4;
				header.RTR = (usbBuf[3] == '0') ? 0 : 2;
				if (header.IDE) {
					header.ExtId = HexToInt(usbBuf + 4, 8);
					if (error) goto loopStart;
					if (header.ExtId > 0x1FFFFFFF) {
						sprintf(errorString, "Extended ID must be lesser than 0x1FFFFFFF (it's %lx)", header.ExtId);
						handleError(errorString);
						goto loopStart;
						break;
					}
				} else {
					header.StdId = HexToInt(usbBuf + 4, 3);
					if (error) goto loopStart;
					if (header.StdId > 0x7FF) {
						sprintf(errorString, "Standard ID must be lesser than 0x7FF (it's %lx)", header.StdId);
						handleError(errorString);
						goto loopStart;
						break;
					}
				}

				header.TransmitGlobalTime = 0;
				for (uint8_t i = 0; i < header.DLC; ++i) {
					txData[i] = HexToInt(usbBuf + 7 + (header.IDE != 0) * 5 + 2 * i, 2);
					if (error) goto loopStart;
				}
				if (HAL_CAN_AddTxMessage(&hcan, &header, txData, &mailBox) != HAL_OK) {
					handleError("Can't send CAN message...");
					goto loopStart;
				}
				totalTXCnt++;
				lastTx = HAL_GetTick();
				break;

			case 't':
				usbBuf[0]=0;
				header.DLC = 8;
				header.IDE = 4;
				header.RTR = 0;
				header.TransmitGlobalTime = 0;
				header.ExtId = HexToInt(usbBuf + 1, 8);
				if (error) goto loopStart;
				for (uint8_t i = 0; i < header.DLC; ++i) {
					txData[i] = HexToInt(usbBuf + 9 + 2 * i, 2);
					if (error) goto loopStart;
				}
				HAL_CAN_AddTxMessage(&hcan, &header, txData, &mailBox);
				totalTXCnt++;
				lastTx = HAL_GetTick();
				break;
			case 'V':
				usbBuf[0]=0;
				sprintf(outString, "<V%08x>\r\n", VERSION);
				CDC_Transmit_FS((uint8_t*) outString, (strlen(outString)));
				break;
			case 'N':
				usbBuf[0]=0;
				canMode = CAN_MODE_NORMAL;
				CanCustomInit();
				printLog("Can mode set to NORMAL");
				break;

			case 'S':
				usbBuf[0]=0;
				canMode = CAN_MODE_SILENT;
				CanCustomInit();
				printLog("Can mode set to SILENT");
				break;

			case 'L':
				usbBuf[0]=0;
				canMode = CAN_MODE_LOOPBACK;
				CanCustomInit();
				printLog("Can mode set to LOOPBACK");
				break;

			case 'K':
				usbBuf[0]=0;
				canMode = CAN_MODE_SILENT_LOOPBACK;
				CanCustomInit();
				printLog("Can mode set to SILENT LOOPBACK");
				break;

			default:
				printLog("Command not supported");
				break;
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 12;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, TX_LED_Pin | RX_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : TX_LED_Pin RX_LED_Pin */
	GPIO_InitStruct.Pin = TX_LED_Pin | RX_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */

	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

