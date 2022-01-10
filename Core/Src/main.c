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

#define BAD_FORMAT 1
#define TOO_LONG 2
#define TOO_BIG_EXT_ID 3
#define TOO_BIG_STD_ID 4

typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t buf[8];
} rxMessage_t;

rxMessage_t messages[16];
uint8_t usbBuf[256];
uint8_t usbPtr = 0;
uint8_t gotMessage = 0;

uint8_t ptr = 0;
uint32_t lastRx = 0;
CAN_FilterTypeDef filter;
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t DecToInt(uint8_t *string, uint8_t len) {
	uint32_t value = 0;
	uint8_t i = 0;
	for (i = 0; i < len; i++)
		if (string[i] >= '0' && string[i] <= '9')
			value += (string[i] - '0') * pow10_(len - i - 1);
		else {
			Error_Handler();
			return 0xFFFFFFFF;
		}
	return value;
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
			Error_Handler();
			return 0xFFFFFFFF;
		}
		value += tmp * pow16(len - i - 1);
	}
	return value;
}

uint32_t HexToByte(uint8_t *string) {
	uint32_t value = 0;
	uint32_t tmp = 0;
	uint8_t i = 0;

	while (string[i] != 0 && i < 2) {
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
			Error_Handler();
			return 0xFFFFFFFF;
		}
		value = (value << 4) | tmp;
		i++;
	}
	return value;
}

void reportError(uint8_t errorCode) {
	uint8_t errorBuf[2] = { 'E', errorCode };
	CDC_Transmit_FS(errorBuf, 2);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &messages[ptr].header, messages[ptr].buf) != HAL_OK)
		Error_Handler();

	ptr++;
	if (ptr > 15)
		ptr = 0;
	lastRx = HAL_GetTick();
}

void CDC_ReceiveCallback(uint8_t *Buf, uint32_t len) {
	if (*Buf == '<')
		usbPtr = 0;
	else if (*Buf == '>') {
		usbBuf[usbPtr] = 0;
		gotMessage = 1;
	} else {
		memcpy(usbBuf + usbPtr, Buf, len);
		usbPtr += len;
	}

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
	/* USER CODE BEGIN 2 */

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

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		Error_Handler();

	if (HAL_CAN_Start(&hcan) != HAL_OK)
		Error_Handler();

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR) != HAL_OK)
		Error_Handler();

	while (1) {
		uint8_t len;

		if (ptr > 0) {
			CDC_Transmit_FS(messages[ptr - 1].buf, messages[ptr - 1].header.DLC);
			ptr--;
		}

		if (HAL_GetTick() - lastRx < 20)
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
		else
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);

		if (gotMessage) {
			gotMessage = 0;

			switch (usbBuf[0]) {
			case '1':
				if (HAL_CAN_Start(&hcan) != HAL_OK)
					Error_Handler();
				if (HAL_CAN_ActivateNotification(&hcan,
				CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
					Error_Handler();
				if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR) != HAL_OK)
					Error_Handler();
				break;
			case '2':
				if (HAL_CAN_Stop(&hcan) != HAL_OK)
					Error_Handler();
			case '3':
				if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
					Error_Handler();
				break;
			case '4':
				if (usbBuf[1] == 0)
					filter.FilterMode = CAN_FILTERMODE_IDMASK;
				else if (usbBuf[1] == 1)
					filter.FilterMode = CAN_FILTERMODE_IDLIST;
				else
					reportError(BAD_FORMAT);
				break;
			case '5':
				filter.FilterMaskIdLow = HexToInt(usbBuf + 1);
				break;
			case '6':
				filter.FilterMaskIdHigh = HexToInt(usbBuf + 1);
				break;
			case '7':
				filter.FilterIdLow = HexToInt(usbBuf + 1);
				break;
			case '8':
				filter.FilterIdHigh = HexToInt(usbBuf + 1);
				break;
			case '9':
				uint16_t bitrate = DecToInt(usbBuf + 1, 4);
				break;
			case 'T':

				len = usbBuf[1];
				if (len > 8) {
					reportError(TOO_LONG);
					break;
				}
				uint8_t txData[8] = { 0, };
				CAN_TxHeaderTypeDef header = { 0, };
				header.DLC = DecToInt(usbBuf + 1, 1);
				header.IDE = DecToInt(usbBuf + 2, 1);
				header.RTR = DecToInt(usbBuf + 3, 1);
				if (header.IDE) {
					header.ExtId = HexToInt(usbBuf + 4, 8);
					if (header.ExtId > 0x1FFFFFFF) {
						reportError(TOO_BIG_EXT_ID);
						break;
					}
				} else {
					header.StdId = HexToInt(usbBuf + 4, 3);
					if (header.StdId > 0x7FF) {
						reportError(TOO_BIG_STD_ID);
						break;
					}
				}

				header.TransmitGlobalTime = 0;
				for (uint8_t i = 0; i < header.DLC; ++i) {
					txData[i] = HexToInt(usbBuf + 6 + header.IDE * 5 + 2 * i, 2);
				}
				HAL_CAN_AddTxMessage(&hcan, &header, txData, CAN_TX_MAILBOX0);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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

