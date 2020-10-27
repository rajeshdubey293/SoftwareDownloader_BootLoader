
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BL_DEBUG_MSG_EN
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define C_UART 	&huart3
#define D_UART	&huart1


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void printmsg(char *format,...);
uint32_t crcCalculation(uint32_t crcArray[], uint8_t lenOfArray);
uint8_t word_to_bytes(uint8_t *dest, uint32_t *src);
uint32_t bytes_to_word(uint32_t *dest, uint8_t *src);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint32_t calculateAuthenticationKEY(uint32_t key);
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host);
void userApplication(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t pBuffer = (uint32_t)0x08040000;
#define BL_RX_LEN  10000
uint8_t dataArray[BL_RX_LEN];
uint32_t uartDataRx[3] = {0};
uint32_t uartDataTx[3] = {0};
uint32_t payloadLen = 0;
uint32_t key = 0xFFCCDDCC;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
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
	MX_CRC_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	memset(dataArray,0,10000);
	printmsg("Press User Button within 3 Seconds to boot into BootLoader Mode\r\n");
	//HAL_Delay(3000);
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
	{
		printmsg("BootLoader Mode\r\n");
		uint32_t downloaderKey = calculateAuthenticationKEY(key);
		uartDataTx[0] = key;
		printmsg("Waiting for Tester Connection\r\n");
		uint32_t temp = 0;
		HAL_UART_Receive(&huart3, &temp, 4, HAL_MAX_DELAY);
		printmsg("Sending Authentication Key to Tester\r\n");
		HAL_Delay(500);
		HAL_UART_Transmit(&huart3, &uartDataTx[0], 4, HAL_MAX_DELAY);
		printmsg("Waiting for the Tester Key\r\n");
		HAL_UART_Receive(&huart3, &uartDataRx[0], 4, HAL_MAX_DELAY);
		if(uartDataRx[0] == downloaderKey)
		{
			printmsg("Tester is verified successfully\r\n");
			uartDataTx[1] = 0xFFFFFFFF;
			//HAL_Delay(1000);
			HAL_UART_Transmit(&huart3, &uartDataTx[1], 4, HAL_MAX_DELAY);
			printmsg("Verification msg sent\r\n");
			HAL_UART_Receive(&huart3, &uartDataRx[1], 8, HAL_MAX_DELAY);
			uint32_t payloadCRC = uartDataRx[1];
			payloadLen = uartDataRx[2];
			printmsg("CRC value and Payload Length received\r\n");
			printmsg("Waiting for the payload\r\n");
			HAL_UART_Receive(&huart3, dataArray, payloadLen, HAL_MAX_DELAY);
			//uint32_t crc_Value = HAL_CRC_Accumulate(&hcrc, &dataArray, payloadLen);
			if(!(bootloader_verify_crc(&dataArray, payloadLen, payloadCRC)))
			{
				printmsg("CRC is matched\r\n");
				printmsg("Data is being written\r\n");
				uint8_t statusWrite = 0;
				uint8_t statusErase = 0;
				statusErase = execute_flash_erase(6, 1);
				statusWrite = execute_mem_write(dataArray, pBuffer, payloadLen);
				if(statusWrite == HAL_OK)
				{
					printmsg("data is written successfully\r\n");
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
					printmsg("Booting into User Application\r\n");
					//HAL_Delay(500);
					userApplication();
				}
				else
				{
					printmsg("Data writing is failed\r\n");
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
					printmsg("Data is Erased, Please Go to Bootloader Mode and Download the firmware\r\n");
					//HAL_Delay(500);
				}
			}
			else
			{
				printmsg("CRC is mis-matched\r\n");
				//HAL_Delay(500);
				printmsg("Booting into User Application\r\n");
				userApplication();
			}

		}
		else
		{
			printmsg("Unknown Tester Found\r\n");
			//HAL_Delay(500);
			printmsg("Booting into User Application\r\n");
			userApplication();

		}
	}
	else
	{
		printmsg("Booting into User Application\r\n");
		//HAL_Delay(500);
		userApplication();
	}
}

/* USER CODE END 3 */


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PG13 PG14 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* prints formatted string to console over UART */
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(D_UART,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}
/* CRC Calculation using Software */
uint32_t crcCalculation(uint32_t crcArray[], uint8_t lenOfArray)
{
	uint32_t crc = 0xFF;
	uint8_t bindex = 0;
	uint8_t lenOfData = 0;
	for(;lenOfData < lenOfArray; lenOfData++)
	{
		crc = crc ^ crcArray[lenOfData];
		for(;bindex < (sizeof(crcArray[lenOfData]) * 8); bindex++)
		{
			if(crc & 0x80000000)
			{
				crc = (crc << 1) ^ 0x04C11DB7;
			}
			else
			{
				crc = crc << 1;
			}
		}
		bindex = 0;
	}
	return crc;
}
uint8_t word_to_bytes(uint8_t *dest, uint32_t *src)
{
	memcpy(&dest, src, sizeof(dest));

}
uint32_t bytes_to_word(uint32_t *dest, uint8_t *src)
{
	memcpy(dest, &src, sizeof(dest));
}
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
	uint8_t status = HAL_OK;

	//We have to unlock flash module to get control of registers
	HAL_FLASH_Unlock();

	for(uint32_t i = 0 ; i <len ; i++)
	{
		//Here we program the flash byte by byte
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
	}
	HAL_FLASH_Lock();

	return status;
}

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
	//we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 12 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 11) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
			/*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 12 - sector_number;
			if( number_of_sector > remanining_sector)
			{
				number_of_sector = remanining_sector;
			}
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}


	return INVALID_SECTOR;
}
uint32_t calculateAuthenticationKEY(uint32_t key)
{
	uint32_t returnValue = key ^ 0xAABBCCDD;
	return returnValue;

}
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xFF;

	for (uint32_t i=0 ; i < len ; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	/* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}
void userApplication(void)
{
	void (*app_reset_handler)(void);

	//printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\n");


	// 1. configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR6_BASE_ADDRESS;
	//printmsg("BL_DEBUG_MSG:MSP value : %#x\n",msp_value);

	//This function comes from CMSIS.
	__set_MSP(msp_value);

	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR6_BASE_ADDRESS + 4);

	app_reset_handler = (void*) resethandler_address;

	//printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n",app_reset_handler);

	//3. jump to reset handler of the user application
	app_reset_handler();

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
