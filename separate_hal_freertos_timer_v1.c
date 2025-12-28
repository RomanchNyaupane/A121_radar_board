/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "rt_heap.h"
#include "example_bring_up.h"
#include "example_detector_distance.h"
#include "core_cm4.h"
#include "example_diagnostic_test.h"
#include "stdio.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_tim.h"
#include "stm32wbxx_hal_tim_ex.h"
#include "stm32wbxx_it.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t connected;
/* USER CODE END 0 */
void A121_Task(void *argument)
{
    //acc_example_diagnostic_test(0, NULL);
    while (1)
    {
				acc_example_detector_distance(0, NULL);
			  uint8_t status_bringup = acc_example_bring_up(0, NULL);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void led_task(){
	RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC -> MODER = 0x00000E00;
	while(1){
		GPIOC -> ODR ^= 1U << 4;
		vTaskDelay(1000);
	}
}



int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();

	  xTaskCreate(led_task, "LED", 128, NULL, 1, NULL);
    xTaskCreate(A121_Task, "A121", 0x1100, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) {}
}

//overriding the default systick hardware to use tim2. freertos and hal require separate timers
TIM_HandleTypeDef htim2;

HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz timer clock
    htim2.Init.Period = 1000 - 1; // 1 kHz interrupt = 1 ms HAL tick
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return HAL_OK;
}
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uwTick++;
    }
}




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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






































// Copyright (c) Acconeer AB, 2022-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_rss_a121.h"
#include "acc_version.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_uart.h"


/** \example example_bring_up.c
 * @brief This is an example on how the assembly test can be used to ease bring-up
 * @n
 * The example executes as follows:
 *   - Register the RSS HAL
 *   - Create assembly test
 *   - Enable one assembly test at a time
 *     - Power on the sensor
 *     - Enable the sensor
 *     - Exceute assembly test
 *     - Disable the sensor
 *     - Power off the sensor
 *   - Destroy assembly test
 */

#define SENSOR_ID (1U)

#define SENSOR_TIMEOUT_MS (1000U)

UART_HandleTypeDef huart1;

static bool run_test(acc_rss_assembly_test_t *assembly_test, acc_sensor_id_t sensor_id);

int acc_example_bring_up(int argc, char *argv[]);






int acc_example_bring_up(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	bool test_ok = true;

	//printf("Acconeer software version %s\n", acc_version_get());
	// Replace: printf("Acconeer software version %s\n", acc_version_get());
	char version_msg[100];  // Adjust size as needed
	const char* version = acc_version_get();
	int msg_len = snprintf(version_msg, sizeof(version_msg), "Acconeer software version %s\n", version);
	if (msg_len > 0) {
    HAL_UART_Transmit(&huart1, (uint8_t*)version_msg, msg_len, HAL_MAX_DELAY);
	}

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	void *buffer = acc_integration_mem_alloc(ACC_RSS_ASSEMBLY_TEST_MIN_BUFFER_SIZE);
	if (buffer == NULL)
	{
		//printf("Assembly test: Memory allocation failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Assembly test: Memory allocation failed\n", strlen("Assembly test: Memory allocation failed\n"), HAL_MAX_DELAY);
		return EXIT_FAILURE;
	}

	// Create
	acc_rss_assembly_test_t *assembly_test = acc_rss_assembly_test_create(SENSOR_ID, buffer, ACC_RSS_ASSEMBLY_TEST_MIN_BUFFER_SIZE);
	if (assembly_test == NULL)
	{
		//printf("Bring-up: Could not create assembly test\n");

		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Could not create assembly test\n", strlen("Bring-up: Could not create assembly test\n"), HAL_MAX_DELAY);
		acc_integration_mem_free(buffer);
		return EXIT_FAILURE;
	}

	// Enable and run: Basic read test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_BASIC_READ);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Basic read test failed\n");
		
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Basic read test failed\n", strlen("Bring-up: Basic read test failed\n"), HAL_MAX_DELAY);
		test_ok = false;
	}

	// Enable and run: Communication test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_COMMUNICATION);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Communication test failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Communication test failed\n", strlen("Bring-up: Communication test failed\n"), HAL_MAX_DELAY);		test_ok = false;
	}

	// Enable and run: Enable test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_ENABLE_PIN);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Enable test failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Enable test failed\n", 
    strlen("Bring-up: Enable test failed\n"), HAL_MAX_DELAY);
		test_ok = false;
	}

	// Enable and run: Interrupt test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_INTERRUPT);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Interrupt test failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Interrupt test failed\n", strlen("Bring-up: Interrupt test failed\n"), HAL_MAX_DELAY);
		test_ok = false;
	}

	// Enable and run: Clock and Supply test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_CLOCK_AND_SUPPLY);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Clock and Supply test failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Clock and Supply test failed\n", strlen("Bring-up: Clock and Supply test failed\n"), HAL_MAX_DELAY);
		test_ok = false;
	}

	// Enable and run: Sensor calibration test
	acc_rss_assembly_test_disable_all_tests(assembly_test);
	acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_SENSOR_CALIBRATION);
	if (!run_test(assembly_test, SENSOR_ID))
	{
		//printf("Bring-up: Calibration test failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: Calibration test failed\n", strlen("Bring-up: Calibration test failed\n"), HAL_MAX_DELAY);
		test_ok = false;
	}

	// Destroy
	acc_rss_assembly_test_destroy(assembly_test);

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}

	if (test_ok)
	{
		//printf("Bring-up: All tests passed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Bring-up: All tests passed\n", 
    strlen("Bring-up: All tests passed\n"), HAL_MAX_DELAY);

		//printf("Application finished OK\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Application finished OK\n", strlen("Application finished OK\n"), HAL_MAX_DELAY);
		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
}

static bool run_test(acc_rss_assembly_test_t *assembly_test, acc_sensor_id_t sensor_id)
{
	bool all_passed = true;

	acc_hal_integration_sensor_supply_on(sensor_id);
	acc_hal_integration_sensor_enable(sensor_id);

	acc_rss_test_state_t              assembly_test_state = ACC_RSS_TEST_STATE_ONGOING;
	acc_rss_test_integration_status_t integration_status  = ACC_RSS_TEST_INTEGRATION_STATUS_OK;

	do
	{
		assembly_test_state = acc_rss_assembly_test_execute(assembly_test, integration_status);

		switch (assembly_test_state)
		{
			case ACC_RSS_TEST_STATE_TOGGLE_ENABLE_PIN:
				acc_hal_integration_sensor_disable(sensor_id);
				acc_hal_integration_sensor_enable(sensor_id);
				integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
				break;
			case ACC_RSS_TEST_STATE_WAIT_FOR_INTERRUPT:
				if (!acc_hal_integration_wait_for_sensor_interrupt(sensor_id, SENSOR_TIMEOUT_MS))
				{
					/* Wait for interrupt failed */
					integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_TIMEOUT;
				}
				else
				{
					integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
				}

				break;
			default:
				integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
				break;
		}
	} while (assembly_test_state != ACC_RSS_TEST_STATE_COMPLETE);

	acc_hal_integration_sensor_disable(sensor_id);
	acc_hal_integration_sensor_supply_off(sensor_id);

	uint16_t nbr_of_test_results = 0U;

	const acc_rss_assembly_test_result_t *test_results = acc_rss_assembly_test_get_results(assembly_test, &nbr_of_test_results);

	for (uint16_t idx = 0; idx < nbr_of_test_results; idx++)
	{
		char result_msg[100];
		int msg_len = snprintf(result_msg, sizeof(result_msg), 
													"Bring-up: '%s' [%s]\n", 
													test_results[idx].test_name, 
													test_results[idx].test_result ? "PASS" : "FAIL");
		if (msg_len > 0 && msg_len < sizeof(result_msg)) {
				HAL_UART_Transmit(&huart1, (uint8_t*)result_msg, msg_len, HAL_MAX_DELAY);
		}
		if (!test_results[idx].test_result)
		{
			all_passed = false;
		}
	}

	return all_passed;
}
