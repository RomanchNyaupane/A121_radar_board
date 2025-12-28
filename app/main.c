#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rt_heap.h"
//#include "example_bring_up.h"
#include "example_detector_distance.h"
#include "core_cm4.h"
//#include "example_diagnostic_test.h"
#include "imu_driver.h"
#include "charger_driver.h"
#include "spi_flash.h"
#include "stm32wbxx_hal.h"
#include "lsm6dsv16x_reg.h"
#include "math.h"

#define IMU_I2C_ADDR_WR 0xD5
#define IMU_I2C_ADDR_RD 0xD7
#define IMU_BOOT_TIME 0x0A //10ms
#define FIFO_WATERMARK 32

#ifndef M_PI
#define M_PI 3.1415f
#endif

uint8_t whoamI;
int fputc(int ch, FILE *f)
{
    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)
    {
        while (ITM->PORT[0].u32 == 0);
        ITM->PORT[0].u8 = (uint8_t)ch;
    }
    return ch;
}

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

imu_config_t himu;

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef time_hrtc;
RTC_DateTypeDef date_hrtc;

charger_status_val_t charger_status_val;
charger_flag_val_t charger_flag_val;

stmdev_ctx_t imu_ctx;

static lsm6dsv16x_sflp_gbias_t gbias;
static lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
static lsm6dsv16x_fifo_status_t fifo_status;
static lsm6dsv16x_fifo_out_raw_t f_data;
static lsm6dsv16x_pin_int_route_t pin_int;
uint8_t game_rotation[7];

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void RTC_Init(void);
static void imu_config();
static inline void _enable_debugger ( void );

void print_uart(char *, uint8_t , uint8_t);
uint8_t int_to_char(uint32_t , uint8_t *);
void platform_init(void);
int32_t platform_write(void *, uint8_t , const uint8_t *, uint16_t);
int32_t platform_read(void *, uint8_t, uint8_t *, uint16_t);
void platform_delay(uint32_t);

volatile uint8_t connected;
volatile uint8_t data_ready_flags;
volatile uint8_t charge_exti_flag;

void A121_Task()
{
    while (1)
    {
			acc_example_detector_distance(0, NULL);
      //vTaskDelay(100);
    }
}
void flash_task(){
	flash_spi_init(&hspi2);
	while(1){
		print_uart("data from address 0x00000001: ", sizeof("data from address 0x00000001: "), flash_read_byte(0x00000001));
		print_uart("data from address 0x00000002: ", sizeof("data from address 0x00000002: "), flash_read_byte(0x00000002));
		print_uart("data from address 0x00000003: ", sizeof("data from address 0x00000003: "), flash_read_byte(0x00000003));
		print_uart("data from address 0x00000004: ", sizeof("data from address 0x00000004: "), flash_read_byte(0x00000004));
		vTaskDelay(10);
	}
}
void charger_task(){
	charge_config();
	set_adc();

	
	while(1){
		if(charge_exti_flag){
			HAL_UART_Transmit(&huart1, (uint8_t*)"charger interrupt \n", sizeof("charger interrupt \n"), HAL_MAX_DELAY);
			charge_exti_flag = 0;
			get_flag_status(&charger_flag_val);
		}
			set_adc();
			get_adc();
			vTaskDelay(10);
	}
}
void imu_task(){
		//imu_start();
		float qw, qi, qj, qk;
    float roll, pitch, yaw;
    uint32_t tmp;
    platform_init();
    vTaskDelay(IMU_BOOT_TIME);
	
		lsm6dsv16x_device_id_get(&imu_ctx, &whoamI);
		if (whoamI != LSM6DSV16X_ID)
		while (1);
    lsm6dsv16x_sw_por(&imu_ctx);
	
	  lsm6dsv16x_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_xl_full_scale_set(&imu_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&imu_ctx, LSM6DSV16X_2000dps);
    lsm6dsv16x_fifo_watermark_set(&imu_ctx, FIFO_WATERMARK);

      /* Set FIFO batch of sflp data */
    fifo_sflp.game_rotation = 1;
    fifo_sflp.gravity = 1;
    fifo_sflp.gbias = 1;
    lsm6dsv16x_fifo_sflp_batch_set(&imu_ctx, fifo_sflp);
    lsm6dsv16x_fifo_mode_set(&imu_ctx, LSM6DSV16X_STREAM_MODE);

    //interrupt
    pin_int.fifo_th = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int1_route_set(&imu_ctx, &pin_int);
    /* Set Output Data Rate */
    lsm6dsv16x_xl_data_rate_set(&imu_ctx, LSM6DSV16X_ODR_AT_30Hz);
    lsm6dsv16x_gy_data_rate_set(&imu_ctx, LSM6DSV16X_ODR_AT_30Hz);
    lsm6dsv16x_sflp_data_rate_set(&imu_ctx, LSM6DSV16X_SFLP_30Hz);

    lsm6dsv16x_sflp_game_rotation_set(&imu_ctx, PROPERTY_ENABLE);

    /*
      * here application may initialize offset with latest values
      * calculated from previous run and saved to non volatile memory.
      */
    gbias.gbias_x = 0.0f;
    gbias.gbias_y = 0.0f;
    gbias.gbias_z = 0.0f;
    lsm6dsv16x_sflp_game_gbias_set(&imu_ctx, &gbias);


		while(1){
      if (data_ready_flags == 0x01) {  //accl has data
        uint8_t num = 0;
        lsm6dsv16x_fifo_status_get(&imu_ctx, &fifo_status);

        if (fifo_status.fifo_th == 1) {
          num = fifo_status.fifo_level;

        while (num--) {
          uint8_t *axis;
          float_t quat[4];
          float_t gravity_mg[3];
          float_t gbias_mdps[3];

          /* Read FIFO sensor value */
          lsm6dsv16x_fifo_out_raw_get(&imu_ctx, &f_data);

          //the function reads 7 fifo location starting with tag and 6 data lccations
          /*
          The format for the SFLP-generated sensors in FIFO is listed below:
            • Game rotation vector: X, Y, and Z axes (vector part of the quaternion) are stored in half-precision floating
              point format, where w (scalar part of the quaternion) is computed in software after reading the data from 
              the FIFO, since the game rotation vector is a unit quaternion.
            • Gravity vector: X, Y, and Z axes are stored as 16-bit two's complement number with ±2 g sensitivity.
            • Gyroscope bias: X, Y, and Z axes are stored as 16-bit two's complement number with ±125 dps sensitivity.
          */
         //we should identify data based on tag 
         /* Tag  | data
            0x13 | SFLP game rotation vector
            0x16 | SFLP gyroscope bias
            0x17 | SFLP gravity vector
          */
         /*
          the game rotation vector is half_precision format(16 bit). each fifo read is 7 byte(1 tag, 6 data).
          so it is axiomatic that x,y and z are obtained by consecutive two bytes of the read data array.
         */
         //the first element is always fifo tag
         if(f_data.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG){
           game_rotation[0] = f_data.data[0];
           game_rotation[1] = f_data.data[1];
           game_rotation[2] = f_data.data[2];
           game_rotation[3] = f_data.data[3];
           game_rotation[4] = f_data.data[4];
           game_rotation[5] = f_data.data[5];
           game_rotation[6] = f_data.tag;
         }
         //since the numbers are half precision format, we need to convert to single precision(64 bit)
        tmp = lsm6dsv16x_from_f16_to_f32(
                      (uint16_t)(game_rotation[0] | (game_rotation[1] << 8)));
        memcpy(&qi, &tmp, sizeof(float));

        tmp = lsm6dsv16x_from_f16_to_f32(
                      (uint16_t)(game_rotation[2] | (game_rotation[3] << 8)));
        memcpy(&qj, &tmp, sizeof(float));

        tmp = lsm6dsv16x_from_f16_to_f32(
                      (uint16_t)(game_rotation[4] | (game_rotation[5] << 8)));
        memcpy(&qk, &tmp, sizeof(float));

        float t = 1.0f - (qi*qi + qj*qj + qk*qk);
        if (t > 0.0f)
            qw = sqrtf(t);
        else
            qw = 0.0f;


        /* Roll (X-axis) */
        roll = atan2f(2.0f * (qw*qi + qj*qk),1.0f - 2.0f * (qi*qi + qj*qj));

        /* Pitch (Y-axis) */
        float sinp = 2.0f * (qw*qj - qk*qi);
        if (fabsf(sinp) >= 1.0f)
            pitch = copysignf(M_PI / 2.0f, sinp);
        else
            pitch = asinf(sinp);

        /* Yaw (Z-axis) */
        yaw = atan2f(2.0f * (qw*qk + qi*qj),1.0f - 2.0f * (qj*qj + qk*qk));
        }
      }
				data_ready_flags = 0;
		}
    char uart_buf[64];
    int len;

    len = snprintf(uart_buf, sizeof(uart_buf), "roll: %d\r\n", (int)(roll * 180.0f / M_PI));
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);

    len = snprintf(uart_buf, sizeof(uart_buf), "pitch: %d\r\n", (int)(pitch * 180.0f / M_PI));
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);

    len = snprintf(uart_buf, sizeof(uart_buf), "yaw: %d\r\n", (int)(yaw * 180.0f / M_PI));
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
		vTaskDelay(10);
		}
}
void rtc_task(){
	while(1){
    HAL_RTC_GetTime(&hrtc, &time_hrtc, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &date_hrtc, RTC_FORMAT_BCD);
		print_uart("clock hour: ", sizeof("clock hour: "), time_hrtc.Hours);
		print_uart("clock minutes: ", sizeof("clock minutes: "), time_hrtc.Minutes);
		print_uart("clock seconds: ", sizeof("clock seconds: "), time_hrtc.Seconds);
		vTaskDelay(10);
	}
}
void led_task(){
	//RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	//GPIOC -> MODER = 0x00000E00;
	while(1){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
		vTaskDelay(10);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
		vTaskDelay(10);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		vTaskDelay(10);
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
	//_enable_debugger();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
	RTC_Init();
	//imu_config();

	printf("init complete");

	xTaskCreate(led_task, "LED", 500, NULL, 1, NULL);
  xTaskCreate(A121_Task, "A121", 0x2000, NULL, 1, NULL);
	xTaskCreate(flash_task, "flash", 0x200, NULL, 1, NULL);
	xTaskCreate(imu_task, "imu", 0x300, NULL, 1, NULL);
	xTaskCreate(rtc_task, "rtc", 0x100, NULL, 1, NULL);
	xTaskCreate(charger_task, "charger", 0x100, NULL, 1, NULL);

	vTaskStartScheduler();
  while (1)
  {

  }
}


int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len){
  HAL_I2C_Mem_Write(handle, IMU_I2C_ADDR_WR, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, HAL_MAX_DELAY);

}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
  HAL_I2C_Mem_Read(handle, IMU_I2C_ADDR_WR, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
}

void platform_delay(uint32_t millisec){
  HAL_Delay(millisec);
}

void platform_init(){
  imu_ctx.write_reg = platform_write;
  imu_ctx.read_reg = platform_read;
  imu_ctx.mdelay = platform_delay;
  imu_ctx.handle = &hi2c1;
}


void imu_config(void){
	//himu = IMU_CONFIG_DEFAULT;
	himu.accl_config.module_en = 1;
	himu.accl_config.interrupt_en = 1;
	
	himu.gyro_config.module_en = 1;
	himu.gyro_config.interrupt_en = 1;
	
	imu_init(&hi2c1, &huart1, &himu);
}
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin){
    if (gpio_pin == GPIO_PIN_4) data_ready_flags |= 0x01;  //accel
    if (gpio_pin == GPIO_PIN_5) data_ready_flags |= 0x02;  //gyro
    if (gpio_pin == GPIO_PIN_1) charge_exti_flag = 0x01;
}

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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


static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}


static void MX_USART1_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_12, GPIO_PIN_RESET);
	
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);


  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

static void RTC_Init(void){
RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x5;
  sTime.Minutes = 0x20;
  sTime.Seconds = 0x05;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}
void print_uart(char *text, uint8_t text_size, uint8_t data)
{
    uint8_t int_char[10];
    uint8_t char_size = int_to_char(data, int_char);
    HAL_UART_Transmit(&huart1, (uint8_t *)text, text_size, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, int_char, char_size, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
}

uint8_t int_to_char(uint32_t num, uint8_t *buf)
{
    uint8_t i = 0;

    if(num == 0){
      buf[i++] = 48;  //48 is ascii of '0' (zero)
    } else{
      while(num>0){
        uint8_t digit = num % 16;
        if(digit < 10){
            buf[i++] = digit + 48;
        } else{
            buf[i++] = digit + 55; //converting to A-F if number is greater than 10
        }
        num = num/16;
      }
    }
    buf[i] = 0; //0 is ascii for null terminator

    uint8_t temp;
    //reverse the array
    for(uint8_t j = 0, k = i - 1; j < k; j++, k--){ //k = i - 1 because we do not want to touch the null terminator
        temp = buf[k];
        buf[k] = buf[j];
        buf[j] = temp;
    }
    return i;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
}


static inline void _enable_debugger ( void )
{
    GPIO_InitTypeDef  s_dbg_cfg = 
    {
        .Pin               = GPIO_PIN_3,
        .Mode              = GPIO_MODE_AF_PP,
        .Pull              = GPIO_NOPULL,
        .Speed             = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate     		 = GPIO_AF0_JTD_TRACE
    };
  
  HAL_GPIO_Init(GPIOB, &s_dbg_cfg);
}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
