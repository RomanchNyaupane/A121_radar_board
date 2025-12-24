// #include "stm32wbxx_hal.h"
#include "imu_driver.h"

I2C_HandleTypeDef *g_p_i2c_handle;
UART_HandleTypeDef *g_p_uart_handle;
imu_config_t *g_p_imu_handle;

static uint16_t average[3];
static uint8_t rx_dummy;
typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
}data_t;

data_t accl_data;
data_t gyro_data;

void imu_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, imu_config_t *himu){
  g_p_i2c_handle = hi2c;
  g_p_uart_handle = huart;
	g_p_imu_handle = himu;
	uint8_t data;
	data = 0x08;
	  //enable measurement units
	if(g_p_imu_handle->accl_config.module_en){
		HAL_I2C_Mem_Write(g_p_i2c_handle, (uint8_t)IMU_I2C_ADDRESS, IMU_REG_CTRL_1, I2C_MEMADD_SIZE_8BIT, &data, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	}
	if(g_p_imu_handle->gyro_config.module_en){
		HAL_I2C_Mem_Write(g_p_i2c_handle, (uint8_t)IMU_I2C_ADDRESS, IMU_REG_CTRL_2, I2C_MEMADD_SIZE_8BIT, &data, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	}
	//enable interrupts
	data = 0x80;
	if((g_p_imu_handle ->accl_config.interrupt_en) || g_p_imu_handle ->gyro_config.interrupt_en){
		HAL_I2C_Mem_Write(g_p_i2c_handle,  (uint8_t)IMU_I2C_ADDRESS, IMU_REG_FUNCTIONS_EN, I2C_MEMADD_SIZE_8BIT, &data, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	}
	//interrupt configuration
	data = 0x01;
	if(g_p_imu_handle ->accl_config.interrupt_en){
		HAL_I2C_Mem_Write(g_p_i2c_handle, (uint8_t)IMU_I2C_ADDRESS, IMU_REG_ADDR_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, &data, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	}
	data = 0x02;
	if(g_p_imu_handle ->gyro_config.interrupt_en){
		HAL_I2C_Mem_Write(g_p_i2c_handle, (uint8_t)IMU_I2C_ADDRESS, IMU_REG_ADDR_INT2_CTRL, I2C_MEMADD_SIZE_8BIT, &data, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	}
}

void imu_start(void){
	HAL_I2C_Mem_Read(g_p_i2c_handle, IMU_I2C_ADDRESS, IMU_REG_ADDR_X_ACCL_H, I2C_MEMADD_SIZE_8BIT, &rx_dummy, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(g_p_i2c_handle, IMU_I2C_ADDRESS, IMU_REG_ADDR_X_GYRO_H, I2C_MEMADD_SIZE_8BIT, &rx_dummy, IMU_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
}

void accl_uart(void){
    char buffer[64];

    sprintf(buffer, "imu_accl_x: %d ", accl_data.x);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "imu_accl_y: %d ", accl_data.y);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "imu_accl_z: %d\n", accl_data.z);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void gyro_uart(void){
    char buffer[64];

    sprintf(buffer, "imu_gyro_x: %d ", gyro_data.x);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "imu_gyro_y: %d ", gyro_data.y);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "imu_gyro_z: %d\n", gyro_data.z);
    HAL_UART_Transmit(g_p_uart_handle, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void accl_measure(){
	//accl_data = (accl_data){0};
	int32_t received_byte[3] = {0,0,0};
	uint8_t temp_byte[6] = {0,0,0,0,0,0};
	uint8_t len = 0;
	while(len<IMU_PARAM_SAMPLE_LEN)
	{
		//read 6 bytes after first data register
		HAL_I2C_Mem_Read(g_p_i2c_handle, IMU_I2C_ADDRESS, IMU_REG_ADDR_X_ACCL_H, I2C_MEMADD_SIZE_8BIT, temp_byte, IMU_PARAM_LEN_2_BYTE * 3, HAL_MAX_DELAY);
		received_byte[0] = received_byte[0] + (((uint16_t)temp_byte[0] << IMU_PARAM_BYTE_SHIFT) | temp_byte[1]);
		received_byte[1] = received_byte[1] + (((uint16_t)temp_byte[2] << IMU_PARAM_BYTE_SHIFT) | temp_byte[3]);
		received_byte[2] = received_byte[2] + (((uint16_t)temp_byte[4] << IMU_PARAM_BYTE_SHIFT) | temp_byte[5]);
		len++;
	}
	accl_data.x = (int16_t)(received_byte[0] / IMU_PARAM_SAMPLE_LEN);
	accl_data.y = (int16_t)(received_byte[1] / IMU_PARAM_SAMPLE_LEN);
	accl_data.z = (int16_t)(received_byte[2] / IMU_PARAM_SAMPLE_LEN);
}

void gyro_measure(){
	//gyro_data = (gyro_data){0};
	int32_t received_byte[3] = {0,0,0};
	uint8_t temp_byte[6] = {0,0,0,0,0,0};
	uint8_t len = 0;
	while(len<IMU_PARAM_SAMPLE_LEN)
	{
		//read 6 bytes after first data register
		HAL_I2C_Mem_Read(g_p_i2c_handle, IMU_I2C_ADDRESS, IMU_REG_ADDR_X_GYRO_H, I2C_MEMADD_SIZE_8BIT, temp_byte, IMU_PARAM_LEN_2_BYTE * 3, HAL_MAX_DELAY);
		received_byte[0] = received_byte[0] + (int32_t)((int16_t)((uint16_t)temp_byte[0] << IMU_PARAM_BYTE_SHIFT) | temp_byte[1]);
		received_byte[1] = received_byte[1] + (int32_t)((int16_t)((uint16_t)temp_byte[2] << IMU_PARAM_BYTE_SHIFT) | temp_byte[3]);
		received_byte[2] = received_byte[2] + (int32_t)((int16_t)((uint16_t)temp_byte[4] << IMU_PARAM_BYTE_SHIFT) | temp_byte[5]);
		len++;
	}
	gyro_data.x = (int16_t)(received_byte[0] / IMU_PARAM_SAMPLE_LEN); //casting at my own risk
	gyro_data.y = (int16_t)(received_byte[1] / IMU_PARAM_SAMPLE_LEN);
	gyro_data.z = (int16_t)(received_byte[2] / IMU_PARAM_SAMPLE_LEN);
}
