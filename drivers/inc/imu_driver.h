#ifndef IMU_DRIVER_LIB_H
#define IMU_DRIVER_LIB_H
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_i2c.h"
#include "stm32wbxx_hal_uart.h"

#define IMU_I2C_ADDRESS 0xD5

#define IMU_REG_ADDR_WHO_AM_I 0x0F
#define IMU_REG_STATUS 0x1E
#define IMU_REG_CTRL_1 0x10 // contains accl_en bit
#define IMU_REG_CTRL_2 0x11

#define IMU_REG_ADDR_X_ACCL_H 0x29
#define IMU_REG_ADDR_X_ACCL_L 0x28
#define IMU_REG_ADDR_Y_ACCL_H 0x2B
#define IMU_REG_ADDR_Y_ACCL_L 0x2A
#define IMU_REG_ADDR_Z_ACCL_H 0x2D
#define IMU_REG_ADDR_Z_ACCL_L 0x2C

#define IMU_REG_ADDR_X_GYRO_H 0x23 // pitch
#define IMU_REG_ADDR_X_GYRO_L 0x23
#define IMU_REG_ADDR_Y_GYRO_H 0x25 // roll
#define IMU_REG_ADDR_Y_GYRO_L 0x24
#define IMU_REG_ADDR_Z_GYRO_H 0x27 // yaw
#define IMU_REG_ADDR_Z_GYRO_L 0x26

#define IMU_REG_FUNCTIONS_EN 0x50 // contains interrupt enable bit

#define IMU_REG_ADDR_INT1_CTRL 0x0D // contains bit for enabline accl interrupt over int1 line
#define IMU_REG_ADDR_INT2_CTRL 0x0E ///< contains bit for enabling gyro interrupt over int2 line

#define IMU_PARAM_SLIDING_WINDOW_LEN 50U
#define IMU_PARAM_SAMPLE_LEN 5U
#define IMU_PARAM_BYTE_SHIFT 8U
#define IMU_PARAM_LEN_1_BYTE 1U
#define IMU_PARAM_LEN_2_BYTE 2U


#define IMU_CONFIG_DEFAULT    \
	{                           \
		.gyro_config{              \
			.module_en = 0,    	  	\
			.interrupt_en = 0,      \
		},                        \
		.accl_config{              \
			.module_en = 0,         \
			.interrupt_en = 0,      \
		}                        \
	}

typedef struct
{
	struct
	{
		uint8_t module_en;
		uint8_t interrupt_en;
	} gyro_config;

	struct
	{
		uint8_t module_en;
		uint8_t interrupt_en;
	} accl_config;

} imu_config_t;

void imu_init(I2C_HandleTypeDef *, UART_HandleTypeDef *, imu_config_t *);
void imu_start(void);
void accl_uart(void);
void gyro_uart(void);
void accl_measure(void);
void gyro_measure(void);
#endif