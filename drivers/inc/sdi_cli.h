#ifndef SDI_CLI_H
#define SDI_CLI_H

#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_uart.h"


#define SDI_PARAM_MAX_CMD_LEN 20

#define SDI_TX() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
#define SDI_RX() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

//choose electrical configuration to write data
#define SELECT_RS485_LINE() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define SELECT_SDI_LINE() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)

#define SDI_CRC_POLY 0xA001

typedef enum{
    COMMAND_QUERY = '?',
    COMMAND_ACTION = 'a'
}main_command_type;

typedef enum{
    SENSOR_IMU,
    SENSOR_FLASH,
    SENSOR_RADAR,
    SENSOR_CHARGER,
    SENSOR_RTC,
    SENSOR_UNKNOWN
}SENSOR_ID;

// the action commands include measurement, additionl measurement, concurrent measurement, additional concurrent measurement crc, send data continuously or on request,
// continuous measurement
typedef enum{
    COMMAND_CHANGE_ADDRESS = 'A',
    COMMAND_START_MEASUREMENT = 'M',
    //COMMAND_ADDITIONAL_MEASUREMENT = 0x01,
    COMMAND_CONCURRENT_MEASUREMENT = 'C',
    COMMAND_CONTINUOUS_MEASUREMENT = 'R',
    COMMAND_SEND_DATA = 'D',
    COMMAND_VERIFICATION = 'V',
    COMMAND_CRC = 'C',
    COMMAND_SEND_IDENTIFICATION = 'I',

    COMMAND_ACTION_TAG_0 = '0',
    COMMAND_ACTION_TAG_1 = '1',
    COMMAND_ACTION_TAG_2 = '2',
    COMMAND_ACTION_TAG_3 = '3',
    COMMAND_ACTION_TAG_4 = '4',
    COMMAND_ACTION_TAG_5 = '5',
    COMMAND_ACTION_TAG_6 = '6',
    COMMAND_ACTION_TAG_7 = '7',
    COMMAND_ACTION_TAG_8 = '8',
    COMMAND_ACTION_TAG_9 = '9'
}sub_command_type;

typedef struct{
    main_command_type c1;          //[address][c2][c3][c4]
    sub_command_type address;
    sub_command_type c2;
    sub_command_type c3;
    sub_command_type c4;
}full_command;

uint8_t sdi_cmd_receive(char *);
void sdi_main(void);

void api_status(SENSOR_ID, uint8_t, float*, char*);
uint16_t sdi_crc16(char*, uint16_t);
uint32_t sdi_crc_ascii(uint16_t);

#endif /*SDI_CLI_H*/