#ifndef SPI_FLASH_LIB_H
#define SPI_FLASH_LIB_H
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_spi.h"

#define FLASH_TRUE 00
#define FLASH_FALSE 01

#define FLASH_PARAM_NUM_BLOCK 256U //no. of blocks. there are 256 blocks.
#define FLASH_PARAM_SIZE_BLOCK 65536U //each block has size of 64kb = 64*1024
#define FLASH_PARAM_NUM_SECTOR 4096U //no. of sectors. there are 16 sectors inside each one of the 256 blocks
#define FLASH_PARAM_SIZE_SECTOR 4096U //size of each sector is 4096 bytes
#define FLASH_PARAM_NUM_PAGE 65536U //no. of pages
#define FLASH_PARAM_SIZE_PAGE 256U //size of one page is 256 bytes

#define FLASH_ADDRESS_COMMAND_ERASE 0x00 //only sector erase
#define FLASH_ADDRESS_COMMAND_ADD 0x01

#define FLASH_INSTR_DUMMY_BYTE 0xDD

#define FLASH_INSTR_MFD_ID 0x90
#define FLASH_INSTR_DEVICE_ID 0xAB
#define FLASH_INSTR_JEDEC_ID 0x9F
#define FLASH_INSTR_UNIQUE_ID 0x4B

#define FLASH_INSTR_WRITE_ENABLE 0x06
#define FLASH_INSTR_WRITE_DISABLE 0x04

#define BYTE 1

#define FLASH_PARAM_LEN_1_BYTE 0x01
#define FLASH_PARAM_LEN_2_BYTE 0x02
#define FLASH_PARAM_LEN_3_BYTE 0x03
#define FLASH_PARAM_LEN_4_BYTE 0x04
#define FLASH_PARAM_LEN_5_BYTE 0x05

#define FLASH_INSTR_READ_DATA 0x03
#define FLASH_INSTR_WRITE_DATA 0x02
#define FLASH_INSTR_READ_STATUS_1 0x05
#define FLASH_INSTR_READ_STATUS_2 0x35
#define FLASH_INSTR_READ_STATUS_3 0x15

#define FLASH_INSTR_CHIP_ERASE 0xC7
#define FLASH_INSTR_SECTOR_ERASE 0x20

#define NSS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)

typedef SPI_HandleTypeDef hspi_type_t;

//uint8_t device_id;

void flash_spi_init(SPI_HandleTypeDef*);
void flash_get_mfd_id(void); //manufacture id
void flash_get_jedec_id(void);
void flash_get_unique_id(void);
void flash_get_device_id(void);

void flash_write_en(void);
void flash_write_dis(void);

uint8_t flash_read_status_1(void);
uint8_t flash_read_byte(uint32_t); //simple byte read
uint8_t flash_read_batch(uint32_t ,uint8_t *, uint16_t);
void flash_write_byte(uint32_t, uint8_t); //simple byte write
uint8_t flash_write_batch(uint32_t, uint8_t *, uint16_t);
void flash_chip_erase(void);
void flash_sector_erase(uint32_t);

#endif /*SPI_FLASH_LIB_H*/