#include "spi_flash.h"

static SPI_HandleTypeDef* g_p_hspi;
static volatile uint8_t flash_spi_tx_buf[8], flash_spi_rx_buf[8];

static uint8_t status = 0b00000000;
//bit_0(lsb) - byte write status. 1-failed 0-success
//bit_1 - byte read status. 1-failed 0-success
//bit_3 - batch write status. 1-failed 0-success
//bit_4 - sector erase status. 1-failed 0-success
//bit_5 - illegal erase attempt. 1-attempt 0-no attempts

static volatile uint8_t page_number = 0x00; //store newest page address available
static volatile uint8_t sector_number = 0x00; //store newest sector address available
static volatile uint8_t block_number = 0x00; //store newest block address available
static volatile uint32_t active_address;

static uint8_t back_sector_erase_available = 0;
// writes and reads are intended to be consecutive, not random
static uint8_t flash_is_write_addr_valid(uint8_t block, uint8_t sector, uint8_t page){ 
    if((block == block_number)&&(sector == sector_number)&&(page == page_number)){
      return FLASH_TRUE;
    } else{
      return FLASH_FALSE;
    }
}


//adjust page, sector, block address after a write or erase
static void flash_address_manager(uint8_t block, uint8_t sector, uint8_t page, uint8_t command, uint16_t amount){
  uint32_t address;
  switch (command)
  {
  case FLASH_ADDRESS_COMMAND_ADD:
    address = (0x00 << 24) | (block << 16) | (sector << 8) | (page);
    address = address + amount;
    active_address = address;
    block_number = (active_address >> 16) & 0xFF;
    sector_number = (active_address >> 8) & 0xFF;
    page_number = active_address & 0xFF;
  break;
  case FLASH_ADDRESS_COMMAND_ERASE:
    address = (0x00 << 24) | (block << 16) | (sector << 8) | (page);
    active_address = address & 0x00FFF000;
    block_number = (active_address >>16) & 0xFF;
    sector_number = (active_address >> 8) & 0xFF;
    page_number = 0;
  break;
  default:
    break;
  }
}


void flash_spi_init(SPI_HandleTypeDef *hspi_type_t){
    g_p_hspi = hspi_type_t;
}
void flash_get_mfd_id(void){
    flash_spi_tx_buf[0] = FLASH_INSTR_MFD_ID;
    flash_spi_tx_buf[1] = FLASH_INSTR_DUMMY_BYTE;
    flash_spi_tx_buf[2] = FLASH_INSTR_DUMMY_BYTE;
    //uint8_t tx_buffer[3] = {FLASH_INSTR_MFD_ID, 0xDE, 0xDE};
    //uint8_t rx_buffer;
    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_3_BYTE, HAL_MAX_DELAY);
    HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_3_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();
}

void flash_get_jedec_id(void){
    //uint8_t tx_buffer = FLASH_INSTR_JEDEC_ID;
    //uint8_t rx_buffer[3];
    flash_spi_tx_buf[0] = FLASH_INSTR_JEDEC_ID;
    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_3_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();
}

void flash_get_unique_id(void){
  //uint8_t tx_buffer[5] = {FLASH_INSTR_UNIQUE_ID, 0xAB, 0xBC, 0xCD, 0xDE};
  //uint8_t rx_buffer[4];
  flash_spi_tx_buf[0] = FLASH_INSTR_UNIQUE_ID;
  flash_spi_tx_buf[1] = FLASH_INSTR_DUMMY_BYTE;
  flash_spi_tx_buf[2] = FLASH_INSTR_DUMMY_BYTE;
  flash_spi_tx_buf[3] = FLASH_INSTR_DUMMY_BYTE;
  flash_spi_tx_buf[4] = FLASH_INSTR_DUMMY_BYTE;
  NSS_LOW();
  HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_5_BYTE, HAL_MAX_DELAY);
  HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_1_BYTE * 8, HAL_MAX_DELAY);
  NSS_HIGH();
}

void flash_get_device_id(void){
  //uint8_t tx_buffer[4] = {FLASH_INSTR_DEVICE_ID, 0xCD, 0xDE, 0xEF};
  //uint8_t rx_buffer;
  flash_spi_tx_buf[0] = FLASH_INSTR_DEVICE_ID;
  flash_spi_tx_buf[1] = FLASH_INSTR_DUMMY_BYTE;
  flash_spi_tx_buf[2] = FLASH_INSTR_DUMMY_BYTE;
  flash_spi_tx_buf[3] = FLASH_INSTR_DUMMY_BYTE;
  NSS_LOW();
  HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
  HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
  NSS_HIGH();
}

void flash_write_en(){
    // uint8_t tx_buffer = FLASH_INSTR_WRITE_ENABLE;
    uint8_t write_en = FLASH_INSTR_WRITE_ENABLE;
    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, &write_en, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();
}

void flash_write_dis(){
    uint8_t write_dis = FLASH_INSTR_WRITE_DISABLE;
    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, &write_dis, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t flash_read_byte(uint32_t address){
    //uint8_t rx_buffer;
		//uint8_t tx_address[4];
    uint8_t page, block, sector;
    page = (address) & 0xFF;
    sector = (address >> 8) & 0xFF;
    block = (address >> 16) & 0xFF;
		flash_spi_tx_buf[0] = FLASH_INSTR_READ_DATA;
		flash_spi_tx_buf[1] = block;
		flash_spi_tx_buf[2] = sector;
		flash_spi_tx_buf[3] = page;

    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
    HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();

    return flash_spi_rx_buf[0];
}

uint8_t flash_read_batch(uint32_t start_address, uint8_t * buffer, uint16_t len){
  flash_spi_tx_buf[0] = FLASH_INSTR_READ_DATA;
  flash_spi_tx_buf[1] = (start_address >> 16) & 0xFF;
  flash_spi_tx_buf[2] = (start_address >> 8) & 0xFF;
  flash_spi_tx_buf[3] = (start_address) & 0xFF;
  
  NSS_LOW();
  HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
  HAL_SPI_Receive(g_p_hspi, buffer, (uint16_t)len, HAL_MAX_DELAY);
  NSS_HIGH();
}

void flash_write_byte(uint32_t address, uint8_t data){
    //uint8_t tx_address[5];

    uint8_t page, block, sector;
    page = (address) & 0xFF;
    sector = (address >> 8) & 0xFF;
    block = (address >> 16) & 0xFF;
		flash_spi_tx_buf[0] = FLASH_INSTR_WRITE_DATA;
		flash_spi_tx_buf[1] = block;
		flash_spi_tx_buf[2] = sector;
		flash_spi_tx_buf[3] = page;
    flash_spi_tx_buf[4] = data;

    if(!flash_is_write_addr_valid(block, sector, page)){
      flash_write_en();
      NSS_LOW();
      HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_5_BYTE, HAL_MAX_DELAY);
      NSS_HIGH();
      flash_write_dis();
      flash_address_manager(block, sector, page, FLASH_ADDRESS_COMMAND_ADD, 1);
      status &= 0b11111110;
    } else{
      status |= 0b00000001;
    }
    back_sector_erase_available = 0;
}
//only consecutive writes are allowed. So the page, sectors and blocks are wrapped automatically as reqired.
//if we are on 53rd byte of a page and 250 bytes is to be written, the address should automatically wrap to the next sector and begin from there
uint8_t flash_write_batch(uint32_t start_address, uint8_t *buffer, uint16_t len){
  uint8_t i = 0;
      uint8_t page, block, sector;
  while(len > 0){
      page = (start_address) & 0xFF;
      sector = (start_address >> 8) & 0xFF;
      block = (start_address >> 16) & 0xFF;
      flash_spi_tx_buf[0] = FLASH_INSTR_WRITE_DATA;
      flash_spi_tx_buf[1] = block;
      flash_spi_tx_buf[2] = sector;
      flash_spi_tx_buf[3] = page;

      uint16_t remaining_bytes_in_page = FLASH_PARAM_SIZE_PAGE - page_number;  //kept in track by previous transactions by address managers
      uint16_t transaction_size = (len >= remaining_bytes_in_page)? remaining_bytes_in_page : len;
      if(!flash_is_write_addr_valid(block, sector, page)){ 
        flash_write_en();
        NSS_LOW();
        HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
        HAL_SPI_Transmit(g_p_hspi, buffer, transaction_size, HAL_MAX_DELAY);
        NSS_HIGH();
        flash_write_dis();
        flash_address_manager(block, sector, page, FLASH_ADDRESS_COMMAND_ADD, transaction_size); //keep track of most recent transaction. internal counters only
        status &= 0b11111111;
      } else {
        status |= 0b00000100;
				return 0;
      }

      buffer = buffer + transaction_size;
      len = len - transaction_size;
      start_address = start_address + transaction_size;
  }
      back_sector_erase_available = 0;
}

uint8_t flash_read_status_1(){
    // uint8_t tx_data = FLASH_INSTR_READ_STATUS_1;
    flash_spi_tx_buf[0] = FLASH_INSTR_READ_STATUS_1;
    NSS_LOW();
    HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    HAL_SPI_Receive(g_p_hspi, flash_spi_rx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
    NSS_HIGH();
    return flash_spi_rx_buf[0];
}

void flash_chip_erase(void){
  flash_write_en();
  flash_spi_tx_buf[0] = FLASH_INSTR_CHIP_ERASE;
  NSS_LOW();
  HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_1_BYTE, HAL_MAX_DELAY);
  NSS_HIGH();
  flash_write_dis();
}

void flash_sector_erase(uint32_t address){
    uint8_t page, sector, block;
    block = (address >> 16) & 0xFF;
    sector = (address >> 8) & 0xFF;
    page = address & 0xFF; //page does not matter during sector erase

    uint8_t sector_aligned = sector & 0xF0;
    uint8_t page_aligned = 0x00;
    flash_spi_tx_buf[0] = FLASH_INSTR_SECTOR_ERASE;
    flash_spi_tx_buf[1] = block;
    flash_spi_tx_buf[2] = sector_aligned;
    flash_spi_tx_buf[3] = page_aligned; //set page address to zero to erase a sector.

    if(!flash_is_write_addr_valid(block, sector, page)){ //erase can be only done on active sector
      flash_write_en();
      NSS_LOW();
      HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
      NSS_HIGH();
      flash_write_dis();
      flash_address_manager(block, sector_aligned, page_aligned, FLASH_ADDRESS_COMMAND_ERASE, 1); //single sector erased
      status &= 0b11100111; //erase successful. no illegal attempt
      back_sector_erase_available = 1; //back sector is available to erase. cleared after erasing a back sector afterwards or by attempting a write on active sector
    }
    else{
      //the comparision in if statement below could be simple but this approach considers edge case condition like block 2 sector 0 and block 1 sector 15. 
      if(((active_address & 0x00FFF000) - (address & 0x00FFF000) == 0x1000) && (back_sector_erase_available == 1)){ //check if provided address is one sector behind of current address
        flash_write_en();
        NSS_LOW();
        HAL_SPI_Transmit(g_p_hspi, flash_spi_tx_buf, FLASH_PARAM_LEN_4_BYTE, HAL_MAX_DELAY);
        NSS_HIGH();
        flash_write_dis();
        flash_address_manager(block, sector_aligned, page_aligned, FLASH_ADDRESS_COMMAND_ERASE, 1); //single sector erased
        status &= 0b11100111; //erase successful, no illegal attempt
        back_sector_erase_available = 0; //back sector is not available to erase. perform erase on current sector to erase back sector
      } else{
        status |= 0b00011000; //erase failed and due to illegal erase attempt
      }
      //status |= 0b00001000; //erase failed and not due to illegal attempt
    }
}