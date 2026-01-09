#include "charger_driver.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern void print_uart(char *, uint8_t, uint8_t);
static void print_u16(char *, uint16_t );

extern float charger_data[6];


uint16_t charge_i2c_addr = CHARGER_I2C_ADDR;

uint8_t  charge_part_info;
uint8_t  charge_wd_timer_status;
uint8_t  charge_fault_status_1;

uint8_t  charge_adc_ctrl       = 0xC0;
uint8_t  charge_adc_func_dis_0 = 0x02;
uint8_t  charge_adc_func_dis_1 = 0xF0;

charger_adc adc_val;

void charge_config(void) {
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_PART_INFO_REG, I2C_MEMADD_SIZE_8BIT, &charge_part_info, 1, HAL_MAX_DELAY);

//    print_uart("device part number: ", sizeof("device part number: "), charge_part_info);
}

void get_charger_status(charger_status_val_t *charger_status_val) {
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_STATUS_0_REG, I2C_MEMADD_SIZE_8BIT, &charger_status_val->charger_status_0, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_STATUS_1_REG, I2C_MEMADD_SIZE_8BIT, &charger_status_val->charger_status_1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_STATUS_2_REG, I2C_MEMADD_SIZE_8BIT, &charger_status_val->charger_status_2, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_STATUS_3_REG, I2C_MEMADD_SIZE_8BIT, &charger_status_val->charger_status_3, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_STATUS_4_REG, I2C_MEMADD_SIZE_8BIT, &charger_status_val->charger_status_4, 1, HAL_MAX_DELAY);
}

void get_fault_status(void) {
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FAULT_STATUS_1, I2C_MEMADD_SIZE_8BIT, &charge_wd_timer_status, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FAULT_STATUS_0, I2C_MEMADD_SIZE_8BIT, &charge_fault_status_1, 1, HAL_MAX_DELAY);
}

void get_flag_status(charger_flag_val_t *charger_flag_val) {
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FLAG_0_REG, I2C_MEMADD_SIZE_8BIT, &charger_flag_val->charger_flag_0, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FLAG_1_REG, I2C_MEMADD_SIZE_8BIT, &charger_flag_val->charger_flag_1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FLAG_2_REG, I2C_MEMADD_SIZE_8BIT, &charger_flag_val->charger_flag_2, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_FLAG_3_REG, I2C_MEMADD_SIZE_8BIT, &charger_flag_val->charger_flag_3, 1, HAL_MAX_DELAY);
}

void set_adc(void) {
    HAL_I2C_Mem_Write(&hi2c1, charge_i2c_addr, CHARGER_ADC_CTRL_REG, I2C_MEMADD_SIZE_8BIT, &charge_adc_ctrl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, charge_i2c_addr, CHARGER_ADC_FUNC_DIS_0, I2C_MEMADD_SIZE_8BIT, &charge_adc_func_dis_0, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, charge_i2c_addr, CHARGER_ADC_FUNC_DIS_1, I2C_MEMADD_SIZE_8BIT, &charge_adc_func_dis_1, 1, HAL_MAX_DELAY);
}

void get_adc(void) {
    uint8_t buf[2];

    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_VBAT_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.vbat = (buf[1] | (uint16_t)(buf[0] << 8));
		
    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_VSYS_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.vsys = (buf[1] | (uint16_t)(buf[0] << 8));

    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_VBUS_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.vbus = (buf[1] | (uint16_t)(buf[0] << 8));

    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_IBAT_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.ibat = (buf[1] | (uint16_t)(buf[0] << 8));

    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_IBUS_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.ibus = (buf[1] | (uint16_t)(buf[0] << 8));

    HAL_I2C_Mem_Read(&hi2c1, charge_i2c_addr, CHARGER_ADC_BAT_TEMP_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    adc_val.bat_temp = (buf[1] | (uint16_t)(buf[0] << 8));

    charger_data[0] = (float)(adc_val.vbat) / 1000;
    charger_data[1] = (float)(adc_val.vsys) / 1000;
    charger_data[2] = (float)(adc_val.vbus) / 1000;
    charger_data[3] = (float)(adc_val.ibat) / 1000;
    charger_data[4] = (float)(adc_val.ibus) / 1000;
    charger_data[5] = (float)adc_val.bat_temp;  //temperature to be converted to meaningful value
		// print_u16("VBAT",     adc_val.vbat);
		// print_u16("VSYS",     adc_val.vsys);
		// print_u16("VBUS",     adc_val.vbus);
		// print_u16("IBAT",     adc_val.ibat);
		// print_u16("IBUS",     adc_val.ibus);
		// print_u16("BAT_TEMP", adc_val.bat_temp);
}
static void print_u16(char *label, uint16_t value)
{
    char buf[32];
    int len = sprintf(buf, "%s: %u\r\n", label, value);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}
