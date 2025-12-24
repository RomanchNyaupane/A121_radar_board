#ifndef CHARGER_DRIVER_H
#define CHARGER_DRIVER_H

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define CHARGER_I2C_ADDR          (0x6B << 1)

#define CHARGER_STATUS_0_REG      0x1B
#define CHARGER_STATUS_1_REG      0x1C
#define CHARGER_STATUS_2_REG      0x1D
#define CHARGER_STATUS_3_REG      0x1E
#define CHARGER_STATUS_4_REG      0x1F

#define CHARGER_FLAG_0_REG        0x22
#define CHARGER_FLAG_1_REG        0x23
#define CHARGER_FLAG_2_REG        0x24
#define CHARGER_FLAG_3_REG        0x25

#define CHARGER_PART_INFO_REG     0x48
#define CHARGER_FAULT_STATUS_0    0x20
#define CHARGER_FAULT_STATUS_1    0x21

#define CHARGER_ADC_CTRL_REG      0x2E
#define CHARGER_ADC_FUNC_DIS_0    0x2F
#define CHARGER_ADC_FUNC_DIS_1    0x30

#define CHARGER_ADC_VSYS_REG      0x3D
#define CHARGER_ADC_VBAT_REG      0x3B
#define CHARGER_ADC_IBAT_REG      0x33
#define CHARGER_ADC_VBUS_REG      0x35
#define CHARGER_ADC_IBUS_REG      0x31
#define CHARGER_ADC_BAT_TEMP_REG  0x3F

typedef struct {
    uint8_t charger_status_0;
    uint8_t charger_status_1;
    uint8_t charger_status_2;
    uint8_t charger_status_3;
    uint8_t charger_status_4;
} charger_status_val_t;

typedef struct {
    uint8_t charger_flag_0;
    uint8_t charger_flag_1;
    uint8_t charger_flag_2;
    uint8_t charger_flag_3;
} charger_flag_val_t;

typedef struct{
		uint16_t vbat;
		uint16_t vbus;
		uint16_t vsys;
		uint16_t bat_temp;
		uint16_t ibus;
		uint16_t ibat;
}charger_adc;

void charge_config(void);
void get_charger_status(charger_status_val_t *charger_status_val);
void get_fault_status(void);
void get_flag_status(charger_flag_val_t *charger_flag_val);
void set_adc(void);
void get_adc(void);

#endif /* CHARGER_DRIVER_H */
