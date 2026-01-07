/*
feature remaining: the data ready is asserted as soon as start is asserted. the time calculation
                   feature is not included here. if included, it must wake task and there will be delay
                   between start and data ready signal. this delay should be managed in the library
                   and the a<><> should be provided to master whenever data is ready to be read
*/

#include "sdi_cli.h"
#include "string.h"
#include "stdio.h"
#define SDI_ERROR 1

static char sdi_rx[20];
static uint8_t command_rx_flag = 0;
full_command command;
uint8_t len;
extern UART_HandleTypeDef hlpuart1;

typedef struct{
    uint8_t address;    //address of sensor
    uint8_t busy;       //1-busy, 0-idle
    uint8_t data_ready; //1-data_ready, 0-data not ready
    uint8_t error;      //1-error, 0-no errors
    float data[6];          //actual measured data values
    uint8_t* tx_data_ptr;   //pointer to the tx data buffer containing ascii to send
    uint32_t crc;
}sensor_status;
sensor_status status_t;

typedef struct{
    uint8_t blocking;   //1-execution requires blocking, 0-concurrency allowed
    uint8_t start;      //start bit
    uint8_t crc_en;        //1-crc required, 0-crc not required
    uint8_t read_type;  //0-single read, 1-mass read
    uint8_t terminate;  //terminate
}sensor_state_var;
sensor_state_var state_var_t;

typedef struct{
    sensor_state_var var;
    sensor_status status;
}sensor;
static sensor sdi_imu, sdi_flash, sdi_radar, sdi_charger, sdi_rtc, sdi_generic;

static void sensor_set(sensor);
static void sensor_start(sensor);
static void sensor_reset(sensor);
static void sensor_send(sensor);
static void reset(sensor*);
static uint8_t sensor_status_get(sensor);
static void service_request_msg(char , uint16_t, uint8_t);
static void abort_msg(char , uint16_t, uint8_t);

volatile uint8_t sdi_api_imu_start, sdi_api_charger_start, sdi_api_rtc_start, sdi_api_flash_start, sdi_api_radar_start; //interface variables
volatile uint8_t sdi_api_imu_crc_en, sdi_api_charger_crc_en, sdi_api_radar_crc_en, sdi_api_rtc_crc_en, sdi_api_flash_crc_en;
volatile uint8_t sdi_api_imu_dr_arr[10], sdi_api_charger_dr_arr[10], sdi_api_rtc_dr_arr[10], sdi_api_flash_dr_arr[10], sdi_api_radar_dr_arr[10]; //interface variables

volatile uint8_t blocking_measurement = 0;
volatile uint8_t non_blocking_measurement = 0;

static uint32_t command_to_hex(full_command *command){
    return ((uint32_t)command->address << 24) | ((uint32_t)command->c2 << 16) |
           ((uint32_t)command->c3 << 8)  | ((uint32_t)command->c4);
}

void sdi_cmd_send(char * cmd, uint8_t len){
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)cmd, len, HAL_MAX_DELAY);
}


//receive ascii command until '!' is detected and return received no. of bytes
//the ! is ignored after parsing. and while decoding, 'a' is also ignored after storing it as adress in command

//cmd rx_flag is indicator that complete command is received. it is set to 1 after receiving a complete command and set to 0 by the command
//executing function to indicate executor is ready
uint8_t sdi_cmd_receive(char * cmd){
    static uint8_t i = 0;
    if((*cmd == '!') && (command_rx_flag == 0)){
        command_rx_flag = 1;
        len = i;
        i = 0;
		return 1;
    }
    if((i < SDI_PARAM_MAX_CMD_LEN) && (command_rx_flag == 0)){
        sdi_rx[i] = *cmd;
        i = i + 1;
        command_rx_flag = 0;
		return 0;
    } else{
			i = 0;
        //error case - implement after first basic test
		return 0;
    }
}



void sdi_parse(char *cmd_raw, full_command *command, uint8_t cmd_len){
    command -> c1 = (cmd_raw[0] == '?') ? COMMAND_QUERY: COMMAND_ACTION;
    command->address = (cmd_raw[0] >= '0' && cmd_raw[0] <= 'f') ? cmd_raw[0] : '0'; //store invalid address. execution will check and cancel
    command->c2 = cmd_raw[1];
    command->c3 = cmd_raw[2];
    command->c4 = cmd_raw[3];
}

/*
length 1 instructions:
a!, ?!

length 2 instructions:
aI!, aM!, aV!, aC!

length 3 instructions:
aAb!, aMC!, aD0!..aD9!, aM1!..aM9!, aCC!, aC1!..aC9!, aR0!..aR9!

length 4 instructions:
aMC1!..aMC9!, aCC1!..aCC9!, aRC0!..aRC9!
*/

void sdi_decode(full_command *command, uint8_t len){
    uint32_t command_hex;// = command_to_hex(command);
    switch (len)
    {
    case 1: //len1 address are a! or ?!. it is easy to separate them. ? is 3F. so 0x3F000000 executes ?!.
        if(command -> c1 == COMMAND_QUERY){
            //command_hex = command_to_hex(command);
            sdi_execute(0x3F000000, command);
        }else{
            sdi_execute(0x61000000, command);
        }
    break;

    case 2:
        command_hex = command_to_hex(command) & 0x00FF0000;
        sdi_execute(command_hex, command);
    break;

    case 3: //for three length instruction with numbers, use command.c3 to get numerical value of the instruction
        if(command->c3 != 'C'){
            command_hex = command_to_hex(command) & 0x00FF0000; //length 3 commands with numbers will be distinguished by 1 at fourth byte
            command_hex = command_hex | 0x00001000;
            sdi_execute(command_hex, command);
        }else{
            command_hex = command_to_hex(command) & 0x00FFFF00;
            sdi_execute(command_hex, command);
        }
    break;

    case 4:
        //all length 4 commands will be identified by 1 at second byte
        command_hex = command_to_hex(command) & 0x00FFFF00;
        command_hex = command_hex | 0x00000010;
        sdi_execute(command_hex, command);
    break;

    default:

    break;
    }

}

/*
    reasons for choosing the decoding and executing method is explained below.
    the instruction category of length 1 and length 2 can be separated easily as they are distinct.
    it is tricky for instruction of legth 3. because aMC(for example) is present as a unique instruction and aM1...aM9 is also present as instruction.(aMC, aD0..aD9, aM1..aM9, aCC, aC1..aC9, aR0..aR9)
    we can create unique instruction for aM1 to aM9 too but it would add just too many switch cases. so we need to decode numerical/non numerical instruction in 3 length instructions as well.
    thus, we need to decode and while decoding, i noticed, letter 'C' is what separates numerical instruction from pure letter equation in 3 length instruction. so i compared with 'C'.
    i identified the instruction type and separated with fourth byte as 1 for numerical and as it is for non numerical instruction. so non numerical occupies all 4 bytes and numerical occupies-
    2 bytes and two bytes should be fetched from command->c3.
    4 length instructions are easy as we can separate the text and number. since MC1 decoding may call MC which is case for three length, so, for four length, i added one extra byte to separate it. now MC from three and 
    MC from 4  length instruction have separate meaning in the switch case execution 
*/
static sensor config;

void sdi_execute(uint32_t instr, full_command *command){
    switch (instr){
    case (0x61000000): //acknowledge address a!
        /*code*/
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"ack addr", sizeof("ack addr"), HAL_MAX_DELAY);
        SDI_RX();
        //use command.address to execute from here
    break;
    case (0x3F000000):
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"? mark", sizeof("? mark"), HAL_MAX_DELAY);
        SDI_RX();
    break;
    case(0x00411000): //change address command - aAb!
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"change addr aAb!", sizeof("change addr aAb!"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x004D0000): //start measurement aM
        if(config.var.blocking == 1){ //comparing old state with existing state. do not allow new sensor request if old is not completed
            if(command->address != config.status.address){ //but allow old sensor request
                SDI_TX();
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not allowed\n", sizeof("not allowed\n"), HAL_MAX_DELAY);
                SDI_RX();
                break;
            }else{
                sensor_reset(config); //reset sensor on identical repeated measure command like aM! after previous aM!
                abort_msg((char)(command->address), 2, 1);
            }
        }
        config.status.address = command->address;
        config.status.busy = 1;	//config by data read function
        config.status.data_ready = 0; //config by status function
        config.status.error = 0; //config not implemented
        //config.status.data = 0; //config by data read function
        config.var.blocking = 1; //config here
        config.var.read_type = 0; //single byte read
        config.var.start = 1; //config here, checked on sensor_set, modified on data read
        config.var.terminate = 0; //config not implemented
        config.var.crc_en = 0;
        //blocking_measurement = 1;

        sensor_set(config); //save and check sensor configuration. if target sensor is already busy, nothing happens
        sensor_start(config); //start mesaurement. if sensor busy, restart (in concurrent, if sensor busy then terminate)
    break;

    case(0x004D4300): //start measurement and request crc aMC
        if(config.var.blocking == 1){ 
            if(command->address != config.status.address){ 
                SDI_TX();
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not allowed\n", sizeof("not allowed\n"), HAL_MAX_DELAY);
                SDI_RX();
                break;
            }else{
                sensor_reset(config);
            }
        }
        config.status.address = command->address;
        config.status.busy = 1;
        config.status.data_ready = 0;
        config.status.error = 0;
        config.var.blocking = 1;
        config.var.crc_en = 1;
        config.var.read_type = 1;
        config.var.start = 1;
        config.var.terminate = 0;
        sensor_set(config);
        sensor_start(config);
    break;

    case(0x00441000): //send data aD0...aD9
        config.status.address = command->address;
        SDI_TX();
        if((sensor_status_get(config) == 0)){ //if data not ready then respond with address and stop measurement
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"data not ready \"address\" \n", sizeof("data not ready \"address\" \n"), HAL_MAX_DELAY);
        }
        SDI_RX();
		if(sensor_status_get(config) == 1){
            sensor_send(config);
		    sensor_reset(config);
        }
    break;

    case(0x004D1000): //additional measurements aM0....aM9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aMx", sizeof("not implemented aMx"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x004D4310): //additional measurements and request CRC aMC1..aMC9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aMCx", sizeof("not implemented aMCx"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00560000): //additional verification aV
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aV", sizeof("not implemented aV"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00430000): //additional concurrent measurement aC
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aC", sizeof("not implemented aC"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00434300): //additional concurrent measurement and request CRC //aCC
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aCC", sizeof("not implemented aCC"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00431000): //additional conncurrent measurements aC0...aC9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aCx", sizeof("not implemented aCx"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00434310): //additional concurrent measurements and request CRC aCC1..aCC9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aCCx", sizeof("not implemented aCCx"), HAL_MAX_DELAY);
        SDI_RX();
    break;
    case(0x00521000): //continuous measurement aR0...aR9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aRx", sizeof("not implemented aRx"), HAL_MAX_DELAY);
        SDI_RX();
    break;
    case(0x00524310): //continuous measurements and request crc aRC0...aRC9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aRCx", sizeof("not implemented aRCx"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    default:
        break;
    }
}
void sdi_main(void){
	sdi_parse(sdi_rx, &command, len);
	sdi_decode(&command, len);
	command_rx_flag = 0;
}


static void sensor_set(sensor config){
    switch (config.status.address)
    {
        case 'a':
			sdi_imu = config;
        break;
        case 'b':
            sdi_charger = config;
        break;
        case 'c':
            sdi_radar = config;
        break;
        case 'd':
            sdi_flash= config;
        break;
        case 'e':
           sdi_rtc= config;
        break;
        default:
        break;
    }
    sdi_generic = config;
}

void sensor_start(sensor config){
    switch (config.status.address)
    {
        case 'a':
            sdi_api_imu_start = 1;
            sdi_api_imu_crc_en = sdi_imu.var.crc_en;
            service_request_msg('a',002,1);//send back a response
        break;
        case 'b':
            sdi_api_charger_start = 1;
            sdi_api_charger_crc_en = sdi_charger.var.crc_en;
            service_request_msg('b',002,1);
        break;
        case 'c':
            sdi_api_radar_start = 1;
            sdi_api_radar_crc_en = sdi_radar.var.crc_en;
            service_request_msg('c',002,1);
        break;
        case 'd':
			sdi_api_flash_start = 1;
            sdi_api_flash_crc_en = sdi_flash.var.crc_en;
            service_request_msg('d',002,1);
        break;
        case 'e':
            sdi_api_rtc_start = 1;
            sdi_api_rtc_crc_en = sdi_rtc.var.crc_en;
            service_request_msg('e',002,1);
        break;
        default:
        break;
    }
}

void sensor_reset(sensor conf){
		switch (conf.status.address)
    {
        case 'a':
            sdi_api_imu_start = 0;
            reset(&sdi_imu);
        break;
        case 'b':
            sdi_api_charger_start = 0;
            reset(&sdi_charger);
        break;
        case 'c':
            sdi_api_radar_start = 0;           
            reset(&sdi_radar); 
        break;
        case 'd':
            sdi_api_flash_start = 0;           
            reset(&sdi_flash); 
        break;
        case 'e':
            sdi_api_rtc_start = 0;       
            reset(&sdi_rtc); 
        break;
        default:
        break;
    }
		reset(&config);
}

void api_status(char sensor, uint8_t data_ready, float* data, uint32_t crc, char* ascii_ptr){
    char tx_buf[100];
	switch (sensor)
		{
        case 'a':
            sdi_imu.status.data_ready = data_ready;
            sdi_imu.status.crc = crc;
            sdi_imu.status.tx_data_ptr = ascii_ptr;
            sdi_imu.status.data[0] = data[0];
            sdi_imu.status.data[1] = data[1];
            sdi_imu.status.data[2] = data[2];
        break;
        case 'b':
            sdi_charger.status.data_ready = data_ready;
            sdi_charger.status.crc = crc;
            sdi_charger.status.tx_data_ptr = ascii_ptr;
            sdi_charger.status.data[0] = data[0];
            sdi_charger.status.data[1] = data[1];
            sdi_charger.status.data[2] = data[2];
            sdi_charger.status.data[3] = data[3];
            sdi_charger.status.data[4] = data[4];
            sdi_charger.status.data[5] = data[5];
        break;
        case 'c':
            sdi_radar.status.data_ready = data_ready;
            sdi_radar.status.crc = crc;
            sdi_radar.status.tx_data_ptr = ascii_ptr;
            sdi_radar.status.data[0] = data[0];
            sdi_radar.status.data[1] = data[1];
            sdi_radar.status.data[2] = data[2];
            sdi_radar.status.data[3] = data[3];
            sdi_radar.status.data[4] = data[4];
            sdi_radar.status.data[5] = data[5];
            break;
        case 'd':
            sdi_flash.status.data_ready = data_ready;
            sdi_flash.status.crc = crc;
            sdi_flash.status.tx_data_ptr = ascii_ptr;
            sdi_flash.status.data[0] = data[0];
        break;
        case 'e':
            sdi_rtc.status.data_ready = data_ready;
            sdi_rtc.status.crc = crc;
            sdi_rtc.status.tx_data_ptr = ascii_ptr;
            sdi_rtc.status.data[0] = data[0];
            sdi_rtc.status.data[1] = data[1];
            sdi_rtc.status.data[2] = data[2];
            sdi_rtc.status.data[3] = data[3];
            sdi_rtc.status.data[4] = data[4];
            sdi_rtc.status.data[5] = data[5];
        break;
        default:
        break;
		}
}

//measurement status - the sensor must have start bit 1 and must have data ready 1 to give a valid status  
static uint8_t sensor_status_get(sensor config){
    switch(config.status.address){
        case 'a':
            if(sdi_imu.status.data_ready && sdi_imu.var.start) return 1; else return 0;
        break;
        case 'b':
            if(sdi_charger.status.data_ready && sdi_charger.var.start) return 1; else return 0;
        break;
        case 'c':
            if(sdi_radar.status.data_ready && sdi_radar.var.start) return 1; else return 0;
        break;
        case 'd':
            if(sdi_flash.status.data_ready && sdi_flash.var.start) return 1; else return 0;
        break;
        case 'e':
            if(sdi_rtc.status.data_ready && sdi_rtc.var.start) return 1; else return 0;
        break;
        default:
            return 0;
        break;
    }
}

void sensor_send(sensor config){
    char tx_buf[100];
    uint32_t crc_ascii;
    uint8_t crc_buf[3];
    char crc_dbg[16];

	switch (config.status.address)
    {
        case 'a':
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"imu_data: ", sizeof("imu_data: "), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_imu.status.tx_data_ptr, strlen(sdi_imu.status.tx_data_ptr), HAL_MAX_DELAY);
            if(sdi_imu.var.crc_en == 1){
                crc_ascii = sdi_crc_ascii(sdi_imu.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
                
            }
            SDI_RX();
        break;
        case 'b':
            // snprintf(tx_buf, sizeof(tx_buf), "charger: VBAT=%+0.3f VSYS=%+0.3f VBUS=%+0.3f IBAT=%+0.3f IBUS=%+0.3f BAT_TEMP=%+0.3f\r\n", sdi_charger.status.data[0],
            //                                                                         sdi_charger.status.data[1],
            //                                                                         sdi_charger.status.data[2],
            //                                                                         sdi_charger.status.data[3],
            //                                                                         sdi_charger.status.data[4],
            //                                                                         sdi_charger.status.data[5]);
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"charger_data: ", sizeof("charger_data: "), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_charger.status.tx_data_ptr, strlen(sdi_imu.status.tx_data_ptr), HAL_MAX_DELAY);
           if(sdi_charger.var.crc_en == 1){
                crc_ascii = sdi_crc_ascii(sdi_charger.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            // HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case 'c':
            // snprintf(tx_buf, sizeof(tx_buf), "distances: D1=%+0.3f S1=%+0.3f D2=%+0.3f S2=%+0.3f D3=%+0.3f S3=%+0.3f\r\n", sdi_radar.status.data[0],
            //                                                                         sdi_radar.status.data[1],
            //                                                                         sdi_radar.status.data[2],
            //                                                                         sdi_radar.status.data[3],
            //                                                                         sdi_radar.status.data[4],
            //                                                                         sdi_radar.status.data[5]);
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"radar_data: ", sizeof("radar_data: "), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_radar.status.tx_data_ptr, strlen(sdi_radar.status.tx_data_ptr), HAL_MAX_DELAY);
            if(sdi_charger.var.crc_en == 1){
                crc_ascii = sdi_crc_ascii(sdi_radar.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8) & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            SDI_RX();
        break;
        case 'd':
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"flash data: ", sizeof("flash_data: "), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_flash.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case 'e':
            // snprintf(tx_buf, sizeof(tx_buf), "rtc: sec=%+0.3f min=%+0.3f hour=%+0.3f year=%+0.3f month=%+0.3f day=%+0.3f\r\n", sdi_rtc.status.data[0],
            //                                                                         sdi_rtc.status.data[1],
            //                                                                         sdi_rtc.status.data[2],
            //                                                                         sdi_rtc.status.data[3],
            //                                                                         sdi_rtc.status.data[4],
            //                                                                         sdi_rtc.status.data[5]);
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"rtc: ", sizeof("rtc: "), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_rtc.status.tx_data_ptr, strlen(sdi_rtc.status.tx_data_ptr), HAL_MAX_DELAY);
            if(sdi_rtc.var.crc_en == 1){
                crc_ascii = sdi_crc_ascii(sdi_rtc.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            SDI_RX();
        break;
        default:
        break;
    }
		sensor_reset(config);
}

static void reset(sensor* sensor){
	sensor->status.busy = 0;
	sensor->status.data[0] = 0;
	sensor->status.data[1] = 0;
	sensor->status.data[2] = 0;
	sensor->status.data[3] = 0;
	sensor->status.data[4] = 0;
	sensor->status.data[5] = 0;
	sensor->status.data_ready = 0;
	sensor->status.error = 0;
	sensor->var.blocking = 0;
	sensor->var.read_type = 0;
	sensor->var.start = 0;
	sensor->var.terminate = 0;
}

static void service_request_msg(char address, uint16_t time, uint8_t number){
    char tx_buf[10];
    snprintf(tx_buf, sizeof(tx_buf), "%c%03d%d\r\n",address, time, number );
    SDI_TX();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
    SDI_RX();
}
static void abort_msg(char address, uint16_t time, uint8_t number){ //same as service_request_msg. for readablility
    char tx_buf[10];
    snprintf(tx_buf, sizeof(tx_buf), "%c%03d%d\r\n",address, time, number );
    SDI_TX();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
    SDI_RX();
}

uint32_t sdi_crc_ascii(uint32_t int_crc)
{
    uint32_t ascii = 0;

    uint8_t c1 = 0x40 | ((int_crc >> 12) & 0x0F);
    uint8_t c2 = 0x40 | ((int_crc >> 6)  & 0x3F);
    uint8_t c3 = 0x40 | (int_crc & 0x3F);

    ascii = ((uint32_t)c1 << 16) | ((uint32_t)c2 << 8) | (uint32_t)c3;

    return ascii;
}
uint16_t sdi_crc16(char *data, uint16_t len)
{
    uint16_t crc = 0x0000;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];

        for (uint8_t count = 0; count < 8; count++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

