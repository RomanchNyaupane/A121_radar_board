#include "sdi_cli.h"

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
    uint32_t data;
}sensor_status;
sensor_status status_t;

typedef struct{
    uint8_t blocking;   //1-execution requires blocking, 0-concurrency allowed
    uint8_t start;      //start bit
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

volatile uint8_t sdi_api_imu_start, sdi_api_charger_start, sdi_api_rtc_start, sdi_api_flash_start, sdi_api_radar_start; //interface variables
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
					}
			}
				config.status.address = command->address;
				config.status.busy = 1;	//config by data read function
				config.status.data_ready = 0; //config by status function
				config.status.error = 0; //config not implemented
				config.status.data = 0; //config by data read function
				config.var.blocking = 1; //config here
				config.var.read_type = 0; //single byte read
				config.var.start = 1; //config here, checked on sensor_set, modified on data read
				config.var.terminate = 0; //config not implemented
		
				blocking_measurement = 1;

				sensor_set(config); //save and check sensor configuration. if target sensor is already busy, nothing happens
				sensor_start(config); //start mesaurement. if sensor busy, restart (in concurrent, if sensor busy then terminate)
    break;

    case(0x004D4300): //start measurement and request crc aMC
        //code
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aMC", sizeof("not implemented aMC"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00441000): //send data aD0...aD9
				config.status.address = command->address;
				sensor_send(config);
				sensor_reset(config);

				//SDI_TX();
				//HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aDx", sizeof("not implemented aDx"), HAL_MAX_DELAY);
				//SDI_RX();
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
            //if(sdi_imu.status.busy == 1) break;
            sdi_api_imu_start = 1;
						SDI_TX();
						HAL_UART_Transmit(&hlpuart1, (uint8_t*)"a0021\r\n", sizeof("a0021\r\n"), HAL_MAX_DELAY); //format atttn<cr><lf> ttt=002, n = 1
						SDI_RX();
        break;
        case 'b':
            //if(sdi_charger.status.busy == 1) break;
            sdi_api_charger_start = 1;
						SDI_TX();
						HAL_UART_Transmit(&hlpuart1, (uint8_t*)"b0021\r\n", sizeof("b0021\r\n"), HAL_MAX_DELAY);
						SDI_RX();
        break;
        case 'c':
            //if(sdi_radar.status.busy == 1) break;
            sdi_api_radar_start = 1;
						SDI_TX();
						HAL_UART_Transmit(&hlpuart1, (uint8_t*)"c0021\r\n", sizeof("c0021\r\n"), HAL_MAX_DELAY);
						SDI_RX();
        break;
        case 'd':
            //if(sdi_flash.status.busy == 1) break;
						sdi_api_flash_start = 1;
						SDI_TX();
						HAL_UART_Transmit(&hlpuart1, (uint8_t*)"d0021\r\n", sizeof("d0021\r\n"), HAL_MAX_DELAY);
						SDI_RX();
        break;
        case 'e':
						//if(sdi_rtc.status.busy == 1) break;
            sdi_api_rtc_start = 1;
						SDI_TX();
						HAL_UART_Transmit(&hlpuart1, (uint8_t*)"e0021\r\n", sizeof("e0021\r\n"), HAL_MAX_DELAY);
						SDI_RX();
        break;
        default:
        break;
    }
}

void sensor_reset(sensor config){
		switch (config.status.address)
    {
        case 'a':
            reset(&sdi_imu);
        break;
        case 'b':
						reset(&sdi_charger);
        break;
        case 'c':
            reset(&sdi_radar);            
        break;
        case 'd':
            reset(&sdi_flash);            
        break;
        case 'e':
            reset(&sdi_rtc);            
        break;
        default:
        break;
    }
		reset(&config);
}

void api_status(char sensor, uint8_t data_ready, uint32_t data){
	switch (sensor)
		{
			case 'a':
					sdi_imu.status.data_ready = data_ready;
					sdi_imu.status.data = data;
			break;
			case 'b':
					sdi_charger.status.data_ready = data_ready;
					sdi_charger.status.data = data;
			break;
			case 'c':
					sdi_radar.status.data_ready = data_ready;
					sdi_radar.status.data = data;
			break;
			case 'd':
					sdi_flash.status.data_ready = data_ready;
					sdi_flash.status.data = data;
			break;
			case 'e':
					sdi_rtc.status.data_ready = data_ready;
					sdi_rtc.status.data = data;
			break;
			default:
			break;
		}
}

void sensor_send(sensor config){
		switch (config.status.address)
    {
        case 'a':
					SDI_TX();
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"imu_data: ", sizeof("imu_data: "), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_imu.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
					SDI_RX();
					
        break;
        case 'b':
					SDI_TX();
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"charger_data: ", sizeof("charger_data: "), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_charger.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
					SDI_RX();
        break;
        case 'c':
					SDI_TX();
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"radar_data: ", sizeof("radar_data: "), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_radar.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
					SDI_RX();
        break;
        case 'd':
					SDI_TX();
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"flash: ", sizeof("flash_data: "), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_flash.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
					SDI_RX();
        break;
        case 'e':
					SDI_TX();
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"rtc: ", sizeof("rtc: "), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)&sdi_rtc.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
					SDI_RX();
        break;
        default:
        break;
    }
		sensor_reset(config);
}

static void reset(sensor* sensor){
	sensor->status.busy = 0;
	sensor->status.data = 0;
	sensor->status.data_ready = 0;
	sensor->status.error = 0;
	sensor->var.blocking = 0;
	sensor->var.read_type = 0;
	sensor->var.start = 0;
	sensor->var.terminate = 0;
}