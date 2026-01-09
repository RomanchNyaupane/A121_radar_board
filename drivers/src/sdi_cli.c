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

/*******************PRIVATE DATA STRUCTURES******************** */
typedef struct{
    char address;           //address of sensor
    uint8_t sub_address;    //like x from aMx or x from aMCx
    uint8_t busy;           //1-busy, 0-idle
    uint8_t data_ready;     //1-data_ready, 0-data not ready
    uint8_t error;          //1-error, 0-no errors
    float data[6];          //actual measured data values
    uint8_t* tx_data_ptr;   //pointer to the tx data buffer containing ascii to send
    uint16_t crc;           //crc value
} sensor_status;

typedef struct{
    uint8_t blocking;       //1-execution requires blocking, 0-concurrency allowed
    uint8_t start;          //start bit
    uint8_t sub_reading;    //0-all sensor data, 1-only one type of data. for aM1 command. eg:only roll from imu, not pitch or yaw
    uint8_t crc_en;         //1-crc required, 0-crc not required
    uint8_t read_type;      //0-single read, 1-mass read
    uint8_t terminate;      //terminate
} sensor_state_var;

typedef struct{
    sensor_state_var var;
    sensor_status status;
    SENSOR_ID tag;
} sensor;
SENSOR_ID sensor_tags;

/*****************************PRIVATE VARIABLES*************************** */
static sensor sdi_imu, sdi_flash, sdi_radar, sdi_charger, sdi_rtc;
static sensor sdi_generic; //used this to keep track of sensor that is active at latest execution(for future use)
static sensor config;  //used this to set parameters for all other sensors
//ID sensor tags
static sensor sdi_imu = {
    .status.address = 'a',
    .tag = SENSOR_IMU
};
static sensor sdi_charger = {
    .status.address = 'b',
    .tag = SENSOR_CHARGER
};
static sensor sdi_radar = {
    .status.address = 'c',
    .tag = SENSOR_RADAR
};
static sensor sdi_flash = {
    .status.address = 'd',
    .tag = SENSOR_FLASH
};
static sensor sdi_rtc = {
    .status.address = 'e',
    .tag = SENSOR_RTC
};

static char sdi_rx[20];
static uint8_t command_rx_flag = 0;
static full_command command;
uint8_t len;


/********************PRIVATE FUNCTION DECLARATIONS********************/
static void _sdi_decode(full_command *, uint8_t);       //decode the command by length
static void _sdi_execute(uint32_t, full_command *);     //contains sensor parameters and function calls required to execute a command
static void _sdi_prarse(char *, full_command *, uint8_t);   //stores command received in an struct
static uint32_t __command_to_hex(full_command *command);    //convert a complete command to 32bit hex
static void __reset(sensor*);                               //helper function to reset given sensor

static void _sensor_set(sensor);    //set configuration for a given sensor
static void _sensor_start(sensor);  //start a sensor
static void _sensor_reset(sensor);  //reset a sensor
static void _sensor_send(sensor);   //send the measured value
static uint8_t _sensor_status_get(sensor);  //get status of a sensor

static void __service_request_msg(char , uint16_t, uint8_t); //used to respond address with time and number of mesured data
static void __abort_msg(char , uint16_t, uint8_t);  //same as service request message but for abort. for readability
static void __sensor_address_msg(char address); //used to respond address only

static void _address_changer(char, char);   //change address of a sensor
static uint8_t __address_mapper(char addr); //verify if an address belongs to a valid sensor

/***********PUBLIC VARIABLES******* */
volatile uint8_t sdi_api_imu_start, sdi_api_charger_start, sdi_api_rtc_start, sdi_api_flash_start, sdi_api_radar_start;
volatile uint8_t sdi_api_imu_crc_en, sdi_api_charger_crc_en, sdi_api_radar_crc_en, sdi_api_rtc_crc_en, sdi_api_flash_crc_en;

volatile uint8_t blocking_measurement = 0;      //not used
volatile uint8_t non_blocking_measurement = 0;  //not used
extern UART_HandleTypeDef hlpuart1;



/*************************PUBLIC FUNCTION DEFINITIONS************************* */

//receive ascii command until '!' is detected and return received no. of bytes
//the ! is ignored after parsing. and while decoding, 'a' is also ignored after storing it as adress in command

//cmd rx_flag is indicator that complete command is received. it is set to 1 after receiving a complete command and set to 0 by the command
//executing function to indicate executor is ready
void sdi_main(void)
{
	_sdi_prarse(sdi_rx, &command, len);
	_sdi_decode(&command, len);
	command_rx_flag = 0;
}

uint8_t sdi_cmd_receive(char * cmd)
{
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

uint32_t sdi_crc_ascii(uint16_t int_crc)
{
    uint32_t ascii = 0;

    uint8_t c1 = 0x40 | ((int_crc >> 12));
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
void api_status(SENSOR_ID tags, uint8_t data_ready, float* data, char* ascii_ptr)
{
    sensor_tags = __address_mapper(config.status.address);
    //for now, store data in both float and ascii format
	switch (sensor_tags)
		{
        case SENSOR_IMU:
            sdi_imu.status.data_ready = data_ready;
            // sdi_imu.status.crc = crc;
            sdi_imu.status.tx_data_ptr = ascii_ptr;
            sdi_imu.status.data[0] = data[0]; //roll
            sdi_imu.status.data[1] = data[1]; //pitch
            sdi_imu.status.data[2] = data[2]; //yaw
        break;
        case SENSOR_CHARGER:
            sdi_charger.status.data_ready = data_ready;
            // sdi_charger.status.crc = crc;
            sdi_charger.status.tx_data_ptr = ascii_ptr;
            sdi_charger.status.data[0] = data[0]; 
            sdi_charger.status.data[1] = data[1];
            sdi_charger.status.data[2] = data[2];
            sdi_charger.status.data[3] = data[3];
            sdi_charger.status.data[4] = data[4];
            sdi_charger.status.data[5] = data[5];
        break;
        case SENSOR_RADAR:
            sdi_radar.status.data_ready = data_ready;
            // sdi_radar.status.crc = crc;
            sdi_radar.status.tx_data_ptr = ascii_ptr;
            sdi_radar.status.data[0] = data[0]; //dist1
            sdi_radar.status.data[1] = data[1]; //str1
            sdi_radar.status.data[2] = data[2]; //dist2
            sdi_radar.status.data[3] = data[3]; //str2
            sdi_radar.status.data[4] = data[4]; //dist3
            sdi_radar.status.data[5] = data[5]; //str3
            break;
        case SENSOR_FLASH:
            sdi_flash.status.data_ready = data_ready;
            // sdi_flash.status.crc = crc;
            sdi_flash.status.tx_data_ptr = ascii_ptr;
            sdi_flash.status.data[0] = data[0];
        break;
        case SENSOR_RTC:
            sdi_rtc.status.data_ready = data_ready;
            // sdi_rtc.status.crc = crc;
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



/*
aMC1
c1 -> identify whether it is a "?" or character other than "?" (unnecessary. to be removed later)
address -> (a)
c2 -> M
C3 -> C
C4 -> 1
*/

/**********************PRIVATE FUNCTION DEFINITIONS********************** */
static void _sdi_prarse(char *cmd_raw, full_command *command, uint8_t cmd_len)
{
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

static void _sdi_decode(full_command *command, uint8_t len)
{
    uint32_t command_hex;// = __command_to_hex(command);
    switch (len)
    {
    case 1: //len1 address are a! or ?!. it is easy to separate them. ? is 3F. so 0x3F000000 executes ?!.
        if(command -> c1 == COMMAND_QUERY){
            //command_hex = __command_to_hex(command);
            _sdi_execute(0x3F000000, command);
        }else{
            _sdi_execute(0x61000000, command);
        }
    break;

    case 2:
        command_hex = __command_to_hex(command) & 0x00FF0000;
        _sdi_execute(command_hex, command);
    break;

    case 3: //for three length instruction with numbers, use command.c3 to get numerical value of the instruction
        if(command->c3 != 'C'){
            command_hex = __command_to_hex(command) & 0x00FF0000; //length 3 commands with numbers will be distinguished by 1 at fourth byte
            command_hex = command_hex | 0x00001000;
            _sdi_execute(command_hex, command);
        }else{
            command_hex = __command_to_hex(command) & 0x00FFFF00;
            _sdi_execute(command_hex, command);
        }
    break;

    case 4:
        //all length 4 commands will be identified by 1 at second byte
        command_hex = __command_to_hex(command) & 0x00FFFF00;
        command_hex = command_hex | 0x00000010;
        _sdi_execute(command_hex, command);
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

static void _sdi_execute(uint32_t instr, full_command *command)
{
    switch (instr){
    case (0x61000000): //acknowledge address a!
        /*code*/
        //use command.address to execute from here
        if(__address_mapper(command->address) != SENSOR_UNKNOWN){
            __sensor_address_msg(command->address);
        }
    break;
    case (0x3F000000): //address query ?!
        __sensor_address_msg(sdi_imu.status.address);
        __sensor_address_msg(sdi_charger.status.address);
        __sensor_address_msg(sdi_radar.status.address);
        __sensor_address_msg(sdi_flash.status.address);
        __sensor_address_msg(sdi_rtc.status.address);
    break;
    case(0x00411000): //change address command - aAb!
        if(__address_mapper(command->address) == SENSOR_UNKNOWN){
            break;//do nothing if original address mismatches before changing
        }
        _address_changer(command->address, command->c3);
        //check if changed address still maps to a valid sensor. if yes, revert back the changes
        if(__address_mapper(command->c3) == SENSOR_UNKNOWN) {
            _address_changer(command->c3, command->address);
            __sensor_address_msg(command->address);
        } else {
            __sensor_address_msg(command -> c3);
        }
    break;

    case(0x004D0000): //start measurement aM
        if(config.var.blocking == 1){ //comparing old state with existing state. do not allow new sensor request if old is not completed
            if(command->address != config.status.address){ //but allow old sensor request
                __abort_msg((char)command->address, 2, 1);
                break;
            }else{
                _sensor_reset(config); //reset sensor on identical repeated measure command like aM! after previous aM!
                __service_request_msg((char)(command->address), 2, 1);
            }
        }
        config.status.sub_address = 0;
        config.status.address = command->address;
        config.status.busy = 1;	//config by data read function
        config.status.data_ready = 0; //config by status function
        config.var.sub_reading = 0;
        config.status.error = 0; //config not implemented
        config.var.blocking = 1; //config here
        config.var.read_type = 0; //single byte read
        config.var.start = 1; //config here, checked on _sensor_set, modified on data read
        config.var.terminate = 0; //config not implemented
        config.var.crc_en = 0;

        _sensor_set(config); //save and check sensor configuration. if target sensor is already busy, nothing happens
        _sensor_start(config); //start mesaurement. if sensor busy, restart (in concurrent, if sensor busy then terminate)
    break;

    case(0x004D4300): //start measurement and request crc aMC
        if(config.var.blocking == 1){ 
            if(command->address != config.status.address){ 
                __abort_msg(config.status.address, 2, 1);
                break;
            }else{
                _sensor_reset(config);
                __service_request_msg((char)(command->address), 2, 1);
            }
        }
        config.status.sub_address = 0;
        config.status.address = command->address;
        config.status.busy = 1;
        config.status.data_ready = 0;
        config.status.error = 0;
        config.var.blocking = 1;
        config.var.sub_reading = 0;
        config.var.crc_en = 1;
        config.var.read_type = 1;
        config.var.start = 1;
        config.var.terminate = 0;
        _sensor_set(config);
        _sensor_start(config);
    break;

    case(0x00441000): //send data aD0...aD9
        config.status.address = command->address;
        if((_sensor_status_get(config) == 0)){ //if data not ready then respond with address and stop measurement
            __service_request_msg(config.status.address, 002,1);
        }
		if(_sensor_status_get(config) == 1){
            _sensor_send(config);
		    _sensor_reset(config);
        }
    break;

    case(0x004D1000): //additional measurements aM0....aM9
        if(config.var.blocking == 1){
            if(command->address != config.status.address){
                __abort_msg((char)command->address, 2, 1);
                break;
            }else{
                _sensor_reset(config);
                __service_request_msg((char)(command->address), 2, 1);
            }
        }
        config.status.sub_address = ((command -> c3) - '0'); //converting ascii number to integer value
        config.var.sub_reading = 1;

        config.status.address = command->address;
        config.status.busy = 1;
        config.status.data_ready = 0;
        config.status.error = 0;
        config.var.blocking = 1; 
        config.var.read_type = 0;
        config.var.start = 1;
        config.var.terminate = 0; 
        config.var.crc_en = 0;

        _sensor_set(config); 
        _sensor_start(config);
    break;

    case(0x004D4310): //additional measurements and request CRC aMC1..aMC9
        if(config.var.blocking == 1){
            if(command->address != config.status.address){
                __abort_msg((char)command->address, 2, 1);
                break;
            }else{
                _sensor_reset(config);
                __service_request_msg((char)(command->address), 2, 1);
            }
        }
        config.status.sub_address = (command -> c4) - '0'; //converting ascii  number to integer value;
        config.var.sub_reading = 1;

        config.status.address = command->address;
        config.status.busy = 1;
        config.status.data_ready = 0;
        config.status.error = 0;
        config.var.blocking = 1; 
        config.var.read_type = 0;
        config.var.start = 1;
        config.var.terminate = 0; 
        config.var.crc_en = 1;

        _sensor_set(config); 
        _sensor_start(config);

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

    case(0x00431000): //additional concurrent measurements aC0...aC9
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

static void _sensor_set(sensor config)
{
    sensor_tags = __address_mapper(config.status.address);
    switch (sensor_tags)
    {
        case SENSOR_IMU:
			sdi_imu = config;
            if(sdi_imu.status.sub_address > 3 && sdi_imu.var.sub_reading == 1){
                sdi_imu.status.sub_address = 2;
                __sensor_address_msg(sdi_imu.status.address);
            }
        break;
        case SENSOR_CHARGER:
            sdi_charger = config;
            if(sdi_charger.status.sub_address > 6 && sdi_charger.var.sub_reading == 1){
                sdi_charger.status.sub_address = 5;
                __sensor_address_msg(sdi_charger.status.address);
            }
        break;
        case SENSOR_RADAR:
            sdi_radar = config;
            if(sdi_radar.status.sub_address > 6 && sdi_radar.var.sub_reading == 1){
                sdi_radar.status.sub_address = 5;
                __sensor_address_msg(sdi_radar.status.address);
            }
        break;
        case SENSOR_FLASH:
            sdi_flash= config;
        break;
        case SENSOR_RTC:
           sdi_rtc= config;
            if(sdi_rtc.status.sub_address > 6 && sdi_rtc.var.sub_reading == 1){
                sdi_rtc.status.sub_address = 5;
                __sensor_address_msg(sdi_rtc.status.address);
            }
        break;
        default:
        break;
    }
    sdi_generic = config;
}

void _sensor_start(sensor config)
{
    sensor_tags = __address_mapper(config.status.address);
    switch (sensor_tags)
    {
        case SENSOR_IMU:
            sdi_api_imu_start = 1;
            sdi_api_imu_crc_en = sdi_imu.var.crc_en;
            __service_request_msg(config.status.address,002,1);//send back a response
        break;
        case SENSOR_CHARGER:
            sdi_api_charger_start = 1;
            sdi_api_charger_crc_en = sdi_charger.var.crc_en;
            __service_request_msg(config.status.address,002,1);
        break;
        case SENSOR_RADAR:
            sdi_api_radar_start = 1;
            sdi_api_radar_crc_en = sdi_radar.var.crc_en;
            __service_request_msg(config.status.address,002,1);
        break;
        case SENSOR_FLASH:
			sdi_api_flash_start = 1;
            sdi_api_flash_crc_en = sdi_flash.var.crc_en;
            __service_request_msg(config.status.address,002,1);
        break;
        case SENSOR_RTC:
            sdi_api_rtc_start = 1;
            sdi_api_rtc_crc_en = sdi_rtc.var.crc_en;
            __service_request_msg(config.status.address,002,1);
        break;
        default:
            __abort_msg(config.status.address, 2, 1);
        break;
    }
}

void _sensor_reset(sensor conf)
{
    sensor_tags = __address_mapper(config.status.address);
	switch (sensor_tags)
    {
        case SENSOR_IMU:
            sdi_api_imu_start = 0;
            __reset(&sdi_imu);
        break;
        case SENSOR_CHARGER:
            sdi_api_charger_start = 0;
            __reset(&sdi_charger);
        break;
        case SENSOR_RADAR:
            sdi_api_radar_start = 0;           
            __reset(&sdi_radar); 
        break;
        case SENSOR_FLASH:
            sdi_api_flash_start = 0;           
            __reset(&sdi_flash); 
        break;
        case SENSOR_RTC:
            sdi_api_rtc_start = 0;       
            __reset(&sdi_rtc); 
        break;
        default:
        break;
    }
		__reset(&config);
}


//measurement status - the sensor must have start bit 1 and must have data ready 1 to give a valid status  
static uint8_t _sensor_status_get(sensor config)
{
    sensor_tags = __address_mapper(config.status.address);
	switch (sensor_tags)
    {
        case SENSOR_IMU:
            if(sdi_imu.status.data_ready && sdi_imu.var.start) return 1; else return 0;
        break;
        case SENSOR_CHARGER:
            if(sdi_charger.status.data_ready && sdi_charger.var.start) return 1; else return 0;
        break;
        case SENSOR_RADAR:
            if(sdi_radar.status.data_ready && sdi_radar.var.start) return 1; else return 0;
        break;
        case SENSOR_FLASH:
            if(sdi_flash.status.data_ready && sdi_flash.var.start) return 1; else return 0;
        break;
        case SENSOR_RTC:
            if(sdi_rtc.status.data_ready && sdi_rtc.var.start) return 1; else return 0;
        break;
        default:
            return 0;
        break;
    }
}

uint32_t imu_crc, radar_crc, charger_crc, rtc_crc, flash_crc;
void _sensor_send(sensor config)
{
    char tx_buf[100];
    uint32_t crc_ascii;
    uint8_t crc_buf[3];
    char crc_dbg[16];
    sensor_tags = __address_mapper(config.status.address);
	switch (sensor_tags)
    { //instead of if, multiplication with 1 or 0 is done to reduce no. of if statements while making code as much as readable
        case SENSOR_IMU:
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)&sdi_imu.status.address, sizeof(char), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_imu.status.tx_data_ptr + ((8 * sdi_imu.status.sub_address) * sdi_imu.var.sub_reading),
                                strlen(sdi_imu.status.tx_data_ptr) - 16 * sdi_imu.var.sub_reading, HAL_MAX_DELAY);
            if(sdi_imu.var.crc_en == 1){
                //to cover crc claculation case for both aM and aM0 type of data
                sdi_imu.status.crc = sdi_crc16(sdi_imu.status.tx_data_ptr + 8 * sdi_imu.status.sub_address,
                                                strlen(sdi_imu.status.tx_data_ptr)-16 * sdi_imu.var.sub_reading);
                crc_ascii = sdi_crc_ascii(sdi_imu.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", sizeof("\r\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case SENSOR_CHARGER:
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)&sdi_charger.status.address, sizeof(char), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_charger.status.tx_data_ptr + ((8 * sdi_charger.status.sub_address) * sdi_charger.var.sub_reading), 
                                strlen(sdi_charger.status.tx_data_ptr) - 40 * sdi_charger.var.sub_reading, HAL_MAX_DELAY);
           if(sdi_charger.var.crc_en == 1){
                sdi_charger.status.crc = (uint32_t) sdi_crc16(sdi_charger.status.tx_data_ptr + 8 * sdi_charger.status.sub_address,
                                                                strlen(sdi_charger.status.tx_data_ptr)-16 * sdi_charger.var.sub_reading);
                crc_ascii = sdi_crc_ascii(sdi_charger.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", sizeof("\r\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case SENSOR_RADAR:
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)&sdi_radar.status.address, sizeof(char), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_radar.status.tx_data_ptr + ((8 * sdi_radar.status.sub_address) * sdi_radar.var.sub_reading),
                                strlen(sdi_radar.status.tx_data_ptr) - 40 * sdi_radar.var.sub_reading, HAL_MAX_DELAY);
            if(sdi_radar.var.crc_en == 1){
                sdi_radar.status.crc = (uint32_t) sdi_crc16(sdi_radar.status.tx_data_ptr + 8 * sdi_radar.status.sub_address,
                                                            strlen(sdi_radar.status.tx_data_ptr)-16 * sdi_radar.var.sub_reading);
                crc_ascii = sdi_crc_ascii(sdi_radar.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8) & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", sizeof("\r\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case SENSOR_FLASH:
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)'d', sizeof('d'), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)sdi_flash.status.data, sizeof(uint32_t), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", sizeof("\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        case SENSOR_RTC:
            SDI_TX();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)&sdi_rtc.status.address, sizeof(char), HAL_MAX_DELAY);
            HAL_UART_Transmit(&hlpuart1, sdi_rtc.status.tx_data_ptr + ((8 * sdi_rtc.status.sub_address) * sdi_rtc.var.sub_reading),
                                strlen(sdi_rtc.status.tx_data_ptr) - 40 * sdi_rtc.var.sub_reading, HAL_MAX_DELAY);
            if(sdi_rtc.var.crc_en == 1){
                sdi_rtc.status.crc = (uint32_t) sdi_crc16(sdi_rtc.status.tx_data_ptr + 8 * sdi_rtc.status.sub_address,
                                                            strlen(sdi_rtc.status.tx_data_ptr)-16 * sdi_rtc.var.sub_reading);
                crc_ascii = sdi_crc_ascii(sdi_rtc.status.crc);
                crc_buf[0] = (crc_ascii >> 16) & 0xFF;
                crc_buf[1] = (crc_ascii >> 8)  & 0xFF;
                crc_buf[2] =  crc_ascii & 0xFF;
                //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"crc: ", strlen("crc: "), HAL_MAX_DELAY);
                HAL_UART_Transmit(&hlpuart1, crc_buf, 3, HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", sizeof("\r\n"), HAL_MAX_DELAY);
            SDI_RX();
        break;
        default:
        break;
    }
		_sensor_reset(config);
}
static void _address_changer(char old_addr, char new_addr)
{
    sensor_tags = __address_mapper(old_addr);
    switch (sensor_tags)
    {
    case SENSOR_IMU:
        sdi_imu.status.address = new_addr;
    break;
    case SENSOR_CHARGER:
        sdi_charger.status.address = new_addr;
    break;
    case SENSOR_RADAR:
        sdi_radar.status.address = new_addr;
    break;
    case SENSOR_FLASH:
        sdi_flash.status.address = new_addr;
    break;
    case SENSOR_RTC:
        sdi_rtc.status.address = new_addr;
    break;
    
    default:
        break;
    }
}

/**********************INTERNAL HELPER FUNCTIONS********************* */
static uint32_t __command_to_hex(full_command *command)
{
    return ((uint32_t)command->address << 24) | ((uint32_t)command->c2 << 16) |
           ((uint32_t)command->c3 << 8)  | ((uint32_t)command->c4);
}

static void __reset(sensor* sensor)
{
	sensor->status.busy = 0;
    sensor->status.sub_address = 0;
    sensor->status.crc = 0;
    sensor->status.tx_data_ptr = 0;
	sensor->status.data[0] = 0;
	sensor->status.data[1] = 0;
	sensor->status.data[2] = 0;
	sensor->status.data[3] = 0;
	sensor->status.data[4] = 0;
	sensor->status.data[5] = 0;
	sensor->status.data_ready = 0;
	sensor->status.error = 0;
	sensor->var.blocking = 0;
    sensor->var.sub_reading = 0;
	sensor->var.read_type = 0;
	sensor->var.start = 0;
	sensor->var.terminate = 0;
}

static void __service_request_msg(char address, uint16_t time, uint8_t number)
{
    char tx_buf[10];
    snprintf(tx_buf, sizeof(tx_buf), "%c%03d%d\r\n",address, time, number );
    SDI_TX();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
    SDI_RX();
}
static void __abort_msg(char address, uint16_t time, uint8_t number)
{ //same as __service_request_msg. for readablility
    char tx_buf[10];
    snprintf(tx_buf, sizeof(tx_buf), "%c%03d%d\r\n",address, time, number );
    SDI_TX();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
    SDI_RX();
}
static void __sensor_address_msg(char address)
{
    char tx_buf[10];
    snprintf(tx_buf, sizeof(tx_buf), "%c\r\n", address);
    SDI_TX();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
    SDI_RX();
}

static uint8_t __address_mapper(char addr)
{
    if(addr == sdi_imu.status.address ) return SENSOR_IMU;
    if(addr == sdi_charger.status.address) return SENSOR_CHARGER;
    if(addr == sdi_radar.status.address) return SENSOR_RADAR;
    if(addr == sdi_flash.status.address) return SENSOR_FLASH;
    if(addr == sdi_rtc.status.address) return SENSOR_RTC;

    return SENSOR_UNKNOWN;
}
