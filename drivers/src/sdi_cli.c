#include "sdi_cli.h"

#define SDI_ERROR 1

static char sdi_rx[20];
static uint8_t command_rx_flag = 0;
full_command command;
uint8_t len;
extern UART_HandleTypeDef hlpuart1;

typedef struct{
    uint8_t address;
    uint8_t busy; //1 - busy, 0 - idle
    uint8_t data_ready;
    uint8_t error;
}sensor_state;

typedef enum{
    STATE_IDLE = 0,
    STATE_MEASURING = 1,
    STATE_DATA_READY = 2,
    STATE_DATA_TRANSFER = 3,
    STATE_END = 5 //optional
}sdi_states;
sdi_states next_state, return_state;
sdi_states system_state = STATE_IDLE; //initial system state

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
    command->address = (cmd_raw[0] >= '0' && cmd_raw[0] <= '9') ? cmd_raw[0] : '0'; //store invalid address. execution will check and cancel
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
        if(system_state != STATE_IDLE) break;
		//uint8_t prev_addr = command->address;
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"change addr aAb!", sizeof("change addr aAb!"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x004D0000): //start measurement aM
        //code
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aM", sizeof("not implemented aM"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x004D4300): //start measurement and request crc aMC
        //code
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aMC", sizeof("not implemented aMC"), HAL_MAX_DELAY);
        SDI_RX();
    break;

    case(0x00441000): //send data aD0...aD9
        SDI_TX();
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"not implemented aDx", sizeof("not implemented aDx"), HAL_MAX_DELAY);
        SDI_RX();
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
