#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "utils.h"
#include "uart.h"
#include "handler.h"
#include "ff.h"
#include "diskio.h"
#include "twi.h"
#include "main.h"
#include "xitoa.h"
#include "timer.h"
#include <avr/eeprom.h>

//globals - for now! - try to cut down on these
uint8_t  Stop = 1;
uint8_t  display_flag = 0;
uint8_t  x_gt_y = 0;
uint8_t  Laser_active = 0;
uint8_t  FSMutex = 0;

uint16_t x_num, y_num;  //the max index for x and y
volatile uint16_t index_x, index_y; // the current index of x and y
uint8_t  gs_value, bytes_per_panel_frame, row_compress, ident_compress;
uint8_t  num_panels = 0;
uint8_t  x_mode, y_mode;
volatile uint16_t frame_num = 0;
uint16_t frame_num_old = 999;  //just chosen at random
int16_t  function_X[FUNCTION_LENGTH];
int16_t  function_Y[FUNCTION_LENGTH];
uint16_t function_counter_x = 0;
uint16_t function_counter_y = 0;
uint32_t func_global_counter_x = 0;
uint32_t func_global_counter_y = 0;

uint16_t functionX_rate = FUNCTION_RATE;
uint16_t functionY_rate = FUNCTION_RATE;

uint8_t  laserPattern[96];
int8_t   gain_x, gain_y, bias_x, bias_y;
int16_t  X_val, Y_val;
int16_t  X_pos_index, Y_pos_index;
uint16_t trigger_rate = 200;

uint32_t start_block; // start block of a pattern on SD-card
uint32_t funcSize_x = 2*FUNCTION_LENGTH; //function file size
uint32_t funcSize_y = 2*FUNCTION_LENGTH;
FIL      file1, file2, file3, file4;       // File object
//file1 pattern file; file 2 function file for x channel
//file2 function file for y channel, file 4 for SD.mat or arena config file
FATFS    fatfs;       // File system object
//max frame size = 32bpp * 4rows * 12col = 1536
//uint8_t  Buff[1536];  // File System working buffer
//uint8_t  buff4Panel[256];   //working buffer for programming panel

uint8_t  adrMap[129]; // panel twi address mapping, we can have same address in different channels
uint8_t  quiet_mode_on=0;

uint16_t func_ID_X = 0;
uint16_t func_ID_Y = 0;

uint16_t loadXBuffer = 2*FUNCTION_LENGTH;
uint16_t loadYBuffer = 2*FUNCTION_LENGTH;

static const uint8_t VERSION[] = "1.0\0";
static const uint8_t SDInfo[] = "SD.mat\0";


/*---------------------------------------------------------*/
/* TWIC Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIC_TWIM_vect) {
    TWI_MasterInterruptHandler(&twi1);
}

/*---------------------------------------------------------*/
/* TWID Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWID_TWIM_vect) {
    TWI_MasterInterruptHandler(&twi2);
}

/*---------------------------------------------------------*/
/* TWIE Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIE_TWIM_vect) {
    TWI_MasterInterruptHandler(&twi3);
}

/*---------------------------------------------------------*/
/* TWIF Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIF_TWIM_vect) {
    TWI_MasterInterruptHandler(&twi4);
}




int main(void) {
    uint8_t sta, res, b1, temp;
    uint16_t cnt;
    uint16_t message_length;
    uint16_t lcv;
    uint8_t  tempBuff[128];
	uint8_t workingModes;
	
	workingModes = eeprom_read_byte(work_mode);
	
    TWI_MasterCreateBuff(&twi1,workingModes);
	TWI_MasterCreateBuff(&twi2,workingModes);
	TWI_MasterCreateBuff(&twi3,workingModes);
	TWI_MasterCreateBuff(&twi4,workingModes);
    
// Initialize TWI master #1
    TWI_MasterInit(&twi1,
            &TWIC,
            TWI_MASTER_INTLVL_LO_gc,
            TWI_BAUDSETTING);
    
// Initialize TWI master #2
    TWI_MasterInit(&twi2,
            &TWID,
            TWI_MASTER_INTLVL_LO_gc,
            TWI_BAUDSETTING);
    
// Initialize TWI master #3
    TWI_MasterInit(&twi3,
            &TWIE,
            TWI_MASTER_INTLVL_LO_gc,
            TWI_BAUDSETTING);
    
// Initialize TWI master #4
    TWI_MasterInit(&twi4,
            &TWIF,
            TWI_MASTER_INTLVL_LO_gc,
            TWI_BAUDSETTING);
    
// Initialize the rest of the system
	createRxBuff(workingModes);
    init_all();

    
    /* Join xitoa module to uart module */
    xfunc_out = (void (*)(char))uart_put;
    
    for (lcv = 0; lcv < FUNCTION_LENGTH; lcv++)
    { function_X[lcv] = function_Y[lcv] = 10;  }// here we use 10 as the equivalent for 1 V.
	
	
	//initialize laser pattern
	for (lcv = 0; lcv < 96; lcv++)
	{ 
		if (((lcv >=0) && (lcv <= 23)) || ((lcv >= 48)&&(lcv <= 71)))
	        laserPattern[lcv] = 1;
		else
			laserPattern[lcv] = 0;
	}
	
    
    //initilializations
    X_pos_index = Y_pos_index = index_x = index_y = 0;
    bias_x = bias_y = 0;
    gain_x = gain_y = 10;
    x_mode = y_mode = 0;
    X_val = Y_val = 0;
    gs_value = 1;
    row_compress = 0;
    ident_compress = 0; // enable this to substitute simpler panael pattern for uniform pattern patches
    
    temp = eeprom_read_byte(arena_config);
    if (temp == 0xff)     //there is no configuration file and use default value 
	{// create default panel mapping
		for (b1 = 0; b1 <= 128; b1++) {
			chMap[b1] = b1 % 4;
			if (b1 && (chMap[b1] == 0))
				chMap[b1] = 4;
			adrMap[b1] = b1; // panel address identity mapping
		}
	} else
	{//load panel mapping from EEPROM	
		for (b1 = 0; b1 <= 128; b1++) {
			chMap[b1] = eeprom_read_byte(arena_config + b1);
			adrMap[b1] = b1; // panel address identity mapping
		}
    }
    
    xputs(PSTR("\nMain Controller Works\n"));
    
    // get the fat file system mounted
    ledWrite(LED1, ON);
    sta = STA_NOINIT;
    while (sta & STA_NOINIT) {
        sta = disk_status(0);
        if(sta & STA_NODISK) {
            xputs(PSTR("Insert SD card"));
            uart_crlf();
            while (sta & STA_NODISK) {
                sta = disk_status(0);
            }
        }
        if(sta & STA_PROTECT) {
            xputs(PSTR("SD card is Write Protected!\n"));
        }
        // Initialize SD Card, do 4 attempts
        for(b1 = 0; b1 < 4; ) {
            sta = disk_initialize(0);
            if(sta & STA_NOINIT) b1++;
            else break;
            _delay_ms(50);
        }
        if(sta & STA_NOINIT) {
            xputs(PSTR("Initialization failed!!!\n"));
            sta = disk_status(0);
            while (!(sta & STA_NODISK)) {
                sta = disk_status(0);
            }
        }
    }
    xputs(PSTR("SD card is initialized\n"));
    
    if (disk_ioctl(0, MMC_GET_TYPE, &b1) == RES_OK) {
        xputs(PSTR("Card type: "));
        switch(b1) {
            case CT_MMC: xputs(PSTR("MMC\n")); break;
            case CT_SD1: xputs(PSTR("SD\n")); break;
            case CT_SD2: xputs(PSTR("SD2\n")); break;
            case CT_SDC: xputs(PSTR("SDC\n")); break;
            case CT_BLOCK: xputs(PSTR("BLK\n")); break;
            case CT_SD2_BLK: xputs(PSTR("SDHC\n")); break;
            default: xputs(PSTR("???\n"));
        }
    }
    
    xputs(PSTR("Initializing FAT Disk..."));
    res = f_mount(0, &fatfs);
    switch(res) {
        case RES_OK: xputs(PSTR(" FAT OK!\n")); break;
        case RES_ERROR: xputs(PSTR(" ERROR!\n")); break;
        case RES_WRPRT: xputs(PSTR(" WRITE PROTECTED!\n")); break;
        case RES_NOTRDY: xputs(PSTR(" NOT READY!\n")); break;
        default: xputs(PSTR("???\n"));
    }
    
    
    i2cMasterSend(0x00, 8, ALL_OFF);
    ledWrite(LED1, OFF);
    ledBlink();
    // Here the EEPROM location 0 is used as a switch between Controller and PCDump mode
    // An alternative is to base the switch on the SD config file
    if (workingModes == 0xff) {
        uint8_t msg_buffer[55];
		xputs(PSTR("Current working mode is the Controller mode!\n"));
		
        while(1) {  // this is the main loop, here we wait for communication from PC over UART
            if (uart_test()) {
                message_length = fill_Rx_buffer(&msg_buffer[0]);
                switch(message_length) {
                    case 1:  // if length 1, then decode...
                        handle_message_length_1(&msg_buffer[0]);
                        break;
                    case 2: // if length 2, then decode, could be reset, display num, or change pat
                        handle_message_length_2(&msg_buffer[0]);
                        break;
                    case 3: // if length 3, then decode...address change or ...
                        handle_message_length_3(&msg_buffer[0]);
                        break;
                    case 5: // if length 5, then decode, set x,y index, or set gain, bias
                        handle_message_length_5(&msg_buffer[0]);
                        break;
                    case 12: //if length 12, then set laser trigger pattern
                        handle_message_length_12(&msg_buffer[0]);
                        break;
                    default:
                        i2cMasterSend(0x00, 8, ERROR_CODES[6]);
                } //end of switch
            }// end of if, goes to top if nothing received on UART
            
            // at bottom of while(1) loop, check to see if stop is 0, then unpdate display if the frame has changed.
            if (Stop == 0){  //only send out new pattern if the pattern index has changed
                if (frame_num != frame_num_old) {
                    frame_num_old = frame_num; //update the 'old' frame number
                    fetch_display_frame(frame_num);
                }
            }
        }
    } else {
        uint8_t msg_buffer[1550];
		xputs(PSTR("Current working mode is the PC dumping mode!\n"));
        while(1) {  // this is the main loop, here we wait for communication from PC over UART
            if (uart_test()) {
                message_length = fill_Rx_buffer(&msg_buffer[0]);
                switch(message_length) {
                    case 1:  // if length 1, then decode...
                        handle_message_length_1(&msg_buffer[0]);
                        break;
                    //case 2: // if length 2, then decode, could be reset, display num, or change pat
                    //    handle_message_length_2(&msg_buffer[0]);
                    //    break;
                    case 50: //
                        display_dumped_frame(&msg_buffer[0]);
                        break;
                    default:
xprintf(PSTR("message_length = %u\n"), message_length);					
                        i2cMasterSend(0x00, 8, ERROR_CODES[6]);
                } //end of switch
            }// end of if, goes to top if nothing received on UART
        }
    }
	
	TWI_MasterReleaseBuff(&twi1);
	TWI_MasterReleaseBuff(&twi2);
	TWI_MasterReleaseBuff(&twi3);
	TWI_MasterReleaseBuff(&twi4);
	releaseRxBuff();
}

void handle_message_length_1(uint8_t *msg_buffer) {
    uint8_t CMD[2];
    uint8_t i;
    
    switch(msg_buffer[0]) {
        case 0x20:  //Start display: 0x20
            //set these to zero so that start at beginning of function - useful for putting in a set amount of expansion
            function_counter_x = 0;
            function_counter_y = 0;
            Stop = 0;
            display_flag = 0;  //clear the display flag
            Reg_Handler(Update_display, UPDATE_RATE, 1, 1);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0); //initilize the 2 and 3 priority interupts to a fast rate so that
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0); // the countdown is fast until the setting of the next rate
            break;                        //by the Update_display interupt.
            
        case 0x30: //stop display
            Stop = 1;
            //turn off the interupts
            Reg_Handler(Update_display, UPDATE_RATE, 1, 0);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
            
            break;
            
        case 0x25:  //Start display & trigger - same as regular, but this also does trigger
            //set these to zero so that start at beginning of function - useful for putting in a set amount of expansion
            function_counter_x = 0;
            function_counter_y = 0;
            Stop = 0;
            display_flag = 0;  //clear the display flag
            Reg_Handler(Update_display, UPDATE_RATE, 1, 1);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
            Reg_Handler(toggle_trigger, (uint32_t)OVERFLOW_RATE/trigger_rate, 0, 1); //turn on the trigger toggle
            break;
            
        case 0x35: //stop display & trigger - same as regular, but this also does trigger
            Stop = 1;
            //turn off the interupts
            Reg_Handler(Update_display, UPDATE_RATE, 1, 0);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
            Reg_Handler(toggle_trigger, OVERFLOW_RATE/trigger_rate, 0, 0); //turn off the trigger toggle
			digitalWrite(3,LOW);    //set the output to low
            break;
            
        case 0x00:  i2cMasterSend(0x00, 8, ALL_OFF); break;
        case 0x40:  i2cMasterSend(0x00, 24, G_LEVELS[0]); break;
        case 0x41:  i2cMasterSend(0x00, 24, G_LEVELS[1]); break;
        case 0x42:  i2cMasterSend(0x00, 24, G_LEVELS[2]); break;
        case 0x43:  i2cMasterSend(0x00, 24, G_LEVELS[3]);   break;
        case 0x44:  i2cMasterSend(0x00, 24, G_LEVELS[4]); break;
        case 0x45:  i2cMasterSend(0x00, 24, G_LEVELS[5]); break;
        case 0x46:  i2cMasterSend(0x00, 24, G_LEVELS[6]); break;
        case 0x47:  i2cMasterSend(0x00, 24, G_LEVELS[7]); break;
        case 0xFF:  i2cMasterSend(0x00, 8, ALL_ON); break;
        case 0x50:  ledBlink(); break;
        case 0x60:  SystemReset();  break;
        case 0x70:  benchmark_pattern(); break;
        case 0x90:  i2cMasterSend(0x00, 32, G_LEVELS_16[0]); break;
        case 0x91:  i2cMasterSend(0x00, 32, G_LEVELS_16[1]); break;
        case 0x92:  i2cMasterSend(0x00, 32, G_LEVELS_16[2]); break;
        case 0x93:  i2cMasterSend(0x00, 32, G_LEVELS_16[3]); break;
        case 0x94:  i2cMasterSend(0x00, 32, G_LEVELS_16[4]); break;
        case 0x95:  i2cMasterSend(0x00, 32, G_LEVELS_16[5]); break;
        case 0x96:  i2cMasterSend(0x00, 32, G_LEVELS_16[6]); break;
        case 0x97:  i2cMasterSend(0x00, 32, G_LEVELS_16[7]); break;
        case 0x98:  i2cMasterSend(0x00, 32, G_LEVELS_16[8]); break;
        case 0x99:  i2cMasterSend(0x00, 32, G_LEVELS_16[9]); break;
        case 0x9A:  i2cMasterSend(0x00, 32, G_LEVELS_16[10]); break;
        case 0x9B:  i2cMasterSend(0x00, 32, G_LEVELS_16[11]); break;
        case 0x9C:  i2cMasterSend(0x00, 32, G_LEVELS_16[12]); break;
        case 0x9D:  i2cMasterSend(0x00, 32, G_LEVELS_16[13]); break;
        case 0x9E:  i2cMasterSend(0x00, 32, G_LEVELS_16[14]); break;
        case 0x9F:  i2cMasterSend(0x00, 32, G_LEVELS_16[15]); break;
        
        case 0x10:  // turn laser on
            Laser_active = 1;
            break;
            
        case 0x11:  // turn laser off
            Laser_active = 0;
            // turn off the lines that may be connected
            digitalWrite(2, LOW);
            break;
            
        case 0x12:  // turn on compression for identical elements
            ident_compress = 1;
            break;
            
        case 0x13:  // turn off compression for identical elements
            ident_compress = 0;
            break;
            
        case 0x14:  //synchronize the SDInfo.mat with the one in the PC
            dump_mat();
            break;
            
        case 0x15:  //get current version
            xprintf(PSTR("Current version number is %s.\n"), VERSION);
            break;
            
        case 0x16:   //show the bus number
            for (i = 1; i <= 128; i++) {
                CMD[0] = 0xFE; CMD[1] = chMap[i];
                i2cMasterSend(i, 2, CMD);
            }
            break;
            
        case 0x17:  // turn on quiet_mode, no message sent out
            quiet_mode_on = 1;
            break;
            
        case 0x18:  // turn off quiet_mode, essage sent out
            quiet_mode_on = 0;
            break;
            
        case 0x19:  // get ADC value from ADC0 to debug ADC
            xprintf(PSTR("ADC_value =  %d:\n"), analogRead(0));
            break;
			
		case 0x21:	// working mode 1 = default mode = controller mode
			eeprom_write_byte(work_mode,0xff);
			xprintf(PSTR("Reset controller to work in the controller mode!\n"));
			break;

		case 0x22:
		    eeprom_write_byte(work_mode,0x00);
			xprintf(PSTR("Reset controller to work in the PC dumping mode!\n"));
			break;
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[1]);
    }
}

void handle_message_length_2(uint8_t *msg_buffer) {
    uint8_t argument_byte;
    
    argument_byte = msg_buffer[1];
    switch(msg_buffer[0]) {
        case 0x01: //sends a reset command out to panel at taget address
            i2cMasterSend(argument_byte, 2, RESET);
            break;
            
        case 0x02: //sends a display command out to panel at taget address
            i2cMasterSend(argument_byte, 2, DISPLAY);
            break;
            
        case 0x03:   //set pattern
            set_pattern(argument_byte);      //pattern x - specified in argument_byte
            break;
            
        case 0x04: // this is an ADC test command
            test_ADC(argument_byte);  //here argument_byte is actually a channel, 0-7 to test ADC/DAC system
            break;
            
        case 0x05: // this is a DIO test command
            test_DIO(argument_byte);  //here argument_byte is actually a channel, 0-7 to test ADC/DAC system
            break;
            
        case 0x06: // this is a trigger rate set command
            trigger_rate = argument_byte*2;  //here argument_byte is a trigger rate
            break;
            
        case 0x07:   //flash panel#
            flash_panel(argument_byte); //here argument_byte is actually a panel number
            break;
            
        case 0x08:   //eeprom panel#
            eeprom_panel(argument_byte); //here argument_byte is actually a panel number
            break;
			
        case 0x09:   //set arena configuration
            set_hwConfig(argument_byte);      //configuration x - specified in argument_byte
            break;	
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[2]);
    }
}

void handle_message_length_3(uint8_t *msg_buffer) {
    uint8_t target_panel_addr;
    uint8_t CMD[2];
    uint16_t funcX_freq, funcY_freq;
    
    switch(msg_buffer[0]) {
        case 0xFF:  //address panel
            target_panel_addr = msg_buffer[1];  //put in error check, in range < 127
            //sends a reset command out to panel at taget address
            
            //Since the panel can be located in any of the four channels, so the command should be sent to all channels
            while (twi1.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi1, target_panel_addr, RESET, 2);
            while (twi2.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi2, target_panel_addr, RESET, 2);
            while (twi3.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi3, target_panel_addr, RESET, 2);
            while (twi4.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi4, target_panel_addr, RESET, 2);
            //i2cMasterSend(target_panel_addr, 2, RESET);
            
            _delay_ms(2200);
            CMD[0] = 0xFF; CMD[1] = msg_buffer[2];   //send change address command
            
            //Since the panel can be located in any of the four channels, so the command should be sent to all channels
            while (twi1.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi1, target_panel_addr, CMD, 2);
            while (twi2.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi2, target_panel_addr, CMD, 2);
            while (twi3.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi3, target_panel_addr, CMD, 2);
            while (twi4.status != TWIM_STATUS_READY);
            TWI_MasterWrite(&twi4, target_panel_addr, CMD, 2);
            //i2cMasterSend(target_panel_addr, 2, CMD);
            
            _delay_ms(50);
            //reset all panels again
            i2cMasterSend(0, 2, RESET);
            
            break;
            
        case 0x10:
            x_mode = msg_buffer[1];
            y_mode = msg_buffer[2];
            //put in an error message if value is not 0, 1, or 2.
            break;
            
        case 0x15:   //this is a set position function
            if (msg_buffer[2] == 0){
                set_default_func(msg_buffer[1]);}
            else
                set_pos_func(msg_buffer[1], msg_buffer[2]);
            break;
            
        case 0x20:   //this is a set velocity function
            if (msg_buffer[2] == 0){
                set_default_func(msg_buffer[1]);}
            else
                set_vel_func(msg_buffer[1], msg_buffer[2]);
            break;
            
        case 0x25: // this is a set function generator frequency
            funcX_freq = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            functionX_rate = OVERFLOW_RATE/funcX_freq;
            if (quiet_mode_on == 0)
                xprintf(PSTR("function X update frequency = %u.\n"), funcX_freq);
            Update_Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);
            break;
            
        case 0x30: // this is a set function generator frequency
            funcY_freq = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            functionY_rate = OVERFLOW_RATE/funcY_freq;
            if (quiet_mode_on == 0)
                xprintf(PSTR("function Y update frequency = %u.\n"), funcY_freq);
            Update_Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);
            break;
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[3]);
    }
}

void handle_message_length_5(uint8_t *msg_buffer) {
    switch(msg_buffer[0]) {
        case 0x70:   //put in a bunch of type casts, because of mysterious error dealling with frame index above 128.
            //'set_position'
            index_x = (uint8_t)msg_buffer[1] + (256*(uint8_t)msg_buffer[2]);
            index_y = (uint8_t)msg_buffer[3] + (256*(uint8_t)msg_buffer[4]);
            
            X_pos_index = index_x; // these only used during position func. control mode, but
            Y_pos_index = index_y; //update here should not slow things down much and no need for sep. function.
            frame_num = index_y* x_num + index_x;
            display_flag = 0;  //clear the display flag
            if (quiet_mode_on == 0)
                xprintf(PSTR("set_position: index_x= %u,  index_y= %u, and frame_num= %u\n"), index_x, index_y, frame_num);
            fetch_display_frame(frame_num);
            break;
            
        case 0x71:
            //'send_gain_bias', all of these are signed byte values
            gain_x = msg_buffer[1];
            bias_x = msg_buffer[2];
            gain_y = msg_buffer[3];
            bias_y = msg_buffer[4];
            break;
            
            
        default:
            i2cMasterSend(0x00, 8, ERROR_CODES[4]);
    }
}


//load laser trigger pattern. Laer patter has 96 bytes, but since
//the value is either 0 or 1, we can combined them in 12 bytes to 
//save serial communicaiton time

void handle_message_length_12(uint8_t *msg_buffer)
{   
	uint8_t i,j, tempVal;
	
	for (i=0; i<12; i++)
	{ 
	   tempVal = msg_buffer[i];
	   for (j=0; j<8; j++)
	   {
		if ((tempVal & (1<<(7-j))) == 0)
			laserPattern[i*8+j] = 0;
		else
			laserPattern[i*8+j] = 1;
	   }
	}
	
	if (quiet_mode_on == 0)
        xputs(PSTR("Success set the new laser pattern.\n"));

}

void display_dumped_frame (uint8_t *msg_buffer) {
    uint8_t panel_index;
    uint16_t buffer_index, x_dac_val, y_dac_val;
	
    //The first two byte is the x_dac_val
    //The second two byte is the y_dac_val
    //The fifth byte is the number of panels
    //the sixth byte is the gray scale level
    //the seventh byte is the flag of row compression 
    x_dac_val = (uint16_t)msg_buffer[0] + 256*(uint16_t)msg_buffer[1];
    y_dac_val = (uint16_t)msg_buffer[2] + 256*(uint16_t)msg_buffer[3];
    num_panels = msg_buffer[4];
    gs_value =msg_buffer[5];
    row_compress = msg_buffer[6];


    if (row_compress)
        bytes_per_panel_frame = gs_value;
    else
        bytes_per_panel_frame = gs_value*8;
  
    buffer_index = 7;
    display_flag = 0;  //clear the display flag
    digitalWrite(1, HIGH); // set line high at beginning of frame write
    
    for (panel_index = 1; panel_index <= num_panels; panel_index++){
        i2cMasterSend(panel_index, bytes_per_panel_frame, &msg_buffer[buffer_index]);
        buffer_index = buffer_index + bytes_per_panel_frame;
    }
    analogWrite(0, x_dac_val); // make it a value in the range 0 - 2047 (0 - 5V)
    analogWrite(1, y_dac_val); 
    digitalWrite(1, LOW); // set line low at end of frame write
}






void fetch_display_frame(uint16_t f_num){
    // this function will fetch the current frame from the SD-card and display it.
    // pass in f_num instead of using global frame_num to ensure that the value of
    // frame_num does not change during this function's run
    // suppose f_num is from 0 to (n_num * y_num - 1)
    uint8_t j, panel_index, packet_sent;
    uint8_t gscale[4];
    uint8_t FLASH[32];
    uint16_t len, cnt, buff_index;
    uint32_t offset;
    uint8_t res;
    uint16_t X_dac_val, Y_dac_val;
    uint8_t sreg = SREG;
	uint8_t block_per_frame;
    
    digitalWrite(1, HIGH); // set line high at start of frame write
    len = num_panels * bytes_per_panel_frame;
	block_per_frame = len/512 + 1;
    uint8_t  frameBuff[len];
    //offset = 512 + (uint32_t)f_num * (uint32_t)len;  //byte offset into the file
    offset = 512 + (uint32_t)f_num * 512 * block_per_frame;
	
    Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0); //straigforward way to avoid fs reentrant
    Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //straigforward way to avoid fs reentrant
    
    res = f_lseek(&file1, offset);
    if ((res == FR_OK) && (file1.fptr == offset)) {
        res = f_read(&file1, frameBuff, len, &cnt);
        if ((res == FR_OK) && (cnt == len)) {	
		    if (func_ID_X != 0)
				Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);  //straigforward way to avoid fs reentrant
        
			if (func_ID_Y != 0)
				Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); //straigforward way to avoid fs reentrant
        
		
            if (display_flag > 1){      //if flag gets bigger than 1 -> frame skipped
                ledToggle(1);    //toggle LED 1
            }
            
            display_flag = 0;  //clear the display flag
            buff_index = 0;
            
            for (panel_index = 1; panel_index <= num_panels; panel_index++){
                for(j = 0;j < bytes_per_panel_frame;j++){
                    FLASH[j] = frameBuff[buff_index++]; //not good for performance, no need to copy the data
                }
                packet_sent = 0; //used with compression to simplify coniditionals.
                if (ident_compress == 1) {
                    if (bytes_per_panel_frame == 8){
                        if( (FLASH[0] == FLASH[1])&&(FLASH[2] == FLASH[3])&&(FLASH[4] == FLASH[5])&&(FLASH[6] == FLASH[7]) ){
                            if( (FLASH[1] == FLASH[2])&&(FLASH[3] == FLASH[4])&&(FLASH[5] == FLASH[6]) ){
                                i2cMasterSend(panel_index, 1, &FLASH[0]); //send a 1 byte packet with the correct row_compressed value.
                                packet_sent = 1;
                            } //end of second round of comparisons
                        } //end of first round of byte comparisons
                    } // end of check if bytes_per_panel_frame is 8
                    
                    if (bytes_per_panel_frame == 24){
                        if( (FLASH[0] == FLASH[1])&&(FLASH[2] == FLASH[3])&&(FLASH[4] == FLASH[5])&&(FLASH[6] == FLASH[7]) ){
                            if( (FLASH[1] == FLASH[2])&&(FLASH[3] == FLASH[4])&&(FLASH[5] == FLASH[6]) ){
                                if( (FLASH[8+0] == FLASH[8+1])&&(FLASH[8+2] == FLASH[8+3])&&(FLASH[8+4] == FLASH[8+5])&&(FLASH[8+6] == FLASH[8+7]) ){
                                    if( (FLASH[8+1] == FLASH[8+2])&&(FLASH[8+3] == FLASH[8+4])&&(FLASH[8+5] == FLASH[8+6]) ){
                                        if( (FLASH[16+0] == FLASH[16+1])&&(FLASH[16+2] == FLASH[16+3])&&(FLASH[16+4] == FLASH[16+5])&&(FLASH[16+6] == FLASH[16+7]) ){
                                            if( (FLASH[16+1] == FLASH[16+2])&&(FLASH[16+3] == FLASH[16+4])&&(FLASH[16+5] == FLASH[16+6]) ){
                                                gscale[0] = FLASH[0];
                                                gscale[1] = FLASH[8];
                                                gscale[2] = FLASH[16];
                                                i2cMasterSend(panel_index, 3, &gscale[0]); //send a 3 byte packet with the correct row_compressed value.
                                                packet_sent = 1;
                                            } //end of sixth round of comparisons
                                        } //end of fifth round of comparisons
                                    } //end of fourth round of comparisons
                                } //end of third round of comparisons
                            } //end of second round of comparisons
                        } //end of first round of byte comparisons
                    } // end of check if bytes_per_panel_frame is 24
                    
                    if (bytes_per_panel_frame == 32){
                        if( (FLASH[0] == FLASH[1])&&(FLASH[2] == FLASH[3])&&(FLASH[4] == FLASH[5])&&(FLASH[6] == FLASH[7]) ){
                            if( (FLASH[1] == FLASH[2])&&(FLASH[3] == FLASH[4])&&(FLASH[5] == FLASH[6]) ){
                                if( (FLASH[8+0] == FLASH[8+1])&&(FLASH[8+2] == FLASH[8+3])&&(FLASH[8+4] == FLASH[8+5])&&(FLASH[8+6] == FLASH[8+7]) ){
                                    if( (FLASH[8+1] == FLASH[8+2])&&(FLASH[8+3] == FLASH[8+4])&&(FLASH[8+5] == FLASH[8+6]) ){
                                        if( (FLASH[16+0] == FLASH[16+1])&&(FLASH[16+2] == FLASH[16+3])&&(FLASH[16+4] == FLASH[16+5])&&(FLASH[16+6] == FLASH[16+7]) ){
                                            if( (FLASH[16+1] == FLASH[16+2])&&(FLASH[16+3] == FLASH[16+4])&&(FLASH[16+5] == FLASH[16+6]) ){
                                                if( (FLASH[24+0] == FLASH[24+1])&&(FLASH[24+2] == FLASH[24+3])&&(FLASH[24+4] == FLASH[24+5])&&(FLASH[24+6] == FLASH[24+7]) ){
                                                    if( (FLASH[24+1] == FLASH[24+2])&&(FLASH[24+3] == FLASH[24+4])&&(FLASH[24+5] == FLASH[24+6]) ){
                                                        gscale[0] = FLASH[0];
                                                        gscale[1] = FLASH[8];
                                                        gscale[2] = FLASH[16];
                                                        gscale[3] = FLASH[24];
                                                        i2cMasterSend(panel_index, 4, &gscale[0]); //send a 4 byte packet with the correct row_compressed value.
                                                        packet_sent = 1;
                                                    }//end
                                                }//end
                                            } //end of sixth round of comparisons
                                        } //end of fifth round of comparisons
                                    } //end of fourth round of comparisons
                                } //end of third round of comparisons
                            } //end of second round of comparisons
                        } //end of first round of byte comparisons
                    } // end of check if bytes_per_panel_frame is 32
                } //end of if ident_compress == 1
                
                if (packet_sent == 0){ //above conditionals rejected sending a simple pattern patch
                    i2cMasterSend(panel_index, bytes_per_panel_frame, &FLASH[0]);
                }
            } //end of for all panels loop
        }
        else {
            if (quiet_mode_on == 0){
                xputs(PSTR("Error in f_read in fetch_display_frame!\n"));
                xprintf(PSTR("RES = %u, f_num= %u, cnt= %u\n"), res, f_num, cnt);
            }
        }
    } else {
        //SREG = sreg;
        if (func_ID_X != 0)
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);	//straigforward way to avoid fs reentrant
        
        if (func_ID_Y != 0)
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);   //straigforward way to avoid fs reentrant
        
        if (quiet_mode_on == 0){
            xputs(PSTR("Error in f_lseek in fetch_display_frame!\n"));
            xprintf(PSTR("RES = %u, f_num= %u, offset = %lu\n"), res, f_num, offset);
        }
    }
    
    //update analog out
    if (x_mode != 5){
        X_dac_val = (index_x*65535)/x_num;
        analogWrite(0, X_dac_val>>5); // make it a value in the range 0 - 2047 (0 - 5V)
    }
    
    if (y_mode != 5){
        Y_dac_val = (index_y*65535)/y_num;
        
        
        analogWrite(1, Y_dac_val>>5); // make it a value in the range 0 - 2047 (0 - 5V)
    }
	
	  //also update the output lines for quadrant-type learning patterns
  if (Laser_active == 1)
  {
    if (laserPattern[index_x] == 0) 
      digitalWrite(2, LOW);  // turn on laser
    else
      digitalWrite(2, HIGH);   // turn off laser
  }  
	
    digitalWrite(1, LOW); // set line low at end of frame write
}

void Update_display(void) {
    int16_t X_rate = 0;
    int16_t Y_rate = 0;
    int16_t X_ADC1, X_ADC2, Y_ADC1, Y_ADC2;
    int16_t temp_ADC_val;
    
    //there are five modes 0 - OL, 1 - CL, 2 - CL w Bias, 3 - POS mode with ch5, 4 - POS mode from pos func 5 - function DBG mode
    
    switch(x_mode) {
        case 0:   // open loop - use function generator to set x rate
            X_val = 2*function_X[function_counter_x];
            X_rate = ((X_val*gain_x)/10 + 5*bias_x)/2;
            break;
        case 1: //closed loop, use CH0 - CH1 to set x rate
            X_ADC1 = analogRead(0)/4;  X_ADC2 = analogRead(1)/4; // 1 volt = 102 frames/sec
            // set to 0 if negative to be compatibel with old code
            if (X_ADC1 < 0)
                X_ADC1 = 0;
            if (X_ADC2 < 0)
                X_ADC2 = 0;
            temp_ADC_val = X_val; //the previous value
            X_val = ( 6*temp_ADC_val + 4*(X_ADC1 - X_ADC2) )/10;   //this is a 60% old value, 40% new value smoother
            X_rate = (int16_t)((int32_t)(X_val*gain_x)/10 + 5*bias_x)/2;  //X_val can go as high as 4095, gain_x 100fiu and bias_x 250
            
            //set a frame rate limit 256fps
            if (X_rate >256)
                X_rate = 256;
            else if (X_rate < -256)
                X_rate = -256;
            
            break;
        case 2: //closed loop w bias - use CH0 - CH1, and function gen. to set x rate
            X_ADC1 = analogRead(0)/4;  X_ADC2 = analogRead(1)/4; // 1 volt = 102
            // set to 0 if negative to be compatibel with old code
            if (X_ADC1 < 0)
                X_ADC1 = 0;
            if (X_ADC2 < 0)
                X_ADC2 = 0;
            temp_ADC_val = X_val; //the previous value
            X_val = ( 6*temp_ADC_val + 4*(X_ADC1 - X_ADC2) )/10;   //this is a 60% old value, 40% new value smoother
            //add in the bias to CL mode on ch X
            X_rate = (int16_t)((int32_t)(X_val*gain_x)/10 + 2*function_X[function_counter_x] + 5*bias_x)/2;
            break;
        case 3: // POS mode, use CH4 to set the frame position (pos ctrl, not vel ctrl)
            X_ADC1 = analogRead(4)/2;  //used to use CH0, changed this so no need to change connections
            // set to 0 if negative to be compatibel with old code
            if (X_ADC1 < 0)
                X_ADC1 = 0;
            index_x = X_ADC1/gain_x + bias_x;
            if (index_x >= x_num)  {index_x = x_num - 1;} //check if too big
            if (index_x <= 0)  {index_x = 0;} //or too small
            frame_num = index_y*x_num + index_x;
            X_rate = 0;
            break;
        case 4:
            //only use temp_ADC_val as a temp variable, just not to create an additional one
            temp_ADC_val = (X_pos_index + function_X[function_counter_x]);
            if (temp_ADC_val >= 0) {index_x = temp_ADC_val%x_num; }
            if (temp_ADC_val < 0) {index_x = x_num - ((abs(temp_ADC_val))%x_num) -1;} //index_x should already smaller than x_num
            frame_num = index_y*x_num + index_x;
            X_rate = 0;
            break;
        case 5:  // this is the function gen DBG mode - don't run x, set rate to zero
            X_rate = 0;
            break;
            //do something with errors here for default case
    }
    
    
    switch(y_mode) {
        case 0:   // open loop - use function generator to set x rate
            Y_val = 2*function_Y[function_counter_y];
            Y_rate = ((Y_val*gain_y)/10 + 5*bias_y)/2;
            break;
        case 1: //closed loop, use CH2 - CH3 to set x rate
            Y_ADC1 = analogRead(2)/4;  Y_ADC2 = analogRead(3)/4; // 1 volt = 102fps
            // set to 0 if negative to be compatibel with old code
            if (Y_ADC1 < 0)
                Y_ADC1 = 0;
            if (Y_ADC2 < 0)
                Y_ADC2 = 0;
            temp_ADC_val = Y_val; //the previous value
            Y_val = ( 6*temp_ADC_val + 4*(Y_ADC1 - Y_ADC2) )/10;   //this is a 60% old value, 40% new value smoother
            Y_rate = (int16_t)((int32_t)(Y_val*gain_y)/10 + 5*bias_y)/2; //Y_val can go as high as 4095, gain_y 100, and bias_y 250.
            
            //set a frame rate limit 256fps
            if (Y_rate > 256)
                Y_rate = 256;
            else if (Y_rate < -256)
                Y_rate = -256;
            
            break;
        case 2: //closed loop w bias - use CH2 - CH3, and function gen. to set x rate
            Y_ADC1 = analogRead(2)/4;  Y_ADC2 = analogRead(3)/4; // 1 volt = 102
            // set to 0 if negative to be compatibel with old code
            if (Y_ADC1 < 0)
                Y_ADC1 = 0;
            if (Y_ADC2 < 0)
                Y_ADC2 = 0;
            temp_ADC_val = Y_val; //the previous value
            Y_val = ( 6*temp_ADC_val + 4*(Y_ADC1 - Y_ADC2) )/10;   //this is a 60% old value, 40% new value smoother
            //add in the bias to CL mode on ch Y
            Y_rate = (int16_t)((int32_t)(Y_val*gain_y)/10 + 2*function_Y[function_counter_y] + 5*bias_y)/2; //Y_val can go as high as 4095
            break;
        case 3: // POS mode, use CH5 to set the frame position (pos ctrl, not vel ctrl)
            Y_ADC1 = analogRead(5)/2;
            // set to 0 if negative to be compatibel with old code
            if (Y_ADC1 < 0)
                Y_ADC1 = 0;
            index_y = Y_ADC1/gain_y + bias_y;
            if (index_y >= y_num)  {index_y = y_num - 1;} //check if too big
            if (index_y <= 0)  {index_y = 0;} //or too small
            frame_num = index_y*x_num + index_x;
            Y_rate = 0;
            break;
        case 4:
            //only use temp_ADC_val as a temp variable, just not to create an additional one
            temp_ADC_val = (Y_pos_index + function_Y[function_counter_y]);
            if (temp_ADC_val >= 0) {index_y = temp_ADC_val%y_num; }
            if (temp_ADC_val < 0) {index_y = y_num - ((abs(temp_ADC_val))%y_num) - 1;  } //index_y should always smaller than y_num
            frame_num = index_y*x_num + index_x;
            Y_rate = 0;
            break;
        case 5:  // this is the function gen DBG mode - don't run y, set rate to zero
            Y_rate = 0;
            break;
            //do something with errors here for default case
    }
    
    //in the above x,y_val computation, there is a div by 10 to take away gain scaling
    //so gain_x of 10 is 1X gain, gain_x of 20 = 2X ...
    
    //here the 2* the rate is because we want 20 = 1V to correspond to 10 fps. could probably do without,
    // and just divide the a2dConvert output by 4, and not scale function_x,y by 2
    if (Stop == 1){
        X_rate = Y_rate = 0;
    }
    
    if (X_rate > 0)
        Update_Reg_Handler(increment_index_x, (uint32_t)OVERFLOW_RATE/abs(X_rate), 2, 1);
    else if (X_rate < 0)
        Update_Reg_Handler(decrement_index_x, (uint32_t)OVERFLOW_RATE/abs(X_rate), 2, 1);
    else     //X_rate == 0
        Update_Reg_Handler(decrement_index_x, (UPDATE_RATE), 2, 0);
    
    
    if (Y_rate > 0)
        Update_Reg_Handler(increment_index_y, (uint32_t)OVERFLOW_RATE/abs(Y_rate), 3, 1);
    else if (Y_rate < 0)
        Update_Reg_Handler(decrement_index_y, (uint32_t)OVERFLOW_RATE/abs(Y_rate), 3, 1);
    else      //Y_rate == 0
        Update_Reg_Handler(decrement_index_y, (UPDATE_RATE), 3, 0);
    
    //if the rates are too high, track the largest one to set warning LED
    x_gt_y = (X_rate >= Y_rate);
}


void increment_index_x(void) {
    
    index_x++;
    if (index_x >= x_num)
    {index_x = 0;}
    
    
    frame_num = index_y*x_num + index_x;
	
    if (x_gt_y) display_flag++;
}


void increment_index_y(void) {
    index_y++;
    if (index_y >= y_num)
    {index_y = 0;}
    
    frame_num = index_y*x_num + index_x;
	
    if (x_gt_y == 0) display_flag++;
}


void decrement_index_x(void) {
    
    if (index_x <= 0)    //just to be safe, use less than
    {index_x = x_num - 1;}    //but these are unsigned
    else
    {index_x--;}
    
    frame_num = index_y*x_num + index_x;
    if (x_gt_y) display_flag++;
}


void decrement_index_y(void) {
    if (index_y <= 0)    //just to be safe, use less than
    {index_y = y_num - 1;}    //but these are unsigned
    else
    {index_y--;}
    
    frame_num = index_y*x_num + index_x;
    if (x_gt_y == 0) display_flag++;
}


void toggle_trigger(void) {
    
    digitalToggle(3); //toggle digital 3 to trigger camera
}



void set_pattern(uint8_t pat_num) {
    //sets the pattern ID, in future return 0 or 1 if error/succeed
    uint16_t num_frames;
    uint16_t cnt;
    static uint8_t str[12];
    uint8_t  pattDataBuff[512];
    uint8_t res;
    
    if (pat_num < 10)
        sprintf(str, "pat000%d.pat\0", pat_num);
    else if (pat_num < 100)
        sprintf(str, "pat00%d.pat\0", pat_num);
    else if (pat_num < 1000)
        sprintf(str, "pat0%d.pat\0", pat_num);
    else
        if (quiet_mode_on == 0)
            xputs(PSTR("pat_num is too big.\n"));
    
    
	Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0); //straigforward way to avoid fs reentrant
    Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //straigforward way to avoid fs reentrant
	
    res = f_close(&file1);
    
    res = f_open(&file1, str, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        res = f_read(&file1, pattDataBuff, 512, &cnt); // read the 10 byte test header info block
        if ((res == FR_OK) && (cnt == 512)) {
		    if (func_ID_X != 0)
				Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);  //straigforward way to avoid fs reentrant
        
			if (func_ID_Y != 0)
				Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); //straigforward way to avoid fs reentrant
			
            // get the test header info
            ((uint8_t*)&x_num)[0] = pattDataBuff[0];
            ((uint8_t*)&x_num)[1] = pattDataBuff[1];
            ((uint8_t*)&y_num)[0] = pattDataBuff[2];
            ((uint8_t*)&y_num)[1] = pattDataBuff[3];
            num_panels = pattDataBuff[4];
            gs_value = pattDataBuff[5];   //11, 12, 13, or 14 means use row compression
            
            
            num_frames = x_num * y_num;
            if ((gs_value >= 11) & (gs_value <= 14)) {
                gs_value = gs_value - 10;
                row_compress = 1;
                bytes_per_panel_frame = gs_value;
            }
            else {
                row_compress = 0;
                bytes_per_panel_frame = gs_value * 8;
            }
            index_x = index_y = 0;
            frame_num = 0;
            Stop = 1;
            display_flag = 0;  //clear the display flag
            if (quiet_mode_on == 0){
                xprintf(PSTR("Setting pattern %u:\n"), pat_num);
                xprintf(PSTR("  x_num = %u\n  y_num = %u\n  num_panels = %u\n  gs_value = %u\n row_compression = %u\n"),
                        x_num, y_num, num_panels, gs_value, row_compress);
            }
            fetch_display_frame(frame_num);
        } else {
            if (quiet_mode_on == 0)
                xputs(PSTR("Error reading in pattern file\n"));
        }
    } else {
        if (quiet_mode_on == 0)
            xputs(PSTR("Error opening pattern file\n"));
    }
}

void set_hwConfig(uint8_t config_num) {
    // try to read in the hardware config file

	static uint8_t str[12];
    uint8_t res, b1;
    uint8_t  tempBuff[128];
	uint16_t cnt;
		
    if (config_num < 10)
        sprintf(str, "cfg000%d.cfg\0", config_num);
    else if (config_num < 100)
        sprintf(str, "cfg00%d.cfg\0", config_num);
    else if (config_num < 1000)
        sprintf(str, "cfg0%d.cfg\0", config_num);
    else
        if (quiet_mode_on == 0)
            xputs(PSTR("config_num is too big.\n"));
			
    res = f_open(&file4, str, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        // looks good, read in the 128 byte panel mapping info
        res = f_read(&file4, tempBuff, 128, &cnt);
        if ((res == FR_OK) && (cnt == 128)) {
            //copy to the mapping tables
            for (b1 = 1; b1 <= 128; b1++) {
				chMap[b1] = tempBuff[b1-1]; // panel 0 doesn't exist
				eeprom_write_byte(arena_config + b1, tempBuff[b1-1]);
            }
		    eeprom_write_byte(arena_config, 0x00); //Mark arena configuration file in EEPROM
        }
        f_close(&file4);
        xputs(PSTR("Successfully load the hardware config file to EEPROM\n"));
    }
    else{
        xputs(PSTR("Cannot find the hardware config file on the SD card.\n"));
    }
}

void benchmark_pattern(void) { // this function assumes that a pattern has been set
    uint16_t num_frames;
    uint16_t frame_ind;
    uint32_t bench_time;
    uint16_t frame_rate;
    
    Stop = 1;
    num_frames = x_num*y_num;
    
    timer_coarse_tic();
    
    for(frame_ind = 0; frame_ind < num_frames; frame_ind++)
        fetch_display_frame(frame_ind);
    
    bench_time = timer_coarse_toc();
    frame_rate = ((uint32_t)num_frames*1000)/bench_time;
    xprintf(PSTR(" bench_time = %lu ms, frame_rate = %u\n"), bench_time, frame_rate);
}

void i2cMasterSend(uint8_t panel, uint8_t len, uint8_t *data) {
    uint8_t ch;
    uint8_t addr;
    TWI_Master_t *twi;
    
    if (panel == 0) {
        while (twi1.status != TWIM_STATUS_READY);
        TWI_MasterWrite(&twi1, 0, data, len);
        while (twi2.status != TWIM_STATUS_READY);
        TWI_MasterWrite(&twi2, 0, data, len);
        while (twi3.status != TWIM_STATUS_READY);
        TWI_MasterWrite(&twi3, 0, data, len);
        while (twi4.status != TWIM_STATUS_READY);
        TWI_MasterWrite(&twi4, 0, data, len);
    }
    else {
        // look up the actual panel address and channel
        ch = chMap[panel];
        addr = adrMap[panel];
        if (ch != 0){
            
            switch (ch) {
                case 1:
                    twi = &twi1;
                    break;
                case 2:
                    twi = &twi2;
                    break;
                case 3:
                    twi = &twi3;
                    break;
                case 4:
                    twi = &twi4;
                    break;
                default: // send to twi1
                    twi = &twi1;
                    break;
            }
            
            while (twi->status != TWIM_STATUS_READY);
            TWI_MasterWrite(twi, addr, data, len);
        }
    }
}

void set_default_func(uint8_t func_channel) {
    uint16_t funcCnt;
    
    switch (func_channel) {
        case 1:
            if (quiet_mode_on == 0)
                xputs(PSTR("Setting default function for X.\n"));
            
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);//disable ISR
            func_ID_X = 0;
            funcSize_x = FUNCTION_LENGTH;
            for (funcCnt = 0; funcCnt < FUNCTION_LENGTH; funcCnt++)
            { function_X[funcCnt] = 10; }
            function_counter_x = 0;
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);//don't need to enable ISR
            break;
        case 2:
            if (quiet_mode_on == 0)
                xputs(PSTR("Setting default function for Y.\n"));
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0);//disable ISR
            func_ID_Y = 0;
            funcSize_y = FUNCTION_LENGTH;
            for (funcCnt = 0; funcCnt < FUNCTION_LENGTH; funcCnt++)
            { function_Y[funcCnt] = 10; }
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);//don't need to enable ISR
            break;
        default:
            if (quiet_mode_on == 0)
                xputs(PSTR("Wrong function channel number.\n"));
    }
    
}


void set_pos_func(uint8_t func_channel, uint8_t func_id) {
    uint16_t cnt;
    uint8_t tmpCnt;
    uint8_t str[12];
    //uint8_t func_name_x[100];
    //uint8_t func_name_y[100];
    uint8_t res, func_name_len;
    uint8_t posFuncBuff[512];
    
    
    if (func_id < 10)
        sprintf(str, "pos000%d.fun\0", func_id);
    else if (func_id < 100)
        sprintf(str, "pos00%d.fun\0", func_id);
    else if (func_id < 1000)
        sprintf(str, "pos0%d.fun\0", func_id);
    else
        if (quiet_mode_on == 0)
            xputs(PSTR("function id is too big.\n"));
    
    switch(func_channel) {
        case 1:    //channel x
            //read the header block and send back the function name
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);//disable ISR
            
            res = f_close(&file2);
            
            res = f_open(&file2, str, FA_OPEN_EXISTING | FA_READ);
            if (res == FR_OK) {
                res = f_read(&file2, posFuncBuff, 512, &cnt);
                if ((res == FR_OK) && (cnt == 512)) {
                    // get the test header info
                    ((uint8_t*)&funcSize_x)[0] = posFuncBuff[0];
                    ((uint8_t*)&funcSize_x)[1] = posFuncBuff[1];
                    ((uint8_t*)&funcSize_x)[2] = posFuncBuff[2];
                    ((uint8_t*)&funcSize_x)[3] = posFuncBuff[3];
                    func_name_len = posFuncBuff[4];
                    
                    
                    //for (tmpCnt=0; tmpCnt<func_name_len; tmpCnt++)
                    //	{func_name_x[tmpCnt] = posFuncBuff[tmpCnt+5];}
                    //func_name_x[func_name_len] = '\0';
                    
                    func_ID_X = func_id;
                    
                    Stop = 1;
                    display_flag = 0;  //clear the display flag
                    if (quiet_mode_on == 0)
                        xprintf(PSTR("Setting position function %u for X\n"), func_id);
                    //xprintf(PSTR("fun X: %s\n function X size: %lu bytes\n"),
                    //        func_name_x, funcSize_x);
                    
                } else {
                    if (quiet_mode_on == 0)
                        xputs(PSTR("Error f_read set_pos_func X\n"));
                }
            } else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_open in set_pos_func X.\n"));
            }
            function_counter_x = 0;
            func_global_counter_x = funcSize_x;
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);//enable ISR
            break;
            
        case 2:
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //disable ISR
            //read the header block and send back the function name
            res = f_close(&file3);
            
            res = f_open(&file3, str, FA_OPEN_EXISTING | FA_READ);
            if (res == FR_OK) {
                res = f_read(&file3, posFuncBuff, 512, &cnt);
                if ((res == FR_OK) && (cnt == 512)) {
                    // get the test header info
                    // get the test header info
                    ((uint8_t*)&funcSize_y)[0] = posFuncBuff[0];
                    ((uint8_t*)&funcSize_y)[1] = posFuncBuff[1];
                    ((uint8_t*)&funcSize_y)[2] = posFuncBuff[2];
                    ((uint8_t*)&funcSize_y)[3] = posFuncBuff[3];
                    func_name_len = posFuncBuff[4];
                    
                    //for (tmpCnt=0; tmpCnt<func_name_len; tmpCnt++)
                    //	{func_name_y[tmpCnt] = posFuncBuff[tmpCnt+5];}
                    //func_name_y[func_name_len] = '\0';
                    
                    func_ID_Y = func_id;
                    
                    Stop = 1;
                    display_flag = 0;  //clear the display flag
                    if (quiet_mode_on == 0)
                        xprintf(PSTR("Setting position function %u for Y\n"), func_id);
                    //xprintf(PSTR("fun Y: %s\n function Y size: %lu bytes\n"),
                    //       func_name_y, funcSize_y);
                    
                } else {
                    if (quiet_mode_on == 0)
                        xputs(PSTR("Error f_read set_pos_func Y.\n"));
                }
            } else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_open in set_pos_func Y.\n"));
            }
            function_counter_y = 0;
            func_global_counter_y = funcSize_y;
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);//enable ISR
            break;
            
        default:
            if (quiet_mode_on == 0)
                xputs(PSTR("Error input for function channel.\n"));
            break;
    }
}


void set_vel_func(uint8_t func_channel, uint8_t func_id) {
    //sets the velocity function id
    uint16_t cnt;
    uint8_t tempCnt;
    uint8_t str[12];
    //uint8_t func_name_x[100];
    //uint8_t func_name_y[100];
    uint8_t res, func_name_len;
    uint8_t velFuncBuff[512];
    
    
    if (func_id < 10)
        sprintf(str, "vel000%d.fun\0", func_id);
    else if (func_id < 100)
        sprintf(str, "vel00%d.fun\0", func_id);
    else if (func_id < 1000)
        sprintf(str, "vel0%d.fun\0", func_id);
    else
        if (quiet_mode_on == 0)
            xputs(PSTR("function id is too big.\n"));
    
    switch(func_channel) {
        case 1:    //channel x
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0); //disable ISR
            //read the header block and send back the function name
            res = f_close(&file2);
            
            res = f_open(&file2, str, FA_OPEN_EXISTING | FA_READ);
            
            if (res == FR_OK) {
                
                res = f_read(&file2, velFuncBuff, 512, &cnt);
                
                if ((res == FR_OK) && (cnt == 512)) {
                    
                    // get the test header info
                    ((uint8_t*)&funcSize_x)[0] = velFuncBuff[0];
                    ((uint8_t*)&funcSize_x)[1] = velFuncBuff[1];
                    ((uint8_t*)&funcSize_x)[2] = velFuncBuff[2];
                    ((uint8_t*)&funcSize_x)[3] = velFuncBuff[3];
                    func_name_len = velFuncBuff[4];
                    
                    func_ID_X = func_id;
                    
                    Stop = 1;
                    display_flag = 0;  //clear the display flag
                    if (quiet_mode_on == 0)
                        xprintf(PSTR("Setting velocity function  %u for X\n"), func_id);
                    
                } else {
                    if (quiet_mode_on == 0)
                        xputs(PSTR("Error f_read in set_vel_func X.\n"));
                }
            } else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_open in set_vel_func X.\n"));
            }
            function_counter_x = 0;
            func_global_counter_x = funcSize_x;
            Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1); //enable ISR
            break;
            
        case 2:
            
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //disable ISR
            
            res = f_close(&file3);
            
            //read the header block and send back the function name
            res = f_open(&file3, str, FA_OPEN_EXISTING | FA_READ);
            
            if (res == FR_OK) {
                
                res = f_read(&file3, velFuncBuff, 512, &cnt);
                
                if ((res == FR_OK) && (cnt == 512)) {
                    // get the test header info
                    ((uint8_t*)&funcSize_y)[0] = velFuncBuff[0];
                    ((uint8_t*)&funcSize_y)[1] = velFuncBuff[1];
                    ((uint8_t*)&funcSize_y)[2] = velFuncBuff[2];
                    ((uint8_t*)&funcSize_y)[3] = velFuncBuff[3];
                    func_name_len = velFuncBuff[4];
                    
                    func_ID_Y = func_id;
                    
                    Stop = 1;
                    display_flag = 0;  //clear the display flag
                    if (quiet_mode_on == 0)
                        xprintf(PSTR("Setting velocity function %u for Y\n"), func_id);
                    
                } else {
                    if (quiet_mode_on == 0)
                        xputs(PSTR("Error f_read in set_vel_func Y.\n"));
                }
            } else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_open in set_vel_func Y.\n"));
            }
            function_counter_y = 0;
            func_global_counter_y = funcSize_y;
            Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); //enable ISR
            break;
            
        default:
            if (quiet_mode_on == 0)
                xputs(PSTR("Error input for function channel.\n"));
            break;
    }
}

void update_funcCnt_x(void) {
    uint16_t X_dac_val;
    uint16_t cnt, j, offset;
    uint8_t res;
    int16_t tempVal;
    uint8_t funcXBuff[2*FUNCTION_LENGTH];
    
    if (func_ID_X != 0){
        
        function_counter_x++;  //min:0, max:FUNCTION_LENGTH
        func_global_counter_x = func_global_counter_x + 2; //min:0, max:funcSize_x Note:size of each function datum is 2 bytes.
        
        if ((function_counter_x >= loadXBuffer/2) || (func_global_counter_x >= funcSize_x)) {
            // need to update function_X[FUNCITION_LENGTH]
            if (func_global_counter_x >= funcSize_x){
                
                offset = 512;
                func_global_counter_x =0;
                function_counter_x = 0;
            }
            else{
                
                //read the next loadBuffer bytes
                offset = 512 + func_global_counter_x;
                function_counter_x = 0;
            }
            
            res = f_lseek(&file2, offset);
            if ((res == FR_OK) && (file2.fptr == offset)) {
                
                if (funcSize_x - func_global_counter_x >= 2*FUNCTION_LENGTH) {
                    loadXBuffer = 2*FUNCTION_LENGTH;
                }
                else {
                    loadXBuffer = funcSize_x - func_global_counter_x;
                }
                
                res = f_read(&file2, funcXBuff, loadXBuffer, &cnt);
                if ((res == FR_OK) && (cnt == loadXBuffer)) {
                    
                    for (j = 0; j < loadXBuffer/2; j++)
                    {   ((uint8_t*)&tempVal)[0] = funcXBuff[2*j];
                        ((uint8_t*)&tempVal)[1] = funcXBuff[2*j+1];
                        function_X [j] = tempVal;
                    }
                    
                } else {
                    if (quiet_mode_on == 0){
                        xprintf(PSTR("res =  %u\n"), res);
                        xputs(PSTR("Error in f_read in in update_funcCnt_x\n"));
                    }
                }
            } else {
                if (quiet_mode_on == 0){
                    xprintf(PSTR("res =  %u\n"), res);
                    xputs(PSTR("Error in f_lseek in update_funcCnt_x\n"));
                }
            }
        } //end if ((function_counter_x >= loadBuffer/2) || (func_global_counter_x >= funcSize_x))
        
    }
    else{
        function_counter_x = (function_counter_x + 1)%FUNCTION_LENGTH;
    }
    
    if (x_mode == 5)   // in function DBG mode - show the function gen
    {
        X_dac_val = (65535/2) + 328*function_X[function_counter_x];   //328 should convert 1V (=20) to 0.5 V
        analogWrite(0, X_dac_val>>5); // make it a value in the range 0 - 2047 (0 - 5V)
    }
}

void update_funcCnt_y(void) {
    uint16_t Y_dac_val;
    uint16_t cnt, j, offset;
    uint8_t res;
    int16_t tempVal;
    uint8_t funcYBuff[2*FUNCTION_LENGTH];
    
    if (func_ID_Y != 0){
        function_counter_y ++;
        func_global_counter_y = func_global_counter_y + 2;
        
        
        if ((function_counter_y >= loadYBuffer/2) || (func_global_counter_y >= funcSize_y)) {
            // need to update function_Y[FUNCITION_LENGTH]
            if (func_global_counter_y >= funcSize_y){
                //read from the start
                offset = 512;
                func_global_counter_y =0;
                function_counter_y = 0;
            }
            else{
                offset = 512 + func_global_counter_y;
                function_counter_y = 0;
            }
            
            res = f_lseek(&file3, offset);
            if ((res == FR_OK) && (file3.fptr == offset)) {
                
                if (funcSize_y - func_global_counter_y >= 2*FUNCTION_LENGTH) {
                    loadYBuffer = 2*FUNCTION_LENGTH;
                }
                else {
                    loadYBuffer = funcSize_y - func_global_counter_y;
                }
                
                res = f_read(&file3, funcYBuff, loadYBuffer, &cnt);
                if ((res == FR_OK) && (cnt == loadYBuffer)) {
                    
                    for (j = 0; j < loadYBuffer/2; j++)
                    {   ((uint8_t*)&tempVal)[0] = funcYBuff[2*j];
                        ((uint8_t*)&tempVal)[1] = funcYBuff[2*j+1];
                        function_Y [j] = tempVal;
                    }
                    
                } else {
                    if (quiet_mode_on == 0){
                        xprintf(PSTR("res =  %u\n"), res);
                        xputs(PSTR("Error in f_read in update_funcCnt_y\n"));
                    }
                }
            } else {
                if (quiet_mode_on == 0){
                    xprintf(PSTR("res =  %u\n"), res);
                    xputs(PSTR("Error in f_lseek in update_funcCnt_y load next buffer\n"));
                }
            }
        } //end if (function_counter >= 1000) && (func_global_counter <= funcSize)
    }
    else{
        function_counter_y = (function_counter_y + 1)%FUNCTION_LENGTH;
    }
    
//update analog output
    if (y_mode == 5){
        Y_dac_val = (65535/2) + 328*function_Y[function_counter_y];   //328 should convert 1V (=20) to 0.5 V
        analogWrite(1, Y_dac_val>>5); // make it a value in the range 0 - 2047 (0 - 5V)
    }
    
}


//synchronize the SD.mat from SD card to PC
void dump_mat(void) {
    uint8_t b1, fileRemain;
    uint32_t iteration, offset;
    uint8_t res;
    uint16_t cnt;
    uint8_t matBuff[50];
    
    // try to read in the SD.mat filfil
    res = f_open(&file4, SDInfo, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        // looks good
        iteration = file4.fsize/50;
        fileRemain = (uint8_t)(file4.fsize - iteration*50);
        //xprintf(PSTR("filesize = %lu, iteration = %lu, fileRemain = %u\n"), file4.fsize, iteration, fileRemain);
        
        // send 50 bytes data for iteration times
        for (b1=1; b1<= iteration; b1++){
            offset = (b1 -1)*50;
            
            res = f_lseek(&file4, offset);
            if ((res == FR_OK) && (file4.fptr == offset)) {
                res = f_read(&file4, matBuff, 50, &cnt);
                if ((res == FR_OK) && (cnt == 50)) {
                    send_Tx_buffer(matBuff, 50);
                }
            }else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_lseek in SDInfo.mat.\n"));
            }
        }
        //send the remained data
        offset = (b1-1)*50;
        res=f_lseek(&file4, offset);
        if ((res == FR_OK) && (file4.fptr == offset)) {
            res = f_read(&file4, matBuff, fileRemain, &cnt);
            if ((res == FR_OK) && (cnt == fileRemain)) {
                send_Tx_buffer(matBuff, fileRemain);
            }
        }else {
            if (quiet_mode_on == 0)
                xputs(PSTR("Error f_lseek in remained data.\n"));
        }
        
        f_close(&file4);
    } else {
        if (quiet_mode_on == 0)
            xputs(PSTR("Error f_open in SDInfo.mat.\n"));//end if (res == FR_OK)
    }
    
}





