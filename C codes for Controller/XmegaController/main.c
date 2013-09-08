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
volatile uint8_t  Stop = 1;
uint8_t  display_flag = 0;
uint8_t  x_gt_y = 0;
uint8_t  Laser_active = 0;
uint8_t  FSMutex = 0;
uint16_t resolution_x = 4095;
uint16_t resolution_y = 4095;
uint8_t  usePreloadedPattern = 0;

uint16_t x_num, y_num;  //the max index for x and y
volatile uint16_t index_x, index_y; // the current index of x and y
uint8_t  gs_value, bytes_per_panel_frame, row_compress, ident_compress;
uint8_t  num_panels = 0;
uint8_t  x_mode, y_mode;
volatile uint16_t frame_num = 0;
int16_t  function_X[BUFFER_LENGTH/2]; //ring buffer
int16_t  function_Y[BUFFER_LENGTH/2];  //ring buffer
volatile uint8_t func_read_index_x = 0;  //read index for the ring buffer function_X[BUFFER_LENGTH/2] 
volatile uint8_t func_read_index_y = 0;	 //read index for the ring buffer function_Y[BUFFER_LENGTH/2]
volatile uint8_t func_write_index_x = 0; //write index for the ring buffer function_X[BUFFER_LENGTH/2]
volatile uint8_t func_write_index_y = 0; //write index for the ring buffer function_X[BUFFER_LENGTH/2]
volatile uint8_t func_buffer_size_x = 0;  //share between main function and update_funcCnt_x
volatile uint8_t func_buffer_size_y = 0;  //share between main function and update_funcCnt_y
//volatile uint32_t func_global_counter_x = 0;
//volatile uint32_t func_global_counter_y = 0;
volatile uint8_t next_block_x = 1;
volatile uint8_t next_block_y = 1;

volatile uint8_t default_func_x = 1;
volatile uint8_t default_func_y = 1;
volatile uint8_t sync_XY_func = 0;
uint16_t functionX_rate = FUNCTION_RATE;
uint16_t functionY_rate = FUNCTION_RATE;

uint8_t  laserPattern[125];
int16_t   gain_x, gain_y, bias_x, bias_y;
int16_t  X_pos_index, Y_pos_index;
uint16_t trigger_rate = 200;

uint32_t start_block; // start block of a pattern on SD-card
uint32_t funcSize_x = BUFFER_LENGTH; //function file size
uint32_t funcSize_y = BUFFER_LENGTH;
FIL      file1, file2, file3, file4;       // File object
//file1 pattern file; file 2 function file for x channel
//file2 function file for y channel, file 4 for SD.mat or arena config file
FATFS    fatfs;       // File system object
//max frame size = 32bpp * 4rows * 12col = 1536
//uint8_t  Buff[1536];  // File System working buffer
//uint8_t  buff4Panel[256];   //working buffer for programming panel

uint8_t  adrMap[129]; // panel twi address mapping, we can have same address in different channels
uint8_t  quiet_mode_on=1;

uint16_t func_ID_X = 0;
uint16_t func_ID_Y = 0;

//volatile uint16_t loadXBuffer = FUNCTION_LENGTH;
//volatile uint16_t loadYBuffer = FUNCTION_LENGTH;

uint16_t num_buffer_load_x = 1;
uint16_t last_load_x = 0;
uint16_t num_buffer_load_y = 1;
uint16_t last_load_y = 0;

static const uint8_t VERSION[] = "1.3\0";
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
	uint8_t rightBufferXLoaded = 0, rightBufferYLoaded = 0;
    uint16_t frame_num_old = 999;  //just chosen at random
	uint8_t func_idx_x_old = 199;
	uint8_t func_idx_y_old = 199;
	
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
	for (lcv = 0; lcv < 125; lcv++)
	{ 
		if (((lcv >=0) && (lcv <= 2)) || ((lcv >= 6)&&(lcv <= 8)))
	        laserPattern[lcv] = 255;
		else
			laserPattern[lcv] = 0;
	}
	
    
    //initilializations
    X_pos_index = Y_pos_index = index_x = index_y = 0;
    bias_x = bias_y = 0;
    gain_x = gain_y = 0;
    x_mode = y_mode = 0;
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
        uint8_t msg_buffer[65];
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
					case 4:
						handle_message_length_4(&msg_buffer[0]);
						break;
                    case 5: // if length 5, then decode
                        handle_message_length_5(&msg_buffer[0]);
                        break;
					case 9: // if length 9, then decode, set x,y index, or set gain, bias
                        handle_message_length_9(&msg_buffer[0]);
                        break;
                    case 62: //if length 62, then set laser trigger pattern first 62 byte
                        handle_message_length_62(&msg_buffer[0]);
                        break;
					case 63: //if length 63, then set laser trigger pattern second 63 byte
                        handle_message_length_63(&msg_buffer[0]);
                        break;	
                    default:
                        i2cMasterSend(0x00, 8, ERROR_CODES[7]);
                } //end of switch
            }// end of if, goes to top if nothing received on UART
            
            // at bottom of while(1) loop, check to see if stop is 0, then unpdate display if the frame has changed.
			// Also update the function buffer
            if (Stop == 0){  //only send out new pattern if the pattern index has change
                if (frame_num != frame_num_old) {
                    frame_num_old = frame_num; //update the 'old' frame number	
					if (usePreloadedPattern == 1)
						display_preload_frame(frame_num, index_x, index_y);
					else
						fetch_display_frame(frame_num, index_x, index_y);
                }
				
				//func_buffer_size_x in word, 2 bytes.
				if ((default_func_x == 0) && (func_buffer_size_x <= FUNCTION_LENGTH/4) && (func_read_index_x != func_idx_x_old))
				{					
					func_idx_x_old = func_read_index_x;
					fetch_update_funcX(0, next_block_x);  
					next_block_x = (next_block_x + 1)%num_buffer_load_x; 
//xprintf(PSTR("func_buffer_size_x=%u, func_read_index_x=%u\n"), func_buffer_size_x, func_read_index_x);
				}

				if ((default_func_y == 0) && (func_buffer_size_y <= FUNCTION_LENGTH/4) && (func_read_index_y != func_idx_y_old))
				{					
					func_idx_y_old = func_read_index_y;
					fetch_update_funcY(0, next_block_y);	
					next_block_y = (next_block_y + 1)%num_buffer_load_y; 
					//xprintf(PSTR("func_buffer_size_y=%u, func_read_index_y=%u\n"), func_buffer_size_y, func_read_index_y);	
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
                        i2cMasterSend(0x00, 8, ERROR_CODES[8]);
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
	uint32_t tmp_x=0;
	uint32_t tmp_y=0;
	uint8_t n_sample=100;
    
    switch(msg_buffer[0]) {
        case 0x20:  //Start display: 0x20
            //set these to zero so that start at beginning of function - useful for putting in a set amount of expansion
			func_read_index_x = 0;
			func_read_index_y = 0;
            Stop = 0;
			next_block_x = 1;
			next_block_y = 1;
            display_flag = 0;  //clear the display flag
            Reg_Handler(Update_display, UPDATE_RATE, 1, 1);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0); //initilize the 2 and 3 priority interupts to a fast rate so that
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0); // the countdown is fast until the setting of the next rate
                                                                //by the Update_display interupt.
																
			if(!default_func_x && !default_func_y && functionX_rate == functionY_rate){ 
				//We want to synchronize function X and function Y updates in this case
				xprintf(PSTR("Function X and Y are synchronized.\n"));
				sync_XY_func = 1;
				update_funcCnt_xy();
				Reg_Handler(update_funcCnt_xy, functionX_rate, 4, 1);	
			}
			else{
				sync_XY_func = 0;
				if (default_func_x)
					Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);
				else{
					update_funcCnt_x();//add this because the function cnt is updated without delay
					Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);
				}
				
				if (default_func_y)
					Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); 
				else{
					update_funcCnt_y();//add this because the function cnt is updated without delay
					Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); 			
				}
			}
			
			break;
            
        case 0x30: //stop display
            Stop = 1;
            //turn off the interupts
            Reg_Handler(Update_display, UPDATE_RATE, 1, 0);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
			
			if(sync_XY_func) 
				Reg_Handler(update_funcCnt_xy, functionX_rate, 4, 0);
			else{
				Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);
				Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); 
			}
			
			if (default_func_x == 0)
				fetch_update_funcX(1,0);
			if (default_func_y == 0)	
				fetch_update_funcY(1,0);
		
            break;
            
        case 0x25:  //Start display & trigger - same as regular, but this also does trigger
            //set these to zero so that start at beginning of function - useful for putting in a set amount of expansion
			func_read_index_x = 0;
			func_read_index_y = 0;
            Stop = 0;
            display_flag = 0;  //clear the display flag
            Reg_Handler(Update_display, UPDATE_RATE, 1, 1);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
			
			if(!default_func_x && !default_func_y && functionX_rate == functionY_rate){ 
				//We want to synchronize function X and function Y updates in this case
				update_funcCnt_xy();
				Reg_Handler(update_funcCnt_xy, functionX_rate, 4, 1);
				
			}else{
																
				if (default_func_x)
					Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);
				else{
					update_funcCnt_x();//add this because the function cnt is updated without delay
					Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);
				}
				
				if (default_func_y)
					Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); 
				else{
					update_funcCnt_y();//add this because the function cnt is updated without delay
					Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); 			
				}
    		}
            Reg_Handler(toggle_trigger, (uint32_t)OVERFLOW_RATE/trigger_rate, 0, 1); //turn on the trigger toggle
            break;
            
        case 0x35: //stop display & trigger - same as regular, but this also does trigger
            Stop = 1;
            //turn off the interupts
            Reg_Handler(Update_display, UPDATE_RATE, 1, 0);
            Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0);
            Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0);
			
			if(sync_XY_func) 
				Reg_Handler(update_funcCnt_xy, functionX_rate, 4, 0);
			else{
				Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);
				Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); 
			}			
			
            Reg_Handler(toggle_trigger, OVERFLOW_RATE/trigger_rate, 0, 0); //turn off the trigger toggle
			digitalWrite(2,LOW);    //set the output to low
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
            digitalWrite(0, LOW);
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
			  
        case 0x19:  // update GUI information
            xprintf(PSTR("update: %d %d %d %d %d %d %d %d:\n"), gain_x, bias_x, X_pos_index, x_mode, gain_y, bias_y, Y_pos_index, y_mode);
          break;
			
		case 0x21:	// working mode 1 = default mode = controller mode
			eeprom_write_byte(work_mode,0xff);
			xprintf(PSTR("Reset controller to work in the controller mode!\n"));
			break;

		case 0x22:
		    eeprom_write_byte(work_mode,0x00);
			xprintf(PSTR("Reset controller to work in the PC dumping mode!\n"));
			break;
            
			
		case 0x23: //using Int3 external trigger mode
			PORTK.INT0MASK = 0x08;      //Int3 is used as source for port interrupt 0
			xprintf(PSTR("Enabled Int3 external trigger mode is on!\n"));
			break;
			
		case 0x24: //disable Int3 external trigger mode
			PORTK.INT0MASK = 0x00;      //Int3 is used as source for port interrupt 0
			xprintf(PSTR("Disabled Int3 external trigger mode!\n"));
			break;		

		case 0x26: //read and set resolution for x and y

			for (i=0; i<n_sample; i++){
			   tmp_x += analogRead(2);
			   tmp_y += analogRead(3);
			   _delay_ms(5);
			};
			resolution_x  =  tmp_x / n_sample;
			resolution_y  =  tmp_y / n_sample;
			xprintf(PSTR("resolution_x =  %d:\n"), resolution_x);
			xprintf(PSTR("resolution_y =  %d:\n"), resolution_y);
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
			
		case 0x10:  // get ADC value from a ADC channel (1-4)
            xprintf(PSTR("ADC_value =  %d:\n"), analogRead(argument_byte - 1));
            break;	
		case 0x11 : 
			loadPattern2Panels(argument_byte);
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
            break;
            
        case 0x30: // this is a set function generator frequency
            funcY_freq = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            functionY_rate = OVERFLOW_RATE/funcY_freq;
            if (quiet_mode_on == 0)
                xprintf(PSTR("function Y update frequency = %u.\n"), funcY_freq);
            break;
			
		case 0x35: //set resoultion_x and resolution_y	
			resolution_x = (uint32_t)msg_buffer[1] * 4095/10;
			resolution_y = (uint32_t)msg_buffer[2] * 4095/10;
			break;			
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[3]);
    }
}

void handle_message_length_4(uint8_t *msg_buffer) {
    int16_t setVal;
	//'set_ao'
    switch(msg_buffer[0]) {
        case 0x10: //set a value ranging from 0-32767 (0-10V) to one of the DAC1~4. 
		    setVal = (int16_t) msg_buffer[2] + (256*msg_buffer[3]);
            analogWrite(msg_buffer[1] - 1, setVal);
            break;
		case 0x11:  //set a value ranging from -32767 to 0(-10V-0)  to one of the DAC1-4 
			setVal = (int16_t) msg_buffer[2] + (256*msg_buffer[3]);
			setVal = -setVal;
            analogWrite(msg_buffer[1] - 1, setVal);
            break;
        default:   
			i2cMasterSend(0x00, 8, ERROR_CODES[4]);
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
				
			if (usePreloadedPattern == 1)
				display_preload_frame(frame_num, index_x, index_y);
			else
				fetch_display_frame(frame_num, index_x, index_y);
            
			break;
            
            
        default:
            i2cMasterSend(0x00, 8, ERROR_CODES[5]);
    }
}

//set gain and bias
void handle_message_length_9(uint8_t *msg_buffer) {
    switch(msg_buffer[0]) {
	//load laser trigger pattern first 62 byte data. Laer patter has 128 bytes, but since
	//the value is either 0 or 1, we can combined them in 12 bytes to 
	//save serial communicaiton time

        case 0x01:
            //'send_gain_bias', all of these are signed byte values
            gain_x = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            bias_x = (uint16_t) msg_buffer[3] + (256*msg_buffer[4]);
            gain_y = (uint16_t) msg_buffer[5] + (256*msg_buffer[6]);
            bias_y = (uint16_t) msg_buffer[7] + (256*msg_buffer[8]);
			if (quiet_mode_on == 0)
                xprintf(PSTR("set_gain_bias: gain_x= %d,  bias_x= %d, gain_y= %d, bias_y=%d\n"), gain_x, bias_x, gain_y, bias_y);
            break;
		
		default:
            i2cMasterSend(0x00, 8, ERROR_CODES[6]);
			
	}
}

			
void handle_message_length_62(uint8_t *msg_buffer)
{   
	uint8_t i;

	for (i = 0; i<62; i++)
	{
		laserPattern[i] = msg_buffer[i];
	}

}

//load laser trigger pattern second 63 byte data. Laer patter has 128 bytes, but since
//the value is either 0 or 1, we can combined them in 12 bytes to 
//save serial communicaiton time

void handle_message_length_63(uint8_t *msg_buffer)
{   
	uint8_t i;
	
	 for (i = 0; i<63; i++)
	{
	laserPattern[62 + i] = msg_buffer[i];
	}
	
	if (quiet_mode_on == 0)
        xputs(PSTR("Success set the new laser pattern.\n"));

}

void display_dumped_frame (uint8_t *msg_buffer) {
    uint8_t panel_index;
    uint16_t buffer_index, x_dac_val, y_dac_val;
    //The first two bytes are the x_dac_val, only support positive number 
    //The second two bytes are the y_dac_val, only support positive number 
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
    analogWrite(0, x_dac_val); // make it a value in the range 0 - 32767 (0V - 10V)
    analogWrite(1, y_dac_val);  // make it a value in the range 0 - 32767 (0V - 10V)
    digitalWrite(1, LOW); // set line low at end of frame write
}

void display_preload_frame(uint16_t f_num, uint16_t Xindex, uint16_t Yindex){
    // this function will fetch the current frame from the SD-card and display it.
    // pass in f_num instead of using global frame_num to ensure that the value of
    // frame_num does not change during this function's run
    // suppose f_num is from 0 to (n_num * y_num - 1)
    uint16_t X_dac_val, Y_dac_val;
	uint8_t CMD[2];

	//when preload pattern to panels (super fast mode), we update frame and analog output in this function in stead of
	//in fetch_display_frame because this fuction is ISR and has a higher priority than fetch_display_frame called by 
	//main funciton. In this way, we can keep update frame during f_read to in order to update function data 
	digitalWrite(1, HIGH); 
	//ask all panels to load f_num
	CMD[0] = *((uint8_t *)&f_num + 1) | 0xf0;  // this is the high byte
	CMD[1] = *(uint8_t *)&f_num; //this is the low byte
		
	i2cMasterSend(0, 2, CMD); 	//use 2 to follow the old protocol temporarily
		
	//update analog output after updating frames 		
	X_dac_val = ((uint32_t)Xindex + 1)*32767/x_num; 
	analogWrite(0, X_dac_val); // make it a value in the range 0 - 32767 (0 - 10V)
	Y_dac_val = ((uint32_t)Yindex + 1)*32767/y_num; 
	analogWrite(1, Y_dac_val); // make it a value in the range 0 - 32767 (0 - 10V)
	digitalWrite(1, LOW); // set line low at end of frame write	
}
			

void fetch_display_frame(uint16_t f_num, uint16_t Xindex, uint16_t Yindex){
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
	uint8_t tempVal, bitIndex, arrayIndex;
	
	digitalWrite(1, HIGH); // set line high at start of frame write			
	if (display_flag > 1){      //if flag gets bigger than 1 -> frame skipped
			ledToggle(1);    //toggle LED 1
	}
				
	display_flag = 0;  //clear the display flag

	len = num_panels * bytes_per_panel_frame;
	
	if (len%512 != 0)
		block_per_frame = len/512 + 1;
	else 
		block_per_frame = len/512;
		
	uint8_t  frameBuff[len];
	offset = 512 + (uint32_t)f_num * 512 * block_per_frame;

	res = f_lseek(&file1, offset);
	if ((res == FR_OK) && (file1.fptr == offset)) {
		res = f_read(&file1, frameBuff, len, &cnt);
		if ((res == FR_OK) && (cnt == len)) {	
		

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
		if (quiet_mode_on == 0){
			xputs(PSTR("Error in f_lseek in fetch_display_frame!\n"));
			xprintf(PSTR("RES = %u, f_num= %u, offset = %lu\n"), res, f_num, offset);
		}
	}
		
    //update analog out only when we update a frame
    if (x_mode != 5){
		X_dac_val = ((uint32_t)Xindex + 1)*32767/x_num;
		analogWrite(0, X_dac_val); // make it a value in the range 0 - 32767 (0 - 10V)
	}

    
    if (y_mode != 5){
        Y_dac_val = ((uint32_t)Yindex + 1)*32767/y_num; 
        analogWrite(1, Y_dac_val); // make it a value in the range 0 - 32767 (0 - 10V)
    }

	
	//also update the output lines for quadrant-type learning patterns
	if (Laser_active == 1)
	{
		arrayIndex = index_x/8;  // find the index in laserPattern array for index_x
		bitIndex = index_x - arrayIndex*8;  // find the bit index in a laserPattern byte for index_x
	
		tempVal = laserPattern[arrayIndex];
	
		if ((tempVal & (1<<(7-bitIndex))) == 0)
			digitalWrite(0, LOW);  // turn off laser
		else
			digitalWrite(0, HIGH);   // turn on laser
	}
	
	digitalWrite(1, LOW); // set line low at end of frame write

}

void Update_display(void) {
    int16_t X_rate = 0;
    int16_t Y_rate = 0;
    int16_t X_ADC1, Y_ADC1, X_ADC2, Y_ADC2;
    int16_t temp_ADC_val;
	int32_t temp_index_x, temp_index_y;	
	int16_t  X_val, Y_val;

    //there are five modes 0 - OL, 1 - CL, 2 - CL w Bias, 3 - POS mode with ch5, 4 - POS mode from pos func 5 - function DBG mode
    
    switch(x_mode) {
        case 0:   // open loop - use function generator to set x rate
            X_val = 2*function_X[func_read_index_x];
            X_rate = ((X_val*gain_x)/10 + 5*bias_x)/2;
            break;
        case 1: //closed loop, use CH0 - CH1 to set x rate
            X_ADC1 = analogRead(0)/4;  // 1 volt = 102 frames/sec
            temp_ADC_val = X_val; //the previous value
            X_val = ( 6*temp_ADC_val + 4*X_ADC1 )/10;   //this is a 60% old value, 40% new value smoother
            X_rate = (int16_t)((int32_t)(X_val*gain_x)/10 + 5*bias_x)/2;  //X_val can go as high as 4095, gain_x 100fiu and bias_x 250
            break;
        case 2: //closed loop w bias - use CH0 - CH1, and function gen. to set x rate
            X_ADC1 = analogRead(0)/4; // 1 volt = 102
            temp_ADC_val = X_val; //the previous value
            X_val = ( 6*temp_ADC_val + 4*X_ADC1 )/10;   //this is a 60% old value, 40% new value smoother
            //add in the bias to CL mode on ch X
            X_rate = (int16_t)((int32_t)(X_val*gain_x)/10 + 2*function_X[func_read_index_x] + 5*bias_x)/2;
            break;
        case 3: // POS mode, use CH2 to set the frame position (pos ctrl, not vel ctrl)
	        X_ADC2 = analogRead(2);  //X_ADC2 ranges from 0-4095 (12bit ADC) when input 0-10V

			if (X_ADC2>resolution_x) {X_ADC2 = resolution_x;}
			
			//calculate the index_x                                               
			temp_index_x = ((int32_t)X_ADC2 * x_num * 2 + resolution_x) / ((int32_t) resolution_x * 2) - 1;
				
            if (temp_index_x >= x_num)  {temp_index_x = x_num - 1;} //check if too big
            if (temp_index_x <= 0)  {temp_index_x = 0;} //or too small
			index_x = temp_index_x;
			
            frame_num = index_y*x_num + index_x;
            X_rate = 0;	
			break;
			
		case 4:
		case 5:
			X_rate = 0;
			break;			
    }
    
    switch(y_mode) {
        case 0:   // open loop - use function generator to set x rate
            Y_val = 2*function_Y[func_read_index_y];
            Y_rate = ((Y_val*gain_y)/10 + 5*bias_y)/2;
            break;
        case 1: //closed loop, use CH2 - CH3 to set x rate
            Y_ADC1 = analogRead(1)/4; // 1 volt = 102fps
            temp_ADC_val = Y_val; //the previous value
            Y_val = ( 6*temp_ADC_val + 4*Y_ADC1)/10;   //this is a 60% old value, 40% new value smoother
            Y_rate = (int16_t)((int32_t)(Y_val*gain_y)/10 + 5*bias_y)/2; //Y_val can go as high as 4095, gain_y 100, and bias_y 250.   
            break;
        case 2: //closed loop w bias - use CH2 - CH3, and function gen. to set x rate
            Y_ADC1 = analogRead(1)/4; // 1 volt = 102
            temp_ADC_val = Y_val; //the previous value
            Y_val = ( 6*temp_ADC_val + 4*Y_ADC1)/10;   //this is a 60% old value, 40% new value smoother
            //add in the bias to CL mode on ch Y
            Y_rate = (int16_t)((int32_t)(Y_val*gain_y)/10 + 2*function_Y[func_read_index_y] + 5*bias_y)/2; //Y_val can go as high as 4095
            break;
            //do something with errors here for default case
        case 3: // POS mode, use CH3 to set the frame position (pos ctrl, not vel ctrl)
            Y_ADC2 = analogRead(3);   //Y_ADC2 ranges from 0-4095 when input 0-10V
			
			if (Y_ADC2>resolution_y) {Y_ADC2 = resolution_y;}
			
			//calculate the index_x                                               
			temp_index_y = ((int32_t)Y_ADC2 * y_num * 2 + resolution_y) / ((int32_t) resolution_y * 2) - 1;
			
            if (temp_index_y >= y_num)  {temp_index_y = y_num - 1;} //check if too big
            if (temp_index_y <= 0)  {temp_index_y = 0;} //or too small
			index_y = temp_index_y;
            frame_num = index_y*x_num + index_x;
            Y_rate = 0;				
			
            break;
			
		case 4:
		case 5:
			Y_rate = 0;
			break;
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
		index_x = 0;
    
    
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

void loadPattern2Panels(uint8_t pat_num) {
    //sets the pattern ID, in future return 0 or 1 if error/succeed
	//currently we only support gs = 1 and non-row compression mode
    uint16_t num_frames;
    static uint8_t str[12];
    uint8_t  pattDataBuff[512];
    uint8_t res;
	uint8_t j, panel_index;
	uint16_t len, cnt, buff_index;
	uint32_t offset;
	//uint16_t X_dac_val, Y_dac_val;
	uint8_t sreg = SREG;
	uint8_t block_per_frame;
	uint8_t tempVal, bitIndex, arrayIndex;
	uint8_t CMD[34];
	uint16_t f_num;
	uint16_t bytes_per_panel_patten;
				
    
    if (pat_num < 10)
        sprintf(str, "pat000%d.pat\0", pat_num);
    else if (pat_num < 100)
        sprintf(str, "pat00%d.pat\0", pat_num);
    else if (pat_num < 1000)
        sprintf(str, "pat0%d.pat\0", pat_num);
    else
        if (quiet_mode_on == 0)
            xputs(PSTR("pat_num is too big.\n"));
    
    res = f_open(&file1, str, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        res = f_read(&file1, pattDataBuff, 512, &cnt); // read the 10 byte test header info block
        if ((res == FR_OK) && (cnt == 512)) {

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
			bytes_per_panel_patten = num_frames*bytes_per_panel_frame;
            if (quiet_mode_on == 0){
                xprintf(PSTR("preload pattern %u:\n"), pat_num);
                xprintf(PSTR("  x_num = %u\n  y_num = %u\n  num_panels = %u\n  gs_value = %u\n row_compression = %u\n"),
                        x_num, y_num, num_panels, gs_value, row_compress);
				xprintf(PSTR("  bytes_per_panel_frame = %u\n  bytes_per_panel_pattern = %u\n"),
                        bytes_per_panel_frame, bytes_per_panel_patten);
            }
			
			//since panel has only 800 byte for the pattern per panel, we need to check before tranfer
			if (bytes_per_panel_patten <= 800)
			{
				for(f_num = 0; f_num < num_frames; ++f_num)
				{
					len = num_panels * bytes_per_panel_frame;
					block_per_frame = len/512 + 1;
					uint8_t  frameBuff[len];
					offset = 512 + (uint32_t)f_num * 512 * block_per_frame;
					//xprintf(PSTR("offset %lu, f_num %u\n"), offset, f_num);
					res = f_lseek(&file1, offset);
					if ((res == FR_OK) && (file1.fptr == offset)) {
						res = f_read(&file1, frameBuff, len, &cnt);
						if ((res == FR_OK) && (cnt == len)) {							
						
							buff_index = 0;
							
							CMD[bytes_per_panel_frame] = *(uint8_t *)&f_num;
							CMD[bytes_per_panel_frame+1] = *((uint8_t *)&f_num + 1);	
								
							for (panel_index = 1; panel_index <= num_panels; panel_index++){
								//FLASH = &frameBuff[buff_index];

								for(j=0;j<bytes_per_panel_frame;++j)
								{
									CMD[j] = frameBuff[buff_index + j];									
								}
					
								
								if (row_compress == 0)
									i2cMasterSend(panel_index, bytes_per_panel_frame+2, CMD);
								else{
									switch(gs_value) {
									case 1: //the data format is [5, data, f_num_LB, f_num_HB , x, x]
										i2cMasterSend(panel_index, 5, CMD);
										break;
									case 3: //the data format is [6, data1, data2, data 3, f_num_LB, f_num_HB , x]
										i2cMasterSend(panel_index, 6, CMD);
										break;
									case 4: //the data format is [7, data1, data2, data 3, data4, f_num_LB, f_num_HB , x]
										i2cMasterSend(panel_index, 7, CMD);
										break;
									default:
										break;
									}
								}
								
								buff_index += bytes_per_panel_frame;

							} //end of for all panels loop
						//xprintf(PSTR("f_num_LB %u, f_num_HB %u\n"), CMD[bytes_per_panel_frame], CMD[bytes_per_panel_frame+1]);		
						}
						else {
							if (quiet_mode_on == 0){
								xputs(PSTR("Error in f_read in loadPattern2Panels!\n"));
								xprintf(PSTR("RES = %u, f_num= %u, cnt= %u\n"), res, f_num, cnt);
								return;
							}
						}
					} else {
						
						if (quiet_mode_on == 0){
							xputs(PSTR("Error in f_lseek in loadPattern2Panels!\n"));
							xprintf(PSTR("RES = %u, f_num= %u, offset = %lu\n"), res, f_num, offset);
							return;
						}
					}
				}
			}
			else{
				xprintf(PSTR("Pattern size is upto 800 byte per panel.\n"));
				xprintf(PSTR("This pattern size is %lu\n"), bytes_per_panel_patten);
				xprintf(PSTR("Failed to load this Pattern to Panels\n"));
				usePreloadedPattern = 0;
				//set_pattern(pat_num); didn't work
				return;
			}
			
        } else {
            if (quiet_mode_on == 0)
                xputs(PSTR("Error reading in pattern file\n"));
			return;
        }
    } else {
        if (quiet_mode_on == 0)
            xputs(PSTR("Error opening pattern file\n"));
			return;
    }
	
	res = f_close(&file1);
	usePreloadedPattern = 1;
	xprintf(PSTR("Successfully load pattern %u to the panels\n"), pat_num);
	
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
            xputs(PSTR("pat_num is too big.\n"));
   
	
    res = f_close(&file1);
    
    res = f_open(&file1, str, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        res = f_read(&file1, pattDataBuff, 512, &cnt); // read the 10 byte test header info block
        if ((res == FR_OK) && (cnt == 512)) {

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
            fetch_display_frame(frame_num, index_x, index_y);
        } else
		
        xputs(PSTR("Error reading in pattern file\n"));
		
    } else
    xputs(PSTR("Error opening pattern file\n"));

	usePreloadedPattern = 0;
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
	
	for(index_y = 0; index_y < y_num; index_y++)
		for(index_x = 0; index_x < x_num; index_x++)
		{
			frame_ind = index_y*x_num + index_x;

			if(usePreloadedPattern == 1)
				display_preload_frame(frame_ind, index_x, index_y);
			else
				fetch_display_frame(frame_ind, index_x, index_y);
		}
	
    bench_time = timer_coarse_toc();
    frame_rate = ((uint32_t)num_frames*1000)/bench_time;
    xprintf(PSTR(" bench_time = %lu ms, frame_rate = %u\n"), bench_time, frame_rate);
	
	//reset index_x and index_y
	index_x=0;
	index_y=0;
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
            
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);//disable ISR
            func_ID_X = 0;
            funcSize_x = FUNCTION_LENGTH;
            for (funcCnt = 0; funcCnt < FUNCTION_LENGTH; funcCnt++)
				{ function_X[funcCnt] = 10; }
            func_read_index_x = 0;
			default_func_x = 1;
			num_buffer_load_x = 1;
			last_load_x = 0;
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);//don't need to enable ISR
            break;
        case 2:
            if (quiet_mode_on == 0)
                xputs(PSTR("Setting default function for Y.\n"));
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0);//disable ISR
            func_ID_Y = 0;
            funcSize_y = FUNCTION_LENGTH;
            for (funcCnt = 0; funcCnt < FUNCTION_LENGTH; funcCnt++)
            { function_Y[funcCnt] = 10; }
			func_read_index_y = 0;
			default_func_y = 1;
			num_buffer_load_y = 1;
			last_load_y = 0;
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);//don't need to enable ISR
            break;
        default:
                xputs(PSTR("Wrong function channel number.\n"));
			break;
    }
    
}


void set_pos_func(uint8_t func_channel, uint8_t func_id) {
    uint16_t cnt;
    uint8_t tmpCnt;

    uint8_t str[12];
    uint8_t res, func_name_len;
    uint8_t posFuncBuff[512];
    
    
    if (func_id < 10)
        sprintf(str, "pos000%d.fun\0", func_id);
    else if (func_id < 100)
        sprintf(str, "pos00%d.fun\0", func_id);
    else if (func_id < 1000)
        sprintf(str, "pos0%d.fun\0", func_id);
    else
            xputs(PSTR("function id is too big.\n"));
    
    switch(func_channel) {
        case 1:    //channel x
            //read the header block and send back the function name
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);//disable ISR
            
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
                    
                } else 
                        xputs(PSTR("Error f_read set_pos_func X\n"));
            } else 
                    xputs(PSTR("Error f_open in set_pos_func X.\n"));
					
	
			last_load_x = funcSize_x % FUNCTION_LENGTH;
			if (last_load_x == 0)
				num_buffer_load_x = funcSize_x / FUNCTION_LENGTH;
			else
				num_buffer_load_x = funcSize_x / FUNCTION_LENGTH + 1;

			if (quiet_mode_on == 0)
			{
				xprintf(PSTR("funcSize_x = %u\n"), funcSize_x);
				xprintf(PSTR("last_load_x = %u\n"), last_load_x);
				xprintf(PSTR("num_buffer_load_x = %u\n"), num_buffer_load_x);			
			}
			
			default_func_x = 0;
			
			//update the function buffer
			fetch_update_funcX(1,0);
			
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);//enable ISR
            break;
            
        case 2:
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //disable ISR
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
                    
                    func_ID_Y = func_id;
                    
                    Stop = 1;
                    display_flag = 0;  //clear the display flag
                    if (quiet_mode_on == 0)
                        xprintf(PSTR("Setting position function %u for Y\n"), func_id);
                    //xprintf(PSTR("fun Y: %s\n function Y size: %lu bytes\n"),
                    //       func_name_y, funcSize_y);
                    
                } else {
                        xputs(PSTR("Error f_read set_pos_func Y.\n"));
                }
            } else {
                if (quiet_mode_on == 0)
                    xputs(PSTR("Error f_open in set_pos_func Y.\n"));
            }
			
			last_load_y = funcSize_y % FUNCTION_LENGTH;
			if (last_load_y == 0)
				num_buffer_load_y = funcSize_y / FUNCTION_LENGTH;
			else
				num_buffer_load_y = funcSize_y / FUNCTION_LENGTH + 1;
		
	
			if (quiet_mode_on == 0){
				xprintf(PSTR("funcSize_y = %u\n"), funcSize_y);
				xprintf(PSTR("last_load_y = %u \n"), last_load_y);
				xprintf(PSTR("num_buffer_load_y = %u\n"), num_buffer_load_y);			
			}
			
			default_func_y = 0;
			
			//update the function buffer
			fetch_update_funcY(1,0);
			
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1);//enable ISR
            break;
            
        default:
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
            xputs(PSTR("function id is too big.\n"));
    
    switch(func_channel) {
        case 1:    //channel x
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0); //disable ISR
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
                        xputs(PSTR("Error f_read in set_vel_func X.\n"));
                }
            } else {
                    xputs(PSTR("Error f_open in set_vel_func X.\n"));
            }
			
			last_load_x = funcSize_x % FUNCTION_LENGTH;
			
			if(last_load_x == 0)
					num_buffer_load_x = funcSize_x/FUNCTION_LENGTH;
			else
					num_buffer_load_x = funcSize_x / FUNCTION_LENGTH + 1;
				
			
			if (quiet_mode_on == 0)
			{
				xprintf(PSTR("funcSize_x = %u\n"), funcSize_x);
				xprintf(PSTR("last_load_x = %u\n"), last_load_x);
				xprintf(PSTR("num_buffer_load_x = %u\n"), num_buffer_load_x);			
			}
			
			default_func_x = 0;
			
			//update the function buffer
			fetch_update_funcX(1,0);
			
            //Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1); //enable ISR
            break;
            
        case 2:
            
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); //disable ISR
            
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
                        xputs(PSTR("Error f_read in set_vel_func Y.\n"));
                }
            } else {
                    xputs(PSTR("Error f_open in set_vel_func Y.\n"));
            }
			
			last_load_y = funcSize_y % FUNCTION_LENGTH;
			
			if (last_load_y == 0)
				num_buffer_load_y = funcSize_y / FUNCTION_LENGTH;		
			else
				num_buffer_load_y = funcSize_y / FUNCTION_LENGTH + 1;
			
		
			if (quiet_mode_on == 0){
				xprintf(PSTR("funcSize_y = %u\n"), funcSize_y);
				xprintf(PSTR("last_load_y = %u\n"), last_load_y);
				xprintf(PSTR("num_buffer_load_y = %u\n"), num_buffer_load_y);			
			}
			
			default_func_y = 0;
			
			//update function buffer
            fetch_update_funcY(1,0);
			
            //Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); //enable ISR
            break;
            
        default:
                xputs(PSTR("Error input for function channel.\n"));
            break;
    }
}
	

void update_funcCnt_x(void) {
    int16_t X_dac_val;
    int16_t temp_ADC_val;


	
	if (!func_buffer_size_x){
		xputs(PSTR("Ring buffer function_x is empty\n"));
		return;
	}
	
	
	//Move mode 4 and 5 from update_display() to here since the update frequency can be changed
	switch(x_mode){
	
		case 1:
		case 2:
		case 3:
			break;
			
		case 4:
			//only use temp_ADC_val as a temp variable, just not to create an additional one		
			temp_ADC_val = X_pos_index + function_X[func_read_index_x];
			if (temp_ADC_val >= 0) {index_x = temp_ADC_val%x_num; }
			if (temp_ADC_val < 0) {index_x = x_num - ((abs(temp_ADC_val))%x_num) -1;} //index_x should already smaller than x_num
			frame_num = index_y*x_num + index_x;
			break;
			
		case 5:   // in function DBG mode - show the function gen
			//3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V 
			X_dac_val = function_X[func_read_index_x]*33;
			analogWrite(0, X_dac_val); // make it a value in the range -32767 - 32767 (-10V - 10V)
			break;
		
    }
	
	
    func_read_index_x++; 
	if (func_read_index_x >= BUFFER_LENGTH/2)
		func_read_index_x = 0;
		
    func_buffer_size_x--;
}

void fetch_update_funcX(uint8_t fReset, uint8_t num_of_load_x) {
    //num_of_load_x ranges from 0 to (num_buffer_load_x - 1)	
	//fReset = 1 mean we need to load the first FUNCTIONLLENGTH of function data into buffer
	//because users always want the functions start from the beginning every time send start command
	uint16_t cnt, j, offset;
	uint8_t res;
	//uint8_t funcXBuff[2*FUNCTION_LENGTH];
	uint8_t tempBuff[FUNCTION_LENGTH];
	uint16_t loadXBufferSize;
	//digitalWrite(0, HIGH);	
			
	//xprintf(PSTR("num_of_load_x =  %u\n"), num_of_load_x);			
	if (func_buffer_size_x >= BUFFER_LENGTH/2){
		xputs(PSTR("Ring buffer function_x is full\n"));
		return;
		}
		
	if (fReset == 1){ 
		func_read_index_x = 0;
		func_write_index_x = 0;
		func_buffer_size_x = 0;       
	}
	
	offset = 512 + num_of_load_x * FUNCTION_LENGTH;
            
	res = f_lseek(&file2, offset);
	if ((res == FR_OK) && (file2.fptr == offset)) {
		//num_of_load_x ranges from 0 to num_buffer_load_x - 1
		if ((num_of_load_x ==  num_buffer_load_x - 1) && (last_load_x != 0))
			loadXBufferSize = last_load_x;
		else 
			loadXBufferSize = FUNCTION_LENGTH;
		
		//load 100 bytes data to temBuff
		res = f_read(&file2, tempBuff, loadXBufferSize, &cnt);
		if (!((res == FR_OK) && (cnt == loadXBufferSize))) {
				xprintf(PSTR("res =  %u\n"), res);
				xputs(PSTR("Error in f_read in in fetch_update_funcX\n"));
			}

		
		for (j = 0; j< cnt; j+=2){
		//for (j = 0; j< loadXBufferSize; j+=2){
			function_X[func_write_index_x] = (uint16_t)tempBuff[j] + (uint16_t)tempBuff[j+1]*256 ; 		
			func_write_index_x++;  
			if (func_write_index_x >= BUFFER_LENGTH/2) //0-100
				func_write_index_x = 0;
				
			func_buffer_size_x ++;  //atomic operation
		}
		
            //xprintf(PSTR("func_write_index_x =  %u\n"), func_write_index_x);
	} else {
			xprintf(PSTR("res =  %u\n"), res);
			xputs(PSTR("Error in f_lseek in fetch_update_funcX\n"));
	}
	//digitalWrite(0, LOW);
}

void update_funcCnt_y(void) {
    int16_t Y_dac_val;
    int16_t temp_ADC_val;

	
	if (!func_buffer_size_y){
		xputs(PSTR("Ring buffer function_Y is empty\n"));
		return;
		}
    
	switch(y_mode){
		case 1:
		case 2:
		case 3:
			break;
			
        case 4:
            //only use temp_ADC_val as a temp variable, just not to create an additional one
            temp_ADC_val = (Y_pos_index + function_Y[func_read_index_y]);
            if (temp_ADC_val >= 0) {index_y = temp_ADC_val%y_num; }
            if (temp_ADC_val < 0) {index_y = y_num - ((abs(temp_ADC_val))%y_num) - 1;  } //index_y should always smaller than y_num
            frame_num = index_y*x_num + index_x;		
            break;
	
		case 5:   // in function DBG mode - show the function gen
			//3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V 
			Y_dac_val = function_Y[func_read_index_y]*33;  
			analogWrite(1, Y_dac_val); // make it a value in the range -32767 - 32767 (-10V - 10V)
			break;
		
    }
	
	func_read_index_y++; 
	if (func_read_index_y >= BUFFER_LENGTH/2)
		func_read_index_y = 0;
	func_buffer_size_y--;
}

void fetch_update_funcY(uint8_t fReset, uint8_t num_of_load_y) {
    //num_of_load_y ranges from 0 to (num_buffer_load_x - 1)	
	//fReset = 1 mean we need to load the first FUNCTIONLLENGTH of function data into buffer
	//because users always want the functions start from the beginning every time send start command
	uint16_t cnt, j, offset;
	uint8_t res;
	//uint8_t funcXBuff[2*FUNCTION_LENGTH];
	uint8_t tempBuff[FUNCTION_LENGTH];
    uint16_t loadYBufferSize;
    //xprintf(PSTR("num_of_load_y =  %u\n"), num_of_load_y);		
	if (func_buffer_size_y >= BUFFER_LENGTH/2){
		xputs(PSTR("Ring buffer function_Y is full\n"));
		return;
		}
		
	if (fReset == 1){
        func_buffer_size_y = 0;
		func_read_index_y = 0;
		func_write_index_y = 0;        
	}
	
	offset = 512 + num_of_load_y * FUNCTION_LENGTH;
            
	res = f_lseek(&file3, offset);
	if ((res == FR_OK) && (file3.fptr == offset)) {
		
		if ((num_of_load_y ==  num_buffer_load_y - 1) && (last_load_y != 0))
			loadYBufferSize = last_load_y;
		else 
			loadYBufferSize = FUNCTION_LENGTH;
		
		//load 100 bytes data to temBuff
		res = f_read(&file3, tempBuff, loadYBufferSize, &cnt);
		if (!((res == FR_OK) && (cnt == loadYBufferSize))) {
		
			xprintf(PSTR("res =  %u\n"), res);
			xputs(PSTR("Error in f_read in in update_funcCnt_y\n"));
		}
		
		for (j = 0; j< cnt; j+=2){
			function_Y[func_write_index_y] = (uint16_t)tempBuff[j] + tempBuff[j+1]*256; 
			func_write_index_y++; 
			if (func_write_index_y >= BUFFER_LENGTH/2)
				func_write_index_y = 0;
			func_buffer_size_y++;  //atomic operation
		}
	//xprintf(PSTR("func_write_index_y =  %u\n"), func_write_index_y);			
	} else {
			xprintf(PSTR("res =  %u\n"), res);
			xputs(PSTR("Error in f_lseek in update_funcCnt_y load next buffer\n"));
	}
} 

void update_funcCnt_xy(void) {
    int16_t X_dac_val, Y_dac_val;
    int16_t temp_ADC_val;

	
	if (!func_buffer_size_x){
		xputs(PSTR("Ring buffer function_x is empty\n"));
		return;
	}
	
	if (!func_buffer_size_y){
		xputs(PSTR("Ring buffer function_Y is empty\n"));
		return;
	}
	
	//if sync_xy_func== 1, we preassume Y_mode is always equal x_mode are the same
	switch(x_mode){
	
		case 1:
		case 2:
		case 3:
			break;			
			
		case 4:
			//only use temp_ADC_val as a temp variable, just not to create an additional one		
			temp_ADC_val = X_pos_index + function_X[func_read_index_x];
			if (temp_ADC_val >= 0) {index_x = temp_ADC_val%x_num; }
			if (temp_ADC_val < 0) {index_x = x_num - ((abs(temp_ADC_val))%x_num) -1;} //index_x should already be smaller than x_num
			
			temp_ADC_val = (Y_pos_index + function_Y[func_read_index_y]);
			if (temp_ADC_val >= 0) {index_y = temp_ADC_val%y_num; }
			if (temp_ADC_val < 0) {index_y = y_num - ((abs(temp_ADC_val))%y_num) - 1;} //index_y should always be smaller than y_num
			
			frame_num = index_y*x_num + index_x;
			break;
			
		case 5:   // in function DBG mode - show the function gen
			//3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V 
			X_dac_val = function_X[func_read_index_x]*33;
			analogWrite(0, X_dac_val); // make it a value in the range -32767 - 32767 (-10V - 10V)
			//3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V 
			Y_dac_val = function_Y[func_read_index_y]*33;  
			analogWrite(1, Y_dac_val); // make it a value in the range -32767 - 32767 (-10V - 10V)			
			break;
    }
	
	
    func_read_index_x++; 
	if (func_read_index_x >= BUFFER_LENGTH/2)
		func_read_index_x = 0;
		
    func_buffer_size_x--;
	
	func_read_index_y++; 
	if (func_read_index_y >= BUFFER_LENGTH/2)
		func_read_index_y = 0;
	
	func_buffer_size_y--;
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
            }else 
                xputs(PSTR("Error f_lseek in SDInfo.mat.\n"));
        }
        //send the remained data
        offset = (b1-1)*50;
        res=f_lseek(&file4, offset);
        if ((res == FR_OK) && (file4.fptr == offset)) {
            res = f_read(&file4, matBuff, fileRemain, &cnt);
            if ((res == FR_OK) && (cnt == fileRemain)) {
                send_Tx_buffer(matBuff, fileRemain);
            }
        }else
            xputs(PSTR("Error f_lseek in remained data.\n"));
        
        f_close(&file4);
    } else 
            xputs(PSTR("Error f_open in SDInfo.mat.\n"));//end if (res == FR_OK
    
}

//external trigger mode for Int3 to start playing pattern

ISR(PORTK_INT0_vect)
{

//set these to zero so that start at beginning of function - useful for putting in a set amount of expansion
func_read_index_x = 0;
func_read_index_y = 0;

next_block_x = 1;
next_block_y = 1;

Stop = 0;
display_flag = 0;  //clear the display flag
Reg_Handler(Update_display, UPDATE_RATE, 1, 1);
Reg_Handler(increment_index_x, UPDATE_RATE, 2, 0); //initilize the 2 and 3 priority interupts to a fast rate so that
Reg_Handler(increment_index_y, UPDATE_RATE, 3, 0); // the countdown is fast until the setting of the next rate
													//by the Update_display interupt.
if (default_func_x)
	Reg_Handler(update_funcCnt_x, functionX_rate, 4, 0);
else{
	update_funcCnt_x();//add this because the function cnt is updated without delay
	Reg_Handler(update_funcCnt_x, functionX_rate, 4, 1);
	}
if (default_func_y)
	Reg_Handler(update_funcCnt_y, functionY_rate, 5, 0); 
else{
	update_funcCnt_y();//add this because the function cnt is updated without delay
	Reg_Handler(update_funcCnt_y, functionY_rate, 5, 1); 					
	}

xputs(PSTR("Int3 catches a rising edge trigger!\n"));
}
