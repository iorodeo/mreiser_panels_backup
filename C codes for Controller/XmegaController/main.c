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

#define FALSE	0
#define TRUE	(!FALSE)

// Slot numbers of the functions called by the timer interrupt, one function per slot.
#define ISR_TOGGLE_TRIGGER		0
#define ISR_UPDATE_DISPLAY		1
#define ISR_INCREMENT_INDEX_X	2
#define ISR_INCREMENT_INDEX_Y	3
#define ISR_INCREMENT_FUNC_X	4
#define ISR_INCREMENT_FUNC_Y	5

#define FRAMEFROMXY(x,y)    ((y)*g_x_max + (x))
#define MIN(x,y)            ((x)<(y)?(x):(y))
#define MAX(x,y)            ((x)>(y)?(x):(y))
#define CLIP(x,lo,hi)       MAX((lo),MIN((x),(hi)))

#define DAQRESOLUTION       ((1<<12)-1)

// Globals - try to cut down on these.
volatile uint8_t  g_b_running = FALSE;
uint8_t           g_b_laseractive = FALSE;
uint8_t           g_b_quiet_mode=TRUE;

uint8_t           g_display_count = 0;
uint16_t          g_adc_x_max = DAQRESOLUTION;	// 12 bit ADC
uint16_t          g_adc_y_max = DAQRESOLUTION;

uint16_t          g_x_max=1, g_y_max=1;  //the max index for x and y
volatile uint16_t g_x, g_y;          // the current index of x and y
int16_t           g_x_initial, g_y_initial;  // The initial position set by the user.

uint8_t           gs_value, g_bytes_per_panel_frame, g_row_compress, g_ident_compress;
uint8_t           g_num_panels = 0;
uint8_t           g_mode_x, g_mode_y;
volatile uint16_t g_index_frame = 0;

uint32_t          g_nbytes_func_x = RINGBUFFER_LENGTH; //function file size
uint32_t          g_nbytes_func_y = RINGBUFFER_LENGTH;
int16_t           g_buf_func_x[RINGBUFFER_LENGTH];  // Ring buffer for function data.
int16_t           g_buf_func_y[RINGBUFFER_LENGTH];
volatile uint8_t  g_index_func_x_read = 0;  // Read index for the ring buffer g_buf_func_x[RINGBUFFER_LENGTH]
volatile uint8_t  g_index_func_y_read = 0;
volatile uint8_t  g_index_func_x_write = 0; // Write index for the ring buffer g_buf_func_x[RINGBUFFER_LENGTH]
volatile uint8_t  g_index_func_y_write = 0;
volatile uint8_t  g_filllevel_buf_func_x = 0;  // How far ahead of the read index is the write index.
volatile uint8_t  g_filllevel_buf_func_y = 0;

volatile uint8_t  g_iblock_func_x = 1;			// Index of the current block.
volatile uint8_t  g_iblock_func_y = 1;
uint16_t          g_nblocks_func_x = 1;			// Total number of blocks.
uint16_t          g_nblocks_func_y = 1;
uint16_t          g_nbytes_final_block_x = 0;	// Length of the final block.
uint16_t          g_nbytes_final_block_y = 0;

volatile uint8_t  g_b_default_func_x = TRUE;
volatile uint8_t  g_b_default_func_y = TRUE;
volatile uint8_t  sync_XY_func = 0;
uint16_t          g_id_func_x = 0;
uint16_t          g_id_func_y = 0;
uint16_t          g_period_func_x = FUNCTION_PERIOD;
uint16_t          g_period_func_y = FUNCTION_PERIOD;

uint8_t           g_laserpattern[125];
int8_t            g_gain_x, g_gain_y;
int8_t            g_bias_x, g_bias_y;
int8_t            g_b_xrate_greater_yrate=FALSE;
uint16_t          g_trigger_rate = 200;

FIL               g_file_pattern;        //g_file_pattern pattern file;
FIL               g_file_func_x;        //g_file_func_x function file for x channel
FIL               g_file_func_y;        //g_file_func_y function file for y channel,
FIL               g_file_arenaconfig;   //g_file_arenaconfig for SD.mat or arena config file

//max frame size = 32bpp * 4rows * 12col = 1536
uint8_t           g_ch_from_panel[129];
uint8_t           g_adr_from_panel[129]; // panel twi address mapping, we can have same address in different channels



static const uint8_t VERSION[] = "1.3\0";
static const uint8_t SDInfo[] = "SD.mat\0";


/*---------------------------------------------------------*/
/* TWIC Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIC_TWIM_vect)
{
    TWI_MasterInterruptHandler(&twi1);
}

/*---------------------------------------------------------*/
/* TWID Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWID_TWIM_vect)
{
    TWI_MasterInterruptHandler(&twi2);
}

/*---------------------------------------------------------*/
/* TWIE Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIE_TWIM_vect)
{
    TWI_MasterInterruptHandler(&twi3);
}

/*---------------------------------------------------------*/
/* TWIF Master Interrupt vector.                           */
/*---------------------------------------------------------*/

ISR(TWIF_TWIM_vect)
{
    TWI_MasterInterruptHandler(&twi4);
}


//external trigger mode for int2 to start playing pattern
ISR(PORTK_INT0_vect)
{
	//set these to zero so that we start at beginning of function - useful for putting in a set amount of expansion.
	g_index_func_x_read = 0;
	g_index_func_y_read = 0;
	g_b_running = TRUE;
	g_display_count = 0;  //clear the display count

	Reg_Handler(update_display,    UPDATE_PERIOD,   ISR_UPDATE_DISPLAY,    TRUE);
	Reg_Handler(increment_index_x, UPDATE_PERIOD,   ISR_INCREMENT_INDEX_X, FALSE); //initialize ISRs to a fast rate so that the countdown is fast until the
	Reg_Handler(increment_index_y, UPDATE_PERIOD,   ISR_INCREMENT_INDEX_Y, FALSE); // setting of the next rate by the update_display interrupt.

    if (g_b_default_func_x)
    	Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);
    else{
    	update_position_x();//add this because the function cnt is updated without delay
    	Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, TRUE);
	}
	
    if (g_b_default_func_y)
    	Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE); 
    else{
    	update_position_y();//add this because the function cnt is updated without delay
    	Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, TRUE); 			
	}


	xputs(PSTR("INT3 catches a rising edge trigger!\n"));
}



int main(void)
{
	FATFS       g_fatfs;       // File system object
    uint8_t     sta, res, b1, temp;
    uint16_t    cnt;
    uint16_t    message_length;
    uint16_t    i;
    uint8_t     bufTemp[128];
    uint8_t     workingModes;
    uint8_t     rightBufferXLoaded = 0, rightBufferYLoaded = 0;
    uint16_t    index_frame_prev = 999;  //just chosen at random
    uint8_t     index_func_x_read_prev = 199;
    uint8_t     index_func_y_read_prev = 199;
    
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
    
    for (i=0; i<RINGBUFFER_LENGTH; i++)
    { 
    	g_buf_func_x[i] = g_buf_func_y[i] = 10;
    }// here we use 10 as the equivalent for 1 V.
    
    
    //initialize laser pattern
    for (i=0; i<125; i++)
    { 
        if (((i>=0) && (i<=2)) || ((i>=6) && (i<=8)))
            g_laserpattern[i] = 255;
        else
            g_laserpattern[i] = 0;
    }
    
    
    //initializations
    g_x_initial = g_y_initial = g_x = g_y = 0;
    g_bias_x = g_bias_y = 0;
    g_gain_x = g_gain_y = 0;
    g_mode_x = g_mode_y = 0;
    gs_value = 1;
    g_row_compress = FALSE;
    g_ident_compress = FALSE; // enable this to substitute simpler panel pattern for uniform pattern patches
    
    temp = eeprom_read_byte(arena_config);
    if (temp == 0xff)     //there is no configuration file and use default value 
    {
    	// create default panel mapping
        for (b1 = 0; b1 <= 128; b1++)
        {
            g_ch_from_panel[b1] = b1 % 4;
            if (b1 && (g_ch_from_panel[b1] == 0))
                g_ch_from_panel[b1] = 4;
            g_adr_from_panel[b1] = b1; // panel address identity mapping
        }
    } 
    else //load panel mapping from EEPROM
    {
        for (b1 = 0; b1 <= 128; b1++)
        {
            g_ch_from_panel[b1] = eeprom_read_byte(arena_config + b1);
            g_adr_from_panel[b1] = b1; // panel address identity mapping
        }
    }
    
    xputs(PSTR("\nMain Controller Works\n"));
    
    // get the fat file system mounted
    ledWrite(LED1, ON);
    sta = STA_NOINIT;
    while (sta & STA_NOINIT)
    {
        sta = disk_status(0);
        if(sta & STA_NODISK)
        {
            xputs(PSTR("Insert SD card"));
            uart_crlf();
            while (sta & STA_NODISK)
            {
                sta = disk_status(0);
            }
        }
        if(sta & STA_PROTECT)
        {
            xputs(PSTR("SD card is Write Protected!\n"));
        }
        // Initialize SD Card, do 4 attempts
        for(b1 = 0; b1 < 4; )
        {
            sta = disk_initialize(0);
            if(sta & STA_NOINIT) 
            	b1++;
            else 
            	break;
            _delay_ms(50);
        }
        if(sta & STA_NOINIT)
        {
            xputs(PSTR("Initialization failed!!!\n"));
            sta = disk_status(0);
            while (!(sta & STA_NODISK))
            {
                sta = disk_status(0);
            }
        }
    }
    xputs(PSTR("SD card is initialized\n"));
    
    if (disk_ioctl(0, MMC_GET_TYPE, &b1) == RES_OK)
    {
        xputs(PSTR("Card type: "));
        switch(b1)
        {
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
    res = f_mount(0, &g_fatfs);
    switch(res)
    {
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
    if (workingModes == 0xff)
    {
        uint8_t msg_buffer[65];
        xputs(PSTR("Current working mode is the Controller mode!\n"));
        
        // Main loop.  Wait for communication from PC over UART
        while(1)
        {
            if (uart_test())
            {
                message_length = fill_Rx_buffer(&msg_buffer[0]);
                switch(message_length)
                {
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
                    case 5: // if length 5, then decode, set x,y index, or set gain, bias
                        handle_message_length_5(&msg_buffer[0]);
                        break;
                    case 62: //if length 62, then set laser trigger pattern first 62 byte
                        handle_message_length_62(&msg_buffer[0]);
                        break;
                    case 63: //if length 63, then set laser trigger pattern second 63 byte
                        handle_message_length_63(&msg_buffer[0]);
                        break;    
                    default:
                        i2cMasterSend(0x00, 8, ERROR_CODES[7]);
                } // switch()
            } // if (uart_test())
            
            // If running, then update display.
            if (g_b_running)
            {
            	// Only send out new pattern if the pattern index has changed.
                if (g_index_frame != index_frame_prev)
                {
                    index_frame_prev = g_index_frame; //update the 'old' frame number
                    fetch_and_display_frame(&g_file_pattern, g_index_frame, g_x, g_y);
                }
                
                //g_filllevel_buf_func_x in word, 2 bytes.
                if (!g_b_default_func_x && (g_filllevel_buf_func_x <= RINGBUFFER_LENGTH/4) && (g_index_func_x_read != index_func_x_read_prev))
                {                    
                    index_func_x_read_prev = g_index_func_x_read;
                    fetch_block_func_x(&g_file_func_x, FALSE, g_iblock_func_x);
                    g_iblock_func_x = (g_iblock_func_x+1) % g_nblocks_func_x;
                    //xprintf(PSTR("g_filllevel_buf_func_x=%u, g_index_func_x_read=%u\n"), g_filllevel_buf_func_x, g_index_func_x_read);
                }

                if (!g_b_default_func_y && (g_filllevel_buf_func_y <= RINGBUFFER_LENGTH/4) && (g_index_func_y_read != index_func_y_read_prev))
                {                    
                    index_func_y_read_prev = g_index_func_y_read;
                    fetch_block_func_y(&g_file_func_y, FALSE, g_iblock_func_y);
                    g_iblock_func_y = (g_iblock_func_y + 1)%g_nblocks_func_y;
                    //xprintf(PSTR("g_filllevel_buf_func_y=%u, g_index_func_y_read=%u\n"), g_filllevel_buf_func_y, g_index_func_y_read);
                }

            }
        }
    } //if (workingModes == 0xff)
    else
    {
        uint8_t msg_buffer[1550];
        xputs(PSTR("Current working mode is the PC dumping mode!\n"));

        // Main loop.  Wait for communication from PC over UART
        while(1)
        {
            if (uart_test())
            {
                message_length = fill_Rx_buffer(&msg_buffer[0]);
                switch(message_length)
                {
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
                } // switch()
            } // if (uart_test())
        } // while(1)
    } // else
    
    TWI_MasterReleaseBuff(&twi1);
    TWI_MasterReleaseBuff(&twi2);
    TWI_MasterReleaseBuff(&twi3);
    TWI_MasterReleaseBuff(&twi4);
    releaseRxBuff();
}


// Starts the pattern running.
void start_running(void)
{
    g_b_running = TRUE;

    // Set indices to the beginning of the function - useful for putting in a set amount of expansion.
    g_index_func_x_read = 0;
    g_index_func_y_read = 0;
    g_iblock_func_x = 1;
    g_iblock_func_y = 1;
    g_display_count = 0;  //clear the display count
    Reg_Handler(update_display, UPDATE_PERIOD, ISR_UPDATE_DISPLAY, TRUE);
    Reg_Handler(increment_index_x, UPDATE_PERIOD, ISR_INCREMENT_INDEX_X, FALSE); //initialize the 2 and 3 priority interupts to a fast rate so that
    Reg_Handler(increment_index_y, UPDATE_PERIOD, ISR_INCREMENT_INDEX_Y, FALSE); // the countdown is fast until the setting of the next rate
                                                        //by the update_display interupt.
    if (g_b_default_func_x)
        Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);
    else
    {
        update_position_x();
        Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, TRUE);
    }

    if (g_b_default_func_y)
        Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE);
    else
    {
        update_position_y();
        Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, TRUE);
    }
}


void handle_message_length_1(uint8_t *msg_buffer)
{
    uint8_t CMD[2];
    uint8_t i;
    uint32_t tmp_x=0;
    uint32_t tmp_y=0;
    uint8_t n_sample=100;
    
    switch(msg_buffer[0])
    {
        case 0x20:  //Start display.
        	start_running();
            break;
            
        case 0x25:  //Start display & trigger - same as regular, but this also does trigger
        	start_running();
        	Reg_Handler(toggle_trigger, (uint32_t)OVERFLOW_PERIOD/g_trigger_rate, ISR_TOGGLE_TRIGGER, TRUE); //turn on the trigger toggle
            break;

        case 0x30: //stop display
            g_b_running = FALSE;
            //turn off the interupts
            Reg_Handler(update_display, UPDATE_PERIOD, ISR_UPDATE_DISPLAY, FALSE);
            Reg_Handler(increment_index_x, UPDATE_PERIOD, ISR_INCREMENT_INDEX_X, FALSE);
            Reg_Handler(increment_index_y, UPDATE_PERIOD, ISR_INCREMENT_INDEX_Y, FALSE);
            Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);
            Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE);
            if (!g_b_default_func_x)
                fetch_block_func_x(&g_file_func_x, TRUE, 0);
            if (!g_b_default_func_y)
                fetch_block_func_y(&g_file_func_y, TRUE, 0);
            break;
            
        case 0x35: //stop display & trigger - same as regular, but this also does trigger
            g_b_running = FALSE;
            //turn off the interupts
            Reg_Handler(update_display, UPDATE_PERIOD, ISR_UPDATE_DISPLAY, FALSE);
            Reg_Handler(increment_index_x, UPDATE_PERIOD, ISR_INCREMENT_INDEX_X, FALSE);
            Reg_Handler(increment_index_y, UPDATE_PERIOD, ISR_INCREMENT_INDEX_Y, FALSE);
            Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);
            Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE);
            Reg_Handler(toggle_trigger, OVERFLOW_PERIOD/g_trigger_rate, ISR_TOGGLE_TRIGGER, FALSE); //turn off the trigger toggle
            digitalWrite(DIO_TRIGGEROUT,LOW);    //set the trigger output to low
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
            g_b_laseractive = TRUE;
            break;
            
        case 0x11:  // turn laser off
            g_b_laseractive = FALSE;
            // turn off the lines that may be connected
            digitalWrite(DIO_LASER, LOW);
            break;
            
        case 0x12:  // turn on compression for identical elements
            g_ident_compress = TRUE;
            break;
            
        case 0x13:  // turn off compression for identical elements
            g_ident_compress = FALSE;
            break;
            
        case 0x14:  //synchronize the SDInfo.mat with the one in the PC
            dump_mat();
            break;
            
        case 0x15:  //get current version
            xprintf(PSTR("Current version number is %s.\n"), VERSION);
            break;
            
        case 0x16:   //show the bus number
            for (i = 1; i <= 128; i++)
            {
                CMD[0] = 0xFE; CMD[1] = g_ch_from_panel[i];
                i2cMasterSend(i, 2, CMD);
            }
            break;
            
        case 0x17:  // turn on quiet_mode, no message sent out
            g_b_quiet_mode = TRUE;
            break;
            
        case 0x18:  // turn off quiet_mode, essage sent out
            g_b_quiet_mode = FALSE;
            break;
              
        case 0x19:  // update GUI information
            xprintf(PSTR("update: %d %d %d %d %d %d %d %d:\n"), g_gain_x, g_bias_x, g_x_initial, g_mode_x, g_gain_y, g_bias_y, g_y_initial, g_mode_y);
          break;
            
        case 0x21:    // working mode 1 = default mode = controller mode
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
            
        case 0x24: //disable int3 external trigger mode
            PORTK.INT0MASK = 0x00;      //Int3 is used as source for port interrupt 0
            xprintf(PSTR("Disabled Int3 external trigger mode!\n"));
            break;        

        case 0x26: //read and set resolution for x and y

            for (i=0; i<n_sample; i++)
            {
               tmp_x += analogRead(2);
               tmp_y += analogRead(3);
               _delay_ms(5);
            };
            g_adc_x_max  =  tmp_x / n_sample;
            g_adc_y_max  =  tmp_y / n_sample;
            xprintf(PSTR("resolution_x =  %d:\n"), g_adc_x_max);
            xprintf(PSTR("resolution_y =  %d:\n"), g_adc_y_max);
            break;
    
        default: i2cMasterSend(0x00, 8, ERROR_CODES[1]);
    }
}

void handle_message_length_2(uint8_t *msg_buffer)
{
    uint8_t argument_byte;
    
    argument_byte = msg_buffer[1];
    switch(msg_buffer[0])
    {
        case 0x01: //sends a reset command out to panel at target address
            i2cMasterSend(argument_byte, 2, RESET);
            break;
            
        case 0x02: //sends a display command out to panel at target address
            i2cMasterSend(argument_byte, 2, DISPLAY);
            break;
            
        case 0x03:   //set pattern
            set_pattern(argument_byte);      //pattern x - specified in argument_byte
            break;
            
        case 0x04: // this is an ADC test command
            test_ADC(argument_byte);  //here argument_byte is a channel, 0-7 to test ADC/DAC system
            break;
            
        case 0x05: // this is a DIO test command
            test_DIO(argument_byte);  //here argument_byte is a channel, 0-7 to test ADC/DAC system
            break;
            
        case 0x06: // this is a trigger rate set command
            g_trigger_rate = argument_byte*2;  //here argument_byte is a trigger rate
            break;
            
        case 0x07:   //flash panel#
            flash_panel(argument_byte); //here argument_byte is a panel number
            break;
            
        case 0x08:   //eeprom panel#
            eeprom_panel(argument_byte); //here argument_byte is a panel number
            break;
            
        case 0x09:   //set arena configuration
            set_hwConfig(argument_byte);      //configuration x - specified in argument_byte
            break;    
            
        case 0x10:  // get ADC value from a ADC channel (1-4)
            xprintf(PSTR("ADC_value =  %d:\n"), analogRead(argument_byte - 1));
            break;    
            
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[2]);
    }
}

void handle_message_length_3(uint8_t *msg_buffer)
{
    uint8_t target_panel_addr;
    uint8_t CMD[2];
    uint16_t funcX_freq, funcY_freq;
    
    switch(msg_buffer[0])
    {
        case 0xFF:  //address panel
            target_panel_addr = msg_buffer[1];  //put in error check, in range < 127
            //sends a reset command out to panel at target address
            
            // Since the panel can be located in any of the four channels,
            // the command should be sent to all channels.
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
            
            //Since the panel can be located in any of the four channels,
            // the command should be sent to all channels.
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
            
        case 0x10:   // set_mode()
            g_mode_x = msg_buffer[1];
            g_mode_y = msg_buffer[2];
            //put in an error message if value is not 0, 1, or 2.
            break;
            
        case 0x15:   //this is a set position function
            if (msg_buffer[2] == 0)
                set_default_func(msg_buffer[1]);
            else
                set_pos_func(msg_buffer[1], msg_buffer[2]);
            break;
            
        case 0x20:   //this is a set velocity function
            if (msg_buffer[2] == 0)
                set_default_func(msg_buffer[1]);
            else
                set_vel_func(msg_buffer[1], msg_buffer[2]);
            break;
            
        case 0x25: // this is a set function generator frequency
            funcX_freq = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            g_period_func_x = OVERFLOW_PERIOD/funcX_freq;
            if (!g_b_quiet_mode)
                xprintf(PSTR("function X update frequency = %u.\n"), funcX_freq);
            break;
            
        case 0x30: // this is a set function generator frequency
            funcY_freq = (uint16_t) msg_buffer[1] + (256*msg_buffer[2]);
            g_period_func_y = OVERFLOW_PERIOD/funcY_freq;
            if (!g_b_quiet_mode)
                xprintf(PSTR("function Y update frequency = %u.\n"), funcY_freq);
            break;
            
        case 0x35: //set resolution_x and g_adc_y_max
            g_adc_x_max = (uint32_t)msg_buffer[1] * DAQRESOLUTION/10;
            g_adc_y_max = (uint32_t)msg_buffer[2] * DAQRESOLUTION/10;
            break;
            
        default: i2cMasterSend(0x00, 8, ERROR_CODES[3]);
    }
}

void handle_message_length_4(uint8_t *msg_buffer)
{
    int16_t setVal;
    //'set_ao'
    switch(msg_buffer[0])
    {
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

void handle_message_length_5(uint8_t *msg_buffer)
{
    switch(msg_buffer[0])
    {
        case 0x70:   //put in a bunch of type casts, because of mysterious error dealing with frame index above 128.
            //'set_position'
            g_x = (uint8_t)msg_buffer[1] + (256*(uint8_t)msg_buffer[2]);
            g_y = (uint8_t)msg_buffer[3] + (256*(uint8_t)msg_buffer[4]);
            
            g_x_initial = g_x; // these only used during position func. control mode, but
            g_y_initial = g_y; //update here should not slow things down much and no need for sep. function.
            g_index_frame = g_y* g_x_max + g_x;
            g_display_count = 0;  //clear the display count
            if (!g_b_quiet_mode)
                xprintf(PSTR("set_position: g_x= %u,  g_y= %u, and g_index_frame= %u\n"), g_x, g_y, g_index_frame);

			if (usePreloadedPattern == 1)
				display_preload_frame(g_index_frame, g_x, g_y);
			else
                fetch_and_display_frame(&g_file_pattern, g_index_frame, g_x, g_y);
            break;

        case 0x71:
            //'send_gain_bias', all of these are signed byte values
            g_gain_x = msg_buffer[1];
            g_bias_x = msg_buffer[2];
            g_gain_y = msg_buffer[3];
            g_bias_y = msg_buffer[4];
            if (!g_b_quiet_mode)
                xprintf(PSTR("set_gain_bias: gain_x= %d,  bias_x= %d, gain_y= %d, bias_y=%d\n"), g_gain_x, g_bias_x, g_gain_y, g_bias_y);
            
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

//load laser trigger pattern first 62 byte data. Laser pattern has 128 bytes, but since
//the value is either 0 or 1, we can combined them in 12 bytes to 
//save serial communicaiton time

void handle_message_length_62(uint8_t *msg_buffer)
{   
    uint8_t i;

    for (i = 0; i<62; i++)
    {
        g_laserpattern[i] = msg_buffer[i];
    }

}

//load laser trigger pattern second 63 byte data. Laser pattern has 128 bytes, but since
//the value is either 0 or 1, we can combined them in 12 bytes to 
//save serial communicaiton time

void handle_message_length_63(uint8_t *msg_buffer)
{   
    uint8_t i;
    
    for (i = 0; i<63; i++)
    {
        g_laserpattern[62 + i] = msg_buffer[i];
    }
    
    
    if (!g_b_quiet_mode)
        xputs(PSTR("Success set the new laser pattern.\n"));

}

void display_dumped_frame (uint8_t *msg_buffer)
{
    uint8_t panel_index;
    uint16_t buffer_index, x_dac_val, y_dac_val;
    //The first two bytes are the x_dac_val, only support positive number 
    //The second two bytes are the y_dac_val, only support positive number 
    //The fifth byte is the number of panels
    //the sixth byte is the gray scale level
    //the seventh byte is the flag of row compression 
    x_dac_val = (uint16_t)msg_buffer[0] + 256*(uint16_t)msg_buffer[1];
    y_dac_val = (uint16_t)msg_buffer[2] + 256*(uint16_t)msg_buffer[3];
    g_num_panels = msg_buffer[4];
    gs_value =msg_buffer[5];
    g_row_compress = msg_buffer[6];


    if (g_row_compress)
        g_bytes_per_panel_frame = gs_value;
    else
        g_bytes_per_panel_frame = gs_value*8;
  
    buffer_index = 7;
    g_display_count = 0;  //clear the display count
    digitalWrite(DIO_FRAMEBUSY, HIGH); // set line high at beginning of frame write
    
    for (panel_index = 1; panel_index <= g_num_panels; panel_index++)
    {
        i2cMasterSend(panel_index, g_bytes_per_panel_frame, &msg_buffer[buffer_index]);
        buffer_index = buffer_index + g_bytes_per_panel_frame;
    }
    analogWrite(0, x_dac_val); // make it a value in the range 0 - 32767 (0V - 10V)
    analogWrite(1, y_dac_val);  // make it a value in the range 0 - 32767 (0V - 10V)
    digitalWrite(DIO_FRAMEBUSY, LOW); // set line low at end of frame write
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
			
void fetch_and_display_frame(FIL *pFile, uint16_t index_frame, uint16_t Xindex, uint16_t Yindex)
{
    // this function will fetch the current frame from the SD-card and display it.
    // pass in index_frame instead of using global g_index_frame to ensure that the value of
    // g_index_frame does not change during this function's run
    // suppose index_frame is from 0 to (n_num * g_y_max - 1)
    uint8_t j, panel_index, packet_sent;
    uint8_t gscale[4];
    uint8_t FLASH[32];
    uint16_t len, cnt, buff_index;
    uint32_t offset;
    FRESULT fresult;
    uint16_t dac_x, dac_y;
    uint8_t sreg = SREG;
    uint8_t block_per_frame;
    uint8_t tempVal, bitIndex, arrayIndex;
    

    digitalWrite(DIO_FRAMEBUSY, HIGH); // set line high at start of frame write
	//if flag gets bigger than 1 -> frame skipped
    if (g_display_count > 1)
        ledToggle(1);    //toggle LED 1
    g_display_count = 0;  //clear the display count

    len = g_num_panels * g_bytes_per_panel_frame;
    
    if (len%512 != 0)
        block_per_frame = len/512 + 1;
    else
        block_per_frame = len/512;  //for gs=4 and rc=0
        
        
    uint8_t  frameBuff[len];
    offset = NBYTES_HEADER + (uint32_t)index_frame * 512 * block_per_frame;

    fresult = f_lseek(pFile, offset);
    if ((fresult == FR_OK) && (pFile->fptr == offset))
    {
        fresult = f_read(pFile, frameBuff, len, &cnt);
        if ((fresult == FR_OK) && (cnt == len))
        {
            buff_index = 0;
            
            for (panel_index = 1; panel_index <= g_num_panels; panel_index++)
            {
                for(j = 0;j < bytes_per_panel_frame;j++){
                    FLASH[j] = frameBuff[buff_index++]; //not good for performance, no need to copy the data
                }

                packet_sent = 0; //used with compression to simplify coniditionals.
                if (g_ident_compress)
                {
                    if (g_bytes_per_panel_frame == 8)
                    {
                        if( (FLASH[0] == FLASH[1])&&(FLASH[2] == FLASH[3])&&(FLASH[4] == FLASH[5])&&(FLASH[6] == FLASH[7]) )
                        {
                            if( (FLASH[1] == FLASH[2])&&(FLASH[3] == FLASH[4])&&(FLASH[5] == FLASH[6]) )
                            {
                                i2cMasterSend(panel_index, 1, &FLASH[0]); //send a 1 byte packet with the correct row_compressed value.
                                packet_sent = 1;
                            } //end of second round of comparisons
                        } //end of first round of byte comparisons
                    } // end of check if g_bytes_per_panel_frame is 8
                    
                    if (g_bytes_per_panel_frame == 24)
                    {
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
                    } // end of check if g_bytes_per_panel_frame is 24
                    
                    if (g_bytes_per_panel_frame == 32){
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
                    } // end of check if g_bytes_per_panel_frame is 32
                } //end of if g_ident_compress
                
                //above conditionals rejected sending a simple pattern patch
                if (packet_sent == 0)
                    i2cMasterSend(panel_index, g_bytes_per_panel_frame, &FLASH[0]);

            } //end of for all panels loop
        }
        else
        {
            if (!g_b_quiet_mode)
            {
                xputs(PSTR("Error in reading file in fetch_and_display_frame!\n"));
                xprintf(PSTR("fresult = %u, index_frame= %u, cnt= %u\n"), fresult, index_frame, cnt);
            }
        }
    }
    else
    {
        if (!g_b_quiet_mode)
        {
            xputs(PSTR("Error seeking in fetch_and_display_frame!\n"));
            xprintf(PSTR("fresult = %u, index_frame= %u, offset = %lu, pFile->fs=%X, pFile->fs->fs_type=%X, pFile->fs->id=%X\n"), fresult, index_frame, offset, pFile->fs, pFile->fs->fs_type, pFile->fs->id);
        }
    }
    
    //update analog out
    if (g_mode_x != 5)
    {
        dac_x = ((uint32_t)Xindex + 1)*32767/g_x_max;
        analogWrite(0, dac_x); // make it a value in the range 0 - 32767 (0 - 10V)
    }

    
    if (g_mode_y != 5)
    {
        dac_y = ((uint32_t)Yindex + 1)*32767/g_y_max;
        analogWrite(1, dac_y); // make it a value in the range 0 - 32767 (0 - 10V)
    }

    
      //also update the output lines for quadrant-type learning patterns
    if (g_b_laseractive)
    {
        arrayIndex = g_x/8;  // find the index in g_laserpattern array for g_x
        bitIndex = g_x - arrayIndex*8;  // find the bit index in a g_laserpattern byte for g_x
    
        tempVal = g_laserpattern[arrayIndex];
    
        if ((tempVal & (1<<(7-bitIndex))) == 0)
            digitalWrite(DIO_LASER, LOW);  // turn off laser
        else
            digitalWrite(DIO_LASER, HIGH);   // turn on laser
    }
    
    digitalWrite(DIO_FRAMEBUSY, LOW); // set line low at end of frame write

}


// set_rates()
// Set an ISR to perform vx and vy.
//
void set_rates(int16_t xRate, int16_t yRate)
{
    if (xRate > 0)
        Update_Reg_Handler(increment_index_x, (uint32_t)OVERFLOW_PERIOD/abs(xRate), ISR_INCREMENT_INDEX_X, TRUE);
    else if (xRate < 0)
        Update_Reg_Handler(decrement_index_x, (uint32_t)OVERFLOW_PERIOD/abs(xRate), ISR_INCREMENT_INDEX_X, TRUE);
    else // xRate == 0
        Update_Reg_Handler(decrement_index_x, UPDATE_PERIOD, ISR_INCREMENT_INDEX_X, FALSE);


    if (yRate > 0)
        Update_Reg_Handler(increment_index_y, (uint32_t)OVERFLOW_PERIOD/abs(yRate), ISR_INCREMENT_INDEX_Y, TRUE);
    else if (yRate < 0)
        Update_Reg_Handler(decrement_index_y, (uint32_t)OVERFLOW_PERIOD/abs(yRate), ISR_INCREMENT_INDEX_Y, TRUE);
    else // yRate == 0
        Update_Reg_Handler(decrement_index_y, UPDATE_PERIOD, ISR_INCREMENT_INDEX_Y, FALSE);

}


// update_display()
// This function calculates and sets the x,y rates.
//
void update_display(void)
{
    static int16_t x, y;
    int16_t xRate = 0;
    int16_t yRate = 0;
    int16_t adc_x, adc_y;
    int16_t val_prev;
    uint16_t x, y;

    //there are six modes:
    // 0 - Open loop, 
    // 1 - Closed loop, 
    // 2 - Closed loop w/ Bias, 
    // 3 - POS mode with ch5, 
    // 4 - POS mode from pos func 
    // 5 - function DBG mode
    
    switch(g_mode_x)
    {
        case 0:   // open loop - use function generator to set x rate
            x = 2*g_buf_func_x[g_index_func_x_read];
            xRate = ((x*g_gain_x)/10 + 5*g_bias_x)/2;
            break;

        case 1: //closed loop, use CH0 - CH1 to set x rate
            adc_x = analogRead(0)/4;  // 1 volt = 102.4 frames/sec
            val_prev = x; //the previous value
            x = ( 6*val_prev + 4*adc_x )/10;   // A fast exponentially weighted moving average.
            xRate = (int16_t)((int32_t)(x*g_gain_x)/10 + 5*g_bias_x)/2;  //x can go as high as DAQRESOLUTION, g_gain_x 100fiu and g_bias_x 250
            
            //set a frame rate limit 256fps
            //if (xRate >256)
            //    xRate = 256;
            //else if (xRate < -256)
            //    xRate = -256;
            break;

        case 2: //closed loop w bias - use CH0 - CH1, and function gen. to set x rate
            adc_x = analogRead(0)/4; // 1 volt = 102
            val_prev = x; //the previous value
            x = ( 6*val_prev + 4*adc_x )/10;   // A fast exponentially weighted moving average.
            //add in the bias to CL mode on ch X
            xRate = (int16_t)((int32_t)(x*g_gain_x)/10 + 2*g_buf_func_x[g_index_func_x_read] + 5*g_bias_x)/2;
            break;

        case 3: 
            adc_x = analogRead(2);  //adc_x ranges from 0-DAQRESOLUTION (12bit ADC).
            adc_x = CLIP(adc_x, 0, g_adc_x_max);

            // Calculate the position.
            x = (uint16_t)(2 * (int32_t)adc_x * (int32_t)g_x_max + (int32_t)g_adc_x_max) / ((int32_t) g_adc_x_max * 2) - 1;
            x = CLIP(x, 0, g_x_max-1);
            g_x = x;

            g_index_frame = FRAMEFROMXY(g_x, g_y);
        	xRate = 0;
            break;

        case 4:
        	xRate = 0;
            break;

        case 5:
            xRate = 0;
            break;
    }

    
    
    switch(g_mode_y)
    {
        case 0:   // open loop - use function generator to set y rate
            y = 2*g_buf_func_y[g_index_func_y_read];
            yRate = ((y*g_gain_y)/10 + 5*g_bias_y)/2;
            break;

        case 1: //closed loop, use CH2 - CH3 to set y rate
            adc_y = analogRead(1)/4; // 1 volt = 102.4 fps
            val_prev = y; //the previous value
            y = (uint16_t)( 6*val_prev + 4*adc_y)/10;   //this is a 60% old value, 40% new value smoother
            yRate = (int16_t)((int32_t)(y*g_gain_y)/10 + 5*g_bias_y)/2; //y can go as high as DAQRESOLUTION, g_gain_y 100, and g_bias_y 250.
            
            //set a frame rate limit 256fps
            //if (yRate > 256)
            //    yRate = 256;
            //else if (yRate < -256)
            //    yRate = -256;
            
            break;

        case 2: //closed loop w bias - use CH2 - CH3, and function gen. to set y rate
            adc_y = analogRead(1)/4; // 1 volt = 102
            val_prev = y; //the previous value
            y = ( 6*val_prev + 4*adc_y)/10;   //this is a 60% old value, 40% new value smoother
            //add in the bias to CL mode on ch Y
            yRate = (int16_t)((int32_t)(y*g_gain_y)/10 + 2*g_buf_func_y[g_index_func_y_read] + 5*g_bias_y)/2; //y can go as high as DAQRESOLUTION
            break;

        case 3:
            adc_y = analogRead(3);   //adc_y ranges from 0-DAQRESOLUTION.
            adc_y = CLIP(adc_y, 0, g_adc_y_max);

            // Calculate the position.
            y = ((int32_t)adc_y * g_y_max * 2 + g_adc_y_max) / ((int32_t) g_adc_y_max * 2) - 1;
            y = CLIP(y, 0, g_y_max-1);
            g_y = y;

            g_index_frame = FRAMEFROMXY(g_x, g_y);
        	yRate = 0;
            break;

        case 4:
        	yRate = 0;
            break;

        case 5:  // this is the function gen DBG mode - don't run y, set rate to zero
            yRate = 0;
            break;
            //do something with errors here for default case
    }
    
    //in the above x,y_val computation, there is a div by 10 to take away gain scaling
    //so g_gain_x of 10 is 1X gain, g_gain_x of 20 = 2X ...
    
    //here the 2* the rate is because we want 20 = 1V to correspond to 10 fps. could probably do without,
    // and just divide the a2dConvert output by 4, and not scale function_x,y by 2
    if (!g_b_running)
        xRate = yRate = 0;
    
    g_b_xrate_greater_yrate = (xRate >= yRate);
    set_rates(xRate, yRate);
}


void increment_index_x(void)
{
    
    g_x++;
    g_x %= g_x_max;
    
    g_index_frame = FRAMEFROMXY(g_x, g_y);
    
    if (g_b_xrate_greater_yrate)
    	g_display_count++;
}


void increment_index_y(void)
{
    g_y++;
    g_y %= g_y_max;
    
    g_index_frame = FRAMEFROMXY(g_x, g_y);
    
    if (!g_b_xrate_greater_yrate)
    	g_display_count++;
}


void decrement_index_x(void)
{
    
    if (g_x <= 0)    //just to be safe, use less than
        g_x = g_x_max - 1;    //but these are unsigned
    else
        g_x--;
    
    g_index_frame = FRAMEFROMXY(g_x, g_y);
    if (g_b_xrate_greater_yrate)
    	g_display_count++;
}


void decrement_index_y(void)
{
    if (g_y <= 0)    //just to be safe, use less than
        g_y = g_y_max - 1;    //but these are unsigned
    else
        g_y--;
    
    g_index_frame = FRAMEFROMXY(g_x, g_y);
    if (!g_b_xrate_greater_yrate)
    	g_display_count++;
}


void toggle_trigger(void)
{
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
void set_pattern(uint8_t pat_num)
{
    //sets the pattern ID, in future return 0 or 1 if error/succeed
    uint16_t num_frames;
    uint16_t cnt;
    static uint8_t str[12];
    uint8_t  bufHeader[NBYTES_HEADER];
    uint8_t res;
    
    if (pat_num < 1000)
        sprintf(str, "pat%04d.pat\0", pat_num);
    else
        xputs(PSTR("pat_num is too big.\n"));
   
    
    f_close(&g_file_pattern);
    res = f_open(&g_file_pattern, str, FA_OPEN_EXISTING | FA_READ); // The file stays open after this function returns.
    if (res == FR_OK)
    {
        res = f_read(&g_file_pattern, bufHeader, NBYTES_HEADER, &cnt); // read the 10 byte header info block
        if ((res == FR_OK) && (cnt == NBYTES_HEADER))
        {
            // get the header info
            ((uint8_t*)&g_x_max)[0] = bufHeader[0];
            ((uint8_t*)&g_x_max)[1] = bufHeader[1];
            ((uint8_t*)&g_y_max)[0] = bufHeader[2];
            ((uint8_t*)&g_y_max)[1] = bufHeader[3];
            g_num_panels = bufHeader[4];
            gs_value = bufHeader[5];   //11, 12, 13, or 14 means use row compression
            
            
            num_frames = g_x_max * g_y_max;
            if ((gs_value >= 11) & (gs_value <= 14))
            {
                gs_value = gs_value - 10;
                g_row_compress = TRUE;
                g_bytes_per_panel_frame = gs_value;
            }
            else
            {
                g_row_compress = FALSE;
                g_bytes_per_panel_frame = gs_value * 8;
            }
            g_x = g_y = 0;
            g_index_frame = 0;
            g_b_running = FALSE;
            g_display_count = 0;  //clear the display count
            if (!g_b_quiet_mode)
            {
                xprintf(PSTR("Setting pattern %u:\n"), pat_num);
                xprintf(PSTR("  g_x_max = %u\n  g_y_max = %u\n  g_num_panels = %u\n  gs_value = %u\n row_compression = %u\n"),
                        g_x_max, g_y_max, g_num_panels, gs_value, g_row_compress);
            }
            fetch_and_display_frame(&g_file_pattern, g_index_frame, g_x, g_y);
        }
        else
        	xputs(PSTR("Error reading in pattern file\n"));
    }
    else
    	xputs(PSTR("Error opening pattern file\n"));

	usePreloadedPattern = 0;
}

void set_hwConfig(uint8_t config_num)
{
    // try to read in the hardware config file

    static uint8_t str[12];
    uint8_t res, b1;
    uint8_t  bufTemp[128];
    uint16_t cnt;
        
    if (config_num < 1000)
        sprintf(str, "cfg%04d.cfg\0", config_num);
    else
        xputs(PSTR("config_num is too big.\n"));
            
    res = f_open(&g_file_arenaconfig, str, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK)
    {
        // looks good, read in the 128 byte panel mapping info
        res = f_read(&g_file_arenaconfig, bufTemp, 128, &cnt);
        if ((res == FR_OK) && (cnt == 128))
        {
            //copy to the mapping tables
            for (b1 = 1; b1 <= 128; b1++)
            {
                g_ch_from_panel[b1] = bufTemp[b1-1]; // panel 0 doesn't exist
                eeprom_write_byte(arena_config + b1, bufTemp[b1-1]);
            }
            eeprom_write_byte(arena_config, 0x00); //Mark arena configuration file in EEPROM
        }
        f_close(&g_file_arenaconfig);
        xputs(PSTR("Successfully load the hardware config file to EEPROM\n"));
    }
    else
    {
        xputs(PSTR("Cannot find the hardware config file on the SD card.\n"));
    }
}

// this function assumes that a pattern has been set
void benchmark_pattern(void)
{
    uint16_t num_frames;
    uint16_t frame_ind;
    uint32_t bench_time;
    uint16_t frame_rate;
    
    g_b_running = FALSE;
    num_frames = g_x_max*g_y_max;
    
    timer_coarse_tic();
    
    for(frame_ind = 0; frame_ind < num_frames; frame_ind++)
        fetch_and_display_frame(&g_file_pattern, frame_ind, g_x, g_y);
    
    bench_time = timer_coarse_toc();
    frame_rate = ((uint32_t)num_frames*1000)/bench_time;
    xprintf(PSTR(" bench_time = %lu ms, frame_rate = %u\n"), bench_time, frame_rate);
	
	//reset index_x and index_y
	index_x=0;
	index_y=0;
}

void i2cMasterSend(uint8_t panel, uint8_t len, uint8_t *data)
{
    uint8_t ch;
    uint8_t addr;
    TWI_Master_t *twi;
    
    if (panel == 0)
    {
        while (twi1.status != TWIM_STATUS_READY)
            ; // Do nothing.
        TWI_MasterWrite(&twi1, 0, data, len);
        while (twi2.status != TWIM_STATUS_READY)
            ; // Do nothing.
        TWI_MasterWrite(&twi2, 0, data, len);
        while (twi3.status != TWIM_STATUS_READY)
            ; // Do nothing.
        TWI_MasterWrite(&twi3, 0, data, len);
        while (twi4.status != TWIM_STATUS_READY)
            ; // Do nothing.
        TWI_MasterWrite(&twi4, 0, data, len);
    }
    else
    {
        // look up the actual panel address and channel
        ch = g_ch_from_panel[panel];
        addr = g_adr_from_panel[panel];
        if (ch != 0)
        {
            
            switch (ch)
            {
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
            
            while (twi->status != TWIM_STATUS_READY)
                ; // Do nothing.
            TWI_MasterWrite(twi, addr, data, len);
        }
    }
}


// set_default_func()
// The default function is a constant 10.
//
void set_default_func(uint8_t channel)
{
    uint16_t i;
    
    switch (channel)
    {
        case 1:
            if (!g_b_quiet_mode)
                xputs(PSTR("Setting default function for X.\n"));

            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);// disable ISR
            g_id_func_x = 0;
            g_b_default_func_x = TRUE;
            g_nbytes_func_x = RINGBUFFER_LENGTH;

            for (i=0; i<RINGBUFFER_LENGTH; i++)
                g_buf_func_x[i] = 10;

            g_index_func_x_read = 0;
            g_nblocks_func_x = 1;
            g_nbytes_final_block_x = 0;
            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, TRUE);// enable ISR
            break;
        case 2:
            if (!g_b_quiet_mode)
                xputs(PSTR("Setting default function for Y.\n"));

            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE);// disable ISR
            g_id_func_y = 0;
            g_b_default_func_y = TRUE;
            g_nbytes_func_y = RINGBUFFER_LENGTH;

            for (i=0; i<RINGBUFFER_LENGTH; i++)
                g_buf_func_y[i] = 10;
            
            g_index_func_y_read = 0;
            g_nblocks_func_y = 1;
            g_nbytes_final_block_y = 0;
            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, TRUE);// enable ISR
            break;
        default:
            xputs(PSTR("Function channel number must be 1 for x, or 2 for y.\n"));
            break;
    }
    
}


void set_pos_func(uint8_t channel, uint8_t id_func)
{
    uint16_t cnt;
    uint8_t str[12];
    //uint8_t i;
    //uint8_t func_name_x[100];
    //uint8_t func_name_y[100];
    //uint8_t func_name_len;
    FRESULT fresult=FR_INVALID_OBJECT;
    FIL		*pFile=NULL;	// The file we're working on: &g_file_func_x or &g_file_func_y.
    uint8_t bufHeader[NBYTES_HEADER];
    
    
    if (id_func < 1000)
        sprintf(str, "pos%04d.fun\0", id_func);
    else
        xputs(PSTR("function id is too big.\n"));
    
    switch(channel)
    {
        case 1:    //channel x
            //read the 512 byte header block and send back the function name
            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE);//disable ISR
            
            pFile = &g_file_func_x;
            f_close(pFile);
            fresult = f_open(pFile, str, FA_OPEN_EXISTING | FA_READ); // The file stays open after this function returns.
            if (fresult == FR_OK)
            {
            	// Read the header into the buffer.
                fresult = f_read(pFile, bufHeader, NBYTES_HEADER, &cnt);
                if ((fresult == FR_OK) && (cnt == NBYTES_HEADER))
                {
                    // Get the 32 bit length of the function.
                    ((uint8_t*)&g_nbytes_func_x)[0] = bufHeader[0];
                    ((uint8_t*)&g_nbytes_func_x)[1] = bufHeader[1];
                    ((uint8_t*)&g_nbytes_func_x)[2] = bufHeader[2];
                    ((uint8_t*)&g_nbytes_func_x)[3] = bufHeader[3];
                    
                    // Get the function name.
                    //func_name_len = bufHeader[4];
                    //for (i=0; i<func_name_len; i++)
                    //    {func_name_x[i] = bufHeader[i+5];}
                    //func_name_x[func_name_len] = '\0';
                    

                    // Indicate that the function is ready.
                    g_id_func_x = id_func;
                    g_b_default_func_x = FALSE;
                    g_b_running = FALSE;
                    g_display_count = 0;

                    if (!g_b_quiet_mode)
                    {
                        xprintf(PSTR("Setting position function %u for X\n"), id_func);
						//xprintf(PSTR("fun X: %s\n function X size: %lu bytes\n"), func_name_x, g_nbytes_func_x);
                    }
                    
                    g_nbytes_final_block_x = g_nbytes_func_x % RINGBUFFER_LENGTH;

                    if (g_nbytes_final_block_x == 0)
                        g_nblocks_func_x = g_nbytes_func_x / RINGBUFFER_LENGTH;
                    else
                        g_nblocks_func_x = g_nbytes_func_x / RINGBUFFER_LENGTH + 1;

                    if (!g_b_quiet_mode)
                    {
                        xprintf(PSTR("g_nbytes_func_x = %u\n"), g_nbytes_func_x);
                        xprintf(PSTR("g_nbytes_final_block_x = %u\n"), g_nbytes_final_block_x);
                        xprintf(PSTR("g_nblocks_func_x = %u\n"), g_nblocks_func_x);
                    }


                    //update the function buffer
                    fetch_block_func_x(pFile, TRUE, 0);

                }
                else
					xputs(PSTR("Error reading file in set_pos_func(): X\n"));
            }
            else
				xputs(PSTR("Error opening file in set_pos_func(): X.\n"));
                
            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, TRUE);//enable ISR
            break;
            
        case 2:	// channel y.
            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE); //disable ISR
            //read the header block and send back the function name
            pFile = &g_file_func_y;
            f_close(pFile);
            fresult = f_open(pFile, str, FA_OPEN_EXISTING | FA_READ); // The file stays open after this function returns.
            if (fresult == FR_OK)
            {
                fresult = f_read(pFile, bufHeader, NBYTES_HEADER, &cnt);
                if ((fresult == FR_OK) && (cnt == NBYTES_HEADER))
                {
                	// Get the 32 bit length of the function.
                    ((uint8_t*)&g_nbytes_func_y)[0] = bufHeader[0];
                    ((uint8_t*)&g_nbytes_func_y)[1] = bufHeader[1];
                    ((uint8_t*)&g_nbytes_func_y)[2] = bufHeader[2];
                    ((uint8_t*)&g_nbytes_func_y)[3] = bufHeader[3];
                    //func_name_len = bufHeader[4];
                    
                    // Indicate that the function is ready.
                    g_id_func_y = id_func;
                    g_b_default_func_y = FALSE;
                    g_b_running = FALSE;
                    g_display_count = 0;

                    if (!g_b_quiet_mode)
                        xprintf(PSTR("Setting position function %u for Y\n"), id_func);
                    //xprintf(PSTR("fun Y: %s\n function Y size: %lu bytes\n"),
                    //       func_name_y, g_nbytes_func_y);
                    
                    g_nbytes_final_block_y = g_nbytes_func_y % RINGBUFFER_LENGTH;

                    if (g_nbytes_final_block_y == 0)
                        g_nblocks_func_y = g_nbytes_func_y / RINGBUFFER_LENGTH;
                    else
                        g_nblocks_func_y = g_nbytes_func_y / RINGBUFFER_LENGTH + 1;

                    if (!g_b_quiet_mode)
                    {
                        xprintf(PSTR("g_nbytes_func_y = %u\n"), g_nbytes_func_y);
                        xprintf(PSTR("g_nbytes_final_block_y = %u \n"), g_nbytes_final_block_y);
                        xprintf(PSTR("g_nblocks_func_y = %u\n"), g_nblocks_func_y);
                    }

                    //update the function buffer
                    fetch_block_func_y(pFile, TRUE, 0);

                }
                else
					xputs(PSTR("Error reading file in set_pos_func(): Y.\n"));
            }
            else
				xputs(PSTR("Error opening file in set_pos_func(): Y.\n"));
            
            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, TRUE);//enable ISR
            break;
            
        default:
                xputs(PSTR("Error: channel must be 1 for x, or 2 for y.\n"));
            break;
    }
}


void set_vel_func(uint8_t channel, uint8_t id_func)
{
    //sets the velocity function id
    uint16_t cnt;
    uint8_t str[12];
    //uint8_t func_name_x[100];
    //uint8_t func_name_y[100];
    //uint8_t func_name_len;
    FRESULT fresult=FR_INVALID_OBJECT;
    FIL		*pFile=NULL;	// The file we're working on: &g_file_func_x or &g_file_func_y.
    uint8_t bufHeader[NBYTES_HEADER];
    
    
    if (id_func < 1000)
        sprintf(str, "vel%04d.fun\0", id_func);
    else
        xputs(PSTR("function id is too big.\n"));
    
    switch(channel)
    {
        case 1:    // Channel x.
            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, FALSE); //disable ISR
            pFile = &g_file_func_x;
            f_close(pFile);
        	fresult = f_open(pFile, str, FA_OPEN_EXISTING | FA_READ); // The file stays open after this function returns.
            
            if (fresult == FR_OK)
            {
                // Read the header block and send back the function name
                fresult = f_read(pFile, bufHeader, NBYTES_HEADER, &cnt);
                
                if ((fresult == FR_OK) && (cnt == NBYTES_HEADER))
                {
                    // get the header info
                    ((uint8_t*)&g_nbytes_func_x)[0] = bufHeader[0];
                    ((uint8_t*)&g_nbytes_func_x)[1] = bufHeader[1];
                    ((uint8_t*)&g_nbytes_func_x)[2] = bufHeader[2];
                    ((uint8_t*)&g_nbytes_func_x)[3] = bufHeader[3];
                    //func_name_len = bufHeader[4];
                    
                    g_id_func_x = id_func;
                    g_b_default_func_x = FALSE;
                    g_b_running = FALSE;
                    g_display_count = 0;  //clear the display count

                    if (!g_b_quiet_mode)
                        xprintf(PSTR("Setting velocity function  %u for X\n"), id_func);
                    
                    if (!g_b_quiet_mode)
                    {
                        xprintf(PSTR("g_nbytes_func_x = %u\n"), g_nbytes_func_x);
                        g_nbytes_final_block_x = g_nbytes_func_x % RINGBUFFER_LENGTH;
                        xprintf(PSTR("g_nbytes_final_block_x = %u\n"), g_nbytes_final_block_x);
                        if(!g_nbytes_final_block_x)
                        {
                            g_nblocks_func_x = g_nbytes_func_x/RINGBUFFER_LENGTH;
                            xprintf(PSTR("g_nblocks_func_x = %u\n"), g_nblocks_func_x);
                        }
                        else
                        {
                            g_nblocks_func_x = g_nbytes_func_x / RINGBUFFER_LENGTH + 1;
                            xprintf(PSTR("g_nblocks_func_x = %u\n"), g_nblocks_func_x);
                        }
                    }


                    // Reset the function, and read the first block of bytes into the function buffer.
                    fetch_block_func_x(pFile, TRUE, 0);

                }
                else
                	xputs(PSTR("Error reading file in set_vel_func(): X.\n"));
            }
            else
				xputs(PSTR("Error opening file in set_vel_func(): X.\n"));

            //Reg_Handler(update_position_x, g_period_func_x, ISR_INCREMENT_FUNC_X, TRUE); //enable ISR
            break;
            
            
        case 2:	// Channel y.
            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, FALSE); //disable ISR
        	pFile = &g_file_func_y;
            f_close(pFile);
        	fresult = f_open(pFile, str, FA_OPEN_EXISTING | FA_READ); // The file stays open after this function returns.
            
            if (fresult == FR_OK)
            {
                // Read the header block and send back the function name
                fresult = f_read(pFile, bufHeader, NBYTES_HEADER, &cnt);
                
                if ((fresult == FR_OK) && (cnt == NBYTES_HEADER))
                {
                    // get the header info
                    ((uint8_t*)&g_nbytes_func_y)[0] = bufHeader[0];
                    ((uint8_t*)&g_nbytes_func_y)[1] = bufHeader[1];
                    ((uint8_t*)&g_nbytes_func_y)[2] = bufHeader[2];
                    ((uint8_t*)&g_nbytes_func_y)[3] = bufHeader[3];
                    //func_name_len = bufHeader[4];
                    
                    g_id_func_y = id_func;
                    g_b_default_func_y = FALSE;
                    g_b_running = FALSE;
                    g_display_count = 0;  //clear the display count

                    if (!g_b_quiet_mode)
                        xprintf(PSTR("Setting velocity function %u for Y\n"), id_func);
                    
                    if (!g_b_quiet_mode)
                    {
                        xprintf(PSTR("g_nbytes_func_y = %u\n"), g_nbytes_func_y);
                        g_nbytes_final_block_y = g_nbytes_func_y % RINGBUFFER_LENGTH;
                        xprintf(PSTR("g_nbytes_final_block_y = %u\n"), g_nbytes_final_block_y);
                        if (g_nbytes_final_block_y == 0)
                        {
                            g_nblocks_func_y = g_nbytes_func_y / RINGBUFFER_LENGTH;
                            xprintf(PSTR("g_nblocks_func_y = %u\n"), g_nblocks_func_y);
                        }
                        else
                        {
                            g_nblocks_func_y = g_nbytes_func_y / RINGBUFFER_LENGTH + 1;
                            xprintf(PSTR("g_nblocks_func_y = %u\n"), g_nblocks_func_y);
                        }
                    }


                    // Reset the function, and read the first block of bytes into the function buffer.
                    fetch_block_func_y(pFile, TRUE, 0);

                }
                else
					xputs(PSTR("Error reading file in set_vel_func(): Y.\n"));
            }
            else
				xputs(PSTR("Error opening file in set_vel_func(): Y.\n"));
            
            //Reg_Handler(update_position_y, g_period_func_y, ISR_INCREMENT_FUNC_Y, TRUE); //enable ISR
            break;
            
        default:
			xputs(PSTR("Error input for function channel.\n"));
            break;
    }
}
    

// update_position_x()
// Calculates and sets the x position.
//
void update_position_x(void)
{
    uint16_t adc_x;
    int16_t  dac_x;
    int16_t  x_tmp;
    int32_t  x;

    
    if (g_filllevel_buf_func_x)
    {
        g_index_func_x_read++;
        g_index_func_x_read %= RINGBUFFER_LENGTH;
        g_filllevel_buf_func_x--;

        switch(g_mode_x)
        {
            case 3:
                break;

            case 4:
                x_tmp = g_x_initial + g_buf_func_x[g_index_func_x_read];

                // Wrap around if necessary.
                if (x_tmp >= 0)
                    g_x = x_tmp%g_x_max;
                else
                    g_x = g_x_max - ((abs(x_tmp))%g_x_max) - 1;

                g_index_frame = FRAMEFROMXY(g_x, g_y);
                break;

            case 5:   // in function DBG mode - show the function gen
                //3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V
                dac_x = g_buf_func_x[g_index_func_x_read]*33;
                analogWrite(0, dac_x); // make it a value in the range -32767 - 32767 (-10V - 10V)
                break;
            
        }
    }
    else
        xputs(PSTR("Function buffer for x is empty\n"));

}


// update_position_y()
// Calculates and sets the x position.
//
void update_position_y(void)
{
    uint16_t adc_y;
    int16_t  dac_y;
    int16_t  y_tmp;
    int32_t  y;


    if (g_filllevel_buf_func_y)
    {
        g_index_func_y_read++;
        g_index_func_y_read %= RINGBUFFER_LENGTH;
        g_filllevel_buf_func_y--;

        switch(g_mode_y)
        {
            case 3:
                break;

            case 4:
                //only use y_tmp as a temp variable, just not to create an additional one
                y_tmp = (g_y_initial + g_buf_func_y[g_index_func_y_read]);

                // Wrap around if necessary.
                if (y_tmp >= 0)
                    g_y = y_tmp%g_y_max;
                else
                	g_y = g_y_max - ((abs(y_tmp))%g_y_max) - 1;

                g_index_frame = FRAMEFROMXY(g_x, g_y);
                break;

            case 5:   // in function DBG mode - show the function gen
                //3277 is converted to 1V by DAC, we amplify function value so value 100 is about 1V
                dac_y = g_buf_func_y[g_index_func_y_read]*33;
                analogWrite(1, dac_y); // make it a value in the range -32767 - 32767 (-10V - 10V)
                break;

        }
    }
    else
        xputs(PSTR("Function buffer for y is empty\n"));

}


// fetch_block_func_x()
// Read a block from the given open file into the Y function buffer.
//
void fetch_block_func_x(FIL *pFile, uint8_t bReset, uint8_t i_block_func_x)
{
    // i_block_func_x ranges from 0 to (g_nblocks_func_x - 1)
    // bReset==TRUE mean we need to load the first FUNCTIONLLENGTH of function data into buffer,
    //         because users always want the functions to start from the beginning every time they send start command.

    uint16_t	cnt, j, offset;
    FRESULT 	fresult;
    uint8_t 	bufTemp[RINGBUFFER_LENGTH];
    uint16_t 	nbytesBuf;
                
    //xprintf(PSTR("i_block_func_x =  %u\n"), i_block_func_x);
    if (g_filllevel_buf_func_x < RINGBUFFER_LENGTH)
    {
        if (bReset)
        {
            g_index_func_x_read = 0;
            g_index_func_x_write = 0;
            g_filllevel_buf_func_x = 0;
        }
        
        offset = NBYTES_HEADER + i_block_func_x * RINGBUFFER_LENGTH;

        fresult = f_lseek(pFile, offset);
        if ((fresult == FR_OK) && (pFile->fptr == offset))
        {
            //i_block_func_x rangesx from 0 to g_nblocks_func_x - 1
            if ((i_block_func_x ==  g_nblocks_func_x - 1) && (g_nbytes_final_block_x != 0))
                nbytesBuf = g_nbytes_final_block_x;
            else
                nbytesBuf = RINGBUFFER_LENGTH;

            //load N bytes data to bufTemp
            fresult = f_read(pFile, bufTemp, nbytesBuf, &cnt);
            if (!((fresult == FR_OK) && (cnt == nbytesBuf)))
            {
                xprintf(PSTR("fresult =  %u\n"), fresult);
                xputs(PSTR("Error reading in fetch_block_func_x().\n"));
            }

            for (j=0; j<cnt; j+=2)
            {
                g_buf_func_x[g_index_func_x_write] = (uint16_t)bufTemp[j] + (uint16_t)bufTemp[j+1]*256 ;
                g_index_func_x_write++;
                g_index_func_x_write %= RINGBUFFER_LENGTH;
                g_filllevel_buf_func_x++;
            }
            
            //xprintf(PSTR("g_index_func_x_write =  %u\n"), g_index_func_x_write);
        }
        else
        {
    		xprintf(PSTR("fresult =  %u\n"), fresult);
    		xputs(PSTR("Error seeking in fetch_block_func_x().\n"));
        }
    }
    else
        xputs(PSTR("Function buffer for x is full\n"));

}

// fetch_block_func_y()
// Read a block from the given open file into the Y function buffer.
//
void fetch_block_func_y(FIL *pFile, uint8_t bReset, uint8_t i_block_func_y)
{
    // i_block_func_y ranges from 0 to (g_nblocks_func_x - 1)
    // bReset==TRUE mean we need to load the first FUNCTIONLLENGTH of function data into buffer
    // because users always want the functions to start from the beginning every time they send start command
    uint16_t	cnt, j, offset;
    FRESULT 	fresult;
    uint8_t 	bufTemp[RINGBUFFER_LENGTH];
    uint16_t 	nbytesBuf;

    //xprintf(PSTR("i_block_func_y =  %u\n"), i_block_func_y);
    if (g_filllevel_buf_func_y < RINGBUFFER_LENGTH)
    {
        if (bReset)
        {
            g_filllevel_buf_func_y = 0;
            g_index_func_y_read = 0;
            g_index_func_y_write = 0;
        }
        
        offset = NBYTES_HEADER + i_block_func_y * RINGBUFFER_LENGTH;

        fresult = f_lseek(pFile, offset);
        if ((fresult == FR_OK) && (pFile->fptr == offset))
        {

            if ((i_block_func_y ==  g_nblocks_func_y - 1) && (g_nbytes_final_block_y != 0))
                nbytesBuf = g_nbytes_final_block_y;
            else
                nbytesBuf = RINGBUFFER_LENGTH;

            //load 100 bytes data to bufTemp
            fresult = f_read(pFile, bufTemp, nbytesBuf, &cnt);
            if (!((fresult == FR_OK) && (cnt == nbytesBuf)))
            {
                xprintf(PSTR("fresult =  %u\n"), fresult);
                xputs(PSTR("Error reading in fetch_block_func_y().\n"));
            }
            
            for (j = 0; j< cnt; j+=2)
            {
                g_buf_func_y[g_index_func_y_write] = (uint16_t)bufTemp[j] + (uint16_t)bufTemp[j+1]*256;
                g_index_func_y_write++;
                g_index_func_y_write %= RINGBUFFER_LENGTH;
                g_filllevel_buf_func_y++;
            }
            //xprintf(PSTR("g_index_func_y_write =  %u\n"), g_index_func_y_write);
        }
        else
        {
    		xprintf(PSTR("fresult =  %u\n"), fresult);
    		xputs(PSTR("Error seeking in fetch_block_func_y().\n"));
        }
    }
    else
        xputs(PSTR("Function buffer for y is full.\n"));

} 

//synchronize the SD.mat from SD card to PC
void dump_mat(void)
{
    uint8_t b1, fileRemain;
    uint32_t iteration, offset;
    FRESULT fresult;
    uint16_t cnt;
    uint8_t matBuff[50];
    
    // try to read in the SD.mat filfil
    fresult = f_open(&g_file_arenaconfig, SDInfo, FA_OPEN_EXISTING | FA_READ);
    if (fresult == FR_OK)
    {
        // looks good
        iteration = g_file_arenaconfig.fsize/50;
        fileRemain = (uint8_t)(g_file_arenaconfig.fsize - iteration*50);
        //xprintf(PSTR("filesize = %lu, iteration = %lu, fileRemain = %u\n"), g_file_arenaconfig.fsize, iteration, fileRemain);
        
        // send 50 bytes data for iteration times
        for (b1=1; b1<= iteration; b1++)
        {
            offset = (b1 -1)*50;
            
            fresult = f_lseek(&g_file_arenaconfig, offset);
            if ((fresult == FR_OK) && (g_file_arenaconfig.fptr == offset))
            {
                fresult = f_read(&g_file_arenaconfig, matBuff, 50, &cnt);
                if ((fresult == FR_OK) && (cnt == 50))
                {
                    send_Tx_buffer(matBuff, 50);
                }
            }
            else
            {
                xputs(PSTR("Error f_lseek in SDInfo.mat.\n"));
            }
        }
        //send the remained data
        offset = (b1-1)*50;
        fresult=f_lseek(&g_file_arenaconfig, offset);
        if ((fresult == FR_OK) && (g_file_arenaconfig.fptr == offset))
        {
            fresult = f_read(&g_file_arenaconfig, matBuff, fileRemain, &cnt);
            if ((fresult == FR_OK) && (cnt == fileRemain))
            {
                send_Tx_buffer(matBuff, fileRemain);
            }
        }
        else
            xputs(PSTR("Error f_lseek in remained data.\n"));
        
        f_close(&g_file_arenaconfig);
    } 
    else 
        xputs(PSTR("Error f_open in SDInfo.mat.\n"));//end if (fresult == FR_OK
    
}

