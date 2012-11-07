/* 	LED Panel with addressing
	Modification by Michael Reiser
		
	This is a modified version for the new (12/04) SMD version of the board
	made on 12/09/04 by MBR
	
	Yet another modification made to this program 04/06 by MBR to test the functiontality 
	of new ATmega 168 chip.
*/

#include "panel.h"

#include <avr/io.h>

volatile unsigned char CurrentCol;
unsigned char CurrentFrame;   //for GS frame

unsigned char DisplayBuffer[8];
unsigned char GS_Buffer[4][8];
unsigned char PATTERNS[RAM_PAT_NUMS][8];
unsigned char currentnum;
unsigned char deviceAddr;


unsigned char Map_flag = 0;   // = 0; debug only!
unsigned char Gray_Scale = 1;
//unsigned char Zero_map = 0;
//unsigned char One_map = 1;
unsigned char start_up = 1;

//#define MIRRORX	// Uncomment this to mirror the pattern horizontally.
//#define MIRRORY   // Uncomment this to mirror the pattern vertically.

	
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData);

int main(void)
{	
	int i;
	int j;
	
	deviceAddr = eeprom_read_byte(Panel_ID);
	
	outp(MASK_05, LED_DATA_PORT_DIRECTION_05);		/* set LED DATA, bits 0 - 5, to write mode */
	sbi(DDRC,PC0);
	sbi(DDRC,PC1);
	//outp(MASK_67, LED_DATA_PORT_DIRECTION_67);		/* set LED DATA, bits 6 - 7, to write mode */
		
	outp(DD_WRITE,LED_ADDRESS_PORT_DIRECTION);	/* set LED ADDRESS to write mode */
	sbi(DDRC,PC3);
	sbi(PORTC,PC3);
	
	//take care of Watchdog timer
	MCUSR &= ~(1<<WDRF); //clear WDRF in MCUSR
	WDTCSR |= (1<<WDCE) | (1<<WDE);  //send logical one to WDCE and WDE
	//turn off WDT
	WDTCSR = 0x00;	
		
	//CurrentCol = 0;			/* init current column variable */
	CurrentCol = 5;			/* init current column variable */
	CurrentFrame = 0;
	Gray_Scale = 1;				
	
	Handler_Init();					/* setup handlers */
	Reg_Handler(UpdateDisplay,2,1);	//runs at about 1/(4*256*8*(1/16E6)) = 2.44 Khz

	
	DisplayNum();
	
	i2cInit(); 
	i2cSetBitrate(400);
 	i2cSetLocalDeviceAddr(deviceAddr << 1, TRUE);
	i2cSetSlaveReceiveHandler( i2cSlaveReceiveService );
	
	long_delay(400);
	start_up = 0;	
	
	//to test for maximum number of patterns that can be held in PATTERNS - zero all of them
	for (j = 0; j < RAM_PAT_NUMS; j++){
		for (i=0; i< 8; i++){
		PATTERNS[j][i] = 0x00;
		}
	}
	
	
	while(1) { }
	return 0;
}


void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData)
{	
	u08 EEpat_num, frame_num;
		
	//use the length of the received packet to determine action
	switch(receiveDataLength) {
	case 1: // if length 1, then this is 1 byte - need to stretch to 8 bytes
		LoadPattern_1byte(*receiveData);
		break; 		
	case 2:   
	//a reset is 0x00, 0x01; display ID is: 0x00 0x02 Update ID is 0xFF, New Address
	//diplay bus number is 0xFE, bus#
	// for first digit different than 0x00, first char is ID for pattern in eeprom - 0x10, 
	// and second char is pattern number.
	//note - for row compressed patterns, 2 byte patterns are not supported!
		switch(*receiveData) {
		
		case 0x00: 
			if (*(receiveData+1) == 0x01) SystemReset();
			else if (*(receiveData+1) == 0x02) DisplayNum();
			break;
		
		case 0xF0:
			// just load that pattern from PATTERNS
			LoadPattern(PATTERNS[*(receiveData+1)]);
			break; 
			
		case 0xFE:
		    DisplayBusNum(*(receiveData+1));
			break;
	
		case 0xFF:   //updates the address, checks for gencall
			if ( start_up && ( ( i2cGetGenCall() && ( deviceAddr == 0 ) ) || (i2cGetGenCall() == 0 ) ) ){ 
				deviceAddr = *(receiveData+1);
				eeprom_write_byte( Panel_ID, deviceAddr);
				i2cSetLocalDeviceAddr(deviceAddr << 1, TRUE);
				DisplayNum();
				}
			else {LoadPatternEEP(SYMBOLS[1]);}	//first dot means not addressed correctly to update
			break;
			
			 
		default:
			EEpat_num = (*receiveData) - 0x10;
			frame_num = *(receiveData+1);
			
			if ( (EEpat_num < EEPAT_NUMS) & (frame_num < EEPAT_LENGTH[EEpat_num]) )			
				LoadPatternEEP(EEPATTERNS[EEPAT_START[EEpat_num]+frame_num]);
			else			
			LoadPatternEEP(SYMBOLS[2]);	//second dot means 'pattern referrenced to is invalid'
		}
		break;
	
	case 3: // if length 3, then this is a compressed g_scale pattern need to stretch to 24 bytes
		LoadPattern_3byte(receiveData);
		Gray_Scale = 7;     // this is a 7 level pattern - so Gray_Scale is 7
		break; 
		
	case 4: // if length 4, then this is a compressed g_scale pattern need to stretch to 32 bytes
        LoadPattern_4byte(receiveData);
        Gray_Scale = 15;
        break;
	
	case 8: LoadPattern(receiveData);	//stream in pattern		
		//Gray_Scale = 1;  , This is now set in LoadPattern.
		break;	
	
	case 9:	StorePattern(*(receiveData+8), receiveData);	
		break;		
		
			
	case 16: LoadPattern16(receiveData);	//stream in pattern		
		Gray_Scale = 3;     // this is a 2 byte pattern - so Gray_Scale is 3
		break;	
		
	case 24: LoadPattern24(receiveData);	//stream in pattern		
		Gray_Scale = 7;     // this is a 7 level pattern - so Gray_Scale is 7
		break;	

  	case 32: LoadPattern32(receiveData);	//stream in pattern
		Gray_Scale = 15;     // this is a 4 byte pattern - so Gray_Scale is 15
		break;	
		
	default:
		LoadPatternEEP(SYMBOLS[3]);   //third dot means wierd packet size received
		}
}


// ReverseBits() - Reverse the bit order of each byte in the given array of bytes.
void ReverseBits (unsigned char *px, unsigned char nBytes)
{
	unsigned char i;
	unsigned char x;
	
	for (i=0; i<nBytes; i++)
	{
		x = *(px+i);	// Copy the byte first, as opposed to multiple derefs & adds.
		x = ((x>>4) & 0x0F) | ((x & 0x0F) << 4); 
		x = ((x>>2) & 0x33) | ((x & 0x33) << 2);
		x = ((x>>1) & 0x55) | ((x & 0x55) << 1);
		*(px+i) = x;
	}
}


void UpdateDisplay(void)
{
	volatile unsigned char	temp_frame;
	unsigned char			colData;
	//const unsigned char	addrval[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
		

	
#ifdef MIRRORX
	colData = 7-CurrentCol;
#else
	colData = CurrentCol;
#endif
	
	// clear the data ports
	LED_DATA_PORT_05 = 0x00;
	LED_DATA_PORT_67 = 0x08;
	//delay(10);
	LED_ADDRESS_PORT = (1 << CurrentCol); // i.e. addrval[CurrentCol]; 

	if (Gray_Scale == 1) 
	{
		LED_DATA_PORT_05 = MASK_05 & DisplayBuffer[colData];
		LED_DATA_PORT_67 |= (MASK_67 & DisplayBuffer[colData]) >> 6;
	} 
	else if (Gray_Scale == 3) 
	{
	
		switch(CurrentFrame) 
		{
			case 0:
				temp_frame = GS_Buffer[0][colData];
				break;
			
			case 1:
				temp_frame = GS_Buffer[0][colData];
				break;
				
			case 2:
				temp_frame = GS_Buffer[1][colData];
				break;
			default:
				break;
		}
				
		LED_DATA_PORT_05 = MASK_05 & temp_frame;
		LED_DATA_PORT_67 |= (MASK_67 & temp_frame) >> 6;

	} else if (Gray_Scale == 7) {

		switch(CurrentFrame) {
				
			case 0:
				temp_frame = GS_Buffer[0][colData];
				break;
			case 1:
				temp_frame = GS_Buffer[0][colData];
				break;
			case 2:
				temp_frame = GS_Buffer[0][colData];	
				break;
			case 3:
				temp_frame = (GS_Buffer[0][colData]);	
				break;
			case 4:
				temp_frame = GS_Buffer[1][colData];	
				break;	
			case 5:
				temp_frame = GS_Buffer[1][colData];
				break;
			case 6:
				temp_frame = GS_Buffer[2][colData];
				break;				
			default:
				break;
		}

		LED_DATA_PORT_05 = MASK_05 & temp_frame;
		LED_DATA_PORT_67 |= (MASK_67 & temp_frame) >> 6;
	} else {

		switch(CurrentFrame) {
				
			case 0:
				temp_frame = GS_Buffer[0][colData];
				break;
			case 1:
				temp_frame = GS_Buffer[0][colData];
				break;
			case 2:
				temp_frame =  GS_Buffer[0][colData];
				break;
			case 3:
				temp_frame =  GS_Buffer[0][colData];
				break;
			case 4:
				temp_frame =  GS_Buffer[0][colData];
				break;	
			case 5:
				temp_frame =  GS_Buffer[0][colData];
				break;
			case 6:
				temp_frame =  GS_Buffer[0][colData];;
				break;
 			case 7:
				temp_frame =  GS_Buffer[0][colData];
				break;
			case 8:
				temp_frame =  GS_Buffer[1][colData];
				break;
			case 9:
				temp_frame = GS_Buffer[1][colData];
				break;
			case 10:
				temp_frame = GS_Buffer[1][colData];
				break;
			case 11:
				temp_frame = GS_Buffer[1][colData];
				break;	
			case 12:
				temp_frame = GS_Buffer[2][colData];
				break;
			case 13:
				temp_frame = GS_Buffer[2][colData];
				break;
 			case 14:
				temp_frame = GS_Buffer[3][colData];
				break;
			default:
				break;
		}

		LED_DATA_PORT_05 = MASK_05 & temp_frame;
		LED_DATA_PORT_67 |= (MASK_67 & temp_frame) >> 6;
	}

	if (CurrentCol == 0) {
		if(!(++CurrentFrame < Gray_Scale)) 
			CurrentFrame = 0;
	}	
	CurrentCol = (CurrentCol + 1) & 0x7;
} 


void DisplayChar(unsigned char c,unsigned char col)
{
#ifdef MIRRORY
	ReverseBits(&c,1);
#endif
	
	DisplayBuffer[col]=c;
}


void LoadPattern_1byte(unsigned char pattern_byte)
{
#ifdef MIRRORY
	ReverseBits(&pattern_byte,1);
#endif
	
	//should be done as above - but below is apparently 4 times faster!!!
	if (pattern_byte & (1<<0) ) DisplayBuffer[0] = 0xff; else DisplayBuffer[0] = 0x00;	
	if (pattern_byte & (1<<1) ) DisplayBuffer[1] = 0xff; else DisplayBuffer[1] = 0x00;	
	if (pattern_byte & (1<<2) ) DisplayBuffer[2] = 0xff; else DisplayBuffer[2] = 0x00;	
	if (pattern_byte & (1<<3) ) DisplayBuffer[3] = 0xff; else DisplayBuffer[3] = 0x00;	
	if (pattern_byte & (1<<4) ) DisplayBuffer[4] = 0xff; else DisplayBuffer[4] = 0x00;	
	if (pattern_byte & (1<<5) ) DisplayBuffer[5] = 0xff; else DisplayBuffer[5] = 0x00;	 
	if (pattern_byte & (1<<6) ) DisplayBuffer[6] = 0xff; else DisplayBuffer[6] = 0x00;	
	if (pattern_byte & (1<<7) ) DisplayBuffer[7] = 0xff; else DisplayBuffer[7] = 0x00;
	
	Gray_Scale = 1;
}

void LoadPattern_3byte(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,3);
#endif
	
	//should be done as above - but below is apparently 4 times faster!!!
	if (*(pattern) & (1<<0) ) GS_Buffer[0][0] = 0xff; else GS_Buffer[0][0] = 0x00;
	if (*(pattern) & (1<<1) ) GS_Buffer[0][1] = 0xff; else GS_Buffer[0][1] = 0x00;
	if (*(pattern) & (1<<2) ) GS_Buffer[0][2] = 0xff; else GS_Buffer[0][2] = 0x00;
	if (*(pattern) & (1<<3) ) GS_Buffer[0][3] = 0xff; else GS_Buffer[0][3] = 0x00;
	if (*(pattern) & (1<<4) ) GS_Buffer[0][4] = 0xff; else GS_Buffer[0][4] = 0x00;
	if (*(pattern) & (1<<5) ) GS_Buffer[0][5] = 0xff; else GS_Buffer[0][5] = 0x00;
	if (*(pattern) & (1<<6) ) GS_Buffer[0][6] = 0xff; else GS_Buffer[0][6] = 0x00;
	if (*(pattern) & (1<<7) ) GS_Buffer[0][7] = 0xff; else GS_Buffer[0][7] = 0x00;

	if (*(pattern + 1) & (1<<0) ) GS_Buffer[1][0] = 0xff; else GS_Buffer[1][0] = 0x00;
	if (*(pattern + 1) & (1<<1) ) GS_Buffer[1][1] = 0xff; else GS_Buffer[1][1] = 0x00;
	if (*(pattern + 1) & (1<<2) ) GS_Buffer[1][2] = 0xff; else GS_Buffer[1][2] = 0x00;
	if (*(pattern + 1) & (1<<3) ) GS_Buffer[1][3] = 0xff; else GS_Buffer[1][3] = 0x00;
	if (*(pattern + 1) & (1<<4) ) GS_Buffer[1][4] = 0xff; else GS_Buffer[1][4] = 0x00;
	if (*(pattern + 1) & (1<<5) ) GS_Buffer[1][5] = 0xff; else GS_Buffer[1][5] = 0x00;
	if (*(pattern + 1) & (1<<6) ) GS_Buffer[1][6] = 0xff; else GS_Buffer[1][6] = 0x00;
	if (*(pattern + 1) & (1<<7) ) GS_Buffer[1][7] = 0xff; else GS_Buffer[1][7] = 0x00;

	if (*(pattern + 2) & (1<<0) ) GS_Buffer[2][0] = 0xff; else GS_Buffer[2][0] = 0x00;
	if (*(pattern + 2) & (1<<1) ) GS_Buffer[2][1] = 0xff; else GS_Buffer[2][1] = 0x00;
	if (*(pattern + 2) & (1<<2) ) GS_Buffer[2][2] = 0xff; else GS_Buffer[2][2] = 0x00;
	if (*(pattern + 2) & (1<<3) ) GS_Buffer[2][3] = 0xff; else GS_Buffer[2][3] = 0x00;
	if (*(pattern + 2) & (1<<4) ) GS_Buffer[2][4] = 0xff; else GS_Buffer[2][4] = 0x00;
	if (*(pattern + 2) & (1<<5) ) GS_Buffer[2][5] = 0xff; else GS_Buffer[2][5] = 0x00;
	if (*(pattern + 2) & (1<<6) ) GS_Buffer[2][6] = 0xff; else GS_Buffer[2][6] = 0x00;
	if (*(pattern + 2) & (1<<7) ) GS_Buffer[2][7] = 0xff; else GS_Buffer[2][7] = 0x00;
}

void LoadPattern_4byte(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,4);
#endif

	if (*(pattern) & (1<<0) ) GS_Buffer[0][0] = 0xff; else GS_Buffer[0][0] = 0x00;
	if (*(pattern) & (1<<1) ) GS_Buffer[0][1] = 0xff; else GS_Buffer[0][1] = 0x00;
	if (*(pattern) & (1<<2) ) GS_Buffer[0][2] = 0xff; else GS_Buffer[0][2] = 0x00;
	if (*(pattern) & (1<<3) ) GS_Buffer[0][3] = 0xff; else GS_Buffer[0][3] = 0x00;
	if (*(pattern) & (1<<4) ) GS_Buffer[0][4] = 0xff; else GS_Buffer[0][4] = 0x00;
	if (*(pattern) & (1<<5) ) GS_Buffer[0][5] = 0xff; else GS_Buffer[0][5] = 0x00;
	if (*(pattern) & (1<<6) ) GS_Buffer[0][6] = 0xff; else GS_Buffer[0][6] = 0x00;
	if (*(pattern) & (1<<7) ) GS_Buffer[0][7] = 0xff; else GS_Buffer[0][7] = 0x00;

	if (*(pattern + 1) & (1<<0) ) GS_Buffer[1][0] = 0xff; else GS_Buffer[1][0] = 0x00;
	if (*(pattern + 1) & (1<<1) ) GS_Buffer[1][1] = 0xff; else GS_Buffer[1][1] = 0x00;
	if (*(pattern + 1) & (1<<2) ) GS_Buffer[1][2] = 0xff; else GS_Buffer[1][2] = 0x00;
	if (*(pattern + 1) & (1<<3) ) GS_Buffer[1][3] = 0xff; else GS_Buffer[1][3] = 0x00;
	if (*(pattern + 1) & (1<<4) ) GS_Buffer[1][4] = 0xff; else GS_Buffer[1][4] = 0x00;
	if (*(pattern + 1) & (1<<5) ) GS_Buffer[1][5] = 0xff; else GS_Buffer[1][5] = 0x00;
	if (*(pattern + 1) & (1<<6) ) GS_Buffer[1][6] = 0xff; else GS_Buffer[1][6] = 0x00;
	if (*(pattern + 1) & (1<<7) ) GS_Buffer[1][7] = 0xff; else GS_Buffer[1][7] = 0x00;

	if (*(pattern + 2) & (1<<0) ) GS_Buffer[2][0] = 0xff; else GS_Buffer[2][0] = 0x00;
	if (*(pattern + 2) & (1<<1) ) GS_Buffer[2][1] = 0xff; else GS_Buffer[2][1] = 0x00;
	if (*(pattern + 2) & (1<<2) ) GS_Buffer[2][2] = 0xff; else GS_Buffer[2][2] = 0x00;
	if (*(pattern + 2) & (1<<3) ) GS_Buffer[2][3] = 0xff; else GS_Buffer[2][3] = 0x00;
	if (*(pattern + 2) & (1<<4) ) GS_Buffer[2][4] = 0xff; else GS_Buffer[2][4] = 0x00;
	if (*(pattern + 2) & (1<<5) ) GS_Buffer[2][5] = 0xff; else GS_Buffer[2][5] = 0x00;
	if (*(pattern + 2) & (1<<6) ) GS_Buffer[2][6] = 0xff; else GS_Buffer[2][6] = 0x00;
	if (*(pattern + 2) & (1<<7) ) GS_Buffer[2][7] = 0xff; else GS_Buffer[2][7] = 0x00;
	
	
	if (*(pattern + 3) & (1<<0) ) GS_Buffer[3][0] = 0xff; else GS_Buffer[3][0] = 0x00;
	if (*(pattern + 3) & (1<<1) ) GS_Buffer[3][1] = 0xff; else GS_Buffer[3][1] = 0x00;
	if (*(pattern + 3) & (1<<2) ) GS_Buffer[3][2] = 0xff; else GS_Buffer[3][2] = 0x00;
	if (*(pattern + 3) & (1<<3) ) GS_Buffer[3][3] = 0xff; else GS_Buffer[3][3] = 0x00;
	if (*(pattern + 3) & (1<<4) ) GS_Buffer[3][4] = 0xff; else GS_Buffer[3][4] = 0x00;
	if (*(pattern + 3) & (1<<5) ) GS_Buffer[3][5] = 0xff; else GS_Buffer[3][5] = 0x00;
	if (*(pattern + 3) & (1<<6) ) GS_Buffer[3][6] = 0xff; else GS_Buffer[3][6] = 0x00;
	if (*(pattern + 3) & (1<<7) ) GS_Buffer[3][7] = 0xff; else GS_Buffer[3][7] = 0x00;
}

void LoadPattern(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,8);
#endif

		DisplayBuffer[0] = (*(pattern + 0));
		DisplayBuffer[1] = (*(pattern + 1));
		DisplayBuffer[2] = (*(pattern + 2));
		DisplayBuffer[3] = (*(pattern + 3));
		DisplayBuffer[4] = (*(pattern + 4));
		DisplayBuffer[5] = (*(pattern + 5));
		DisplayBuffer[6] = (*(pattern + 6));
		DisplayBuffer[7] = (*(pattern + 7));
		Gray_Scale = 1;
}


void LoadPattern16(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,16);
#endif
	
	GS_Buffer[0][0] = (*(pattern + 0));
	GS_Buffer[0][1] = (*(pattern + 1));
	GS_Buffer[0][2] = (*(pattern + 2));
	GS_Buffer[0][3] = (*(pattern + 3));
	GS_Buffer[0][4] = (*(pattern + 4));
	GS_Buffer[0][5] = (*(pattern + 5));
	GS_Buffer[0][6] = (*(pattern + 6));
	GS_Buffer[0][7] = (*(pattern + 7));
	
	GS_Buffer[1][0] = (*(pattern + 8));
	GS_Buffer[1][1] = (*(pattern + 9));
	GS_Buffer[1][2] = (*(pattern + 10));
	GS_Buffer[1][3] = (*(pattern + 11));
	GS_Buffer[1][4] = (*(pattern + 12));
	GS_Buffer[1][5] = (*(pattern + 13));
	GS_Buffer[1][6] = (*(pattern + 14));
	GS_Buffer[1][7] = (*(pattern + 15));
}


void LoadPattern24(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,24);
#endif
	
	GS_Buffer[0][0] = (*(pattern + 0));
	GS_Buffer[0][1] = (*(pattern + 1));
	GS_Buffer[0][2] = (*(pattern + 2));
	GS_Buffer[0][3] = (*(pattern + 3));
	GS_Buffer[0][4] = (*(pattern + 4));
	GS_Buffer[0][5] = (*(pattern + 5));
	GS_Buffer[0][6] = (*(pattern + 6));
	GS_Buffer[0][7] = (*(pattern + 7));
	
	GS_Buffer[1][0] = (*(pattern + 8));
	GS_Buffer[1][1] = (*(pattern + 9));
	GS_Buffer[1][2] = (*(pattern + 10));
	GS_Buffer[1][3] = (*(pattern + 11));
	GS_Buffer[1][4] = (*(pattern + 12));
	GS_Buffer[1][5] = (*(pattern + 13));
	GS_Buffer[1][6] = (*(pattern + 14));
	GS_Buffer[1][7] = (*(pattern + 15));
	
	GS_Buffer[2][0] = (*(pattern + 16));
	GS_Buffer[2][1] = (*(pattern + 17));
	GS_Buffer[2][2] = (*(pattern + 18));
	GS_Buffer[2][3] = (*(pattern + 19));
	GS_Buffer[2][4] = (*(pattern + 20));
	GS_Buffer[2][5] = (*(pattern + 21));
	GS_Buffer[2][6] = (*(pattern + 22));
	GS_Buffer[2][7] = (*(pattern + 23));
}

void LoadPattern32(unsigned char* pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,32);
#endif

	GS_Buffer[0][0] = (*(pattern + 0));
	GS_Buffer[0][1] = (*(pattern + 1));
	GS_Buffer[0][2] = (*(pattern + 2));
	GS_Buffer[0][3] = (*(pattern + 3));
	GS_Buffer[0][4] = (*(pattern + 4));
	GS_Buffer[0][5] = (*(pattern + 5));
	GS_Buffer[0][6] = (*(pattern + 6));
	GS_Buffer[0][7] = (*(pattern + 7));
	
	GS_Buffer[1][0] = (*(pattern + 8));
	GS_Buffer[1][1] = (*(pattern + 9));
	GS_Buffer[1][2] = (*(pattern + 10));
	GS_Buffer[1][3] = (*(pattern + 11));
	GS_Buffer[1][4] = (*(pattern + 12));
	GS_Buffer[1][5] = (*(pattern + 13));
	GS_Buffer[1][6] = (*(pattern + 14));
	GS_Buffer[1][7] = (*(pattern + 15));
	
	GS_Buffer[2][0] = (*(pattern + 16));
	GS_Buffer[2][1] = (*(pattern + 17));
	GS_Buffer[2][2] = (*(pattern + 18));
	GS_Buffer[2][3] = (*(pattern + 19));
	GS_Buffer[2][4] = (*(pattern + 20));
	GS_Buffer[2][5] = (*(pattern + 21));
	GS_Buffer[2][6] = (*(pattern + 22));
	GS_Buffer[2][7] = (*(pattern + 23));

 	GS_Buffer[3][0] = (*(pattern + 24));
	GS_Buffer[3][1] = (*(pattern + 25));
	GS_Buffer[3][2] = (*(pattern + 26));
	GS_Buffer[3][3] = (*(pattern + 27));
	GS_Buffer[3][4] = (*(pattern + 28));
	GS_Buffer[3][5] = (*(pattern + 29));
	GS_Buffer[3][6] = (*(pattern + 30));
	GS_Buffer[3][7] = (*(pattern + 31));
	}

void LoadPatternEEP(unsigned char *pattern)
{
#ifdef MIRRORY
	ReverseBits(pattern,8);
#endif

	DisplayBuffer[0] = eeprom_rb((uint8_t*)(pattern + 0));
	DisplayBuffer[1] = eeprom_rb((uint8_t*)(pattern + 1));
	DisplayBuffer[2] = eeprom_rb((uint8_t*)(pattern + 2));
	DisplayBuffer[3] = eeprom_rb((uint8_t*)(pattern + 3));
	DisplayBuffer[4] = eeprom_rb((uint8_t*)(pattern + 4));
	DisplayBuffer[5] = eeprom_rb((uint8_t*)(pattern + 5));
	DisplayBuffer[6] = eeprom_rb((uint8_t*)(pattern + 6));
	DisplayBuffer[7] = eeprom_rb((uint8_t*)(pattern + 7));
}

void StorePattern(unsigned char patternNumber, unsigned char *pattern)
{	
	PATTERNS[patternNumber][0] = (*(pattern + 0));
	PATTERNS[patternNumber][1] = (*(pattern + 1));
	PATTERNS[patternNumber][2] = (*(pattern + 2));
	PATTERNS[patternNumber][3] = (*(pattern + 3));
	PATTERNS[patternNumber][4] = (*(pattern + 4));
	PATTERNS[patternNumber][5] = (*(pattern + 5));
	PATTERNS[patternNumber][6] = (*(pattern + 6));
	PATTERNS[patternNumber][7] = (*(pattern + 7));
}


// delay for a minimum of <us> microseconds (from P. Stang)
// the time resolution is dependent on the time the loop takes 
// e.g. with 8Mhz and 5 cycles per loop, the resolution is 0.625 us 
void delay(unsigned short us) 
{
	unsigned short delay_loops;
	register unsigned short i;
	delay_loops = (us+3)/5*CYCLES_PER_US; // +3 for rounding up (rough) 
	// one loop takes 5 cpu cycles 
	for (i=0; i < delay_loops; i++) { asm volatile ("nop"); };
} 


//for longer delays, use this function - gives delay
//time in ms. Can delay up to about 1 minute.
void long_delay(unsigned short ms) 
{
	register unsigned short i;
	for (i=0; i < ms; i++)
	{
	delay(1000);	
	};
} 



// resets the chip
void SystemReset()
{
asm volatile ("cli"); // turn off interrupts
//WDTCR = _BV(WDE) | _BV(WDP2); // init WatchDog
//WDTCSR = _BV(WDE) | _BV(WDP2); // init WatchDog
WDTCSR |= (1<<WDCE) | (1<<WDE);
WDTCSR = (1<<WDE) | (1<<WDP0); //32 ms to timeout

while (1) asm volatile ("nop"); // wait for reset
}

void DisplayNum()
//display the current panel ID, for numbers upto 127
//this will in fact coorectly enumerate beyond 127 to 199
{
	int i, dig1, dig2, One_mask;
	//create a mask to append a row on top if the ID > 99
	if (deviceAddr > 99)	One_mask = 0x01;
	else		One_mask = 0x00;	
	
	dig1 = (deviceAddr%100)/10;
	dig2 = (deviceAddr%100)%10;
		
	for (i = 0; i < 4; i++)	//first character
	{ DisplayChar( (eeprom_rb((uint8_t*)(&NUMS[dig1][i])))|One_mask ,i);    }	
	for (i = 4; i < 8; i++) //second character
	{ DisplayChar( (eeprom_rb((uint8_t*)(&NUMS[dig2][i-4])))|One_mask ,i); 	}	
}


void DisplayBusNum(uint8_t busNum)
//display the current bus number
{
	int i, dig1, dig2;	
	
	dig1 = 0;
	dig2 = busNum;
		
	for (i = 0; i < 4; i++)	//first character
	{ DisplayChar( (eeprom_rb((uint8_t*)(&NUMS[dig1][i]))),i);}	
	for (i = 4; i < 8; i++) //second character
	{ DisplayChar( (eeprom_rb((uint8_t*)(&NUMS[dig2][i-4]))),i);}	
}

