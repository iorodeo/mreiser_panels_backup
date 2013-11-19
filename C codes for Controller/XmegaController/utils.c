#include "utils.h"


FIL             g_file_flash, g_file_eeprom;
TWI_Master_t    twi1;    // TWI master module #1
TWI_Master_t    twi2;    // TWI master module #2
TWI_Master_t    twi3;    // TWI master module #3
TWI_Master_t    twi4;    // TWI master module #4

static uint16_t	s_command_rangeregister[2]={0,0};


void init_all()
{ 
  ledWrite(LED0, ON);
  
  PORTC.DIRSET = PIN3_bm; // UART pin
  PORTC.DIRCLR = PIN2_bm; // UART pin

  PORTC.DIRSET = PIN4_bm; // SS pin for SPI unit on port C
  PORTC.DIRSET = PIN5_bm; // MOSI pin for SPI unit on port C
  PORTC.DIRSET = PIN7_bm; // SCL pin for SPI unit on port C
  PORTC.OUTSET = PIN4_bm; // Set SS high

  PORTD.DIRSET = PIN4_bm; // SS pin for SPI unit on port D
  PORTD.DIRSET = PIN5_bm; // MOSI pin for SPI unit on port D
  PORTD.DIRSET = PIN7_bm; // SCL pin for SPI unit on port D
  PORTD.OUTSET = PIN4_bm; // Set SS high

  PORTE.DIR = 0xB0;       // SPI (SD-card)
  PORTE.OUT = 0x00;

  PORTH.DIRSET = PIN5_bm; // LDAC
  PORTH.DIRSET = PIN6_bm; // CLR
  PORTH.DIRSET = PIN7_bm; // BIN
  PORTH.OUTCLR = PIN5_bm; // LDAC low
  PORTH.OUTSET = PIN6_bm; // CLR high
  PORTH.OUTCLR = PIN7_bm; // BIN low

  PORTJ.DIR = 0xf0;       // LEDs are on bits 4 - 7
  PORTJ.OUT = 0xf0;       // All LEDs off


  PORTK.DIR = 0xf0;       // bit 4 - 7 are external signal direction control
  PORTK.OUT = 0xff;       // external signal direction set to input (0 = input)



 // set digital I/O signals to outputs
  digitalMode(DIO_0, OUTPUT);     //used to trigger laser
  digitalMode(DIO_1, OUTPUT);     //used as a sign for the start and end of fetch_display_frame function
  digitalMode(DIO_2, OUTPUT);     //used to trigger camera
  digitalMode(DIO_3, INPUT);      //Used to detect external trigger signal

  PORTK.PIN3CTRL = 0x01;      //INT3 senses rising edge
  PORTK.INT0MASK = 0x00;      //disable Int3 as source for port interrupt 0x00, enable it with 0x08.
  PORTK.INTCTRL = 0x02;       //Set Int3 INT0 Level medium   

  
  OSC.XOSCCTRL = 0x47;    // 0.4-16 MHz XTAL - 1K CLK Start Up
  OSC.PLLCTRL = 0xC4;     // XOSC is PLL Source - 4x Factor (32MHz)
  OSC.CTRL = 0x18;        // Enable PLL & External Oscillator

  // switch to 32 MHz oscillator
  while(!testbit(OSC.STATUS,OSC_PLLRDY_bp));  // wait until PLL stable
  CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // 32MHz from PLL
  _delay_ms(100);         // Debounce ON Switch

  // Timer TCE1: 100Hz timer for SD Card routines
  TCE1.CTRLA = 0x04;      // Prescaler: clk/8
  TCE1.PER   = 39999;     // 100Hz
  TCE1.INTCTRLA = 0x02;   // Timer overflow is a medium level interrupt

  // Set initial default values for analog input voltage ranges.
  // Note: if you change the ranges, also change in analogRead().
  s_command_rangeregister[0] = ADC_WRITE | ADC_RANGEREGISTER1
									| ADC_RR_BITS(0, ADC_RR_VIN_PLUSMINUS5)
									| ADC_RR_BITS(1, ADC_RR_VIN_PLUSMINUS5)
									| ADC_RR_BITS(2, ADC_RR_VIN_PLUS10)
									| ADC_RR_BITS(3, ADC_RR_VIN_PLUS10);

  s_command_rangeregister[1] = ADC_WRITE | ADC_RANGEREGISTER2
									| ADC_RR_BITS(4, ADC_RR_VIN_PLUSMINUS5)
									| ADC_RR_BITS(5, ADC_RR_VIN_PLUSMINUS5)
									| ADC_RR_BITS(6, ADC_RR_VIN_PLUS10)
									| ADC_RR_BITS(7, ADC_RR_VIN_PLUS10);

  // Prepare for SPI communication to the ADC7328.
  SPIC.CTRL = 0x58;       // 0101 1000:  Enable Master Mode, Mode 2, clkper/4

  // Write the range registers.
  writeCommandToADC(s_command_rangeregister[0]);
  writeCommandToADC(s_command_rangeregister[1]);


  // sequence register: all channels off.
  writeCommandToADC(ADC_WRITE | ADC_SEQUENCEREGISTER | ADC_SR_NONE);

  // control register: ch 000, mode = 00, pm = 00, code = 0(twos complement), ref = 1, seq = 00
  writeCommandToADC(ADC_WRITE | ADC_CONTROLREGISTER |
		  	  	  	  ADC_CR_ADDRESS(0) |
		  	  	  	  ADC_CR_MODE_8SINGLEENDED |
		  	  	  	  ADC_CR_PM_NORMAL |
		  	  	  	  ADC_CR_CODING2S |
		  	  	  	  ADC_CR_REFINT |
		  	  	  	  ADC_CR_SEQ_NONE);


  // Prepare for SPI communication to the DAC.
  SPID.CTRL = 0x58;       // 0101 1000:  Enable Master Mode, Mode 2, clkper/4

  // Initialize DAC (SPI master on port D)
  // DAC power control register (all ch + ref powered up)
  PORTD.OUTCLR = PIN4_bm;
  SPID.DATA = 0x10;
  loop_until_bit_is_set(SPID.STATUS, 7);
  SPID.DATA = 0x00;
  loop_until_bit_is_set(SPID.STATUS, 7);
  SPID.DATA = 0x1f;
  loop_until_bit_is_set(SPID.STATUS, 7);
  PORTD.OUTSET = PIN4_bm;

//DAC output range register (all ch +/-10V range)  
  PORTD.OUTCLR = PIN4_bm;
  SPID.DATA = 0x0c;
//	SPID.DATA = 0x08; // only ch 0
  loop_until_bit_is_set(SPID.STATUS, 7);
  SPID.DATA = 0x00;
  loop_until_bit_is_set(SPID.STATUS, 7);
   SPID.DATA = 0x04;
  loop_until_bit_is_set(SPID.STATUS, 7);
  PORTD.OUTSET = PIN4_bm;

  // initialize the UART
  uart_init();
  
  //initialize timer
  timer_init();
  
  Handler_Init();

  // Enable interrupts
  PMIC.CTRL = 0x07;       // Enable High, Medium and Low level interrupts
  sei();
}


// set_voltage_range_channel()
// Sets one channel of the ADC chip to the given voltage range.
// ch: a channel number in the range 0-7
// range:  One of
//         ADC_RR_VIN_PLUSMINUS10
//         ADC_RR_VIN_PLUSMINUS5
//         ADC_RR_VIN_PLUSMINUS2P5
//         ADC_RR_VIN_PLUS10
//
void set_voltage_range_channel(uint8_t ch, uint8_t range)
{
	// Update the bits in the static variable.
	s_command_rangeregister[ADC_RR_FROM_CH(ch)] &= ~ADC_RR_CH_MASK(ch);		// Clear the bits for this channel.
	s_command_rangeregister[ADC_RR_FROM_CH(ch)] |= ADC_RR_BITS(ch, range);  // Set the new range for this channel.

	// Send the bits out to the ADC chip.
	SPIC.CTRL = 0x58;
	writeCommandToADC(s_command_rangeregister[ADC_RR_FROM_CH(ch)]);
}


void writeCommandToADC (uint16_t command)
{
	  PORTC.OUTCLR = PIN4_bm;				// Select the AD7328 DAC chip.
	  SPIC.DATA = MSB(command);
	  loop_until_bit_is_set(SPIC.STATUS, 7);
	  SPIC.DATA = LSB(command);
	  loop_until_bit_is_set(SPIC.STATUS, 7);
	  PORTC.OUTSET = PIN4_bm;				// Execute the command, i.e. deselect the AD7328 DAC chip.

}


int16_t readConversionFromADC (void)
{
	int16_t	w1;

	PORTC.OUTCLR = PIN4_bm;				// Select the AD7328 DAC chip.
	SPIC.DATA = 0;
	loop_until_bit_is_set(SPIC.STATUS, 7);
	((uint8_t*)&w1)[1] = SPIC.DATA;
	SPIC.DATA = 0;
	loop_until_bit_is_set(SPIC.STATUS, 7);
	((uint8_t*)&w1)[0] = SPIC.DATA;
	PORTC.OUTSET = PIN4_bm;				// Deselect the AD7328 DAC chip.

	return w1;
}


// From Application Note AVR1003
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	uint8_t volatile saved_sreg = SREG;
	cli();
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif

	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
	);

	SREG = saved_sreg;
}

void ledWrite( uint8_t led, uint8_t value )
{
	// ignore write if out-ouf-bounds argument
	if (!((led & ~3) || (value & ~1)))
	{
		if (value == 1)
			PORTJ.OUTCLR = 1 << 4 + led;	// turn the led ON
		else
			PORTJ.OUTSET = 1 << 4 + led;	// turn the led OFF
	}
}

void ledToggle( uint8_t led )
{
	// ignore write if out-ouf-bounds argument
	if (!(led & ~3))
		PORTJ.OUTTGL = 1 << 4 + led;	// toggle the led
}

void ledBlink(void)
{ 
  /* blink LED 1, this is a simple debug tool to see if the controller is responsive */
  int j;
  	
  for(j = 0;j < 10;j++)
  {
    ledToggle(1);
    _delay_ms(350);
  }
}

void digitalMode( uint8_t bit, uint8_t mode)
{
	if (!(bit & ~3))
	{
		if (mode == OUTPUT)
		{
			/* set the port as output */
			PORTK.OUTCLR = 1 << 4 + bit; // set the external driver mode to output first
			PORTK.DIRSET = 1 << bit;   // set the uC pin direction to output second
		}
		else if (mode == INPUT)
		{
			/* set the port as input */
			PORTK.DIRCLR = 1 << bit;   // set the uC pin direction to input first
			PORTK.OUTSET = 1 << 4 + bit; // set the external driver direction to input
		}
	}
}

uint8_t digitalRead( uint8_t bit )
{
	uint8_t res;

	// ignore read if out-ouf-bounds argument
	if (!(bit & ~3))
		res = (PORTK.IN >> bit) & 0x01;
	else
		res = 0;

	return(res);
}

void digitalWrite( uint8_t bit, uint8_t value )
{
	// ignore write if out-ouf-bounds argument
	if (!((bit & ~3) || (value & ~1)))
	{
		if (value == 1)
			PORTK.OUTSET = 1 << bit;	// set the bit HIGH
		else
			PORTK.OUTCLR = 1 << bit;	// set the bit LOW
	}
}

void digitalToggle( uint8_t bit )
{
	// ignore write if out-ouf-bounds argument
	if (!(bit & ~3))
		PORTK.OUTTGL = 1 << bit;	// toggle the bit
}

int16_t analogRead( uint8_t ch )
{
	int16_t w1;

	if (ch <= 7)
	{
		// control register: ch = <ch>, mode = 00, pm = 00, code = 0, ref = 1, seq = 00
		//coding = 0,the output coding is twos complement
		writeCommandToADC(ADC_WRITE | ADC_CONTROLREGISTER
							| ADC_CR_ADDRESS(ch)
							| ADC_CR_MODE_8SINGLEENDED
							| ADC_CR_PM_NORMAL
							| ADC_CR_CODING2S
							| ADC_CR_REFINT
							| ADC_CR_SEQ_NONE);

		// do a conversion on the selected channel (no register write)
		w1 = readConversionFromADC();

		//ADC0-1 -5V-5V  //ADC2-7 0-10V
		//in this way, 0v ->0, -5V -> -2047, 5V->2047
		if (ch <= 1)
		{
			//-5V-0V
			if (w1 & 1 << 12)
			{
				//input value is -5V-0v, ADC code from 0x1000 to 0x1fff
				w1 = (w1 & 0x0fff)>>1;
				w1 = w1 | 0xf800;
				//output value now is from 0xf800 to 0xffff
			}
			else
			{   //0-5V
				//since output data : 3 channel id bits + sign bit + 12 conversion result
				//input value is 0-5V, 12 ADC code from 0x0000 to 0x0fff
				//remove the 3 channel id bits
				w1 = (w1 & 0x0fff)>>1;
				//output value is now 0x0000 to 0x07ff
			}
		}
		else // ch > 1
		{
			//in this way, 0v ->0, 5V -> 2047, 10V->4095
			if (w1 & 1 << 12)
			{
				//input value is less than 5V, ADC code from 0x1000 to 0x1fff
				w1 = (w1 & 0x0fff) >> 1;
				//output value now is from 0x0000 to 0x07ff
			}
			else
			{
				//since output data : 3 channel id bits + sign bit + 12 conversion result
				//input value is bigger than 5v, 12 ADC code from 0x0000 to 0x0fff
				w1 = (w1 | 0x1000) >> 1;
				//remove the 3 channel id bits
				w1 = w1 & 0x0fff;
				//output value is now 0x0800 to 0x0fff
			}
		}
		return w1;
	}
	else
		return 0;
}

void analogWrite(uint8_t ch, int16_t value)
{
	//AD5754 16 bit DAC, it also works for AD5724 12 bit DAC
	//chanel number should from 0 to 3 and value ranges from -32767 to 32767
	if (!((ch & ~3) || ((value > 32767) || (value < -32767))))
	{
		/* DAC register*/
		PORTD.OUTCLR = PIN4_bm;        // SPI SS = L
		SPID.DATA = 0x00 | (ch & 0x7);
		loop_until_bit_is_set(SPID.STATUS, 7);
		SPID.DATA = (uint8_t)((value & 0xff00) >> 8);
		loop_until_bit_is_set(SPID.STATUS, 7);
		SPID.DATA = (uint8_t)(value & 0xff) ;
		loop_until_bit_is_set(SPID.STATUS, 7);
		PORTD.OUTSET = PIN4_bm;        // SPI SS = H
	}
}
 
void test_DIO(uint8_t ch)
{ 
	int16_t ADC_val;
	uint8_t k;

	for (k = 0; k < 60; k++)
	{
		//flip all 4 bits
		digitalToggle(0);
		digitalToggle(1);
		digitalToggle(2);
		digitalToggle(3);

		_delay_ms(100);			
		ADC_val = analogRead(ch); // +/-5v range, 1v = 408
		_delay_ms(100);			
		analogWrite(1, ADC_val); // +/- 5v range, 1v = 408

		if (k % 2)
			ledToggle(1);  //toggle LED, once per square wave pulse
	}
}

void SystemReset(void)
{
	//issue software reset
	CCPWrite(&RST.CTRL, RST_SWRST_bm);
}

void test_ADC(uint8_t ch)
{
	int16_t X_dac_val;
	int16_t ADC_val;
	uint8_t j, k;
		
	for (k = 0; k < 30; k++)
	{
		for (j = 0; j < 200; j++)
		{	if (j < 100)
				X_dac_val = 20*(uint16_t)j; 	// build the up part of the triangle wave
			else
				X_dac_val = 20*(uint16_t)(100 + (100 - j)); // build the down part of the triangle wave
 	
            analogWrite(0, X_dac_val);
			ADC_val = analogRead(ch);   //1v = 102 -> ~5 TIMES THE GAIN OF OL
            analogWrite(1, ADC_val);
			_delay_ms(2);			
		}
		ledToggle(1);  //toggle LED, once per triangle wave pulse
	}	 	
}

void progPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff)
{
    xputs(PSTR("#"));
//  xprintf(PSTR("flash page write: 0x%lX 0x%X\n"), paddr, psize);
    buff[0] = 'B';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize>>1;
    buff[4] = 'F';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], psize+HEADER_SIZE, 0);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
        //xputs(PSTR("I am waiting.\n"));
    }
    if (twi->result != TWIM_RESULT_OK)
    {
        xprintf(PSTR("Bad flash write, result = %u\n"), twi->result);
    }
}

void readPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff)
{
    uint8_t i;
    
    xputs(PSTR("#"));
    buff[0] = 'g';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize>>1;
    buff[4] = 'F';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], HEADER_SIZE, psize);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
    }
    if (twi->result != TWIM_RESULT_OK)
        xprintf(PSTR("Bad flash read, result = %u\n"), twi->result);
    
    for (i = 0; i < psize; i++)
        buff[i+HEADER_SIZE] = *(twi->PreadData + i);
}


int verifyPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff)
{
    uint8_t match, i;
    
    xputs(PSTR("#"));
    buff[0] = 'g';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize>>1;
    buff[4] = 'F';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], HEADER_SIZE, psize);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
    }
    if (twi->result != TWIM_RESULT_OK)
        xprintf(PSTR("Bad flash read, result = %u\n"), twi->result);

    match = 1;
    for (i = 0; i < psize; i++)
    {
        if (*(twi->PreadData + i) != buff[i+HEADER_SIZE])
            match = 0;
    }
    return match;
}

void progEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff)
{
    xputs(PSTR("#"));
    buff[0] = 'B';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize;
    buff[4] = 'E';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], psize+HEADER_SIZE, 0);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
    }
    if (twi->result != TWIM_RESULT_OK)
        xprintf(PSTR("Bad eeprom write, result = %u\n"), twi->result);

}

void readEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff)
{
    uint8_t i;
    
    xputs(PSTR("#"));
    buff[0] = 'g';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize;
    buff[4] = 'E';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], HEADER_SIZE, psize);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
    }
    if (twi->result != TWIM_RESULT_OK)
        xprintf(PSTR("Bad flash read, result = %u\n"), twi->result);
    
    for (i = 0; i < psize; i++)
        buff[i+HEADER_SIZE] = *(twi->PreadData + i);

}

int verifyEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff) {
    uint8_t match, i;
    
    xputs(PSTR("#"));
    buff[0] = 'g';
    buff[1] = (uint8_t)((paddr>>8) & 0xff);
    buff[2] = (uint8_t)(paddr & 0xff);
    buff[3] = psize;
    buff[4] = 'E';
    TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &buff[0], HEADER_SIZE, psize);
    while (twi->status != TWIM_STATUS_READY)
    {
        /* Wait until transaction is complete. */
    }
    if (twi->result != TWIM_RESULT_OK)
        xprintf(PSTR("Bad flash read, result = %u\n"), twi->result);

    match = 1;
    for (i = 0; i < psize; i++)
    {
        if (*(twi->PreadData + i) != buff[i+HEADER_SIZE])
            match = 0;
    }
    return match;
}

void flash_panel(uint8_t panel_num)
{
    uint8_t res, rc;
    uint16_t nextaddr, page;
    int32_t paddr;
    uint8_t psize;
    uint8_t pagestartaddr, pageendaddr, pageoffset;
    uint8_t pagebuff[PAGE_SIZE+HEADER_SIZE];
    ihexrec_t ihex;
    uint16_t lineno;
    uint16_t len, i;
    TWI_Master_t *twi;
    uint8_t ch;
    uint8_t flashBuff[256];
    
    ch = g_ch_from_panel[panel_num];
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
        
        // open the hex file for reading
        res = f_open(&g_file_flash, panelFlash, FA_OPEN_EXISTING | FA_READ);
        if (res != FR_OK)
        {
            // could'n open the file
            xputs(PSTR("Error f_open in panel.hex.\n"));
            put_rc(res);
        }
        else
        {
            // file open OK, get the panel address
            //paddr = panel_num;
            if (panel_num >= 128)
            {
                // use hardware reset
                PORTD.OUTCLR = 0x04;
                _delay_ms(10);
                PORTD.OUTSET = 0x04;
            }
            else
            {
                // use cmd reset (assumes panel firmware already loaded)
                xprintf(PSTR("flash panel %u.\n"), panel_num);
                pagebuff[0] = 0x00;
                pagebuff[1] = 0x01;
                TWI_MasterWriteRead(twi, panel_num, &pagebuff[0], 2, 0);
                while (twi->status != TWIM_STATUS_READY)
                {
                    /* Wait until transaction is complete. */
                }
                if (twi->result != TWIM_RESULT_OK)
                    xprintf(PSTR("Bad panel reset, result = %u\n"), twi->result);

            }
            //i2cMasterSend(panel_num, 2, RESET);
            //i2cMasterSend(1, 2, RESET);
            //TWI_MasterWriteRead(&twi1, panel_num, &buff[0], psize+HEADER_SIZE, 0);
        }
        
        // program the panel
        _delay_ms(500); // wait .5 sec for the panel to come out of reset
        lineno = 0;
        page = 65535;
        pagestartaddr = 0;
        pageendaddr = 0;
        xputs(PSTR("\nwriting:   "));
        while (f_gets((char*)flashBuff, sizeof(flashBuff), &g_file_flash) != NULL)
        {
            lineno++;
            len = strlen(flashBuff);
            if (flashBuff[len-1] == '\n')
                flashBuff[--len] = 0;
            if (flashBuff[0] != ':')
                continue;
            rc = ihex_readrec(&ihex, flashBuff);
            if (rc < 0)
            {
                xprintf(PSTR("invalid record at line %d of panel.hex.\n"), lineno);
                break;
            }
            else if (rc != ihex.cksum)
            {
                xprintf(PSTR("ERROR: checksum mismatch at line %d of panel.hex.\n"), lineno);
                xprintf(PSTR("checksum=0x%02x, computed checksum=0x%02x\n"), ihex.cksum, rc);
                break;
            }
            else
            {
                if (ihex.rectyp == 0)
                {
                    // data record
                    nextaddr = ihex.loadofs;
                    if ((nextaddr >> PAGE_SIZE_SHIFT) != page)
                    {
                        // this record is for a different page - check if we need to flush the current page
                        if (pagestartaddr != pageendaddr)
                        {
                            // flush data in page buffer
                            paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                            psize = pageendaddr - pagestartaddr;
                            progPage(twi, paddr, psize, &pagebuff[0]);
                        }
                        page = (nextaddr >> PAGE_SIZE_SHIFT);
                        pagestartaddr = pageendaddr = nextaddr & (PAGE_SIZE - 1);
                    }
                    pageoffset = nextaddr & (PAGE_SIZE - 1);
                    if (pageoffset < pagestartaddr)
                        pagestartaddr = pageoffset;
                    for (i=0; i<ihex.reclen; i++)
                        pagebuff[pageoffset+i+HEADER_SIZE] = ihex.data[i];

                    if ((pageoffset + ihex.reclen) > pageendaddr)
                        pageendaddr = pageoffset+ihex.reclen;

                }
                else if (ihex.rectyp == 1)
                {
                    // end of file record
                    if (pagestartaddr != pageendaddr)
                    {
                        // flush the data in page buffer
                        paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                        psize = pageendaddr - pagestartaddr;
                        progPage(twi, paddr, psize, &pagebuff[0]);
                    }
                    break;
                }
                else
                {
                    xprintf(PSTR("don't know how to deal with rectype=%d at line %d of panel.hex\n"),
                            ihex.rectyp, lineno);
                    break;
                }
            }
        }
        // rewind the the input file to the start for verification
        res = f_lseek(&g_file_flash, 0);
        if (res != FR_OK)
        {
            xputs(PSTR("Error f_lseek in panel.hex.\n"));
            put_rc(res);
        }
        else
        {
            // verify the panel data
            lineno = 0;
            page = 65535;
            pagestartaddr = 0;
            pageendaddr = 0;
            xputs(PSTR("\nverifying: "));
            while (f_gets((char*)flashBuff, sizeof(flashBuff), &g_file_flash) != NULL)
            {
                lineno++;
                len = strlen(flashBuff);
                if (flashBuff[len-1] == '\n')
                    flashBuff[--len] = 0;

                if (flashBuff[0] != ':')
                    continue;

                rc = ihex_readrec(&ihex, flashBuff);
                if (rc < 0)
                {
                    xprintf(PSTR("invalid record at line %d of panel.hex\n"), lineno);
                    break;
                }
                else if (rc != ihex.cksum)
                {
                    xprintf(PSTR("ERROR: checksum mismatch at line %d of panel.hex\n"), lineno);
                    xprintf(PSTR("checksum=0x%02x, computed checksum=0x%02x\n"), ihex.cksum, rc);
                    break;
                }
                else
                {
                    if (ihex.rectyp == 0)
                    {
                        // data record
                        nextaddr = ihex.loadofs;
                        if ((nextaddr >> PAGE_SIZE_SHIFT) != page)
                        {
                            // this record is for a different page - check if we need to flush the current page
                            if (pagestartaddr != pageendaddr)
                            {
                                // flush data in page buffer
                                paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                                psize = pageendaddr - pagestartaddr;
                                if (verifyPage(twi, paddr, psize, &pagebuff[0])==0)
                                    xprintf(PSTR("verify mismatch for page %d\n"), page);

                            }
                            page = (nextaddr >> PAGE_SIZE_SHIFT);
                            pagestartaddr = pageendaddr = nextaddr & (PAGE_SIZE - 1);
                        }
                        pageoffset = nextaddr & (PAGE_SIZE - 1);
                        if (pageoffset < pagestartaddr)
                            pagestartaddr = pageoffset;

                        for (i=0; i<ihex.reclen; i++)
                            pagebuff[pageoffset+i+HEADER_SIZE] = ihex.data[i];

                        if ((pageoffset + ihex.reclen) > pageendaddr)
                            pageendaddr = pageoffset+ihex.reclen;

                    }
                    else if (ihex.rectyp == 1)
                    {
                        // end of file record
                        if (pagestartaddr != pageendaddr)
                        {
                            // flush the data in page buffer
                            paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                            psize = pageendaddr - pagestartaddr;
                            if (verifyPage(twi, paddr, psize, &pagebuff[0])==0)
                                xprintf(PSTR("verify mismatch for page %d\n"), page);

                        }
                        xputs(PSTR("\n"));
                        break;
                    }
                    else
                    {
                        xprintf(PSTR("don't know how to deal with rectype=%d at line %d of panel.hex\n"),
                                ihex.rectyp, lineno);
                        break;
                    }
                }
            }
        }
        // exit bootloader
        pagebuff[0] = 'E';
        TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &pagebuff[0], 1, 0);
        while (twi->status != TWIM_STATUS_READY)
        {
            /* Wait until transaction is complete. */
        }
        if (twi->result != TWIM_RESULT_OK)
            xprintf(PSTR("Bad exit cmd, result = %u\n"), twi->result);
        
        //i2cMasterSend(panel_num, 1, &pagebuff[0]);
        //TWI_MasterWriteRead(&twiMaster1, PANEL_BL_ADDR, &pagebuff[0], 1, 0);
    }
    else
    {
        xprintf(PSTR("Cannot find the panel %d.\n"), panel_num);
    }
};

void eeprom_panel(uint8_t panel_num)
{
    uint8_t res, rc;
    uint16_t nextaddr, page;
    int32_t paddr;
    uint8_t psize;
    uint8_t pagestartaddr, pageendaddr, pageoffset;
    uint8_t pagebuff[PAGE_SIZE+HEADER_SIZE];
    ihexrec_t ihex;
    uint16_t lineno;
    uint16_t len, i;
    TWI_Master_t *twi;
    uint8_t ch;
    uint8_t eepromBuff[256];
    
    ch = g_ch_from_panel[panel_num];
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
        
        // open the hex file for reading
        res = f_open(&g_file_eeprom, panelEEprom, FA_OPEN_EXISTING | FA_READ);
        if (res != FR_OK)
        {
            // could'n open the file
            xputs(PSTR("Error f_open in eeprom.hex.\n"));
            put_rc(res);
        }
        else
        {
            // file open OK, get the panel address
            //paddr = panel_num;
            //xatoi(&argv[1], &paddr);
            if (paddr >= 128)
            {
                // use hardware reset
                PORTD.OUTCLR = 0x04;
                _delay_ms(10);
                PORTD.OUTSET = 0x04;
            }
            else
            {
                // use cmd reset (assumes panel firmware already loaded)
                //i2cMasterSend(panel_num, 2, RESET);
                
                // use cmd reset (assumes panel firmware already loaded)
                xprintf(PSTR("eeprom panel %u.\n"), panel_num);
                pagebuff[0] = 0x00;
                pagebuff[1] = 0x01;
                TWI_MasterWriteRead(twi, panel_num, &pagebuff[0], 2, 0);
                while (twi->status != TWIM_STATUS_READY)
                {
                    /* Wait until transaction is complete. */
                }
                if (twi->result != TWIM_RESULT_OK)
                    xprintf(PSTR("Bad panel reset, result = %u\n"), twi->result);

            }
        }
        // program the panel
        _delay_ms(500); // wait .5 sec for the panel to come out of reset
        lineno = 0;
        page = 65535;
        pagestartaddr = 0;
        pageendaddr = 0;
        xputs(PSTR("\nwriting:   "));
        while (f_gets((char*)eepromBuff, sizeof(eepromBuff), &g_file_eeprom) != NULL)
        {
            lineno++;
            len = strlen(eepromBuff);
            if (eepromBuff[len-1] == '\n')
                eepromBuff[--len] = 0;

            if (eepromBuff[0] != ':')
                continue;

            rc = ihex_readrec(&ihex, eepromBuff);
            if (rc < 0)
            {
                xprintf(PSTR("invalid record at line %d of eeprom.hex.\n"), lineno);
                break;
            }
            else if (rc != ihex.cksum)
            {
                xprintf(PSTR("ERROR: checksum mismatch at line %d of eeprom.hex.\n"), lineno);
                xprintf(PSTR("checksum=0x%02x, computed checksum=0x%02x\n"), ihex.cksum, rc);
                break;
            }
            else
            {
                if (ihex.rectyp == 0)
                {
                    // data record
                    nextaddr = ihex.loadofs;
                    if ((nextaddr >> PAGE_SIZE_SHIFT) != page)
                    {
                        // this record is for a different page - check if we need to flush the current page
                        if (pagestartaddr != pageendaddr)
                        {
                            // flush data in page buffer
                            paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                            psize = pageendaddr - pagestartaddr;
                            progEEPage(twi, paddr, psize, &pagebuff[0]);
                        }
                        page = (nextaddr >> PAGE_SIZE_SHIFT);
                        pagestartaddr = pageendaddr = nextaddr & (PAGE_SIZE - 1);
                    }
                    pageoffset = nextaddr & (PAGE_SIZE - 1);
                    if (pageoffset < pagestartaddr)
                        pagestartaddr = pageoffset;

                    for (i=0; i<ihex.reclen; i++)
                        pagebuff[pageoffset+i+HEADER_SIZE] = ihex.data[i];

                    if ((pageoffset + ihex.reclen) > pageendaddr)
                        pageendaddr = pageoffset+ihex.reclen;

                }
                else if (ihex.rectyp == 1)
                {
                    // end of file record
                    if (pagestartaddr != pageendaddr)
                    {
                        // flush the data in page buffer
                        paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                        psize = pageendaddr - pagestartaddr;
                        progEEPage(twi, paddr, psize, &pagebuff[0]);
                    }
                    break;
                }
                else
                {
                    xprintf(PSTR("don't know how to deal with rectype=%d at line %d of eeprom.hex.\n"),
                            ihex.rectyp, lineno);
                    break;
                }
            }
        }
        // rewind the the input file to the start for verification
        res = f_lseek(&g_file_eeprom, 0);
        if (res != FR_OK)
        {
            xputs(PSTR("Error f_lseek in eeprom.hex.\n"));
            put_rc(res);
        }
        else
        {
            // verify the panel data
            lineno = 0;
            page = 65535;
            pagestartaddr = 0;
            pageendaddr = 0;
            xputs(PSTR("\nverifying: "));
            while (f_gets((char*)eepromBuff, sizeof(eepromBuff), &g_file_eeprom) != NULL)
            {
                lineno++;
                len = strlen(eepromBuff);
                if (eepromBuff[len-1] == '\n')
                    eepromBuff[--len] = 0;

                if (eepromBuff[0] != ':')
                    continue;

                rc = ihex_readrec(&ihex, eepromBuff);
                if (rc < 0)
                {
                    xprintf(PSTR("invalid record at line %d of eeprom.hex\n"), lineno);
                    break;
                }
                else if (rc != ihex.cksum)
                {
                    xprintf(PSTR("ERROR: checksum mismatch at line %d of eeprom.hex\n"), lineno);
                    xprintf(PSTR("checksum=0x%02x, computed checksum=0x%02x\n"), ihex.cksum, rc);
                    break;
                }
                else
                {
                    if (ihex.rectyp == 0)
                    {
                        // data record
                        nextaddr = ihex.loadofs;
                        if ((nextaddr >> PAGE_SIZE_SHIFT) != page)
                        {
                            // this record is for a different page - check if we need to flush the current page
                            if (pagestartaddr != pageendaddr)
                            {
                                // flush data in page buffer
                                paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                                psize = pageendaddr - pagestartaddr;
                                if (verifyEEPage(twi, paddr, psize, &pagebuff[0])==0)
                                    xprintf(PSTR("verify mismatch for page %d\n"), page);

                            }
                            page = (nextaddr >> PAGE_SIZE_SHIFT);
                            pagestartaddr = pageendaddr = nextaddr & (PAGE_SIZE - 1);
                        }
                        pageoffset = nextaddr & (PAGE_SIZE - 1);
                        if (pageoffset < pagestartaddr)
                            pagestartaddr = pageoffset;

                        for (i=0; i<ihex.reclen; i++)
                            pagebuff[pageoffset+i+HEADER_SIZE] = ihex.data[i];

                        if ((pageoffset + ihex.reclen) > pageendaddr)
                            pageendaddr = pageoffset+ihex.reclen;

                    }
                    else if (ihex.rectyp == 1)
                    {
                        // end of file record
                        if (pagestartaddr != pageendaddr)
                        {
                            // flush the data in page buffer
                            paddr = (page << PAGE_SIZE_SHIFT) + pagestartaddr;
                            psize = pageendaddr - pagestartaddr;
                            if (verifyEEPage(twi, paddr, psize, &pagebuff[0])==0)
                                xprintf(PSTR("verify mismatch for page %d\n"), page);

                        }
                        xputs(PSTR("\n"));
                        break;
                    }
                    else
                    {
                        xprintf(PSTR("don't know how to deal with rectype=%d at line %d of eeprom.hex\n"),
                                ihex.rectyp, lineno);
                        break;
                    }
                }
            }
        }
        // exit bootloader
        pagebuff[0] = 'E';
        TWI_MasterWriteRead(twi, PANEL_BL_ADDR, &pagebuff[0], 1, 0);
        while (twi->status != TWIM_STATUS_READY)
        {
            /* Wait until transaction is complete. */
        }
        if (twi->result != TWIM_RESULT_OK)
            xprintf(PSTR("Bad exit cmd, result = %u\n"), twi->result);
        
    }else{
        xprintf(PSTR("Cannot find the panel %d.\n"), panel_num);
    }
    
};

static void put_rc(FRESULT rc)
{
    const prog_char *p;
    static const prog_char str[] =
            "OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
            "INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
            "INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0";
    FRESULT i;
    
    for (p = str, i = 0; i != rc && pgm_read_byte_near(p); i++)
        while(pgm_read_byte_near(p++));

    xprintf(PSTR("%S\n"), p);
}

static int16_t ihex_readrec(ihexrec_t * ihex, char * rec)
{
    int i, j;
    char buf[8];
    int offset, len;
    char * e;
    unsigned char cksum;
    int rc;
    
    len    = strlen(rec);
    offset = 1;
    cksum  = 0;
    
    /* reclen */
    if (offset + 2 > len)
        return -1;

    for (i=0; i<2; i++)
        buf[i] = rec[offset++];

    buf[i] = 0;
    ihex->reclen = strtoul(buf, &e, 16);
    if (e == buf || *e != 0)
        return -1;
    
    /* load offset */
    if (offset + 4 > len)
        return -1;

    for (i=0; i<4; i++)
        buf[i] = rec[offset++];

    buf[i] = 0;
    ihex->loadofs = strtoul(buf, &e, 16);
    if (e == buf || *e != 0)
        return -1;
    
    /* record type */
    if (offset + 2 > len)
        return -1;

    for (i=0; i<2; i++)
        buf[i] = rec[offset++];

    buf[i] = 0;
    ihex->rectyp = strtoul(buf, &e, 16);
    if (e == buf || *e != 0)
        return -1;
    
    cksum = ihex->reclen + ((ihex->loadofs >> 8) & 0x0ff) +
            (ihex->loadofs & 0x0ff) + ihex->rectyp;
    
    /* data */
    for (j=0; j<ihex->reclen; j++)
    {
        if (offset + 2 > len)
            return -1;

        for (i=0; i<2; i++)
            buf[i] = rec[offset++];

        buf[i] = 0;
        ihex->data[j] = strtoul(buf, &e, 16);
        if (e == buf || *e != 0)
            return -1;

        cksum += ihex->data[j];
    }
    
    /* cksum */
    if (offset + 2 > len)
        return -1;

    for (i=0; i<2; i++)
        buf[i] = rec[offset++];

    buf[i] = 0;
    ihex->cksum = strtoul(buf, &e, 16);
    if (e == buf || *e != 0)
        return -1;
    
    rc = -cksum & 0x000000ff;
    
    return rc;
}
