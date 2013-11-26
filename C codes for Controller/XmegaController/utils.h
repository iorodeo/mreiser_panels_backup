#include "ff.h"	
#include "twi.h"
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "timer.h"
#include "xitoa.h"
#include "handler.h"
#include <stdlib.h>

#define testbit(port, bit) (uint8_t)(((uint8_t)port & (uint8_t)_BV(bit)))

#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3

#define ON 1
#define OFF 0

#define OUTPUT 0
#define INPUT 1
#define LOW 0
#define HIGH 1
#define IHEX_MAXDATA 256
#define PANEL_BL_ADDR 0x70
#define HEADER_SIZE 5
#define PAGE_SIZE 128
#define PAGE_SIZE_SHIFT 7


#define MSB(x)	(((x) & 0xFF00) >> 8)
#define LSB(x)	(((x) & 0x00FF))


#define DIO_0		0
#define DIO_1		1
#define DIO_2		2
#define DIO_3		3

#define DIO_LASER			DIO_0
#define DIO_FRAMEBUSY		DIO_1
#define DIO_TRIGGEROUT		DIO_2
#define DIO_TRIGGER			DIO_3



// Define some AD7328 bitfield settings.

#define ADC_READ			0x0000
#define ADC_WRITE			0x8000

#define ADC_CONTROLREGISTER		0x0000
#define ADC_RANGEREGISTER1		0x2000
#define ADC_RANGEREGISTER2		0x4000
#define ADC_SEQUENCEREGISTER	0x6000


// Control register bits.
#define ADC_CR_ADD2				0x1000
#define ADC_CR_ADD1				0x0800
#define ADC_CR_ADD0				0x0400
#define ADC_CR_ADD_BIT			10
#define ADC_CR_MODE1			0x0200
#define ADC_CR_MODE0			0x0100
#define ADC_CR_PM1              0x0080
#define ADC_CR_PM0              0x0040
#define ADC_CR_CODING 			0x0020
#define ADC_CR_REF	 			0x0010
#define ADC_CR_SEQ1		 		0x0008
#define ADC_CR_SEQ2		 		0x0004
#define ADC_CR_DOUT 			0x0002

// Control register bit combinations.
#define ADC_CR_ADDRESS(ch)				(((ch) & 0x7) << ADC_CR_ADD_BIT)
#define ADC_CR_MODE_7PSEUDODIFFERENTIAL	(ADC_CR_MODE1 | ADC_CR_MODE0)
#define ADC_CR_MODE_4FULLYDIFFERENTIAL	(ADC_CR_MODE1 | 0)
#define ADC_CR_MODE_4PSEUDODIFFERENTIAL	(           0 | ADC_CR_MODE0)
#define ADC_CR_MODE_8SINGLEENDED		(           0 | 0)
#define ADC_CR_PM_SHUTDOWN				(ADC_CR_PM1 | ADC_CR_PM0)
#define ADC_CR_PM_AUTOSHUTDOWN			(ADC_CR_PM1 | 0)
#define ADC_CR_PM_AUTOSTANDBY			(         0 | ADC_CR_PM0)
#define ADC_CR_PM_NORMAL				(         0 | 0)
#define ADC_CR_CODING1S 				(ADC_CR_CODING)
#define ADC_CR_CODING2S 				(0)
#define ADC_CR_REFINT	 				(ADC_CR_REF)
#define ADC_CR_REFEXT	 				(0)
#define ADC_CR_SEQ_NONE					(          0 |           0)
#define ADC_CR_SEQ_SEQREG				(          0 | ADC_CR_SEQ1)
#define ADC_CR_SEQ_0TOADDR				(ADC_CR_SEQ1 |           0)
#define ADC_CR_DOUT_DRIVE 				(ADC_CR_DOUT)
#define ADC_CR_DOUT_3STATE 				(0)


// Sequence register bits.
#define ADC_SR_CH0	 			0x1000
#define ADC_SR_CH1	 			0x0800
#define ADC_SR_CH2	 			0x0400
#define ADC_SR_CH3	 			0x0200
#define ADC_SR_CH4	 			0x0100
#define ADC_SR_CH5	 			0x0080
#define ADC_SR_CH6	 			0x0040
#define ADC_SR_CH7	 			0x0020
#define ADC_SR_NONE				0x0000

// Range register bits.
#define ADC_RR_VIN_PLUSMINUS10	0x0
#define ADC_RR_VIN_PLUSMINUS5	0x1
#define ADC_RR_VIN_PLUSMINUS2P5	0x2
#define ADC_RR_VIN_PLUS10		0x3

// Range register calculations.
#define ADC_RR_FROM_CH(ch)     ((ch)<4 ? 0 : 1)                      // Get zero-based register number from the channel.
#define ADC_RR_BIT_FROM_CH(ch) (((5-((ch)-(ch>3?4:0)))<<1) + 1)      // Convert a channel number to a bit position (RR0: ch0->bit11, ch1->bit9, ch2->bit7, ch3->bit5
                                                                     //                                             RR1: ch4->bit11, ch5->bit9, ch6->bit7, ch7->bit5)
#define ADC_RR_BITS(ch,range)  ((range) << ADC_RR_BIT_FROM_CH(ch))	 // Make a bitfield corresponding to a channel number and voltage range.
#define ADC_RR_CH_MASK(ch)     ((0x3) << ADC_RR_BIT_FROM_CH(ch))


static const uint8_t panelFlash[] = "panel.hex\0";
static const uint8_t panelEEprom[] = "panel.eep\0";

extern TWI_Master_t twi1;    // TWI master module #1
extern TWI_Master_t twi2;    // TWI master module #2
extern TWI_Master_t twi3;    // TWI master module #3
extern TWI_Master_t twi4;    // TWI master module #4

extern uint8_t  g_ch_from_panel[];  // panel twi channel mapping

typedef struct ihexrec {
	uint8_t  reclen;
	uint16_t loadofs;
	uint8_t  rectyp;
	uint8_t  data[IHEX_MAXDATA];
	uint8_t  cksum;
} ihexrec_t;

/* Function prototypes */
void     CCPWrite( volatile uint8_t * address, uint8_t value );
void     ledShow4Bits (uint8_t byte);
void     ledWrite( uint8_t led, uint8_t value );
void     ledToggle( uint8_t led );
void     ledBlink(void);
void     digitalMode( uint8_t bit, uint8_t mode);
uint8_t  digitalRead( uint8_t bit );
void     digitalWrite( uint8_t bit, uint8_t value );
void     digitalToggle( uint8_t bit );
uint16_t analogRead( uint8_t ch );
void     analogWrite(uint8_t ch, int16_t value);
void     test_DIO(uint8_t ch);
void     SystemReset(void);
void     test_ADC(uint8_t ch);
void     eeprom_panel(uint8_t panel_num);
void     flash_panel(uint8_t panel_num);
void     writeCommandToADC (uint16_t command);
void     set_voltage_range_channel(uint8_t ch, uint8_t range);

static int16_t ihex_readrec(ihexrec_t * ihex, char * rec);
static void    put_rc (FRESULT rc);

void    progPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);
void    readPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);
int     verifyPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);
void    progEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);
void    readEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);
int     verifyEEPage(TWI_Master_t *twi, uint32_t paddr, uint8_t psize, uint8_t *buff);

