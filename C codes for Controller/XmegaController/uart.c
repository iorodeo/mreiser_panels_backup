
/* UART functions */

#include "uart.h"
#include "xitoa.h"


typedef struct _txfifo {
  uint8_t	idx_w;
  uint8_t	idx_r;
  uint8_t	count;
  uint8_t buff[64];
} txFIFO;


typedef struct _rxfifo {
  uint16_t	idx_w;
  uint16_t	idx_r;
  uint16_t	count;
  uint16_t  size;
  uint8_t * PRxBuff;
} rxFIFO;

static volatile
txFIFO txfifo;

static volatile
rxFIFO rxfifo;


void createRxBuff(uint8_t mode) {

if (mode == 0xff) {
 rxfifo.PRxBuff = malloc(64);
 rxfifo.size = 64;
}
else {
 rxfifo.PRxBuff = malloc(1550);
 rxfifo.size = 1550;
 }
}


void releaseRxBuff( ) {

free(rxfifo.PRxBuff);

}

/* Initialize UART */

void uart_init()
{

  rxfifo.idx_r = 0;
  rxfifo.idx_w = 0;
  rxfifo.count = 0;
  txfifo.idx_r = 0;
  txfifo.idx_w = 0;
  txfifo.count = 0;


//  USARTC0.BAUDCTRLA = 0x17;   // BSCALE = -6, BSEL = 1047
//  USARTC0.BAUDCTRLB = 0xA4;   // ==> 115211 bps (~115.2kbps)

//  USARTC0.BAUDCTRLA = 0xEC;   // BSCALE = -6, BSEL = 492
//  USARTC0.BAUDCTRLB = 0xA1;   // ==> 230400 bps 
  
//  USARTC0.BAUDCTRLA = 0xD6;   // BSCALE = -6, BSEL = 214
//  USARTC0.BAUDCTRLB = 0xA0;   // ==> 460800 bps
  
  USARTC0.BAUDCTRLA = 0x4B;   // BSCALE = -6, BSEL = 75
  USARTC0.BAUDCTRLB = 0xA0;   // ==> 921600 bps
  
//  USARTC0.BAUDCTRLA = 0x05;   // BSCALE = -6, BSEL = 5
//  USARTC0.BAUDCTRLB = 0xA0;   // ==> 1843200 bps
  
  USARTC0.CTRLA = 0x10;       // RX is low level interrupt
  USARTC0.CTRLC = 0x03;       // Async, No Parity, 1 stop bit, 8 data bits
  USARTC0.CTRLB = 0x18;       // Enable RX and TX
}


/* Get a received character */

uint16_t uart_test ()
{
  return rxfifo.count;
}


uint8_t uart_get ()
{
  uint16_t i;
  uint8_t d;

  i = rxfifo.idx_r;
  while(rxfifo.count == 0);
  d = *(rxfifo.PRxBuff+i++);
  cli();
  rxfifo.count--;
  sei();
  if(i >= rxfifo.size)
    i = 0;
  rxfifo.idx_r = i;

  return d;
}


/* Put a character to transmit */

void uart_put (uint8_t d)
{
  uint8_t i;

  i = txfifo.idx_w;
  while(txfifo.count >= sizeof(txfifo.buff));
  txfifo.buff[i++] = d;
  cli();
  txfifo.count++;
  USARTC0.CTRLA = USART_RXCINTLVL0_bm | USART_DREINTLVL0_bm;

  sei();
  if(i >= sizeof(txfifo.buff))
    i = 0;
  txfifo.idx_w = i;
}

// Sends a String to the serial port
void uart_putstr(uint8_t *s)
{
  uint8_t d;
  while(d = pgm_read_byte_near(s)) {
    uart_put(d);
    s++;
  }
}

// Sends a Carriage Return and Line Feed to the serial port
void uart_crlf()
{
  uart_put(CR);
  uart_put(LF);
}




//fill_Rx_buffer: fills an incomming message (Rx) buffer 
//Arguments: uint8_t uart_sel, uint8_t *Rx_buffer
//Return Values: buffer size, or 0 if there is an error.

//Limitations: need to add a timeout here

//Notes: this function expects the first byte in the buffer to be the length of the message
//only call this after a while(!(uart_test())); command

uint16_t fill_Rx_buffer(uint8_t *Rx_buffer)
{	
	uint8_t message_length, temp1, temp2;
	uint16_t data_length, i;
	
	message_length = 0;
	//err_flag = 1;
	
	if (uart_test())
	{
		message_length = uart_get();
		
		if (message_length == 50)
		{
		//dump frame: header and data. read the first two bytes data which are the frame data length

			temp1 = uart_get();
			temp2 = uart_get();

			data_length = (uint16_t)temp1 + 256*(uint16_t)temp2;	

			for (i = 0; i < data_length+7; i++)
			{
				while(!(uart_test()));
				Rx_buffer[i] = uart_get();
			}
			
								
		}	
        else	
			for (i = 0; i < message_length; i++)
			{
				while(!(uart_test()));
				Rx_buffer[i] = uart_get();
			}		
	}
	//else err_flag = 0;  - without other error check, message_length stays at zero.
	//message_length = err_flag*message_length;  - only useful for multiple error sources, but mult is overkill
	return message_length;			

}	

//send_Tx_buffer: sends an outgoing message (Tx) buffer 

//Arguments: uint8_t *Tx_buffer - the buffer to send, 
//           uint8_t message_length - the number of bytes to send
//Return Values: none
//Limitations: messages can only be upto 255 bytes in length
//Notes: this function sends the message length as the first byte of the message

void send_Tx_buffer(uint8_t *Tx_buffer, uint8_t message_length)
{		
	uint8_t i;

	uart_put(message_length);
	for (i = 0; i < message_length;i++)
	{
		uart_put(Tx_buffer[i]);
	}	
}


/* UART RXC interrupt */

ISR(USARTC0_RXC_vect)
{
  uint8_t d;
  uint16_t n, i;

  d = USARTC0.DATA;
  n = rxfifo.count;
  if(n < rxfifo.size) {
    rxfifo.count = ++n;
    i = rxfifo.idx_w;
    *(rxfifo.PRxBuff + i++) = d;
    if(i >= rxfifo.size)
      i = 0;
    rxfifo.idx_w = i;
  }
}


/* UART UDRE interrupt */

ISR(USARTC0_DRE_vect)
{
  uint8_t n, i;

  n = txfifo.count;
  if(n) {
    txfifo.count = --n;
    i = txfifo.idx_r;
    USARTC0.DATA = txfifo.buff[i++];
    if(i >= sizeof(txfifo.buff))
      i = 0;
    txfifo.idx_r = i;
  }
  if(n == 0)
    USARTC0.CTRLA = USART_RXCINTLVL0_bm;
}

