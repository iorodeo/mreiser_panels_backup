
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#define CTRC 3
#define CR 13
#define LF 10
#define BKSPC 8

void createRxBuff(uint8_t);
void releaseRxBuff(void);
void uart_init(void);		/* Initialize UART and Flush FIFOs */
uint8_t uart_get (void);	/* Get a byte from UART Rx FIFO */
uint16_t uart_test(void);	/* Check number of data in UART Rx FIFO */
void uart_put (uint8_t);	/* Put a byte into UART Tx FIFO */
void uart_putstr(uint8_t *s);
void uart_crlf();
uint16_t fill_Rx_buffer(uint8_t *Rx_buffer);
void send_Tx_buffer(uint8_t *Tx_buffer, uint8_t message_length);
