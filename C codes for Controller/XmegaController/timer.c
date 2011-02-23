/*
	Timer Routines
	By Robert Bailey
	
	Revision History:
	09.29.02	RB	Created
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"


volatile uint32_t ticks=0;


ISR(TCF0_OVF_vect)			/*signal Handler for timer ovf 0 */
{
	ticks++;
}


void timer_init(void)
{	
	ticks=0;
  TCF0.CTRLA = 0x04;      // Prescaler: clk/8
  TCF0.PER   = 3999;     // 1000Hz
  TCF0.INTCTRLA = 0x02;   // Timer overflow is a medium level interrupt
}

void Wait(uint16_t delay)
{
	uint32_t temp=ticks;

	while(ticks - temp < delay);
}


void timer_fine_tic(void)
{
// set the 16 bit timer to zero, also set the prescaler to 64
// with prescaler = 64, counts happen every 32E6/64, so 1 counts are 2 us
// full count is (2^16)*(64)/(32E6) = 128 ms

TCF1.CTRLA = 0x05;     /*Prescaler = 64*/
TCF1.CNT = 0; //reset TCF1 
}

uint16_t timer_fine_toc(void)
{
// read and return the 16 bit timer
	uint16_t del_t;
	del_t = TCF1.CNT * 2;
	return del_t;
}	


void timer_coarse_tic(void)
{
// set the 16 bit timer to zero, also set the prescaler to 8
// with prescaler = 8, and per = 4000, overflow happens every 1ms

	
	ticks=0;

  TCF0.CTRLA = 0x04;      // Prescaler: 8
  TCF0.PER   = 3999;     // 1000Hz
  TCF0.INTCTRLA = 0x02;   // Timer overflow is a medium level interrupt
  TCF0.CNT = 0;            //reset TCF0
}

uint32_t timer_coarse_toc(void)
{
// read and return the ticks counter
// to convert this value to ms - divide by 
	unsigned long del_t;	
	del_t = (uint32_t)ticks;
	return del_t;
}	

