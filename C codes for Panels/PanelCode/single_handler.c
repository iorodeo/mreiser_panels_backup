/*
	Interrupt Handler Routines
	By Robert Bailey
	
	Revision History:
	11.15.02	RB	Created
	
	03.05.04  	Modified by MBR to function as only the handler for a single interrupt
			to cut down on overhead.
*/

/*
Description: This file contains the interrupt handler for the system.  The registered interrupts are called according to their time interval.
The units of time are the overflow of the 8 bit counter.
*/

#include "single_handler.h"
//#include <avr/signal.h> - obsolete
#include <avr/interrupt.h>


/* Handler Routine Variables */
volatile unsigned long count;		/* functions counts */
unsigned long start_count;			/* functions start counts */
void (*p_handler_func)(void);		/* handler function pointers */
unsigned char mask;			/* interrupt mask */

/* Handler Routines */

/*
Function Name: Handler_Init

Description: Initializes routines and timer for the interrupt handler.

Arguments: none

Return Values: none

Limitations: 
Notes:
*/
void Handler_Init(void)
{
	cli();
	
	mask = FALSE;	/* Initialize mask to FALSE */
	        
	outp(TCCR0_VAL,TCCR0B);				/* write timer prescaler */
	sbi(TIMSK0,TOIE0);				/* enable timer ovf irq */
	sei();						/* enable interrupts */
}

/*
Function Name: SIG_OVERFLOW0

Description: The interrupt handler function of the timer0 interrupt.

Arguments: none

Return Values: none

Limitations: 
Notes:
*/
SIGNAL(SIG_OVERFLOW0)					/* timer 0 ovf int handler */
{
	cbi(TIMSK0,TOIE0);				/* disable timer ovf irq */
	
	if(mask==TRUE)			/* if int enabled check count */
		count--;			
	if(count==0)		/* if count=0, perform function call and reset */
	{
		count=start_count;
		(*p_handler_func) ();
	}

	sbi(TIMSK0,TOIE0);				/* enable timer ovf irq */
}

/*
Function Name: Reg_Handler

Description: Registers a timed interrupt request with the interrupt handler.

Arguments: 
            void* fptr = function pointer to the handler function
            long s_cnt = start count of the timer
            unsigned char priority = priority of the interrupt request
            unsigned char msk = the mask of the interrupt. TRUE/FALSE value
            
Return Values: none

Limitations: 
Notes:
*/
void Reg_Handler(void* fptr,unsigned long s_cnt,unsigned char msk)
{	
	mask = FALSE;				/* disable while modifying vector */
	p_handler_func =fptr;			/* set function pointer */
	start_count =s_cnt;			/* set start count */
	count=s_cnt;				/* set count */
	mask =msk;				/* set interrupt mask */
}

