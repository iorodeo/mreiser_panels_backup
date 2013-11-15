
// Description: This file contains the interrupt handler for the system.  
// The registered interrupts are called according to their time interval.

#include "handler.h"
#include <avr/interrupt.h>

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif


// Handler Routine Variables
volatile uint32_t 	g_count[HANDLER_MAX];       // functions counts
uint32_t 			g_count_max[HANDLER_MAX];          // functions start counts
void 				(*g_p_handler_func[HANDLER_MAX])(void);  // handler function pointers
uint8_t 			g_enabled[HANDLER_MAX];                  // interrupt enabled


// Handler Routines:


// Handler_Init: Initializes routines and timers for the interrupt handler.

void Handler_Init(void)
{
	uint8_t lcv;

	// Initialize enableds to FALSE
	for(lcv=0;lcv<HANDLER_MAX;lcv++)
		g_enabled[lcv] = FALSE;

	// Timer TCE0: set-up to match old system (32MHz/8/512 = 16MHz/8/256):
	//  TCE0.CTRLA = 0x04;      // Prescaler: clk/8
	//JL03092010 change the prescaler from 8 to 2 so the overflowrate inclrease 4 times
	TCE0.CTRLA = 0x02;      // Prescaler: clk/2
	TCE0.PER   = 511;       // 256*2 - 1
	TCE0.INTCTRLA = 0x03;   // Timer overflow is a the highest level interrupt
}


// ISR()
// The interrupt handler function of the timer E0 interrupt.
// Calls the functions in g_p_handler_func[] if they are enabled.
//
ISR(TCE0_OVF_vect)
{
	uint8_t lcv;

	//ledToggle(1);
	//xputs(PSTR("\nISR Works\n"));

	for (lcv=0; lcv<HANDLER_MAX; lcv++)		/* check and act on all vectors */
	{
		if (g_enabled[lcv])			/* if enabled, check count */
		{
			g_count[lcv]--;
			if (g_count[lcv]==0)		/* if g_count=0, perform function call and reset */
			{
				g_count[lcv] = g_count_max[lcv];
				(*g_p_handler_func[lcv])();
			}
		}
	}
}


//Reg_Handler: Registers a timed interrupt request with the interrupt handler.

//Arguments: 
//  fptr: function pointer to the handler function
//  period: start count of the timer
//  index: index of the interrupt request
//  enabled: TRUE if the function is enabled.
            
void Reg_Handler(void* fptr, uint32_t period, uint8_t index, uint8_t enabled)
{	
	g_enabled[index] = FALSE;         // disable while modifying vector
	g_p_handler_func[index] = fptr;   // set function pointer
	g_count_max[index] = period;	  // set start count
	g_count[index] = period;          // set current count
	g_enabled[index] = enabled;
}

void Update_Reg_Handler(void* fptr, uint32_t period, uint8_t index, uint8_t enabled)
{	
	g_enabled[index] = FALSE;         // disable while modifying vector
	g_p_handler_func[index] = fptr;   // set function pointer
	g_count_max[index] = period;      // set start count

	//if current count is greater than the desired count, then reset current count
	if (g_count[index] > g_count_max[index])
		g_count[index] = g_count_max[index];      // set count

	g_enabled[index] = enabled;
}

