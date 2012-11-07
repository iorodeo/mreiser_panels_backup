
// Description: This file contains the interrupt handler for the system.  
// The registered interrupts are called according to their time interval.

#include "handler.h"
#include <avr/interrupt.h>
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif


// Handler Routine Variables
volatile uint32_t count[HANDLER_MAX];       // functions counts
uint32_t start_count[HANDLER_MAX];          // functions start counts
void (*p_handler_func[HANDLER_MAX])(void);  // handler function pointers
uint8_t mask[HANDLER_MAX];                  // interrupt mask


// Handler Routines:


// Handler_Init: Initializes routines and timers for the interrupt handler.

void Handler_Init(void)
{
  uint8_t lcv;

  // Initialize masks to FALSE
  for(lcv=0;lcv<HANDLER_MAX;lcv++)
  {
    mask[lcv] = FALSE;
  }

// Timer TCE0: set-up to match old system (32MHz/8/512 = 16MHz/8/256):
//  TCE0.CTRLA = 0x04;      // Prescaler: clk/8
//JL03092010 change the prescaler from 8 to 2 so the overflowrate inclrease 4 times
  TCE0.CTRLA = 0x02;      // Prescaler: clk/2
  TCE0.PER   = 511;       // 256*2 - 1
  TCE0.INTCTRLA = 0x03;   // Timer overflow is a the highest level interrupt
}


// The interrupt handler function of the timer E0 interrupt.

ISR(TCE0_OVF_vect)
{
 uint8_t lcv;
  
//ledToggle(1);
//xputs(PSTR("\nISR Works\n"));

   for(lcv=0;lcv<HANDLER_MAX;lcv++)		/* check and act on all vectors */
   {
     if(mask[lcv]==TRUE)			/* if int enabled check count */
     {
       count[lcv]--;			
       if(count[lcv]==0)		/* if count=0, perform function call and reset */
       {
         count[lcv]=start_count[lcv];
		 //*
         (*p_handler_func[lcv]) ();
       }
     }
   }



}


//Reg_Handler: Registers a timed interrupt request with the interrupt handler.

//Arguments: 
//            void* fptr = function pointer to the handler function
//            long s_cnt = start count of the timer
//            unsigned char priority = priority of the interrupt request
//            unsigned char msk = the mask of the interrupt. TRUE/FALSE value
            
void Reg_Handler(void* fptr,uint32_t s_cnt, uint8_t priority, uint8_t msk)
{	
  mask[priority]=FALSE;             // disable while modifying vector
  p_handler_func[priority]=fptr;    // set function pointer
  start_count[priority]=s_cnt;      // set start count
  count[priority]=s_cnt;            // set count
  mask[priority]=msk;				        // set interrupt mask
}

void Update_Reg_Handler(void* fptr, uint32_t s_cnt, uint8_t priority, uint8_t msk)
{	
  mask[priority]=FALSE;             // disable while modifying vector
  p_handler_func[priority]=fptr;    // set function pointer
  start_count[priority]=s_cnt;      // set start count
	
  //if current count is greater than the desired count, then reset current count
  if (count[priority] > s_cnt){
    count[priority]=s_cnt;          // set count
  }

  mask[priority]=msk;               // set interrupt mask
}

