/*
	Interrupt Handler Routines Header File
	By Robert Bailey
	
	Revision History:
	11.15.02	RB	Created
*/

/*
Description: Header file for the interrupt handler routines. Defines constants and contains prototypes for the Interrupt Handler routines.
*/

#include "legacy.h"

/* Interrupt Handler Definitions */
#define TCCR0_VAL 0x01				/* prescale timer by 8 */

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/* Interrupt Handler Routine Prototypes */
void Handler_Init(void);
void Reg_Handler(void* fptr,unsigned long s_cnt,unsigned char msk);

