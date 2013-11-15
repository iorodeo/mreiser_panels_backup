#include <avr/io.h>

// Interrupt Handler Definitions
#define HANDLER_MAX 6                // number of functions handler can handle


/* Interrupt Handler Routine Prototypes */
void Handler_Init(void);
void Reg_Handler(void* fptr, uint32_t period, uint8_t index, uint8_t enabled);
void Update_Reg_Handler(void* fptr, uint32_t period, uint8_t index, uint8_t enabled);
