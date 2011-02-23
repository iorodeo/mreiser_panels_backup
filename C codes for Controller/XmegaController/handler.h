#include <avr/io.h>

// Interrupt Handler Definitions
#define HANDLER_MAX 6                // number of functions handler can handle


/* Interrupt Handler Routine Prototypes */
void Handler_Init(void);
void Reg_Handler(void* fptr, uint32_t s_cnt, uint8_t priority, uint8_t msk);
void Update_Reg_Handler(void* fptr, uint32_t s_cnt, uint8_t priority, uint8_t msk);
