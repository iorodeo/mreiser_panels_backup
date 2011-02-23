#ifndef TWI_MASTER_DRIVER_H
#define TWI_MASTER_DRIVER_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

/* Baud register setting calculation. Formula described in datasheet. */
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)


/* Transaction status defines. */
#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1


/* Transaction result enumeration. */
typedef enum TWIM_RESULT_enum {
  TWIM_RESULT_UNKNOWN          = (0x00<<0),
  TWIM_RESULT_OK               = (0x01<<0),
  TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
  TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
  TWIM_RESULT_BUS_ERROR        = (0x04<<0),
  TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
  TWIM_RESULT_FAIL             = (0x06<<0),
} TWIM_RESULT_t;

/* Buffer size defines */
//#define TWIM_WRITE_BUFFER_SIZE         136
//#define TWIM_READ_BUFFER_SIZE          136


/* TWI master driver struct
 *
 *  TWI master struct. Holds pointer to TWI module,
 *  buffers and necessary varibles.
 */
typedef struct TWI_Master {
  TWI_t *interface;                               /* Pointer to what interface to use */
  register8_t address;                            /* Slave address */
  //register8_t writeData[TWIM_WRITE_BUFFER_SIZE];  /* Data to write */
  //register8_t readData[TWIM_READ_BUFFER_SIZE];    /* Read data */
  uint8_t *PwriteData;  /* Data to write */
  uint8_t *PreadData;    /* Read data */
  uint8_t writeBuffSize; 
  uint8_t readBuffSize; 
  register8_t bytesToWrite;                       /* Number of bytes to write */
  register8_t bytesToRead;                        /* Number of bytes to read */
  register8_t bytesWritten;                       /* Number of bytes written */
  register8_t bytesRead;                          /* Number of bytes read */
  register8_t status;                             /* Status of transaction */
  register8_t result;                             /* Result of transaction */
}TWI_Master_t;


void TWI_MasterCreateBuff(TWI_Master_t *twi, uint8_t mode);
void TWI_MasterReleaseBuff(TWI_Master_t *twi);


void TWI_MasterInit(TWI_Master_t *twi,
                    TWI_t *module,
                    TWI_MASTER_INTLVL_t intLevel,
                    uint8_t baudRateRegisterSetting);
TWI_MASTER_BUSSTATE_t TWI_MasterState(TWI_Master_t *twi);
bool TWI_MasterReady(TWI_Master_t *twi);
bool TWI_MasterWrite(TWI_Master_t *twi,
                     uint8_t address,
                     uint8_t * writeData,
                     uint8_t bytesToWrite);
bool TWI_MasterRead(TWI_Master_t *twi,
                    uint8_t address,
                    uint8_t bytesToRead);
bool TWI_MasterWriteRead(TWI_Master_t *twi,
                         uint8_t address,
                         uint8_t *writeData,
                         uint8_t bytesToWrite,
                         uint8_t bytesToRead);
void TWI_MasterInterruptHandler(TWI_Master_t *twi);
void TWI_MasterArbitrationLostBusErrorHandler(TWI_Master_t *twi);
void TWI_MasterWriteHandler(TWI_Master_t *twi);
void TWI_MasterReadHandler(TWI_Master_t *twi);

#endif /* TWI_MASTER_DRIVER_H */