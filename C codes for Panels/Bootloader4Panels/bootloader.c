/*
Panel boot-loader via twi
This code is located in the bootloader section of the flash memory (0x3800 - 0x3fff).

Fuse bits:
The bootsector size must be set to 1024 words and the reset function must be set to bootloader.

Commands:
E:
Exit boot loader (jump to the start of application data space)
(twi 1 byte write transaction)

e:
erase the application section of the flash memory (0x0000 - 0x37ff).
(twi 1 byte write transaction)

B <addr-high> <addr-low> <size> <memtype> data, data, ...:
Block-write to either flash memory (memtype == 'F') or eeprom memory (memtype == 'E')
If memtype is flash then size is in words, else size is in bytes
Addr is a byte address for both flash and eeprom
The data must be for one page address, i.e. (addr & 0x007f) + size(in bytes) must not be more than 128
The largest data block is a full page (addr & 0x007f == 0, size(in bytes) == 128)
(twi <size>+5 bytes write transaction)

g <addr-high> <addr-low> <size> <memtype>:
Block-read from either flash memory (memtype == 'F') or eeprom memory (memtype == 'E')
If memtype is flash then size is in words, else size is in bytes
Addr is a byte address for both flash and eeprom
The data must be for one page address, i.e. (addr & 0x007f) + size(in bytes) must not be more than 128
The largest data block is a full page (addr & 0x007f == 0, size(in bytes) == 128)
(twi 5 byte write transaction, followed by a twi <size> bytes read transaction)

*/

#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define PANEL_BL_ADDR 0x70
#define APP_END 0x3800
#define PAGE_SIZE 128
#define HEADER_SIZE 5
#define TWI_BUFFER_LENGTH HEADER_SIZE+PAGE_SIZE


uint8_t* twi_txBuffer;
uint8_t twi_txBufferIndex;
uint8_t twi_txBufferLength;

uint8_t* twi_rxBuffer;
uint8_t twi_rxBufferIndex;

uint8_t cmd, size, memtype;
uint16_t address;


void (*jump_to_application) (void) = (void *)0x0000;

/*
 * Function twi_SlaveReceive
 * Desc     Process a twi slave receive buffer
 * Input    pointer to data buffer, length of the data in the buffer
 */
void twi_SlaveReceive(uint8_t *rxBuff, int len) {
  uint16_t data, i;
  // the first byte in the receive buffer is the command
  cmd = rxBuff[0];
  if (cmd == 'E') {
    // EXIT command
    jump_to_application();
  } else if (cmd == 'e') {
    // erase command
    for(i = 0; i < APP_END; i += PAGE_SIZE) {
      boot_page_erase(i);
      boot_spm_busy_wait();
    }
  } else if (cmd == 'g') {
    // block-read command (g addr-high addr-low size memtype)
    // data is fetched by a SlaveTransmit transaction
    address = 0;
    size = 0x80;
    memtype = 'F';
    if(len >= HEADER_SIZE) {
      address = (rxBuff[1] << 8) | rxBuff[2];
      size = rxBuff[3];
      memtype = rxBuff[4];
    }
  } else if (cmd == 'B') {
    // block-write command (B addr-high addr-low size memtype data...)
    address = (rxBuff[1] << 8) | rxBuff[2];
    size = rxBuff[3];
    memtype = rxBuff[4];
    if (memtype == 'E') {
      // EEPROM
      if (size > len - HEADER_SIZE)
        size = len;
      for( i = HEADER_SIZE; i < size + HEADER_SIZE; i++) {
        eeprom_write_byte((uint8_t *) address, rxBuff[i]);
        eeprom_busy_wait();
        address++;
      }
    } else if (memtype == 'F') {
      // FLASH 
      // note that size is in words, not bytes
      if (len < (size<<1) + HEADER_SIZE) {
        // too short length
      } else if ((((uint16_t)size<<1) + (address & 0x007f)) > PAGE_SIZE) {
        // incorrect size
      } else if (address >= APP_END) {
        // don't overwrite the bootloader
      } else {
        // looks OK, move the data from the receive buffer to the boot_page memory
        for (i = 0; i < size<<1; i += 2) {
          data = rxBuff[HEADER_SIZE+i] | (rxBuff[HEADER_SIZE+1+i] << 8);
          boot_page_fill(address+i, data);
        }
        // erase the FLASH page
        boot_page_erase(address);
        boot_spm_busy_wait();
        // program the FLASH page 
        boot_page_write(address); /* Store page buffer in flash memory */
        boot_spm_busy_wait();
        boot_rww_enable();
      }
    }
  }
}


/*
 * Function twi_SlaveTransmit
 * Desc     Fill a twi slave transmit buffer
 */
void twi_SlaveTransmit(void) {
  uint8_t i;

  if (cmd == 'g') {
    // return the data for a block-read command
    if (memtype == 'E') {
      // read eeprom data
      twi_txBufferLength = size;
      for (i = 0; i < size; i++) {
        EEARL = address;
        EEARH = (address) >> 8;
        address++;
        EECR |= (1<<EERE);
        twi_txBuffer[i] = EEDR;
      }
    } else if (memtype == 'F') {
      // read flash data
      // note that size is in words, not bytes
      twi_txBufferLength = size<<1;
      for (i = 0; i < size<<1; i++) {
        twi_txBuffer[i] = pgm_read_byte_near(address++);
      }
    }
  } else {
    // unknown command, just fill the return buffer with some junk
    for (i = 0; i < TWI_BUFFER_LENGTH; i++) {
      twi_txBuffer[i] = i;
    }
    twi_txBufferLength = TWI_BUFFER_LENGTH;
  }
}


/* 
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 */
void twi_reply(uint8_t ack) {
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWINT);
  }
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 */
void twi_stop(void) {
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 */
void twi_releaseBus(void) {
  // release bus
  TWCR = _BV(TWEN) | _BV(TWEA) | _BV(TWINT);
}

int main(void) {
  uint32_t count;

  //take care of Watchdog timer
  MCUSR &= ~(1<<WDRF); //clear WDRF in MCUSR
  WDTCSR |= (1<<WDCE) | (1<<WDE);  //send logical one to WDCE and WDE
  //turn off WDT
  WDTCSR = 0x00;	

  // twi initialization
  TWBR = ((F_CPU / 1000/ 400) - 16) / 2;
  TWAR = (PANEL_BL_ADDR << 1);

  TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);

  // allocate buffers
  twi_txBuffer = (uint8_t*) calloc(TWI_BUFFER_LENGTH, sizeof(uint8_t));
  twi_rxBuffer = (uint8_t*) calloc(TWI_BUFFER_LENGTH, sizeof(uint8_t));

  // short timeout for programming mode detection
  count = 2000000;
  while (!(TWCR & _BV(TWINT))) {
    if (!count--)
    // no twi transaction within the time-out time, jump to start of the application
    jump_to_application();
  }
  for (;;) {
    // twi slave state handling
    // polling version to avoid moving the interrupt table to bootloader space
    while (!(TWCR & _BV(TWINT)));
    switch(TW_STATUS){
      // Slave Receiver
      case TW_SR_SLA_ACK:   // addressed, returned ack
      case TW_SR_GCALL_ACK: // addressed generally, returned ack
      case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
      case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
        // indicate that rx buffer can be overwritten and ack
        twi_rxBufferIndex = 0;
        twi_reply(1);
        break;
      case TW_SR_DATA_ACK:       // data received, returned ack
      case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
        // if there is still room in the rx buffer
        if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
          // put byte in buffer and ack
          twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
          twi_reply(1);
        }else{
          // otherwise nack
          twi_reply(0);
        }
        break;
      case TW_SR_STOP: // stop or repeated start condition received
        // ack future responses right away so not to miss next frame
        twi_reply(1);
        // process the received data packet
        twi_SlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
        break;
      case TW_SR_DATA_NACK:       // data received, returned nack
      case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
        // nack back at master
        twi_reply(0);
        break;
      
      // Slave Transmitter
      case TW_ST_SLA_ACK:          // addressed, returned ack
      case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
        // ready the tx buffer index for iteration
        twi_txBufferIndex = 0;
        // set tx buffer length to be zero, to verify if user changes it
        twi_txBufferLength = 0;
        // request for txBuffer to be filled and length to be set
        twi_SlaveTransmit();
        // if the function didn't change buffer & length, initialize it
        if(0 == twi_txBufferLength){
          twi_txBufferLength = 1;
          twi_txBuffer[0] = 0x00;
        }
        // transmit first byte from buffer, fall
      case TW_ST_DATA_ACK: // byte sent, ack returned
        // copy data to output register
        TWDR = twi_txBuffer[twi_txBufferIndex++];
        // if there is more to send, ack, otherwise nack
        if(twi_txBufferIndex < twi_txBufferLength){
          twi_reply(1);
        }else{
          twi_reply(0);
        }
        break;
      case TW_ST_DATA_NACK: // received nack, we are done 
      case TW_ST_LAST_DATA: // received ack, but we are done already!
        // ack future responses
        twi_reply(1);
        break;

      // All
      case TW_NO_INFO:   // no state information
        break;
      case TW_BUS_ERROR: // bus error, illegal stop/start
        twi_stop();
        break;
      default:
        twi_releaseBus();
    }
  }
}


