// legacy.h
//try these to help compiling errors - there support sbi, cbi legacy
//#define sbi(p,b) (p) |= (1<<(b))
//#define cbi(p,b) (p) &= ~(1<<(b))

#define  cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit)) //clear bit
#define  sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))  //set bit
#define  _BV(bit)   (1 << (bit))
#define  inb(sfr)   _SFR_BYTE(sfr)
#define  inw(sfr)   _SFR_WORD(sfr)
#define  outb(sfr, val)   (_SFR_BYTE(sfr) = (val))
#define  outw(sfr, val)   (_SFR_WORD(sfr) = (val))
#define  outp(val, sfr)   outb(sfr, val)
#define  inp(sfr)   inb(sfr)
#define  BV(bit)   _BV(bit) 

#define  eeprom_rb(addr)   eeprom_read_byte ((uint8_t *)(addr))


// PRG_RDB is no longer defined in avr-libc
#ifndef PRG_RDB
#define 	PRG_RDB(addr)   pgm_read_byte(addr)
#endif