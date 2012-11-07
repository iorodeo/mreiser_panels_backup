/*
	Timer Header File
*/

//#define F_CPU 32000000

extern volatile unsigned long ticks;


void timer_init(void);
void Wait(uint16_t delay);

#define secs(A) ((uint16_t) (A * F_CPU / 4096))
#define msecs(A) ((uint16_t) (A * (F_CPU / 1000) / 4096))


void timer_fine_tic(void);
uint16_t timer_fine_toc(void);

void timer_coarse_tic(void);
uint32_t timer_coarse_toc(void);
