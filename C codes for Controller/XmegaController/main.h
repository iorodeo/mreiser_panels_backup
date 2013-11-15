//in one bytes because function_X and function_Y are int8_t
#define NBYTES_HEADER	  512
#define RINGBUFFER_LENGTH 100

//In the old system OVERFLOW_PERIOD = 16MHz/8(prescaler)/256(timer0)
//In the new system OVERFLOW_PERIOD = 32MHz/8(prescaler)/512(timerE)
//JL03092010 increase OVERFLOW_PERIOD 4 times in order to get a higher resolution clock for the handlers
//#define OVERFLOW_PERIOD 2000000/256
#define OVERFLOW_PERIOD   8000000/256
#define UPDATE_PERIOD     OVERFLOW_PERIOD/400
#define FUNCTION_PERIOD   OVERFLOW_PERIOD/50


#define BAUDRATE	      400000
#define TWI_BAUDSETTING   TWI_BAUD(F_CPU, BAUDRATE)
#define TWI_BUFFER_LENGTH HEADER_SIZE+PAGE_SIZE

#define EEPROM __attribute__((section(".eeprom")))
void init_all(void);

//routines for handling the incomming messages over the UART
void handle_message_length_1(uint8_t *msg_buffer);
void handle_message_length_2(uint8_t *msg_buffer);
void handle_message_length_3(uint8_t *msg_buffer);
void handle_message_length_4(uint8_t *msg_buffer);
void handle_message_length_5(uint8_t *msg_buffer);
void handle_message_length_9(uint8_t *msg_buffer);
void handle_message_length_62(uint8_t *msg_buffer);
void handle_message_length_63(uint8_t *msg_buffer);
void handle_message_length_52(uint8_t *msg_buffer);


//routines for updating display, computing new pattern indices
void update_display(void);
void increment_index_x(void);
void increment_index_y(void);
void decrement_index_x(void);
void decrement_index_y(void);
void fetch_and_display_frame(FIL *pFile, uint16_t f_num, uint16_t, uint16_t);
void display_preload_frame(uint16_t f_num, uint16_t, uint16_t);
void update_ANOUT(void);
void update_funcCnt_x(void);
void update_funcCnt_y(void);
void update_funcCnt_xy(void);

//helper utilities
void toggle_trigger(void);
void delay(uint16_t us);
void long_delay(uint16_t ms);
void send_number(uint8_t target_panel_addr, int32_t number);
void set_pattern(uint8_t pat_num);
void benchmark_pattern(void);
void set_hwConfig(uint8_t config_num);
void i2cMasterSend(uint8_t addr, uint8_t len, uint8_t *data);
void set_vel_func(uint8_t func_channel, uint8_t func_id);
void set_pos_func(uint8_t func_channel, uint8_t func_id);
void set_default_func(uint8_t func_channel);

void loadPattern2Panels(uint8_t pat_num);
void display_dumped_frame (uint8_t *msg_buffer);

void dump_mat(void);
void fetch_block_func_x(FIL *pFile, uint8_t fReset, uint8_t);
void fetch_block_func_y(FIL *pFile, uint8_t fReset, uint8_t);
uint8_t difference(uint8_t write_index, uint8_t read_index);

unsigned char work_mode[1] EEPROM = {0xff};
unsigned char arena_config[129] EEPROM;
static uint8_t RESET[2] = {0x00, 0x01};
static uint8_t DISPLAY[2] = {0x00, 0x02};

static uint8_t TEST_BYTE[8][1] = {0x0F, 0xF0, 0xAA, 0x55, 0x03, 0x0C, 0x30, 0xC0};
static uint8_t TEST_3BYTE[9][3] = {{0x00, 0x00, 0x00}, {0x00, 0x00, 0xFF}, {0x00, 0xFF, 0x00}, {0x00, 0xFF, 0xFF},
{0xFF, 0x00, 0x00}, {0xFF, 0x00, 0xFF}, {0xFF, 0xFF, 0x00}, {0xFF, 0xFF, 0xFF} , {0x0F, 0x33, 0x55} };
static uint8_t G_LEVELS_16[16][32] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} };

static uint8_t ALL_OFF[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t ALL_ON[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t G_LEVELS[8][24] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} };

static uint8_t ERROR_CODES[8][8] = { {0x80, 0x3F, 0x21, 0x21, 0x00, 0x3F, 0x25, 0x21},
{0x00, 0xBF, 0x21, 0x21, 0x00, 0x3F, 0x25, 0x21},
{0x00, 0x3F, 0xA1, 0x21, 0x00, 0x3F, 0x25, 0x21},
{0x00, 0x3F, 0x21, 0xA1, 0x00, 0x3F, 0x25, 0x21},
{0x00, 0x3F, 0x21, 0x21, 0x80, 0x3F, 0x25, 0x21},
{0x00, 0x3F, 0x21, 0x21, 0x00, 0xBF, 0x25, 0x21},
{0x00, 0x3F, 0x21, 0x21, 0x00, 0x3F, 0xA5, 0x21},
{0x00, 0x3F, 0x21, 0x21, 0x00, 0x3F, 0x25, 0xA1} };
