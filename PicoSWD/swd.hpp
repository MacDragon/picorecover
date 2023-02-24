int32_t probe_send_instruction( uint8_t instruction, uint8_t instno );
int32_t probe_wait_reply( uint32_t timeout, uint8_t instno );
bool probe_read_memory( uint32_t addr, uint8_t *data, uint32_t count);
bool probe_write_memory( uint32_t addr, uint8_t *data, uint32_t count );
bool probe_write_memory_slow( uint32_t addr, uint8_t *data, uint32_t count );
bool probe_rescue_reset( void );
bool probe_sendhelper( void );
void probe_init( void );
