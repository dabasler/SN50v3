#ifndef M24M02E_H
#define M24M02E_H

#include <stdint.h>
#include <stdbool.h>

#define M24M02E_I2C_ADDR_BASE  0x50  // 0b1010 000

bool m24m02e_init(void);
uint8_t check_m24m02e_connect(void);
bool m24m02e_read(uint32_t mem_addr, uint8_t* data, uint16_t len);
bool m24m02e_write(uint32_t mem_addr, const uint8_t* data, uint16_t len);
bool m24m02e_sleep(void);

#endif
