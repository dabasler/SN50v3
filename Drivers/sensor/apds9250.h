#ifndef APDS9250_H
#define APDS9250_H

#include <stdint.h>
#include <stdbool.h>

#define APDS9250_I2C_ADDR  0x52  // 7-bit address

typedef struct {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t ir;
} apds9250_t;

bool apds9250_init(void);
uint8_t check_apds9250_connect(void);
bool apds9250_read_raw(uint32_t *r, uint32_t *g, uint32_t *b, uint32_t *ir, uint32_t timeout_ms);
uint64_t apds9250_measure(uint32_t timeout_ms);
bool apds9250_sleep(void);
void apds9250_unpack(uint64_t packed, apds9250_t* data, uint8_t* gain_out);

#endif // APDS9250_H
