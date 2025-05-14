#ifndef __LTC2485_H__
#define __LTC2485_H__

#include <stdint.h>
#include <stdbool.h>
#include <tremo_delay.h>

#include "bsp.h"
#include "log.h"

// LTC2485 I2C address (CA1 = LOW, CA0 = HIGH)

#define LTC2485_I2C_ADDR  0x14 //Global Address 0x77
//define LTC2485_I2C_ADDR  0x14

#ifdef __cplusplus
extern "C" {
#endif

void ltc2485_set_vx(bool enable);

/**
 * @brief Initialize GPIO pins used for LTC2485 control.
 */
void ltc2485_control_gpio_init(void);

/**
 * @brief Initialize the LTC2485 driver (sets GPIOs, default state).
 * @return true if successful
 */
bool ltc2485_init(void);

uint8_t check_ltc2485_connect(void);

int32_t ltc2485_read_adc(uint32_t timeout_ms);


float ltc2485_read_temperature(uint16_t vref, uint32_t timeout_ms);


#ifdef __cplusplus
}
#endif

#endif // __LTC2485_H__
