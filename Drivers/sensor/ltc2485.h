#ifndef __LTC2485_H__
#define __LTC2485_H__

#include <stdint.h>
#include <stdbool.h>
#include <tremo_delay.h>

#include "bsp.h"

// LTC2485 I2C address (CA1 = LOW, CA0 = HIGH)

#define LTC2485_I2C_ADDR  0x28

#ifdef __cplusplus
extern "C" {
#endif

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

bool ltc2485_read_raw(uint32_t *raw_code, uint32_t timeout_ms);

uint32_t ltc2485_measure_once(uint32_t timeout_ms, bool* success);
bool ltc2485_read_temperature_raw(int32_t *adc_code, uint32_t timeout_ms);
float ltc2485_temperature(float vref);
bool ltc2485_sleep(void);

#ifdef __cplusplus
}
#endif

#endif // __LTC2485_H__
