#include "ltc2485.h"
#include "I2C_A.h"
#include "lora_config.h"

// --- Internal helpers ---

static bool ltc2485_read_bytes(uint8_t *buf, uint8_t len) {
    return I2C_Read_Len(LTC2485_I2C_ADDR, 0x00, len, buf) == 0;
}

static bool ltc2485_write_byte(uint8_t byte) {
    return I2C_Write_Byte(LTC2485_I2C_ADDR, byte) == 0;
}

static bool data_ready(uint8_t status_byte) {
    return (status_byte & 0x80) != 0;
}

/**
 * @brief Controls the Vx power switch (active-low).
 *
 * This function drives the gate of a P-channel MOSFET (via LTC2485_VX_PIN)
 * to control power to the LTC2485. The MOSFET is active-low, so:
 * - A LOW output (0) enables power (turns the MOSFET on)
 * - A HIGH output (1) disables power (turns the MOSFET off)
 *
 * @param enable Set to true to enable power (drive pin low), false to disable (drive pin high)
 */
static void ltc2485_set_vx(bool enable) {
    gpio_write(LTC2485_VX_PORT, LTC2485_VX_PIN, enable ? 0 : 1);
}

// --- Public API ---

bool ltc2485_init(void) {
    ltc2485_control_gpio_init();
    ltc2485_set_vx(false);  // Initially off, controlled by application
    return true;
}

uint8_t check_ltc2485_connect(void) {
    uint8_t rxdata[4] = {0};

    I2C_GPIO_MODE_Config();  // Set up I2C mode for GPIO

    if (ltc2485_read_bytes(rxdata, 4) == false)
        return 0;

    return data_ready(rxdata[0]) ? 1 : 0;
}


/**
 * @brief Reads the external analog input from the LTC2485 as a 24-bit unsigned value.
 *
 * This function performs a conversion readout from the LTC2485 and extracts the
 * 25-bit signed ADC result. To simplify transmission and reduce payload size,
 * it discards any negative values (sign bit = 1) and returns only the top 24 bits
 * of the positive conversion result.
 *
 * The result is right-aligned by removing the 7 sub-LSB bits and sign bit, leaving
 * a clean 24-bit unsigned integer suitable for compact encoding in LoRa payloads.
 *
 * @param[out] raw_code    Pointer to store the 24-bit unsigned result (stored in a uint32_t)
 * @param[in]  timeout_ms  Timeout in milliseconds for conversion completion
 * @return true if successful and result is non-negative, false on I2C failure, timeout,
 *         or if the ADC result was negative.
 */
 
bool ltc2485_read_raw(uint32_t *raw_code, uint32_t timeout_ms) {
    uint8_t buffer[4];
    uint32_t elapsed = 0;

    do {
        if (!ltc2485_read_bytes(buffer, 4))
            return false;
        if (data_ready(buffer[0]))
            break;
		delay_ms(10);
        elapsed += 10;
    } while (elapsed < timeout_ms);

    if (!data_ready(buffer[0]))
        return false;



    uint32_t raw = ((uint32_t)buffer[0] << 24) |
                   ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) |
                   ((uint32_t)buffer[3]);

    if (raw & 0x80000000)  // Sign bit set
        return false;      // Negative value, discard

    // Shift off sign bit and sub-LSBs ? 24-bit unsigned positive result
    *raw_code = (raw >> 7) & 0xFFFFFF;


    return ltc2485_sleep();
}

bool ltc2485_sleep(void) {
    return ltc2485_write_byte(0x00);
}



/**
 * @brief Perform a single LTC2485 ADC measurement (positive-only, 24-bit).
 *
 * This function powers up the LTC2485 using the Vx control pin, waits for
 * analog settling, performs a measurement with timeout, powers down Vx again,
 * and returns the raw ADC result.
 *
 * Only positive 24-bit ADC values are supported. Negative readings are discarded.
 *
 * @param[in]  timeout_ms   Timeout for ADC data-ready polling
 * @param[out] success      Optional output flag to indicate success (true) or failure (false)
 * @return 24-bit unsigned ADC result (0 if failed or negative)
 */
uint32_t ltc2485_measure_once(uint32_t timeout_ms, bool* success)
{
    uint32_t raw_code = 0;

    ltc2485_set_vx(true);        // Power on
    delay_ms(40);                // Allow analog input and reference to settle

    bool ok = ltc2485_read_raw(&raw_code, timeout_ms);

    ltc2485_set_vx(false);       // Always power off after attempt

    if (!ok) {
        if (success) *success = false;
        return 0;                // Return 0 if read failed
    }

    if (success) *success = true;
    return raw_code;
}


/**
 * @brief Reads the internal temperature sensor value from the LTC2485.
 *
 * This function initiates a temperature conversion and reads the resulting
 * 32-bit ADC code from the LTC2485. The 25-bit signed ADC result is extracted
 * and returned in full 32-bit form, sign-extended. This value can be used
 * to compute the actual temperature in degrees Celsius using the LTC2485's
 * PTAT transfer function.
 *
 * @param[out] adc_code    Pointer to signed 32-bit storage for full ADC result
 * @param[in]  timeout_ms  Timeout in milliseconds for conversion completion
 * @return true if successful, false if I2C error or data not ready
 */
bool ltc2485_read_temperature_raw(int32_t *adc_code, uint32_t timeout_ms)
{
    if (!ltc2485_write_byte(0xC0))
        return false;

    delay_ms(150);  // Wait for internal conversion

    uint8_t buffer[4];
    if (!ltc2485_read_bytes(buffer, 4))
        return false;

    if (!data_ready(buffer[0]))
        return false;

    uint32_t raw = ((uint32_t)buffer[0] << 24) |
                   ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) |
                   ((uint32_t)buffer[3]);

    // Sign-extend 25-bit ADC result: left-align then arithmetic right shift
    *adc_code = (int32_t)(raw << 1) >> 7;

    return ltc2485_sleep();
}

/**
 * @brief Measures and converts the LTC2485 internal temperature to °C.
 *
 * This function initiates a temperature conversion, reads the signed
 * ADC code, converts it into voltage based on the given VREF, and
 * applies the typical PTAT transfer function:
 *
 *   Vtemp = 420 mV at 27 °C
 *   Slope  ˜ 1.4 mV/°C
 *
 *   Temp(°C) = 27 + (Vadc - 0.420) / 0.0014
 *
 * @param[in] vref Reference voltage used by LTC2485 (e.g., 2.5V)
 * @return Temperature in °C if successful, or -99 on failure
 */
float ltc2485_temperature(float vref)
{
    int32_t adc_code;
    if (!ltc2485_read_temperature_raw(&adc_code, 200))
        return -99;  // Measurement failed

    float voltage = ((float)adc_code * vref) / 16777216.0f; // 2^24
    return 27.0f + ((voltage - 0.420f) / 0.0014f);
}

//see definitions in: lora_config.h
// Terminal   PIN   Function
// Pin 1:     VCC
// Pin 4:     SLC
// Pin 5:     SDA
// Pin 6:     PC13  ENABE VX Active low
// Pin 7:     PB9   NOT USED  (monitor VX)// Set High Impedance set Input, no Pullup
// Pin 8:     PB8   EXT ADC // Set High Impedance no Pullup

void ltc2485_control_gpio_init(void) {
    LTC2485_VX_CLK_ENABLE();
    LTC2485_BLOCK_CLK_ENABLE();

    gpio_set_iomux(LTC2485_VX_PORT, LTC2485_VX_PIN, 0);
    gpio_init(LTC2485_VX_PORT, LTC2485_VX_PIN, GPIO_MODE_OUTPUT_PP_HIGH); //EN is active low

    gpio_set_iomux(LTC2485_BLOCK_PORT, LTC2485_BLOCK_PIN1, 0);
    gpio_init(LTC2485_BLOCK_PORT, LTC2485_BLOCK_PIN1, GPIO_MODE_INPUT_FLOATING);

    gpio_set_iomux(LTC2485_BLOCK_PORT, LTC2485_BLOCK_PIN2, 0);
    gpio_init(LTC2485_BLOCK_PORT, LTC2485_BLOCK_PIN2, GPIO_MODE_INPUT_FLOATING);
}
