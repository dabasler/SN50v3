#include "ltc2485.h"
#include "I2C_A.h"
#include "lora_config.h"
#include "log.h"

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
void ltc2485_set_vx(bool enable) {
    gpio_write(DABAEXT_VX_PORT, DABAEXT_VX_PIN, enable ? 0 : 1);
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
 * a clean 24-bit integer suitable for compact encoding in LoRa payloads.
 *
 * @param[in]  timeout_ms  Timeout in milliseconds for conversion completion
 * @return the ADC result or 0xFFFFFFFF if something went wrong.
 */
 
int32_t ltc2485_read_adc(uint32_t timeout_ms) {
    uint8_t buffer[4];
    uint32_t elapsed = 0;
	uint32_t value = 0;
	
	
   if (!ltc2485_write_byte(0x00)) {
		//LOG_PRINTF(LL_DEBUG, "ltc2485: no response\r\n");
        return 0xFFFFFFFF;}
	
	delay_ms(200);  // Wait for internal conversion
	do {
        if (!ltc2485_read_bytes(buffer, 4))
            return false;
        if (data_ready(buffer[0]))
            break;
		delay_ms(10);
        elapsed += 10;
    } while (elapsed < timeout_ms);

    if (!data_ready(buffer[0])){
		//LOG_PRINTF(LL_DEBUG, "ltc2485: data not ready\r\n");
		return 0xFFFFFFFF;}
 
    uint32_t raw = ((uint32_t)buffer[0] << 24) |
                   ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) |
                   ((uint32_t)buffer[3]);
	
	LOG_PRINTF(LL_DEBUG, "ltc2485: raw 0x%08luX (hex)\r\n", raw);
	// Check over-range: SIG=1, MSB=1
	if (((raw >> 30) & 0x3) == 0x3) {
		//LOG_PRINTF(LL_DEBUG, "Overrange\r\n");
		return 0x7FFFFFFF;
	}

	// Check under-range: SIG=0, MSB=0
	if (((raw >> 30) & 0x3) == 0x0) {
		//LOG_PRINTF(LL_DEBUG, "Underrange\r\n");
		return 0x80000000;
	}
	// Flip sign bit to convert from offset binary to two's complement
    raw ^= 0x80000000;
    // Convert to signed 32-bit and right shift to discard status bits (7 bits)
	value = ((int32_t)raw) >> 7;
	//LOG_PRINTF(LL_DEBUG, "ltc2485: vaule 0x%06lX (hex)\r\n", value);
    return value;
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
float ltc2485_read_temperature(uint16_t vref, uint32_t timeout_ms)
{
    if (!ltc2485_write_byte(0x08)){
		//LOG_PRINTF(LL_DEBUG, "ltc2485: no response\r\n");
        return -99;
		}

    delay_ms(200);  // Wait for internal conversion

    uint8_t buffer[4];
    if (!ltc2485_read_bytes(buffer, 4)) {
		//LOG_PRINTF(LL_DEBUG, "ltc2485: data not ready\r\n");
        return -99;
		}

    if (!data_ready(buffer[0])) {
		//LOG_PRINTF(LL_DEBUG, "ltc2485: no valid data\r\n");
		return -99;
	}
        

    uint32_t raw = ((uint32_t)buffer[0] << 24) |
                   ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) |
                   ((uint32_t)buffer[3]);
	//LOG_PRINTF(LL_DEBUG, "ltc2485: PTAT raw 0x%08luX (hex)\r\n", raw);
    raw ^= 0x80000000;
	float mV = (float)raw * (vref / 2147483648.0);  // 2^31
	//LOG_PRINTF(LL_DEBUG, "ltc2485: PTAT %.2f mV\r\n", mV);
	// Temperature calculation (from LTC2485 datasheet, p.20)
	const float LTC2485_TAVERAGE       = 27.0;    //  °C
	const float LTC2485_MVOLT_TAVERAGE = 420.0;   //  mV
	const float LTC2485_SLOPE          = 1.40;    //  mV/°C
    float TC = LTC2485_TAVERAGE + (mV - LTC2485_MVOLT_TAVERAGE) / LTC2485_SLOPE;
	//LOG_PRINTF(LL_DEBUG, "ltc2485: PTAT %.2f C\r\n", TC);
    return TC;	
}


//see definitions in: lora_config.h
// Terminal   PIN   Function
// Pin 1:     VCC
// Pin 4:     SLC
// Pin 5:     SDA
// Pin 6:     PC13  ENABE VX Active low
// Pin 7:     PB9   NOT USED; Just use the terminal to deliver VX// Set High Impedance set Input, no Pullup
// Pin 8:     PB8   EXT ADC // Set High Impedance no Pullup

void ltc2485_control_gpio_init(void) {
    DABAEXT_VX_CLK_ENABLE();
    DABAEXT_BLOCK_CLK_ENABLE();

    gpio_set_iomux(DABAEXT_VX_PORT, DABAEXT_VX_PIN, 0);
    gpio_init(DABAEXT_VX_PORT, DABAEXT_VX_PIN, GPIO_MODE_OUTPUT_PP_HIGH); //EN is active low

    gpio_set_iomux(DABAEXT_BLOCK_PORT, DABAEXT_BLOCK_PIN1, 0);
    gpio_init(DABAEXT_BLOCK_PORT, DABAEXT_BLOCK_PIN1, GPIO_MODE_INPUT_FLOATING);

    gpio_set_iomux(DABAEXT_BLOCK_PORT, DABAEXT_BLOCK_PIN2, 0);
    gpio_init(DABAEXT_BLOCK_PORT, DABAEXT_BLOCK_PIN2, GPIO_MODE_INPUT_FLOATING);
}
