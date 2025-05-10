#include "apds9250.h"
#include "I2C_A.h"
#include "tremo_delay.h"

// Register addresses
#define MAIN_CTRL_REG         0x00
#define LS_MEAS_RATE_REG      0x04
#define LS_GAIN_REG           0x05
#define MAIN_STATUS_REG       0x07
#define PART_ID_REG           0x06
#define RED_DATA_REG          0x13
#define GREEN_DATA_REG        0x0D
#define BLUE_DATA_REG         0x10
#define IR_DATA_REG           0x0A

// Gain constants
#define APDS9250_GAIN_1X   0x00
#define APDS9250_GAIN_3X   0x01
#define APDS9250_GAIN_6X   0x02
#define APDS9250_GAIN_9X   0x03
#define APDS9250_GAIN_18X  0x04

// Dynamic adjustment thresholds
#define APDS9250_SATURATION_THRESHOLD  240000U
#define APDS9250_LOW_SIGNAL_THRESHOLD   500U

static uint8_t current_gain = APDS9250_GAIN_3X;

static bool read_channel(uint8_t base_reg, uint32_t* out) {
    uint8_t data[3];
    if (I2C_Read_reg_Len(APDS9250_I2C_ADDR, base_reg, 3, data) != 0)
        return false;
    *out = ((data[2] & 0x0F) << 16) | (data[1] << 8) | data[0];
    return true;
}

static bool apds9250_set_gain(uint8_t gain) {
    if (gain == current_gain)
        return true;

    uint8_t val = gain;
    if (I2C_Write_reg_Len(APDS9250_I2C_ADDR, LS_GAIN_REG, 1, &val) != 0)
        return false;

    current_gain = gain;
    delay_ms(10);
    return true;
}

static void apds9250_adjust_gain(uint32_t r, uint32_t g, uint32_t b, uint32_t ir) {
    uint32_t max_val = r;
    if (g > max_val) max_val = g;
    if (b > max_val) max_val = b;
    if (ir > max_val) max_val = ir;

    if (max_val >= APDS9250_SATURATION_THRESHOLD && current_gain > APDS9250_GAIN_1X)
        apds9250_set_gain(current_gain - 1);
    else if (max_val < APDS9250_LOW_SIGNAL_THRESHOLD && current_gain < APDS9250_GAIN_9X)
        apds9250_set_gain(current_gain + 1);
}

bool apds9250_init(void) {
    //I2C_GPIO_MODE_Config();

    uint8_t ctrl = 0x02;
    if (I2C_Write_reg_Len(APDS9250_I2C_ADDR, MAIN_CTRL_REG, 1, &ctrl) != 0)
        return false;

    delay_ms(5);

    uint8_t meas = 0x22;
    if (I2C_Write_reg_Len(APDS9250_I2C_ADDR, LS_MEAS_RATE_REG, 1, &meas) != 0)
        return false;

    current_gain = APDS9250_GAIN_3X;
    return apds9250_set_gain(current_gain);
}

uint8_t check_apds9250_connect(void) {
    uint8_t part_id;
    //I2C_GPIO_MODE_Config();
    if (I2C_Read_reg_Len(APDS9250_I2C_ADDR, PART_ID_REG, 1, &part_id) != 0)
        return 0;
    return (part_id == 0xB5) ? 1 : 0;
}

bool apds9250_read_raw(uint32_t *r, uint32_t *g, uint32_t *b, uint32_t *ir, uint32_t timeout_ms) {
    uint8_t status;
    uint32_t elapsed = 0;

    do {
        if (I2C_Read_reg_Len(APDS9250_I2C_ADDR, MAIN_STATUS_REG, 1, &status) != 0)
            return false;
        if (status & (1 << 3)) break;
        delay_ms(10);
        elapsed += 10;
    } while (elapsed < timeout_ms);

    if (!(status & (1 << 3)))
        return false;

    if (!read_channel(RED_DATA_REG, r) ||
        !read_channel(GREEN_DATA_REG, g) ||
        !read_channel(BLUE_DATA_REG, b) ||
        !read_channel(IR_DATA_REG, ir))
        return false;

    apds9250_adjust_gain(*r, *g, *b, *ir);
    return true;
}

/**
 * @brief Performs a single RGB+IR measurement using the APDS-9250 sensor.
 *
 * This function triggers a light sensor readout, optionally adjusts the gain based
 * on signal level, and returns a 64-bit packed result containing the measured values.
 * Each of the R, G, B, and IR channels is packed into 16 bits, and the current gain
 * setting is stored in the upper 4 bits of the result.
 *
 * Bit layout of returned uint64_t:
 *   [63:60] Gain setting (0–4, corresponds to 1x–18x)
 *   [59:48] Red channel (16-bit)
 *   [47:32] Green channel (16-bit)
 *   [31:16] Blue channel (16-bit)
 *   [15: 0] IR channel (16-bit)
 *
 * @param timeout_ms Timeout for sensor readiness (in milliseconds)
 * @param success Pointer to a bool that will be set to true if the measurement succeeded
 * @return 64-bit packed result containing gain and color channel data;
 *         0xFFFFFFFFFFFFFFFF if measurement failed
 */
uint64_t apds9250_measure(uint32_t timeout_ms, bool* success) {

	apds9250_init();
    uint32_t r, g, b, ir;
    *success = apds9250_read_raw(&r, &g, &b, &ir, timeout_ms);
    if (!*success)
        return 0xFFFFFFFFFFFFFFFF;

    uint64_t packed = 0;
    packed |= ((uint64_t)(current_gain & 0x0F) << 60);
    packed |= ((uint64_t)(r & 0xFFFF) << 48);
    packed |= ((uint64_t)(g & 0xFFFF) << 32);
    packed |= ((uint64_t)(b & 0xFFFF) << 16);
    packed |= ((uint64_t)(ir & 0xFFFF));
	apds9250_sleep();
    return packed;
}

bool apds9250_sleep(void) {
    uint8_t ctrl = 0x00;
    return I2C_Write_reg_Len(APDS9250_I2C_ADDR, MAIN_CTRL_REG, 1, &ctrl) == 0;
}

void apds9250_unpack(uint64_t packed, apds9250_t* data, uint8_t* gain_out) {
    if (gain_out) *gain_out = (packed >> 60) & 0x0F;
    if (data) {
        data->red   = (packed >> 48) & 0xFFFF;
        data->green = (packed >> 32) & 0xFFFF;
        data->blue  = (packed >> 16) & 0xFFFF;
        data->ir    =  packed        & 0xFFFF;
    }
}
