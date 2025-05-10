#include "m24m02e.h"
#include "I2C_A.h"
#include "tremo_delay.h"

#define PAGE_SIZE 256
#define WRITE_CYCLE_TIME_MS 5

static uint8_t calc_device_addr(uint32_t addr) {
    return M24M02E_I2C_ADDR_BASE | ((addr >> 16) & 0x03);
}

bool m24m02e_init(void) {
    I2C_GPIO_MODE_Config();
    return true;
}

uint8_t check_m24m02e_connect(void) {
    uint8_t test;
    I2C_GPIO_MODE_Config();
    return m24m02e_read(0x000000, &test, 1) ? 1 : 0;
}

bool m24m02e_read(uint32_t mem_addr, uint8_t* data, uint16_t len) {
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)((mem_addr >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(mem_addr & 0xFF);
    uint8_t dev_addr = calc_device_addr(mem_addr);

    if (I2C_Write_reg_Len(dev_addr, addr_buf[0], 1, &addr_buf[1]) != 0)
        return false;

    return I2C_Read_Len(dev_addr, 0x00, len, data) == 0;
}

bool m24m02e_write(uint32_t mem_addr, const uint8_t* data, uint16_t len) {
    while (len > 0) {
        uint8_t page_offset = mem_addr % PAGE_SIZE;
        uint8_t chunk = PAGE_SIZE - page_offset;
        if (chunk > len) chunk = len;

        uint8_t dev_addr = calc_device_addr(mem_addr);
        uint8_t buf[chunk + 2];
        buf[0] = (mem_addr >> 8) & 0xFF;
        buf[1] = mem_addr & 0xFF;
        for (uint8_t i = 0; i < chunk; ++i) buf[i + 2] = data[i];

        if (I2C_Write_Len(dev_addr, buf[0], chunk + 1, &buf[1]) != 0)
            return false;

        delay_ms(WRITE_CYCLE_TIME_MS);
        mem_addr += chunk;
        data += chunk;
        len -= chunk;
    }
    return true;
}

bool m24m02e_sleep(void) {
    I2C_GPIO_MODE_ANALOG();  // Optional: put bus in high-Z if needed
    return true;
}







