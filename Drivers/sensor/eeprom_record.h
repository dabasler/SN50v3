#ifndef EEPROM_RECORD_H
#define EEPROM_RECORD_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "log.h"

#define BASE_ADDRESS 0x000000 
#define WORKMODE_101_RECORD 7
#define WORKMODE_102_RECORD 17


#define EEPROM_TOTAL_SIZE   (256 * 1024UL)
#define EEPROM_EMPTY_BYTE   0x00

typedef struct {
    uint32_t base_addr;
    uint16_t record_size;
    uint32_t total_records;
    uint32_t next_write_addr;
} eeprom_record_t;

extern eeprom_record_t record_cfg;

bool eeprom_record_init(uint16_t record_size, uint32_t base_addr);
bool eeprom_record_write(const uint8_t* data);
bool eeprom_record_read(uint32_t index, uint8_t* data_out);
bool eeprom_record_read_relative(int32_t rel_index, uint8_t* data_out);
uint32_t eeprom_record_count(void);

bool eeprom_record_reset(uint16_t record_size);
bool eeprom_record_clear_all(void);
void eeprom_record_print_hex(const uint8_t* buf);


void pack_sensor_record_7byte(const sensor_t* sensor_data, uint8_t* out);
void pack_sensor_record_17byte(const sensor_t* sensor_data, uint8_t* out);
void unpack_sensor_record_7byte(const uint8_t* in, sensor_t* out);
void unpack_sensor_record_17byte(const uint8_t* in, sensor_t* out);

void init_record_storage(uint8_t workmode);
#endif // EEPROM_RECORD_H


