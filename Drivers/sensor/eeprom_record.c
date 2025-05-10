#include "eeprom_record.h"
#include "m24m02e.h"
#include <string.h>

eeprom_record_t record_cfg;


/**
 * @brief Initialize the EEPROM record management system.
 *
 * This function sets up internal configuration for fixed-size record handling
 * and scans the EEPROM memory from the given base address to find the next
 * available write location.
 *
 * The EEPROM is treated as a circular buffer. Each record is assumed to have
 * a constant size (specified by record_size), and records are stored back-to-back
 * starting from base_addr.
 *
 * The function searches sequentially for the first record-sized block that is
 * completely filled with EEPROM_EMPTY_BYTE (typically 0x00), which is considered
 * an "empty" or unwritten slot. This location becomes the next_write_addr.
 *
 * If no empty slot is found (i.e., the EEPROM is fully written), the next write
 * will wrap around and overwrite the first block at base_addr.
 *
 * @param record_size  The fixed number of bytes for each record (must be > 0)
 * @param base_addr    The starting EEPROM address for record storage
 * @return true if initialization and scanning succeeded, false on error
 *
 * @note This function must be called at startup or after reset to configure
 *       the record system and resume writing correctly. It does not clear or
 *       erase any memory.
 */
bool eeprom_record_init(uint16_t record_size, uint32_t base_addr)
{
    if (record_size == 0 || record_size > 255 || base_addr >= EEPROM_TOTAL_SIZE)
        return false;

    record_cfg.base_addr = base_addr;
    record_cfg.record_size = record_size;
    record_cfg.total_records = (EEPROM_TOTAL_SIZE - base_addr) / record_size;
    record_cfg.next_write_addr = base_addr;

    uint8_t buffer[record_size];
    for (uint32_t i = 0; i < record_cfg.total_records; i++) {
        uint32_t addr = base_addr + i * record_size;
        if (!m24m02e_read(addr, buffer, record_size))
            return false;

        bool is_empty = true;
        for (uint16_t j = 0; j < record_size; j++) {
            if (buffer[j] != EEPROM_EMPTY_BYTE) {
                is_empty = false;
                break;
            }
        }

        if (is_empty) {
            record_cfg.next_write_addr = addr;
            return true;
        }
    }

    record_cfg.next_write_addr = base_addr;
    return true;
}

/**
 * @brief Write a new fixed-size record to EEPROM in circular fashion.
 *
 * This function writes a new record at the current write position and 
 * then marks the next slot with 0x00 bytes to indicate it is the next 
 * available write location. This marking serves as a simple and efficient 
 * pointer for determining the most recent entry after a power reset, 
 * without requiring metadata like counters or timestamps.
 *
 * The EEPROM is treated as a circular buffer. When the end of the 
 * memory area is reached, writing wraps around to the base address.
 *
 * @param data Pointer to the data buffer (must be record_size bytes)
 * @return true if the write and marking succeeded, false on error
 */
bool eeprom_record_write(const uint8_t* data)
{
    if (!data || record_cfg.record_size == 0)
        return false;

    if (!m24m02e_write(record_cfg.next_write_addr, data, record_cfg.record_size))
        return false;

    uint32_t next = record_cfg.next_write_addr + record_cfg.record_size;
    if (next >= EEPROM_TOTAL_SIZE)
        next = record_cfg.base_addr;

    uint8_t empty_buf[record_cfg.record_size];
    memset(empty_buf, EEPROM_EMPTY_BYTE, record_cfg.record_size);

    if (!m24m02e_write(next, empty_buf, record_cfg.record_size))
        return false;

    record_cfg.next_write_addr = next;
    return true;
}

bool eeprom_record_read(uint32_t index, uint8_t* data_out)
{
    if (!data_out || index >= record_cfg.total_records)
        return false;

    uint32_t addr = record_cfg.base_addr + index * record_cfg.record_size;
    return m24m02e_read(addr, data_out, record_cfg.record_size);
}

bool eeprom_record_read_relative(int32_t rel_index, uint8_t* data_out)
{
    if (!data_out || rel_index >= 0 || record_cfg.record_size == 0)
        return false;

    uint32_t current_index = (record_cfg.next_write_addr - record_cfg.base_addr) / record_cfg.record_size;
    int32_t abs_index = (int32_t)current_index + rel_index - 1;

    if (abs_index < 0)
        abs_index += record_cfg.total_records;

    if (abs_index < 0 || abs_index >= (int32_t)record_cfg.total_records)
        return false;

    uint32_t addr = record_cfg.base_addr + ((uint32_t)abs_index * record_cfg.record_size);
    return m24m02e_read(addr, data_out, record_cfg.record_size);
}

/**
 * @brief Reset the EEPROM record storage system to start a new logging session.
 *
 * This function resets the internal configuration for record management and
 * marks the start of a new recording session by clearing (writing 0x00 to) 
 * the first record block. This signals to the eeprom_record_init() function
 * that the first slot is available and should be used for the next write.
 *
 * It does not erase the entire EEPROM area or affect existing records beyond
 * the first block. This makes it efficient and low-wear for switching modes
 * or starting a fresh data log while preserving past data unless explicitly
 * overwritten during normal circular logging.
 *
 * Typical use cases include:
 *  - Switching to a new application mode with a different record format
 *  - Starting a new log session after reboot or remote command
 *
 * @param record_size  Size of a single record (must be > 0)
 * @return true if the reset succeeded and first block was cleared, false on error
 *
 * @note After calling this function, you must call eeprom_record_init() to 
 *       re-scan the memory and resume logging correctly.
 */
uint32_t eeprom_record_count(void)
{
    uint8_t buf[record_cfg.record_size];
    for (uint32_t i = 0; i < record_cfg.total_records; i++) {
        uint32_t addr = record_cfg.base_addr + i * record_cfg.record_size;
        if (!m24m02e_read(addr, buf, record_cfg.record_size))
            break;

        bool is_empty = true;
        for (uint16_t j = 0; j < record_cfg.record_size; j++) {
            if (buf[j] != EEPROM_EMPTY_BYTE) {
                is_empty = false;
                break;
            }
        }

        if (is_empty)
            return i;
    }
    return record_cfg.total_records;
}

bool eeprom_record_reset(uint16_t record_size)
{
    if (record_size == 0 || BASE_ADDRESS >= EEPROM_TOTAL_SIZE){
        return false;
		}
	// clean out all remaining data, set all to 0xFF
	eeprom_record_clear_all();
    record_cfg.base_addr = BASE_ADDRESS;
    record_cfg.record_size = record_size;
    record_cfg.total_records = (EEPROM_TOTAL_SIZE - BASE_ADDRESS) / record_size;
    record_cfg.next_write_addr = BASE_ADDRESS;
    // Mark the first block as "next write location" by clearing it with 0x00
    uint8_t marker[record_size];
    memset(marker, EEPROM_EMPTY_BYTE, record_size);
    return m24m02e_write(BASE_ADDRESS, marker, record_size);
}


/**
 * @brief Smartly clear all EEPROM records by writing 0xFF only to used blocks.
 *
 * This function erases all record blocks by setting them to 0xFF (erased state),
 * but only writes to blocks that are not already fully erased. This reduces
 * EEPROM wear. It is intended to be used during mode switches or full resets
 * to ensure stale or incompatible data is cleared before new logging starts.
 *
 * @return true if all blocks were processed successfully, false on any I/O error
 */
bool eeprom_record_clear_all(void)
{
    uint8_t buf[record_cfg.record_size];
    uint8_t blank[record_cfg.record_size];
    memset(blank, 0xFF, record_cfg.record_size);

    for (uint32_t i = 0; i < record_cfg.total_records; i++) {
        uint32_t addr = record_cfg.base_addr + i * record_cfg.record_size;

        if (!m24m02e_read(addr, buf, record_cfg.record_size))
            return false;

        if (memcmp(buf, blank, record_cfg.record_size) != 0) {
            if (!m24m02e_write(addr, blank, record_cfg.record_size))
                return false;
        }
    }

    return true;
}


/**
 * @brief Print a record buffer as a hex string using LOG_PRINTF.
 *
 * This function prints the given buffer as a compact hex string.
 * It uses the current record size from the configuration and assumes
 * the buffer is valid and contains record_cfg.record_size bytes.
 *
 * @param buf Pointer to the buffer containing the record data
 */
void eeprom_record_print_hex(const uint8_t* buf)
{
    if (!buf || record_cfg.record_size == 0)
        return;

    for (uint16_t i = 0; i < record_cfg.record_size; i++) {
        LOG_PRINTF(LL_DEBUG, "%02X", buf[i]);
    }
    LOG_PRINTF(LL_DEBUG, "\r\n");
}

uint32_t eeprom_record_get_total_records(void) {
    return record_cfg.total_records;
}

void pack_sensor_record_7byte(const sensor_t* sensor_data, uint8_t* out) {
    out[0] = (sensor_data->bat_mv >> 8);
    out[1] = sensor_data->bat_mv & 0xFF;

    out[2] = (sensor_data->ADC_ext_24bit >> 16) & 0xFF;
    out[3] = (sensor_data->ADC_ext_24bit >> 8) & 0xFF;
    out[4] = sensor_data->ADC_ext_24bit & 0xFF;

    int16_t t = (int16_t)(sensor_data->temp1 * 10);
    out[5] = (t >> 8) & 0xFF;
    out[6] = t & 0xFF;
}


void pack_sensor_record_17byte(const sensor_t* sensor_data, uint8_t* out) {
    out[0] = (sensor_data->bat_mv >> 8);
    out[1] = sensor_data->bat_mv & 0xFF;

    out[2] = (sensor_data->ADC_ext_24bit >> 16) & 0xFF;
    out[3] = (sensor_data->ADC_ext_24bit >> 8) & 0xFF;
    out[4] = sensor_data->ADC_ext_24bit & 0xFF;

    int16_t t = (int16_t)(sensor_data->temp_sht * 10);
    out[5] = (t >> 8) & 0xFF;
    out[6] = t & 0xFF;

    int16_t h = (int16_t)(sensor_data->hum_sht * 10);
    out[7] = (h >> 8) & 0xFF;
    out[8] = h & 0xFF;

    out[9]  = (sensor_data->RGBIR >> 56) & 0xFF;
    out[10] = (sensor_data->RGBIR >> 48) & 0xFF;
    out[11] = (sensor_data->RGBIR >> 40) & 0xFF;
    out[12] = (sensor_data->RGBIR >> 32) & 0xFF;
    out[13] = (sensor_data->RGBIR >> 24) & 0xFF;
    out[14] = (sensor_data->RGBIR >> 16) & 0xFF;
    out[15] = (sensor_data->RGBIR >> 8) & 0xFF;
    out[16] = sensor_data->RGBIR & 0xFF;
}
//reqired to send the stored data
void unpack_sensor_record_7byte(const uint8_t* in, sensor_t* out) {
    out->bat_mv = (in[0] << 8) | in[1];

    out->ADC_ext_24bit = ((int32_t)in[2] << 16) |
                         ((int32_t)in[3] << 8)  |
                         ((int32_t)in[4]);

    int16_t t = (in[5] << 8) | in[6];
    out->temp1 = t / 10.0f;

    // Leave other fields unchanged
}

//reqired to send the stored data
void unpack_sensor_record_17byte(const uint8_t* in, sensor_t* out) {
    out->bat_mv = (in[0] << 8) | in[1];

    out->ADC_ext_24bit = ((int32_t)in[2] << 16) |
                         ((int32_t)in[3] << 8)  |
                         ((int32_t)in[4]);

    int16_t t = (in[5] << 8) | in[6];
    out->temp_sht = t / 10.0f;

    int16_t h = (in[7] << 8) | in[8];
    out->hum_sht = h / 10.0f;

    out->RGBIR = ((uint64_t)in[9]  << 56) |
                 ((uint64_t)in[10] << 48) |
                 ((uint64_t)in[11] << 40) |
                 ((uint64_t)in[12] << 32) |
                 ((uint64_t)in[13] << 24) |
                 ((uint64_t)in[14] << 16) |
                 ((uint64_t)in[15] << 8)  |
                 ((uint64_t)in[16]);

    // Leave temp1 unchanged
}

/**
 * @brief Initialize EEPROM record storage based on the selected work mode.
 *
 * This function is called at system startup or mode switch to prepare the
 * EEPROM for data logging or record-keeping. It first checks that the EEPROM
 * is connected and responsive. Then, based on the current work mode, it:
 *
 *  - Selects the appropriate record size for the mode
 *  - Calls eeprom_record_reset() to reset the record configuration and
 *    mark the start of the new record series (by clearing the first block)
 *  - Calls eeprom_record_init() to scan for the next available write address
 *    (which will be the cleared block after reset)
 *
 * The function supports multiple modes (e.g., 101, 102), each potentially
 * using different record sizes. If no mode-specific configuration is found,
 * a warning is logged and no action is taken.
 *
 * @param workmode The operational mode that determines how EEPROM storage is configured
 */

void init_record_storage(uint8_t workmode) {
    if (check_m24m02e_connect()) {
		uint8_t record_size;
        if (workmode == 101) {
			record_size=WORKMODE_101_RECORD;
        } else if (workmode == 102) {
            record_size=WORKMODE_102_RECORD;
        } else {
            LOG_PRINTF(LL_WARN, "No record storage defined for workmode %d\r\n", workmode);
            return;
        }
		eeprom_record_reset(record_size);
        if (!eeprom_record_init(record_size, BASE_ADDRESS)) {
            LOG_PRINTF(LL_ERR, "Record init failed\r\n");
            return;
        }

        LOG_PRINTF(LL_DEBUG, "EEPROM record system initialized (size: %d bytes)\r\n", record_size);
    } else {
        LOG_PRINTF(LL_WARN, "EEPROM not detected\r\n");
    }
}

