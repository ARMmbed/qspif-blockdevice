/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "QSPIFBlockDevice.h"
#include <string.h>
#include "mbed_wait_api.h"

#include "mbed_trace.h"
#define TRACE_GROUP "QSPIF"

/* Default QSPIF Parameters */
/****************************/
#define QSPIF_DEFAULT_READ_SIZE  1
#define QSPIF_DEFAULT_PROG_SIZE  1
#define QSPIF_DEFAULT_SE_SIZE    4096
#define QSPI_MAX_STATUS_REGISTER_SIZE 2
#define QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC 50
#define QSPIF_DEFAULT_TIMEOUT_MSEC    1


/* SFDP Header Parsing */
/***********************/
#define QSPIF_SFDP_HEADER_SIZE 8
#define QSPIF_PARAM_HEADER_SIZE 8

/* Basic Parameters Table Parsing */
/**********************************/
#define SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES 64 /* 16 DWORDS */
//READ Instruction support according to BUS Configuration
#define QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE 2
#define QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE 16
#define QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE 27
#define QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE 9
#define QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE 11
#define QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE 23
#define QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE 15
#define QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE 13
#define QSPIF_BASIC_PARAM_TABLE_PAGE_SIZE_BYTE 40
// Quad Enable Params
#define QSPIF_BASIC_PARAM_TABLE_QER_BYTE 58
#define QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE 56
// Erase Types Params
#define QSPIF_BASIC_PARAM_ERASE_TYPE_1_BYTE 29
#define QSPIF_BASIC_PARAM_ERASE_TYPE_2_BYTE 31
#define QSPIF_BASIC_PARAM_ERASE_TYPE_3_BYTE 33
#define QSPIF_BASIC_PARAM_ERASE_TYPE_4_BYTE 35
#define QSPIF_BASIC_PARAM_ERASE_TYPE_1_SIZE_BYTE 28
#define QSPIF_BASIC_PARAM_ERASE_TYPE_2_SIZE_BYTE 30
#define QSPIF_BASIC_PARAM_ERASE_TYPE_3_SIZE_BYTE 32
#define QSPIF_BASIC_PARAM_ERASE_TYPE_4_SIZE_BYTE 34
#define QSPIF_BASIC_PARAM_4K_ERASE_TYPE_BYTE 1


// Erase Types Per Region BitMask
#define ERASE_BITMASK_TYPE4 0x08
#define ERASE_BITMASK_TYPE3 0x04
#define ERASE_BITMASK_TYPE2 0x02
#define ERASE_BITMASK_TYPE1 0x01
#define ERASE_BITMASK_NONE  0x00
#define ERASE_BITMASK_ALL   0x0F

#define IS_MEM_READY_MAX_RETRIES 10000

// Debug Printouts
#define QSPIF_DEBUG_ERROR   1
#define QSPIF_DEBUG_WARNING 2
#define QSPIF_DEBUG_INFO    3
#define QSPIF_DEBUG_DEBUG   4
#define QSPIF_DEBUG_TRACE   5

namespace mbed {

enum qspif_default_instructions {
    QSPIF_NOP  = 0x00, // No operation
    QSPIF_PP = 0x02, // Page Program data
    QSPIF_READ = 0x03, // Read data
    QSPIF_SE   = 0x20, // 4KB Sector Erase
    QSPIF_SFDP = 0x5a, // Read SFDP
    QSPIF_WRSR = 0x01, // Write Status/Configuration Register
    QSPIF_WRDI = 0x04, // Write Disable
    QSPIF_RDSR = 0x05, // Read Status Register
    QSPIF_WREN = 0x06, // Write Enable
    QSPIF_RSTEN = 0x66, // Reset Enable
    QSPIF_RST = 0x99, // Reset
    QSPIF_RDID = 0x9f, // Read Manufacturer and JDEC Device ID
};

// Mutex is used for some QSPI Driver commands that must be done sequentially with no other commands in between
// e.g. (1)Set Write Enable, (2)Program, (3)Wait Memory Ready
SingletonPtr<PlatformMutex> QSPIFBlockDevice::_mutex;

// Local Function
static int local_math_power(int base, int exp);

/********* Public API Functions *********/
/****************************************/

QSPIFBlockDevice::QSPIFBlockDevice(PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName csel,
                                   qspif_polarity_mode clock_mode, int freq)
    : _qspi(io0, io1, io2, io3, sclk, csel, clock_mode), _device_size_bytes(0)
{
    _is_initialized = false;
    _min_common_erase_size = 0;
    _regions_count = 1;
    _region_erase_types_bitfield[0] = ERASE_BITMASK_NONE;

    //Default Bus Setup 1_1_1 with 0 dummy and mode cycles
    _inst_width = QSPI_CFG_BUS_SINGLE;
    _address_width = QSPI_CFG_BUS_SINGLE;
    _address_size = QSPI_CFG_ADDR_SIZE_24;
    _data_width = QSPI_CFG_BUS_SINGLE;
    _dummy_and_mode_cycles = 0;

    if (QSPI_STATUS_OK != _qsp_set_frequency(freq)) {
        tr_error("ERROR: QSPI Set Frequency Failed");
        tr_error("ERROR: QSPI Set Frequency Failed");
    }
}


int QSPIFBlockDevice::init()
{

    uint8_t vendor_device_ids[4];
    size_t data_length = 3;
    int status = QSPIF_BD_ERROR_OK;
    uint32_t basic_table_addr = NULL;
    size_t basic_table_size = 0;
    uint32_t sector_map_table_addr = NULL;
    size_t sector_map_table_size = 0;
    int qspi_status = QSPI_STATUS_OK;

    _mutex->lock();
    if (_is_initialized == true) {
        goto Exit_Point;
    }

    // Soft Reset
    if ( -1 == _reset_flash_mem()) {
        tr_error("ERROR: init - Unable to initialize flash memory, tests failed\n");
        status = QSPIF_BD_ERROR_DEVICE_ERROR;
        goto Exit_Point;
    } else {
        tr_info("INFO: Initialize flash memory OK\n");
    }

    /* Read Manufacturer ID (1byte), and Device ID (2bytes)*/
    qspi_status = _qspi_send_read_command(QSPIF_RDID, (char *)vendor_device_ids, 0x0 /*address*/, data_length);
    if (qspi_status != QSPI_STATUS_OK) {
        tr_error("ERROR: init - Read Vendor ID Failed");
        status = QSPIF_BD_ERROR_DEVICE_ERROR;
        goto Exit_Point;
    }

    switch (vendor_device_ids[0]) {
        case 0xbf:
            // SST devices come preset with block protection
            // enabled for some regions, issue write disable instruction to clear
            _set_write_enable();
            _qspi_send_general_command(QSPIF_WRDI, -1, NULL, 0, NULL, 0);
            break;
    }

    //Synchronize Device
    if ( false == _is_mem_ready()) {
        tr_error("ERROR: init - _is_mem_ready Failed");
        status = QSPIF_BD_ERROR_READY_FAILED;
        goto Exit_Point;
    }

    /**************************** Parse SFDP Header ***********************************/
    if ( 0 != _sfdp_parse_sfdp_headers(basic_table_addr, basic_table_size, sector_map_table_addr, sector_map_table_size)) {
        tr_error("ERROR: init - Parse SFDP Headers Failed");
        status = QSPIF_BD_ERROR_PARSING_FAILED;
        goto Exit_Point;
    }


    /**************************** Parse Basic Parameters Table ***********************************/
    if ( 0 != _sfdp_parse_basic_param_table(basic_table_addr, basic_table_size) ) {
        tr_error("ERROR: init - Parse Basic Param Table Failed");
        status = QSPIF_BD_ERROR_PARSING_FAILED;
        goto Exit_Point;
    }

    /**************************** Parse Sector Map Table ***********************************/
    _region_size_bytes[0] =
        _device_size_bytes; // If there's no region map, we have a single region sized the entire device size
    _region_high_boundary[0] = _device_size_bytes - 1;

    if ( (sector_map_table_addr != NULL) && (0 != sector_map_table_size) ) {
        tr_info("INFO: init - Parsing Sector Map Table - addr: 0x%xh, Size: %d", sector_map_table_addr,
                sector_map_table_size);
        if ( 0 != _sfdp_parse_sector_map_table(sector_map_table_addr, sector_map_table_size) ) {
            tr_error("ERROR: init - Parse Sector Map Table Failed");
            status = QSPIF_BD_ERROR_PARSING_FAILED;
            goto Exit_Point;
        }
    }

    // Configure  BUS Mode to 1_1_1 for all commands other than Read
    _qspi_configure_format(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                           QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);

    _is_initialized = true;

Exit_Point:
    _mutex->unlock();

    return status;
}

int QSPIFBlockDevice::deinit()
{
    _mutex->lock();
    if (_is_initialized == false) {
        _mutex->unlock();
        return QSPIF_BD_ERROR_OK;
    }

    // Disable Device for Writing
    qspi_status_t status = _qspi_send_general_command(QSPIF_WRDI, -1, NULL, 0, NULL, 0);
    int result = QSPIF_BD_ERROR_OK;

    if (status != QSPI_STATUS_OK)  {
        tr_error("ERROR: Write Disable failed");
        result = QSPIF_BD_ERROR_DEVICE_ERROR;
    }
    _is_initialized = false;
    _mutex->unlock();

    return result;
}


int QSPIFBlockDevice::read(void *buffer, bd_addr_t addr, bd_size_t size)
{


    int status = QSPIF_BD_ERROR_OK;

    tr_info("INFO Inst: 0x%xh", _read_instruction);

    _mutex->lock();

    // Configure Bus for Reading
    _qspi_configure_format(
        _inst_width, //Bus width for Instruction phase
        _address_width, //Bus width for Address phase
        _address_size,
        QSPI_CFG_BUS_SINGLE, //Bus width for Alt phase
        QSPI_CFG_ALT_SIZE_8,
        _data_width, //Bus width for Data phase
        _dummy_and_mode_cycles);

    if (QSPI_STATUS_OK != _qspi_send_read_command(_read_instruction, buffer, addr, size)) {
        status = QSPIF_BD_ERROR_DEVICE_ERROR;
        tr_error("ERROR: Read failed\n");
    }

    // All commands other than Read use default 1-1-1 Bus mode (Program/Erase are constrained by flash memory performance less than that of the bus)
    _qspi_configure_format(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                           QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);

    _mutex->unlock();
    return status;

}


int QSPIFBlockDevice::program(const void *buffer, bd_addr_t addr, bd_size_t size)
{
    qspi_status_t result = QSPI_STATUS_OK;
    bool program_failed = false;
    int status = QSPIF_BD_ERROR_OK;
    uint32_t offset = 0;
    uint32_t chunk = 0;
    bd_size_t writtenBytes = 0;

    tr_debug("DEBUG: program - Buff: 0x%x, addr: %llu, size: %llu", buffer, addr, size);

    while (size > 0) {

        // Write on _page_size_bytes boundaries (Default 256 bytes a page)
        offset = addr % _page_size_bytes;
        chunk = (offset + size < _page_size_bytes) ? size : (_page_size_bytes - offset);
        writtenBytes = chunk;

        _mutex->lock();

        //Send WREN
        if (_set_write_enable() != 0) {
            tr_error("ERROR: Write Enabe failed\n");
            program_failed = true;
            status = QSPIF_BD_ERROR_WREN_FAILED;
            goto Exit_Point;
        }

        if ( false == _is_mem_ready()) {
            tr_error("ERROR: Device not ready, write failed\n");
            program_failed = true;
            status = QSPIF_BD_ERROR_READY_FAILED;
            goto Exit_Point;
        }

        result = _qspi_send_program_command(_prog_instruction, buffer, addr, &writtenBytes);
        if ( (result != QSPI_STATUS_OK) || (chunk != writtenBytes) ) {
            tr_error("ERROR: Write failed");
            program_failed = true;
            status = QSPIF_BD_ERROR_DEVICE_ERROR;
            goto Exit_Point;
        }

        buffer = static_cast<const uint8_t *>(buffer) + chunk;
        addr += chunk;
        size -= chunk;

        wait_ms(QSPIF_DEFAULT_TIMEOUT_MSEC);

        if ( false == _is_mem_ready()) {
            tr_error("ERROR: Device not ready after write, failed\n");
            program_failed = true;
            status = QSPIF_BD_ERROR_READY_FAILED;
            goto Exit_Point;
        }
        _mutex->unlock();


    }

Exit_Point:
    if (program_failed) {
        _mutex->unlock();
    }

    return status;
}


int QSPIFBlockDevice::erase(bd_addr_t addr, bd_size_t inSize)
{

    int type = 0;
    uint32_t chunk = 4096;
    unsigned int cur_erase_inst = _erase_instruction;
    int size = (int)inSize;
    bool erase_failed = false;
    int status = QSPIF_BD_ERROR_OK;
    // Find region of erased address
    int region = _utils_find_addr_region(addr);
    // Erase Types of selected region
    uint8_t bitfield = _region_erase_types_bitfield[region];

    tr_debug("DEBUG: erase - addr: %llu, inSize: %llu", addr, inSize);

    // For each iteration erase the largest section supported by current region
    while (size > 0) {

        // iterate to find next Largest erase type ( a. supported by region, b. smaller than size)
        // find the matching instruction and erase size chunk for that type.
        type = _utils_iterate_next_largest_erase_type(bitfield, size, (int)addr, _region_high_boundary[region]);
        cur_erase_inst = _erase_type_inst_arr[type];
        chunk = _erase_type_size_arr[type];

        tr_debug("DEBUG: erase - addr: llu, size:%d, Inst: 0x%xh, chunk: %d , ",
                 addr, size, cur_erase_inst, chunk);
        tr_debug("DEBUG: erase - Region: %d, Type:%d",
                 region, type);

        _mutex->lock();

        if (_set_write_enable() != 0) {
            tr_error("ERROR: QSPI Erase Device not ready - failed");
            erase_failed = true;
            status = QSPIF_BD_ERROR_READY_FAILED;
            goto Exit_Point;
        }

        if (QSPI_STATUS_OK != _qspi_send_erase_command(cur_erase_inst, addr, size) ) {
            tr_error("ERROR: QSPI Erase command failed!");
            erase_failed = true;
            status = QSPIF_BD_ERROR_DEVICE_ERROR;
            goto Exit_Point;
        }

        addr += chunk;
        size -= chunk;

        if ( (size > 0) && (addr > _region_high_boundary[region]) ) {
            // erase crossed to next region
            region++;
            bitfield = _region_erase_types_bitfield[region];
        }
        wait_ms(QSPIF_DEFAULT_TIMEOUT_MSEC);
        if ( false == _is_mem_ready()) {
            tr_error("ERROR: QSPI After Erase Device not ready - failed\n");
            erase_failed = true;
            status = QSPIF_BD_ERROR_READY_FAILED;
            goto Exit_Point;
        }

        _mutex->unlock();
    }

Exit_Point:
    if (erase_failed) {
        _mutex->unlock();
    }

    return status;
}



bd_size_t QSPIFBlockDevice::get_read_size() const
{
    // Assuming all devices support 1byte read granularity
    return QSPIF_DEFAULT_READ_SIZE;
}

bd_size_t QSPIFBlockDevice::get_program_size() const
{
    // Assuming all devices support 1byte program granularity
    return QSPIF_DEFAULT_PROG_SIZE;
}

bd_size_t QSPIFBlockDevice::get_erase_size() const
{
    // return minimal erase size supported by all regions (0 if none exists)
    return _min_common_erase_size;
}


// Find minimal erase size supported by the region to which the address belongs to
bd_size_t QSPIFBlockDevice::get_erase_size(bd_addr_t addr)
{
    // Find region of current address
    int region = _utils_find_addr_region(addr);

    int min_region_erase_size = _min_common_erase_size;
    int8_t type_mask = ERASE_BITMASK_TYPE1;
    int i_ind = 0;


    if (region != -1) {
        type_mask = 0x01;

        for (i_ind = 0; i_ind < 4; i_ind++) {
            // loop through erase types bitfield supported by region
            if (_region_erase_types_bitfield[region] & type_mask) {

                min_region_erase_size = _erase_type_size_arr[i_ind];
                break;
            }
            type_mask = type_mask << 1;
        }

        if (i_ind == 4) {
            tr_error("ERROR: no erase type was found for region addr");
        }
    }

    return (bd_size_t)min_region_erase_size;

}

bd_size_t QSPIFBlockDevice::size() const
{
    return _device_size_bytes;
}


/*********************************************************/
/********** SFDP Parsing and Detection Functions *********/
/*********************************************************/

int QSPIFBlockDevice::_sfdp_parse_sector_map_table(uint32_t sector_map_table_addr, size_t sector_map_table_size)
{
    uint8_t sector_map_table[SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES]; /* Up To 16 DWORDS = 64 Bytes */
    uint32_t tmp_region_size = 0;
    int i_ind = 0;
    int prev_boundary = 0;
    // Default set to all type bits 1-4 are common
    int min_common_erase_type_bits = ERASE_BITMASK_ALL;


    qspi_status_t status = _qspi_send_read_command(QSPIF_SFDP, (char *)sector_map_table, sector_map_table_addr /*address*/,
                           sector_map_table_size);
    if (status != QSPI_STATUS_OK) {
        tr_error("ERROR: init - Read SFDP First Table Failed");
        return -1;
    }

    // Currently we support only Single Map Descriptor
    if (! ( (sector_map_table[0] & 0x3) == 0x03 ) && (sector_map_table[1]  == 0x0) ) {
        tr_error("ERROR: Sector Map - Supporting Only Single! Map Descriptor (not map commands)");
        return -1;
    }

    _regions_count = sector_map_table[2] + 1;
    if (_regions_count > QSPIF_MAX_REGIONS) {
        tr_error("ERROR: Supporting up to %d regions, current setup to %d regions - fail",
                 QSPIF_MAX_REGIONS, _regions_count);
        return -1;
    }

    // Loop through Regions and set for each one: size, supported erase types, high boundary offset
    // Calculate minimum Common Erase Type for all Regions
    for (i_ind = 0; i_ind < _regions_count; i_ind++) {
        tmp_region_size = ((*((uint32_t *)&sector_map_table[(i_ind + 1) * 4])) >> 8) & 0x00FFFFFF; // bits 9-32
        _region_size_bytes[i_ind] = (tmp_region_size + 1) * 256; // Region size is 0 based multiple of 256 bytes;
        _region_erase_types_bitfield[i_ind] = sector_map_table[(i_ind + 1) * 4] & 0x0F; // bits 1-4
        min_common_erase_type_bits &= _region_erase_types_bitfield[i_ind];
        _region_high_boundary[i_ind] = (_region_size_bytes[i_ind] - 1) + prev_boundary;
        prev_boundary = _region_high_boundary[i_ind] + 1;
    }

    // Calc minimum Common Erase Size from min_common_erase_type_bits
    uint8_t type_mask = ERASE_BITMASK_TYPE1;
    for (i_ind = 0; i_ind < 4; i_ind++) {
        if (min_common_erase_type_bits & type_mask) {
            _min_common_erase_size = _erase_type_size_arr[i_ind];
            break;
        }
        type_mask = type_mask << 1;
    }

    if (i_ind == 4) {
        // No common erase type was found between regions
        _min_common_erase_size = 0;
    }

    return 0;
}


int QSPIFBlockDevice::_sfdp_parse_basic_param_table(uint32_t basic_table_addr, size_t basic_table_size)
{
    uint8_t param_table[SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES]; /* Up To 16 DWORDS = 64 Bytes */

    qspi_status_t status = _qspi_send_read_command(QSPIF_SFDP, (char *)param_table, basic_table_addr /*address*/,
                           basic_table_size);
    if (status != QSPI_STATUS_OK) {
        tr_error("ERROR: init - Read SFDP First Table Failed");
        return -1;
    }

    // Check address size, currently only supports 3byte addresses
    if ((param_table[2] & 0x4) != 0 || (param_table[7] & 0x80) != 0) {
        tr_error("ERROR: init - verify 3byte addressing Failed");
        return -1;
    }

    // Get device density (stored in bits - 1)
    uint32_t density_bits = (
                                (param_table[7] << 24) |
                                (param_table[6] << 16) |
                                (param_table[5] << 8 ) |
                                param_table[4] );
    _device_size_bytes = (density_bits + 1) / 8;

    // Set Default read/program/erase Instructions
    _read_instruction = QSPIF_READ;
    _prog_instruction = QSPIF_PP;
    _erase_instruction = QSPIF_SE;

    // Set Page Size (QSPI write must be done on Page limits)
    _page_size_bytes = _sfdp_detect_page_size(param_table);

    // Detect and Set Erase Types
    bool shouldSetQuadEnable = false;
    bool is_qpi_mode = false;
    _sfdp_detect_erase_types_inst_and_size(param_table, _erase4k_inst, _erase_type_inst_arr, _erase_type_size_arr);
    _erase_instruction = _erase4k_inst;


    // Detect and Set fastest Bus mode (default 1-1-1)
    _sfdp_detect_best_bus_read_mode(param_table, shouldSetQuadEnable, is_qpi_mode, _read_instruction);

    if (true == shouldSetQuadEnable) {
        // Set Quad Enable and QPI Bus modes if Supported
        tr_info("INFO: init - Setting Quad Enable");
        _sfdp_set_quad_enabled(param_table);
        if (true == is_qpi_mode) {
            tr_info("INFO: init - Setting QPI mode");
            _sfdp_set_qpi_enabled(param_table);
        }
    }
    return 0;
}

int QSPIFBlockDevice::_sfdp_parse_sfdp_headers(uint32_t& basic_table_addr, size_t& basic_table_size,
        uint32_t& sector_map_table_addr, size_t& sector_map_table_size)
{
    uint8_t sfdp_header[QSPIF_SFDP_HEADER_SIZE];
    uint8_t param_header[QSPIF_PARAM_HEADER_SIZE];
    size_t data_length = QSPIF_SFDP_HEADER_SIZE;
    bd_addr_t addr = 0x0;

    // Set 1-1-1 bus mode for SFDP header parsing
    _qspi_configure_format(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                           QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 8);

    qspi_status_t status = _qspi_send_read_command(QSPIF_SFDP, (char *)sfdp_header, addr /*address*/, data_length);
    if (status != QSPI_STATUS_OK) {
        tr_error("ERROR: init - Read SFDP Failed");
        return -1;
    }

    // Verify SFDP signature for sanity
    // Also check that major/minor version is acceptable
    if (!(memcmp(&sfdp_header[0], "SFDP", 4) == 0 && sfdp_header[5] == 1)) {
        tr_error("ERROR: init - _verify SFDP signature and version Failed");
        return -1;
    } else {
        tr_info("INFO: init - verified SFDP Signature and version Successfully");
    }

    // Discover Number of Parameter Headers
    int number_of_param_headers = (int)(sfdp_header[6]) + 1;
    tr_debug("DEBUG: number of Param Headers: %d", number_of_param_headers);


    addr += QSPIF_SFDP_HEADER_SIZE;
    data_length = QSPIF_PARAM_HEADER_SIZE;

    // Loop over Param Headers and parse them (currently supported Basic Param Table and Sector Region Map Table)
    for (int i_ind = 0; i_ind < number_of_param_headers; i_ind++) {

        status = _qspi_send_read_command(QSPIF_SFDP, (char *)param_header, addr, data_length);
        if (status != QSPI_STATUS_OK) {
            tr_error("ERROR: init - Read Param Table %d Failed", i_ind + 1);
            return -1;
        }

        // The SFDP spec indicates the standard table is always at offset 0
        // in the parameter headers, we check just to be safe
        if (param_header[2] != 1) {
            tr_error("ERROR: Param Table %d - Major Version should be 1!", i_ind + 1);
            return -1;
        }

        if ((param_header[0] == 0) && (param_header[7] == 0xFF)) {
            // Found Basic Params Table: LSB=0x00, MSB=0xFF
            tr_debug("DEBUG: Found Basic Param Table at Table: %d", i_ind + 1);
            basic_table_addr = ( (param_header[6] << 16) | (param_header[5] << 8) | (param_header[4]) );
            // Supporting up to 64 Bytes Table (16 DWORDS)
            basic_table_size = ((param_header[3] * 4) < SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES) ? (param_header[3] * 4) : 64;

        } else if ((param_header[0] == 81) && (param_header[7] == 0xFF)) {
            // Found Sector Map Table: LSB=0x81, MSB=0xFF
            tr_debug("DEBUG: Found Sector Map Table at Table: %d", i_ind + 1);
            sector_map_table_addr = ( (param_header[6] << 16) | (param_header[5] << 8) | (param_header[4]) );
            sector_map_table_size = param_header[3] * 4;

        }
        addr += QSPIF_PARAM_HEADER_SIZE;

    }
    return 0;
}



int QSPIFBlockDevice::_sfdp_set_qpi_enabled(uint8_t *basic_param_table_ptr)
{
    uint8_t config_reg[1];

    // QPI 4-4-4 Enable Procedure is specified in 5 Bits
    uint8_t en_seq_444_value = ( ((basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE] & 0xF0) >> 4) | ((
                                     basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE + 1] & 0x01) << 4 ));

    switch (en_seq_444_value) {
        case 1:
        case 2:
            tr_debug("DEBUG: _setQPIEnabled - send command 38h");
            _qspi_send_general_command(0x38, -1, NULL, 0, NULL, 0);
            break;

        case 4:
            tr_debug("DEBUG: _setQPIEnabled - send command 35h");
            _qspi_send_general_command(0x35, -1, NULL, 0, NULL, 0);
            break;

        case 8:
            tr_debug("DEBUG: _setQPIEnabled - set config bit 6 and send command 71h");
            _qspi_send_general_command(0x65, 0x800003, NULL, 0, (char *)config_reg, 1);
            config_reg[0] |= 0x40; //Set Bit 6
            _qspi_send_general_command(0x71, 0x800003, NULL, 0, (char *)config_reg, 1);
            break;

        case 16:
            tr_debug("DEBUG: _setQPIEnabled - reset config bits 0-7 and send command 61h");
            _qspi_send_general_command(0x65, -1, NULL, 0, (char *)config_reg, 1);
            config_reg[0] &= 0x7F; //Reset Bit 7 of CR
            _qspi_send_general_command(0x61, -1, NULL, 0, (char *)config_reg, 1);
            break;

        default:
            tr_warning("WARNING: _setQPIEnabled - Unsuported En Seq 444 configuration");
            break;

    }
    return 0;
}



int QSPIFBlockDevice::_sfdp_set_quad_enabled(uint8_t *basic_param_table_ptr)
{

    int sr_read_size = QSPI_MAX_STATUS_REGISTER_SIZE;
    int sr_write_size = QSPI_MAX_STATUS_REGISTER_SIZE;

    int status_reg_setup[QSPI_MAX_STATUS_REGISTER_SIZE];
    uint8_t status_reg[QSPI_MAX_STATUS_REGISTER_SIZE];
    unsigned int write_register_inst = QSPIF_WRSR;
    unsigned int read_register_inst = QSPIF_RDSR;

    // QUAD Enable procedure is specified by 3 bits
    uint8_t qer_value = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_QER_BYTE] & 0x70) >> 4;


    switch (qer_value) {
        case 0:
            tr_debug("DEBUG: Device Does not Have a QE Bit, continue based on Read Inst");
            return 0;

        case 1:
        case 4:
            status_reg_setup[0] = 0;
            status_reg_setup[1] = 0x02;  //Bit 1 of Status Reg 2
            tr_debug("DEBUG: Setting QE Bit, Bit 1 of Status Reg 2");
            break;

        case 2:
            status_reg_setup[0] = 0x40; // Bit 6 of Status Reg 1
            status_reg_setup[1] = 0;
            sr_write_size = 1;
            sr_read_size = 1;
            tr_debug("DEBUG: Setting QE Bit, Bit 6 of Status Reg 1");
            break;

        case 3:
            status_reg_setup[0] = 0x80; // Bit 7 of Status Reg 1
            status_reg_setup[1] = 0;
            sr_write_size = 1;
            sr_read_size = 1;
            write_register_inst = 0x3E;
            read_register_inst = 0x3F;
            tr_debug("DEBUG: Setting QE Bit, Bit 7 of Status Reg 1");
            break;
        case 5:
            status_reg_setup[0] = 0;
            status_reg_setup[1] = 0x02; //Bit 1 of status Reg 2
            read_register_inst = 0x35;
            sr_read_size = 1;
            tr_debug("DEBUG: Setting QE Bit, Bit 1 of Status Reg 2 -special read command");
            break;
        default:
            tr_warning("WARNING: _setQuadEnable - Unsuported QER configuration");
            break;


    }

    // Read Status Register
    if (QSPI_STATUS_OK == _qspi_send_general_command(read_register_inst, -1, NULL, 0, (char *)status_reg,
            sr_read_size) ) {  // store received values in status_value
        tr_debug("DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_reg[0]);
    } else {
        tr_error("ERROR: Reading Status Register failed");
        return -1;
    }

    // Set Bits for Quad Enable
    status_reg[0] |= status_reg_setup[0];
    status_reg[1] |= status_reg_setup[1];


    // Write new Status Register Setup
    if (_set_write_enable() != 0) {
        tr_error("ERROR: Write Enabe failed\n");
        return -1;
    }

    if ( false == _is_mem_ready()) {
        tr_error("ERROR: Device not ready, write failed");
        return -1;
    }

    if (QSPI_STATUS_OK == _qspi_send_general_command(write_register_inst, -1, (char *)status_reg, sr_write_size, NULL,
            0) ) {  // Write QE to status_register
        tr_debug("DEBUG: _setQuadEnable - Writing Status Register Success: value = 0x%x",
                 (int)status_reg[0]);
    } else {
        tr_error("ERROR: _setQuadEnable - Writing Status Register failed");
        return -1;
    }

    wait_ms(QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC);

    if ( false == _is_mem_ready()) {
        tr_error("ERROR: Device not ready after write, failed");
        return -1;
    }


    // For Debug
    memset(status_reg, 0, QSPI_MAX_STATUS_REGISTER_SIZE);
    if (QSPI_STATUS_OK == _qspi_send_general_command(read_register_inst, -1, NULL, 0, (char *)status_reg,
            sr_read_size) ) {  // store received values in status_value
        tr_debug("DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_reg[0]);
    } else {
        tr_error("ERROR: Reading Status Register failed");
        return -1;
    }


    return 0;
}


int QSPIFBlockDevice::_sfdp_detect_page_size(uint8_t *basic_param_table_ptr)
{
    // Page Size is specified by 4 Bits (N), calculated by 2^N
    int page_to_power_size = ( (int)basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_PAGE_SIZE_BYTE]) >> 4;
    int page_size = local_math_power(2, page_to_power_size);
    tr_debug("DEBUG: _detectPageSize - Page Size: %d", page_size);
    return page_size;
}



int QSPIFBlockDevice::_sfdp_detect_erase_types_inst_and_size(uint8_t *basic_param_table_ptr, unsigned int& erase4k_inst,
        unsigned int *erase_type_inst_arr, unsigned int *erase_type_size_arr)
{
    erase4k_inst = 0xff;
    bool found_4Kerase_type = false;
    uint8_t bitfield = 0x01;

    // Erase 4K Inst is taken either from param table legacy 4K erase or superseded by erase Instruction for type of size 4K
    erase4k_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_4K_ERASE_TYPE_BYTE];

    // Loop Erase Types 1-4
    for (int i_ind = 0; i_ind < 4; i_ind++) {
        erase_type_inst_arr[i_ind] = 0xff; //0xFF default for unsupported type
        erase_type_size_arr[i_ind] = local_math_power(2,
                                     basic_param_table_ptr[QSPIF_BASIC_PARAM_ERASE_TYPE_1_SIZE_BYTE + 2 * i_ind]); // Size given as 2^N
        tr_info("DEBUG: Erase Type(A) %d - Inst: 0x%xh, Size: %d", (i_ind + 1), erase_type_inst_arr[i_ind],
                erase_type_size_arr[i_ind]);
        if (erase_type_size_arr[i_ind] > 1) {
            // if size==1 type is not supported
            erase_type_inst_arr[i_ind] = basic_param_table_ptr[QSPIF_BASIC_PARAM_ERASE_TYPE_1_BYTE + 2 * i_ind];

            if ((erase_type_size_arr[i_ind] < _min_common_erase_size) || (_min_common_erase_size == 0) ) {
                //Set default minimal common erase for singal region
                _min_common_erase_size = erase_type_size_arr[i_ind];
            }

            // SFDP standard requires 4K Erase type to exist and its instruction to be identical to legacy field erase instruction
            if (erase_type_size_arr[i_ind] == 4096) {
                found_4Kerase_type = true;
                if (erase4k_inst != erase_type_inst_arr[i_ind]) {
                    //Verify 4KErase Type is identical to Legacy 4K erase type specified in Byte 1 of Param Table
                    erase4k_inst = erase_type_inst_arr[i_ind];
                    tr_warning("WARNING: _detectEraseTypesInstAndSize - Default 4K erase Inst is different than erase type Inst for 4K");

                }
            }
            _region_erase_types_bitfield[0] |= bitfield; // If there's no region map, set region "0" types bitfield as defualt;
        }

        tr_info("INFO: Erase Type %d - Inst: 0x%xh, Size: %d", (i_ind + 1), erase_type_inst_arr[i_ind],
                erase_type_size_arr[i_ind]);
        bitfield = bitfield << 1;
    }

    if (false == found_4Kerase_type) {
        tr_warning("WARNING: Couldn't find Erase Type for 4KB size");
    }
    return 0;
}


int QSPIFBlockDevice::_sfdp_detect_best_bus_read_mode(uint8_t *basic_param_table_ptr, bool& set_quad_enable,
        bool& is_qpi_mode,
        unsigned int& read_inst)
{

    bool is_done = false;

    set_quad_enable = false;
    is_qpi_mode = false;
    uint8_t examined_byte = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE];

    do { // compound statement is the loop body



        if (examined_byte & 0x10) {
            // QPI 4-4-4 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE];
            set_quad_enable = true;
            is_qpi_mode = true;
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE - 1] & 0x1F);
            tr_debug("/nDEBUG: Read Bus Mode set to 4-4-4, Instruction: 0x%xh", _read_instruction);
            //_inst_width = QSPI_CFG_BUS_QUAD;
            _address_width = QSPI_CFG_BUS_QUAD;
            _data_width = QSPI_CFG_BUS_QUAD;

            break;
        }


        examined_byte = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE];
        if (examined_byte & 0x40) {
            //  Fast Read 1-4-4 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE];
            set_quad_enable = true;
            // dummy cycles + mode cycles = Dummy Cycles
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_QUAD;
            _data_width = QSPI_CFG_BUS_QUAD;
            tr_debug("/nDEBUG: Read Bus Mode set to 1-4-4, Instruction: 0x%xh", _read_instruction);
            break;
        }
        if (examined_byte & 0x80) {
            //  Fast Read 1-1-4 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE];
            set_quad_enable = true;
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE - 1] & 0x1F);
            _data_width = QSPI_CFG_BUS_QUAD;
            tr_debug("/nDEBUG: Read Bus Mode set to 1-1-4, Instruction: 0x%xh", _read_instruction);
            break;
        }
        examined_byte = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE];
        if (examined_byte & 0x01) {
            //  Fast Read 2-2-2 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_DUAL;
            _data_width = QSPI_CFG_BUS_DUAL;
            tr_info("/nINFO: Read Bus Mode set to 2-2-2, Instruction: 0x%xh", _read_instruction);
            break;
        }

        examined_byte = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE];
        if (examined_byte & 0x20) {
            //  Fast Read 1-2-2 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_DUAL;
            _data_width = QSPI_CFG_BUS_DUAL;
            tr_debug("/nDEBUG: Read Bus Mode set to 1-2-2, Instruction: 0x%xh", _read_instruction);
            break;
        }
        if (examined_byte & 0x01) {
            // Fast Read 1-1-2 Supported
            read_inst = basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE - 1] >> 5)
                                     + (basic_param_table_ptr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE - 1] & 0x1F);
            _data_width = QSPI_CFG_BUS_DUAL;
            tr_debug("/nDEBUG: Read Bus Mode set to 1-1-2, Instruction: 0x%xh", _read_instruction);
            break;
        }
        tr_debug("/nDEBUG: Read Bus Mode set to 1-1-1, Instruction: 0x%xh", _read_instruction);
        is_done = true;
    } while (is_done == false);

    return 0;
}


int QSPIFBlockDevice::_reset_flash_mem()
{
    // Perform Soft Reset of the Device prior to initialization
    int status = 0;
    char status_value[2] = {0};
    tr_info("INFO: _reset_flash_mem:\n");
    //Read the Status Register from device
    if (QSPI_STATUS_OK == _qspi_send_general_command(QSPIF_RDSR, -1, NULL, 0, status_value,
            1) ) {  // store received values in status_value
        tr_debug("DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_value[0]);
    } else {
        tr_debug("ERROR: Reading Status Register failed\n");
        status = -1;
    }

    if (0 == status) {
        //Send Reset Enable
        if (QSPI_STATUS_OK == _qspi_send_general_command(QSPIF_RSTEN, -1, NULL, 0, NULL,
                0) ) {   // store received values in status_value
            tr_debug("DEBUG: Sending RSTEN Success\n");
        } else {
            tr_error("ERROR: Sending RSTEN failed\n");
            status = -1;
        }


        if (0 == status) {
            //Send Reset
            if (QSPI_STATUS_OK == _qspi_send_general_command(QSPIF_RST, -1, NULL, 0, NULL,
                    0)) {   // store received values in status_value
                tr_debug("DEBUG: Sending RST Success\n");
            } else {
                tr_error("ERROR: Sending RST failed\n");
                status = -1;
            }

            _is_mem_ready();
        }
    }

    return status;
}


bool QSPIFBlockDevice::_is_mem_ready()
{
    // Check Status Register Busy Bit to Verify the Device isn't Busy
    char status_value[2];
    int retries = 0;
    bool mem_ready = true;

    do {
        retries++;
        //Read the Status Register from device
        if (QSPI_STATUS_OK != _qspi_send_general_command(QSPIF_RDSR, -1, NULL, 0, status_value,
                2)) {   // store received values in status_value
            tr_error("ERROR: Reading Status Register failed\n");
        }
    } while ( (status_value[0] & 0x1) != 0 && retries < IS_MEM_READY_MAX_RETRIES );

    if ((status_value[0] & 0x1) != 0) {
        tr_error("ERROR: _is_mem_ready FALSE\n");
        mem_ready = false;
    }
    return mem_ready;
}


int QSPIFBlockDevice::_set_write_enable()
{
    int status = 0;
    if (QSPI_STATUS_OK !=  _qspi_send_general_command(QSPIF_WREN, -1, NULL, 0, NULL, 0)) {
        tr_error("ERROR:Sending WREN command FAILED\n");
        status = -1;
    }
    return status;
}



/*********************************************/
/************* Utility Functions *************/
/*********************************************/
int QSPIFBlockDevice::_utils_find_addr_region(bd_size_t offset)
{
    //Find the region to which the given offset belong to
    if ((offset > _device_size_bytes) || (_regions_count == 0)) {
        return -1;
    }

    if (_regions_count == 1) {
        return 0;
    }

    for (int i_ind = _regions_count - 2; i_ind >= 0; i_ind--) {

        if (offset > _region_high_boundary[i_ind]) {
            return (i_ind + 1);
        }
    }
    return -1;

}


int QSPIFBlockDevice::_utils_iterate_next_largest_erase_type(uint8_t& bitfield, int size, int offset, int boundry)
{
    // Iterate on all supported Erase Types of the Region to which the offset belong to.
    // Iterates from highest type to lowest
    uint8_t type_mask = ERASE_BITMASK_TYPE4;
    int i_ind  = 0;
    int largest_erase_type = 0;
    for (i_ind = 3; i_ind >= 0; i_ind--) {
        if (bitfield & type_mask) {
            largest_erase_type = i_ind;
            if ( (size > _erase_type_size_arr[largest_erase_type]) &&
                    ((boundry - offset) > _erase_type_size_arr[largest_erase_type]) ) {
                break;
            } else {
                bitfield &= ~type_mask;
            }
        }
        type_mask = type_mask >> 1;
    }

    if (i_ind == 4) {
        tr_error("ERROR: no erase type was found for current region addr");
    }
    return largest_erase_type;

}


/***************************************************/
/*********** QSPI Driver API Functions *************/
/***************************************************/

qspi_status_t QSPIFBlockDevice::_qsp_set_frequency(int freq)
{
    return _qspi.set_frequency(freq);
}


qspi_status_t QSPIFBlockDevice::_qspi_send_read_command(unsigned int read_inst, void *buffer, bd_addr_t addr,
        bd_size_t size)
{
    // Send Read command to device driver
    size_t buf_len = size;

    if (_qspi.read(read_inst, -1, (unsigned int )addr, (char *)buffer, &buf_len) != QSPI_STATUS_OK ) {
        tr_error("ERROR: Read failed");
        return QSPI_STATUS_ERROR;
    }

    return QSPI_STATUS_OK;

}


qspi_status_t QSPIFBlockDevice::_qspi_send_program_command(unsigned int progInst, const void *buffer, bd_addr_t addr,
        bd_size_t *size)
{
    // Send Program (write) command to device driver
    qspi_status_t result = QSPI_STATUS_OK;

    result = _qspi.write(progInst, -1, addr, (char *)buffer, (size_t *)size);
    if (result != QSPI_STATUS_OK) {
        tr_error("ERROR: QSPI Write failed");
    }

    return result;
}


qspi_status_t QSPIFBlockDevice::_qspi_send_erase_command(unsigned int eraseInst, bd_addr_t addr, bd_size_t size)
{
    // Send Erase Instruction command to driver
    qspi_status_t result = QSPI_STATUS_OK;

    tr_info("INFO: Inst: 0x%xh, addr: %llu, size: %llu", eraseInst, addr, size);

    result = _qspi.command_transfer(eraseInst, // command to send
                                    (((int)addr) & 0x00FFF000), // Align addr to 4096
                                    NULL,                 // do not transmit
                                    0,              // do not transmit
                                    NULL,                 // just receive two bytes of data
                                    0); // store received values in status_value

    if (QSPI_STATUS_OK != result) {
        tr_error("ERROR: QSPI Erase failed");
    }

    return result;

}


qspi_status_t QSPIFBlockDevice::_qspi_send_general_command(unsigned int instruction, bd_addr_t addr,
        const char *tx_buffer,
        size_t tx_length, const char *rx_buffer, size_t rx_length)
{
    // Send a general command Instruction to driver
    qspi_status_t status = _qspi.command_transfer(instruction, (int)addr, tx_buffer, tx_length, rx_buffer, rx_length);

    if (QSPI_STATUS_OK != status) {
        tr_error("ERROR:Sending Generic command: %x", instruction);
    }

    return status;
}


qspi_status_t QSPIFBlockDevice::_qspi_configure_format(qspi_bus_width_t inst_width, qspi_bus_width_t address_width,
        qspi_address_size_t address_size, qspi_bus_width_t alt_width, qspi_alt_size_t alt_size, qspi_bus_width_t data_width,
        int dummy_cycles)
{
    // Configure QSPI driver Bus format
    qspi_status_t status = _qspi.configure_format(inst_width, address_width, address_size, alt_width, alt_size, data_width,
                           dummy_cycles);

    return status;
}


/*********************************************/
/************** Local Functions **************/
/*********************************************/
static int local_math_power(int base, int exp)
{
    // Integer X^Y function, used to calculate size fields given in 2^N format
    int result = 1;
    while (exp) {
        result *= base;
        exp--;
    }
    return result;
}

} //namespace mbed



