/*
    Driver for the W25Q128JV Flash Memory.
    Basic implementation for initialization, read, write, and erase operations.

    Reference: Winbond W25Q128JV Datasheet Rev. M
    Device: 128 Mbit (16 MB) flash memory
    - 65,536 pages (256 bytes each)
    - 4,096 sectors (4 KB each)
    - 256 blocks (64 KB each)

    @ UBC Rocket
*/

#ifndef W25Q128_FLASH_H
#define W25Q128_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"

#define W25Q128_CMD_WRITE_ENABLE         0x06
#define W25Q128_CMD_WRITE_DISABLE        0x04
#define W25Q128_CMD_READ_STATUS_REG1     0x05
#define W25Q128_CMD_READ_STATUS_REG2     0x35
#define W25Q128_CMD_READ_STATUS_REG3     0x15
#define W25Q128_CMD_WRITE_STATUS_REG1    0x01
#define W25Q128_CMD_WRITE_STATUS_REG2    0x31
#define W25Q128_CMD_WRITE_STATUS_REG3    0x11
#define W25Q128_CMD_PAGE_PROGRAM         0x32
#define W25Q128_CMD_SECTOR_ERASE         0x20
#define W25Q128_CMD_BLOCK_ERASE_32K      0x52
#define W25Q128_CMD_BLOCK_ERASE_64K      0xD8
#define W25Q128_CMD_CHIP_ERASE           0xC7
#define W25Q128_CMD_READ_DATA            0x03
#define W25Q128_CMD_FAST_READ            0x0B
#define W25Q128_CMD_READ_JEDEC_ID        0x9F
#define W25Q128_CMD_READ_UNIQUE_ID       0x4B
#define W25Q128_CMD_POWER_DOWN           0xB9
#define W25Q128_CMD_RELEASE_POWER_DOWN   0xAB

typedef struct {
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity;
} jedec_id_t;

typedef enum {
    W25Q128_STATUS_REG_1 = 1,
    W25Q128_STATUS_REG_2 = 2,
    W25Q128_STATUS_REG_3 = 3,
} w25q128_status_reg_t;

typedef struct {
    bool srp;  ///< Status Register Protect
    bool sec;  ///< Sector/Block Protect bit
    bool tb;   ///< Top/Bottom Block Protect
    bool bp2;  ///< Block Protect Bits (2)
    bool bp1;  ///< Block Protect Bits (1)
    bool bp0;  ///< Block Protect Bits (0)
    bool wel;  ///< Write Enable Latch
    bool busy; ///< Erase/Write In Progress
} w25q128_status_reg_1_t;

typedef struct {
    bool sus; ///< Erase/Program Suspend Status
    bool cmp; ///< Complement Protect
    bool lb3; ///< Security Register Lock Bits (3)
    bool lb2; ///< Security Register Lock Bits (2)
    bool lb1; ///< Security Register Lock Bits (1)
    bool qe;  ///< Quad Enable
    bool srl; ///< Status Register Lock
} w25q128_status_reg_2_t;

typedef struct {
    bool drv1; ///< Output Driver Strength (1)
    bool drv0; ///< Output Driver Strength (0)
    bool wps;  ///< Write Protect Selection
} w25q128_status_reg_3_t;

typedef struct {
    uint64_t               unique_id;    ///< 64-bit unique ID
    w25q128_status_reg_1_t status_reg1;  ///< Status Register 1 cache
    w25q128_status_reg_2_t status_reg2;  ///< Status Register 2 cache
    w25q128_status_reg_3_t status_reg3;  ///< Status Register 3 cache
    jedec_id_t             jedec_id;     ///< JEDEC ID
    bool                   initialized;  ///< Initialization status
} w25q128_t;

/**
 * @brief Command configuration for W25Q128 with XSPI/OCTOSPI
 * This struct contains the XSPI command parameters and optional TX data buffer
 */
typedef struct {
    XSPI_RegularCmdTypeDef cmd; ///< XSPI command configuration
    const uint8_t *tx_data;     ///< Optional TX data buffer (NULL if no data to send)
    uint16_t tx_data_size;      ///< Size of TX data buffer (0 if no data)
} w25q128_xspi_cmd_t;

/**
 * @brief Build command to read JEDEC ID (3 bytes: Manufacturer, Memory Type, Capacity)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_read_jedec_id(w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build command to read Unique ID (64-bit)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_read_unique_id(w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Write Enable command
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_write_enable(w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Write Disable command
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_write_disable(w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build command to read Status Register
 * @param reg_num Register number
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_read_status_reg(w25q128_status_reg_t reg_num, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build command to read data (standard read, up to 50 MHz)
 * @param address 24-bit address to read from
 * @param num_bytes Number of bytes to read
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_read_data(uint32_t address, uint32_t num_bytes, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build command to read data (fast read, up to 104 MHz, requires 1 dummy byte)
 * @param address 24-bit address to read from
 * @param num_bytes Number of bytes to read
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_fast_read(uint32_t address, uint32_t num_bytes, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Page Program command (writes up to 256 bytes)
 * @param address 24-bit address to write to (must be within same page)
 * @param data Pointer to data to write
 * @param num_bytes Number of bytes to write (max 256)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_page_program(uint32_t address, const uint8_t *data,
                                uint16_t num_bytes, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Sector Erase command (erases 4 KB)
 * @param sector_address Sector address (0 to 4095)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_sector_erase(uint32_t sector_address, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Block Erase command (erases 64 KB)
 * @param block_address Block address (0 to 255)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_block_erase(uint32_t block_address, w25q128_xspi_cmd_t *cmd);

/**
 * @brief Build Chip Erase command (erases entire chip)
 * @param cmd Pointer to command structure to populate
 */
void w25q128_build_chip_erase(w25q128_xspi_cmd_t *cmd);

/* -------------------------------------------------------------------------- */
/* Response parsers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse JEDEC ID response
 * @param rx_buf Receive buffer from SPI transaction
 * @param dev Device structure to update
 */
void w25q128_parse_jedec_id(const uint8_t *rx_buf, w25q128_t *dev);

/**
 * @brief Parse Unique ID response
 * @param rx_buf Receive buffer from SPI transaction
 * @param dev Device structure to update
 */
void w25q128_parse_unique_id(const uint8_t *rx_buf, w25q128_t *dev);

/**
 * @brief Parse Status Register response
 * @param rx_buf Receive buffer from SPI transaction
 * @param reg_num Register number
 * @param dev Device structure to update
 */
void w25q128_parse_status_reg(const uint8_t *rx_buf, w25q128_status_reg_t reg_num, w25q128_t *dev);

/**
 * @brief Parse read data response
 * @param rx_buf Receive buffer from SPI transaction
 * @param data_out Output buffer for read data
 * @param num_bytes Number of bytes to copy
 */
void w25q128_parse_read_data(const uint8_t *rx_buf, uint8_t *data_out, uint16_t num_bytes);

/**
 * @brief Check if device is busy (polling status register)
 * @param dev The device structure
 * @return true if busy, false if ready
 */
bool w25q128_is_busy(w25q128_t *dev);

/**
 * @brief Check if write enable latch is set
 * @param dev The device structure
 * @return true if write enabled, false otherwise
 */
bool w25q128_is_write_enabled(w25q128_t *dev);

#endif // W25Q128_FLASH_H
