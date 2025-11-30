/*
    W25Q128JV Flash Memory Driver Implementation
    Command builders for XSPI/OCTOSPI interface (bus-agnostic)

    @ UBC Rocket
*/

#include "W25Q128.h"
#include "stm32h5xx_hal_xspi.h"
#include <stdint.h>
#include <string.h>

void w25q128_build_read_jedec_id(w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_READ_JEDEC_ID;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = W25Q128_DATA_BYTES_READ_JEDEC_ID;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_READ_JEDEC_ID;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_write_enable(w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_WRITE_ENABLE;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DataLength = W25Q128_DATA_BYTES_WRITE_ENABLE;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_WRITE_ENABLE;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_write_disable(w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_WRITE_DISABLE;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DataLength = W25Q128_DATA_BYTES_WRITE_DISABLE;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_WRITE_DISABLE;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_read_status_reg(w25q128_status_reg_t reg_num, w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;

    switch (reg_num) {
        case W25Q128_STATUS_REG_1:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG1;
            cmd->cmd.DataLength = W25Q128_DATA_BYTES_READ_STATUS_REG1;
            cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_READ_STATUS_REG1;
            break;
        case W25Q128_STATUS_REG_2:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG2;
            cmd->cmd.DataLength = W25Q128_DATA_BYTES_READ_STATUS_REG2;
            cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_READ_STATUS_REG2;
            break;
        case W25Q128_STATUS_REG_3:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG3;
            cmd->cmd.DataLength = W25Q128_DATA_BYTES_READ_STATUS_REG3;
            cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_READ_STATUS_REG3;
            break;
    }

    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_fast_read_quad_io(uint32_t address, uint32_t num_bytes, w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_FAST_READ_QUAD_IO;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.Address = address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;

    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_4_LINES;
    cmd->cmd.DataLength = num_bytes;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_FAST_READ_QUAD_IO;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_quad_input_page_program(uint32_t address, const uint8_t *data,
                                           uint16_t num_bytes, w25q128_xspi_cmd_t *cmd)
{
    // Limit to page size (256 bytes)
    if (num_bytes > 256) {
        num_bytes = 256;
    }

    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_QUAD_INPUT_PAGE_PROGRAM;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.Address = address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;

    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_4_LINES;
    cmd->cmd.DataLength = num_bytes;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_QUAD_INPUT_PAGE_PROGRAM;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = data;
    cmd->tx_data_size = num_bytes;
}

void w25q128_build_sector_erase(uint32_t sector_address, w25q128_xspi_cmd_t *cmd)
{
    // Convert sector address to byte address
    uint32_t byte_address = sector_address * 4096;  // 4KB per sector

    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_SECTOR_ERASE_4K;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.Address = byte_address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;

    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DataLength = W25Q128_DATA_BYTES_SECTOR_ERASE_4K;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_SECTOR_ERASE_4K;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_block_erase_64k(uint32_t block_address, w25q128_xspi_cmd_t *cmd)
{
    // Convert block address to byte address
    uint32_t byte_address = block_address * 65536;  // 64KB per block

    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_BLOCK_ERASE_64K;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.Address = byte_address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;

    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_BLOCK_ERASE_64K;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_chip_erase(w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));

    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_CHIP_ERASE;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DummyCycles = W25Q128_DUMMY_CYCLES_CHIP_ERASE;

    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

/* -------------------------------------------------------------------------- */
/* Response parsers                                                           */
/* -------------------------------------------------------------------------- */

static w25q128_status_reg_1_t parse_status_reg_1(const uint8_t *rx_buf)
{
    const uint8_t status_byte = rx_buf[0];

    w25q128_status_reg_1_t status_reg = {
        .srp  = status_byte & (1 << 7),
        .sec  = status_byte & (1 << 6),
        .tb   = status_byte & (1 << 5),
        .bp2  = status_byte & (1 << 4),
        .bp1  = status_byte & (1 << 3),
        .bp0  = status_byte & (1 << 2),
        .wel  = status_byte & (1 << 1),
        .busy = status_byte & (1 << 0),
    };

    return status_reg;
}

static w25q128_status_reg_2_t parse_status_reg_2(const uint8_t *rx_buf)
{
    const uint8_t status_byte = rx_buf[0];

    w25q128_status_reg_2_t status_reg = {
        .sus = status_byte & (1 << 7),
        .cmp = status_byte & (1 << 6),
        .lb3 = status_byte & (1 << 5),
        .lb2 = status_byte & (1 << 4),
        .lb1 = status_byte & (1 << 3),
        .qe  = status_byte & (1 << 1),
        .srl = status_byte & (1 << 0),
    };

    return status_reg;
}

static w25q128_status_reg_3_t parse_status_reg_3(const uint8_t *rx_buf)
{
    const uint8_t status_byte = rx_buf[0];

    w25q128_status_reg_3_t status_reg = {
        .drv1 = status_byte & (1 << 6),
        .drv0 = status_byte & (1 << 5),
        .wps  = status_byte & (1 << 2),
    };

    return status_reg;
}

void w25q128_parse_jedec_id(const uint8_t *rx_buf, w25q128_t *dev)
{
    dev->jedec_id.manufacturer_id = rx_buf[0];
    dev->jedec_id.memory_type = rx_buf[1];
    dev->jedec_id.capacity = rx_buf[2];
}

void w25q128_parse_status_reg(const uint8_t *rx_buf, w25q128_status_reg_t reg_num, w25q128_t *dev)
{
    switch (reg_num)
    {
        case W25Q128_STATUS_REG_1:
            dev->status_reg1 = parse_status_reg_1(rx_buf);
            break;
        case W25Q128_STATUS_REG_2:
            dev->status_reg2 = parse_status_reg_2(rx_buf);
            break;
        case W25Q128_STATUS_REG_3:
            dev->status_reg3 = parse_status_reg_3(rx_buf);
            break;
    }
}

bool w25q128_is_busy(w25q128_t *dev)
{
    return dev->status_reg1.busy;
}

bool w25q128_is_write_enabled(w25q128_t *dev)
{
    return dev->status_reg1.wel;
}
