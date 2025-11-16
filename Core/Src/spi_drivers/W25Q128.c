/*
    W25Q128JV Flash Memory Driver Implementation
    Command builders for XSPI/OCTOSPI interface (bus-agnostic)
    
    @ UBC Rocket
*/

#include "W25Q128.h"
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
    cmd->cmd.DataLength = 3;  // 3 bytes: Manufacturer, Memory Type, Capacity
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_read_unique_id(w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));
    
    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_READ_UNIQUE_ID;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    
    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = 8;  // 8 bytes for unique ID
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 32;  // 4 dummy bytes = 32 dummy cycles
    
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
    cmd->cmd.DummyCycles = 0;
    
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
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_read_status_reg(uint8_t reg_num, w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));
    
    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    
    switch (reg_num) {
        case 1:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG1;
            break;
        case 2:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG2;
            break;
        case 3:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG3;
            break;
        default:
            cmd->cmd.Instruction = W25Q128_CMD_READ_STATUS_REG1;
            break;
    }
    
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    
    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = 1;  // 1 byte status register
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_read_data(uint32_t address, uint32_t num_bytes, w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));
    
    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_READ_DATA;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.Address = address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    
    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = num_bytes;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_fast_read(uint32_t address, uint32_t num_bytes, w25q128_xspi_cmd_t *cmd)
{
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));
    
    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_FAST_READ;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.Address = address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    
    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = num_bytes;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 8;  // 1 dummy byte = 8 dummy cycles
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_page_program(uint32_t address, const uint8_t *data, 
                                uint16_t num_bytes, w25q128_xspi_cmd_t *cmd)
{
    // Limit to page size (256 bytes)
    if (num_bytes > 256) {
        num_bytes = 256;
    }
    
    memset(&cmd->cmd, 0, sizeof(XSPI_RegularCmdTypeDef));
    
    cmd->cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->cmd.Instruction = W25Q128_CMD_PAGE_PROGRAM;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.Address = address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    
    cmd->cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd->cmd.DataLength = num_bytes;
    cmd->cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->cmd.DummyCycles = 0;
    
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
    cmd->cmd.Instruction = W25Q128_CMD_SECTOR_ERASE;
    cmd->cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    
    cmd->cmd.Address = byte_address;
    cmd->cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd->cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd->cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    
    cmd->cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd->cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}

void w25q128_build_block_erase(uint32_t block_address, w25q128_xspi_cmd_t *cmd)
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
    cmd->cmd.DummyCycles = 0;
    
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
    cmd->cmd.DummyCycles = 0;
    
    cmd->cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    cmd->cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
    
    cmd->tx_data = NULL;
    cmd->tx_data_size = 0;
}
