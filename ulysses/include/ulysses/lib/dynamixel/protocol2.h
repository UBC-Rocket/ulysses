#ifndef ULYSSES_LIB_DYNAMIXEL_PROTOCOL2
#define ULYSSES_LIB_DYNAMIXEL_PROTOCOL2

#include <stdint.h>

#define DYNAMIXEL_PROTOCOL2_BROADCAST_ID (0xFE)

enum dynamixel_protocol2_instruction {
    DYNAMIXEL_PROTOCOL2_INST_PING = 0x01,                 ///< Read device model and firmware
    DYNAMIXEL_PROTOCOL2_INST_READ = 0x02,                 ///< Read control table bytes
    DYNAMIXEL_PROTOCOL2_INST_WRITE = 0x03,                ///< Write control table bytes
    DYNAMIXEL_PROTOCOL2_INST_REG_WRITE = 0x04,            ///< Stage a write until Action
    DYNAMIXEL_PROTOCOL2_INST_ACTION = 0x05,               ///< Execute a staged write
    DYNAMIXEL_PROTOCOL2_INST_FACTORY_RESET = 0x06,        ///< Restore factory settings
    DYNAMIXEL_PROTOCOL2_INST_REBOOT = 0x08,               ///< Reboot a device
    DYNAMIXEL_PROTOCOL2_INST_CLEAR = 0x10,                ///< Clear selected device data
    DYNAMIXEL_PROTOCOL2_INST_CONTROL_TABLE_BACKUP = 0x20, ///< Back up or restore registers
    DYNAMIXEL_PROTOCOL2_INST_STATUS = 0x55,     ///< REPLY INSTRUCTION ONLY! Device status reply
    DYNAMIXEL_PROTOCOL2_INST_SYNC_READ = 0x82,  ///< Read common registers across devices
    DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE = 0x83, ///< Write common registers across devices
    // DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ = 0x8A, ///< Sync Read with a combined reply
    DYNAMIXEL_PROTOCOL2_INST_BULK_READ = 0x92,  ///< Read different device registers
    DYNAMIXEL_PROTOCOL2_INST_BULK_WRITE = 0x93, ///< Write different device registers
    // DYNAMIXEL_PROTOCOL2_INST_FAST_BULK_READ = 0x9A, ///< Bulk Read with a combined reply
};

enum dynamixel_protocol2_status {
    DYNAMIXEL_PROTOCOL2_OK,             ///< Packet encoded or decoded successfully
    DYNAMIXEL_PROTOCOL2_NO_SPACE,       ///< Output buffer or protocol length limit exceeded
    DYNAMIXEL_PROTOCOL2_INVALID_PACKET, ///< Received packet has invalid framing, stuffing, or CRC
};

/**
 * @brief Encode one Protocol 2.0 instruction packet, including byte stuffing and CRC
 *
 * No allocation or I/O is performed. Parameter data must already be in wire order
 * (multi-byte register addresses and values are little endian). Source and output
 * buffers must not overlap. The packet buffer may be partially written on failure.
 * packet_size is only written on success.
 * Full packet sizes include the seven-byte prefix outside the 16-bit length field.
 *
 * @param id Device ID (0..252), or DYNAMIXEL_PROTOCOL2_BROADCAST_ID
 * @param instruction Controller opcode from enum dynamixel_protocol2_instruction; excludes Status
 * @param parameters Parameter bytes; may be NULL when parameters_length is zero
 * @param parameters_length Length of parameter data in bytes, before stuffing
 * @param[out] packet Non-NULL output buffer; at most
 *                    parameters_length + parameters_length / 3 + 11 bytes are needed,
 *                    including framing and worst-case stuffing
 * @param capacity Capacity of packet in bytes
 * @param[out] packet_size Non-NULL output for the complete encoded packet size
 * @retval DYNAMIXEL_PROTOCOL2_OK Packet is ready to transmit
 * @retval DYNAMIXEL_PROTOCOL2_NO_SPACE Output buffer or 16-bit packet length limit exceeded
 */
enum dynamixel_protocol2_status
dynamixel_protocol2_encode_request(uint8_t id, uint8_t instruction, const uint8_t *parameters,
                                   uint16_t parameters_length, uint8_t *packet, uint32_t capacity,
                                   uint32_t *packet_size);

/**
 * @brief Validate and decode one complete, ordinary Protocol 2.0 status packet
 *
 * The caller frames the UART stream and passes exactly one packet, with no leading
 * noise or trailing bytes. CRC is checked before removing byte stuffing. Fast Sync
 * Read and Fast Bulk Read responses use a different format and are not supported.
 * Source and output buffers must not overlap. The parameter buffer may be partially
 * written on failure. Other outputs are only written on success.
 * A nonzero device error still returns DYNAMIXEL_PROTOCOL2_OK.
 *
 * @param packet Non-NULL received packet, including header and CRC
 * @param packet_size Number of received bytes
 * @param[out] id Non-NULL output for the responding device ID
 * @param[out] error Non-NULL output for the raw error byte, including the alert bit (bit 7)
 * @param[out] parameters Output for unstuffed parameters; may be NULL when capacity is zero
 * @param capacity Capacity of parameters in bytes
 * @param[out] parameters_length Non-NULL output for the decoded parameter data length in bytes
 * @retval DYNAMIXEL_PROTOCOL2_OK Packet decoded; inspect error for the device result
 * @retval DYNAMIXEL_PROTOCOL2_NO_SPACE Parameter buffer is too small
 * @retval DYNAMIXEL_PROTOCOL2_INVALID_PACKET Framing, length, ID, instruction, stuffing, or CRC
 *                                          is invalid. Truncated packets are also invalid
 */
enum dynamixel_protocol2_status
dynamixel_protocol2_decode_response(const uint8_t *packet, uint32_t packet_size, uint8_t *id,
                                    uint8_t *error, uint8_t *parameters, uint16_t capacity,
                                    uint16_t *parameters_length);

#endif // ULYSSES_LIB_DYNAMIXEL_PROTOCOL2
