#ifndef ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_PACKET
#define ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_PACKET

#include <stdint.h>

#include "ulysses/lib/buffer.h"
#include "ulysses/status.h"

#define DYNAMIXEL_PROTOCOL2_BROADCAST_ID (0xFE)
#define DYNAMIXEL_PROTOCOL2_MAX_PARAMETERS_COUNT (100)
#define DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE (7)

enum dynamixel_protocol2_instruction {
    DYNAMIXEL_PROTOCOL2_INST_PING = 0x01,                 ///< Read device model and firmware
    DYNAMIXEL_PROTOCOL2_INST_READ = 0x02,                 ///< Read control table bytes
    DYNAMIXEL_PROTOCOL2_INST_FACTORY_RESET = 0x06,        ///< Restore factory settings
    DYNAMIXEL_PROTOCOL2_INST_REBOOT = 0x08,               ///< Reboot a device
    DYNAMIXEL_PROTOCOL2_INST_CLEAR = 0x10,                ///< Clear selected device data
    DYNAMIXEL_PROTOCOL2_INST_CONTROL_TABLE_BACKUP = 0x20, ///< Back up or restore registers
    DYNAMIXEL_PROTOCOL2_INST_STATUS = 0x55,               ///< Status reply opcode
    DYNAMIXEL_PROTOCOL2_INST_SYNC_READ = 0x82,            ///< Read common registers across devices
    DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE = 0x83,           ///< Write common registers across devices
    DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ = 0x8A,       ///< Sync Read with a combined reply
    DYNAMIXEL_PROTOCOL2_INST_BULK_READ = 0x92,            ///< Read different device registers
    DYNAMIXEL_PROTOCOL2_INST_BULK_WRITE = 0x93,           ///< Write different device registers
    DYNAMIXEL_PROTOCOL2_INST_FAST_BULK_READ = 0x9A,       ///< Bulk Read with a combined reply
};

/**
 * @brief Size the output buffer for dynamixel_protocol2_encode_request().
 * @param parameters_length Serialized parameter byte count
 * @return Minimum capacity in bytes, including room for worst-case byte stuffing
 */
static inline uint32_t dynamixel_protocol2_required_packet_capacity(uint16_t parameters_length)
{
    return 10 + parameters_length + parameters_length / 3;
}

/**
 * @brief Read the packet length field.
 * @param packet Packet buffer with the seven-byte prefix available
 * @return Number of bytes to read after the prefix, including the CRC
 */
static inline uint16_t dynamixel_protocol2_read_packet_length(const uint8_t *packet)
{
    return buffer_read_uint16_le(
        &packet[DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE - sizeof(uint16_t)]);
}

/**
 * @brief Encode a Protocol 2.0 instruction packet.
 *
 * @param id Target device ID or DYNAMIXEL_PROTOCOL2_BROADCAST_ID. Sync and Bulk instructions,
 *           including their Fast variants, require the broadcast ID.
 * @param instruction Request opcode
 * @param parameters Serialized parameters. May be NULL for a request with no parameters.
 * @param parameters_length Number of serialized parameter bytes
 * @param[out] packet Encoded packet on success
 * @param capacity Output buffer capacity in bytes. Use
 *                 dynamixel_protocol2_required_packet_capacity() to size the buffer.
 * @param[out] packet_size Number of encoded bytes on success
 * @retval ULS_STATUS_OK Packet is ready to transmit
 * @retval ULS_STATUS_NO_SPACE Packet buffer is too small. Both outputs stay untouched.
 */
enum uls_status dynamixel_protocol2_encode_request(uint8_t id, uint8_t instruction,
                                                   const uint8_t *restrict parameters,
                                                   uint16_t parameters_length,
                                                   uint8_t *restrict packet, uint32_t capacity,
                                                   uint32_t *packet_size);

/**
 * @brief Decode one complete Protocol 2.0 status packet.
 *
 * Pass the decoded payload to the instruction-specific deserializer.
 *
 * @param packet Received packet, including header and CRC
 * @param packet_size Number of bytes in the received packet
 * @param[out] id Packet ID on success
 * @param[out] parameters Decoded payload, including the device error byte
 * @param capacity Output capacity in bytes. For a complete packet, use packet_size minus 10 bytes
 *                 to fit the payload before unstuffing.
 * @param[out] parameters_length Number of decoded bytes on success
 * @retval ULS_STATUS_OK Packet decoded. The device error byte can be nonzero.
 * @retval ULS_STATUS_NO_SPACE Parameter buffer is too small. All outputs stay untouched.
 * @retval ULS_STATUS_INVALID_INPUT Packet is malformed or truncated. Invalid byte stuffing can
 *                                 leave partial parameters. ID and length stay untouched.
 */
enum uls_status dynamixel_protocol2_decode_response(const uint8_t *restrict packet,
                                                    uint32_t packet_size, uint8_t *id,
                                                    uint8_t *restrict parameters, uint16_t capacity,
                                                    uint16_t *parameters_length);

#endif // ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_PACKET
