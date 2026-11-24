#include "ulysses/lib/dynamixel/protocol2_packet.h"

#include <stdbool.h>
#include <string.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"
#include "ulysses/lib/checksum/crc.h"

// Packet processing: https://emanual.robotis.com/docs/en/dxl/protocol2/#packet-process

#define STATUS_PAYLOAD_OFFSET (8)
#define CRC_LENGTH (2)
#define MAX_DEVICE_ID (0xFC)

static const uint8_t header[] = {0xFF, 0xFF, 0xFD, 0x00};

static inline bool is_header(const uint8_t *data)
{
    return data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFD;
}

static inline bool is_valid_instruction(uint8_t instruction)
{
    switch (instruction) {
    case DYNAMIXEL_PROTOCOL2_INST_PING:
    case DYNAMIXEL_PROTOCOL2_INST_READ:
    case DYNAMIXEL_PROTOCOL2_INST_FACTORY_RESET:
    case DYNAMIXEL_PROTOCOL2_INST_REBOOT:
    case DYNAMIXEL_PROTOCOL2_INST_CLEAR:
    case DYNAMIXEL_PROTOCOL2_INST_CONTROL_TABLE_BACKUP:
    case DYNAMIXEL_PROTOCOL2_INST_SYNC_READ:
    case DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE:
    case DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ:
    case DYNAMIXEL_PROTOCOL2_INST_BULK_READ:
    case DYNAMIXEL_PROTOCOL2_INST_BULK_WRITE:
    case DYNAMIXEL_PROTOCOL2_INST_FAST_BULK_READ:
        return true;
    default:
    case DYNAMIXEL_PROTOCOL2_INST_STATUS:
        return false;
    }
}

enum uls_status dynamixel_protocol2_encode_request(uint8_t id, uint8_t instruction,
                                                   const uint8_t *restrict parameters,
                                                   uint16_t parameters_length,
                                                   uint8_t *restrict packet, uint32_t capacity,
                                                   uint32_t *packet_size)
{
    ULYSSES_ASSERT(id <= MAX_DEVICE_ID || id == DYNAMIXEL_PROTOCOL2_BROADCAST_ID,
                   "invalid dynamixel instruction ID");
    ULYSSES_ASSERT(is_valid_instruction(instruction), "invalid dynamixel instruction");
    if (instruction == DYNAMIXEL_PROTOCOL2_INST_SYNC_READ ||
        instruction == DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE ||
        instruction == DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ ||
        instruction == DYNAMIXEL_PROTOCOL2_INST_BULK_READ ||
        instruction == DYNAMIXEL_PROTOCOL2_INST_BULK_WRITE ||
        instruction == DYNAMIXEL_PROTOCOL2_INST_FAST_BULK_READ) {
        ULYSSES_ASSERT(id == DYNAMIXEL_PROTOCOL2_BROADCAST_ID,
                       "dynamixel sync and bulk instructions require the broadcast ID");
    }
    ULYSSES_ASSERT(parameters_length <= DYNAMIXEL_PROTOCOL2_MAX_PARAMETERS_COUNT,
                   "dynamixel parameters exceed the supported limit");
    ULYSSES_ASSERT(parameters != NULL || parameters_length == 0,
                   "dynamixel parameters must not be NULL for nonzero length");
    ULYSSES_ASSERT(packet != NULL, "dynamixel packet output must not be NULL");
    ULYSSES_ASSERT(packet_size != NULL, "dynamixel packet size output must not be NULL");

    if (capacity < dynamixel_protocol2_required_packet_capacity(parameters_length)) {
        return ULS_STATUS_NO_SPACE;
    }

    memcpy(packet, header, sizeof(header));
    packet[4] = id;
    packet[7] = instruction;

    uint16_t ptr = 8;
    for (uint16_t i = 0; i < parameters_length; i++) {
        packet[ptr++] = parameters[i];

        if (i >= 2 && is_header(&parameters[i - 2])) {
            packet[ptr++] = 0xFD;
        }
    }

    buffer_write_uint16_le(&packet[DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE - sizeof(uint16_t)],
                           ptr - DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + CRC_LENGTH);
    buffer_write_uint16_le(&packet[ptr], checksum_crc16_umts(packet, ptr));
    *packet_size = ptr + CRC_LENGTH;

    return ULS_STATUS_OK;
}

enum uls_status dynamixel_protocol2_decode_response(const uint8_t *restrict packet,
                                                    uint32_t packet_size, uint8_t *id,
                                                    uint8_t *restrict parameters, uint16_t capacity,
                                                    uint16_t *parameters_length)
{
    ULYSSES_ASSERT(packet != NULL, "dynamixel packet must not be NULL");
    ULYSSES_ASSERT(id != NULL, "dynamixel ID output must not be NULL");
    ULYSSES_ASSERT(parameters != NULL || capacity == 0,
                   "dynamixel parameters output must not be NULL for nonzero capacity");
    ULYSSES_ASSERT(parameters_length != NULL,
                   "dynamixel parameters length output must not be NULL");

    if (packet_size <= STATUS_PAYLOAD_OFFSET + CRC_LENGTH ||
        memcmp(packet, header, sizeof(header)) != 0 ||
        dynamixel_protocol2_read_packet_length(packet) !=
            packet_size - DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE ||
        packet[7] != DYNAMIXEL_PROTOCOL2_INST_STATUS) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint32_t crc_offset = packet_size - CRC_LENGTH;
    uint16_t wire_parameters_length = crc_offset - STATUS_PAYLOAD_OFFSET;
    uint8_t packet_id = packet[4];
    bool fast_instruction = packet_id == DYNAMIXEL_PROTOCOL2_BROADCAST_ID;
    if ((!fast_instruction && packet_id > MAX_DEVICE_ID) ||
        (fast_instruction && wire_parameters_length < 2)) {
        return ULS_STATUS_INVALID_INPUT;
    }

    if (capacity < wire_parameters_length) {
        return ULS_STATUS_NO_SPACE;
    }

    if (checksum_crc16_umts(packet, crc_offset) != buffer_read_uint16_le(&packet[crc_offset])) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint16_t length = 0;
    if (fast_instruction) {
        // Combined replies do not use byte stuffing.
        length = wire_parameters_length;
        memcpy(parameters, &packet[STATUS_PAYLOAD_OFFSET], length);
    } else {
        for (uint32_t i = STATUS_PAYLOAD_OFFSET; i < crc_offset; i++) {
            parameters[length++] = packet[i];

            if (is_header(&packet[i - 2])) {
                if (i + 1 == crc_offset || packet[i + 1] != 0xFD) {
                    return ULS_STATUS_INVALID_INPUT;
                }
                i++;
            }
        }
    }

    *id = packet_id;
    *parameters_length = length;

    return ULS_STATUS_OK;
}
