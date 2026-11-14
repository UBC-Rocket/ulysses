#include "ulysses/lib/dynamixel/protocol2.h"

#include <stdbool.h>
#include <string.h>

#include "ulysses/lib/assert.h"

// Packet format: https://emanual.robotis.com/docs/en/dxl/protocol2/
// SDK reference: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c/src/dynamixel_sdk/protocol2_packet_handler.c

#define PACKET_PREFIX_SIZE (7)
#define ERROR_OFFSET (8)
#define STATUS_PARAMETERS_OFFSET (9)
#define CRC_LENGTH (2)
#define MAX_DEVICE_ID (0xFC)

static const uint8_t header[] = {0xFF, 0xFF, 0xFD, 0x00};

// TODO: this should be in a separate library, also should probably
// switch between HW and SW CRC based on data length if we're really optimizing
static uint16_t crc16_umts(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (uint32_t bit = 0; bit < 8; bit++) {
            uint32_t mask = -(uint32_t)(crc >> 15);
            crc = (crc << 1) ^ (mask & 0x8005);
        }
    }
    return crc;
}

static uint16_t read_u16le(const uint8_t *data)
{
    return data[0] | (data[1] << 8);
}

static void write_u16le(uint8_t *data, uint16_t value)
{
    data[0] = value;
    data[1] = value >> 8;
}

static bool is_header(const uint8_t *data)
{
    return data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFD;
}

static bool is_valid_instruction(uint8_t instruction)
{
    switch (instruction) {
    case DYNAMIXEL_PROTOCOL2_INST_PING:
    case DYNAMIXEL_PROTOCOL2_INST_READ:
    case DYNAMIXEL_PROTOCOL2_INST_WRITE:
    case DYNAMIXEL_PROTOCOL2_INST_REG_WRITE:
    case DYNAMIXEL_PROTOCOL2_INST_ACTION:
    case DYNAMIXEL_PROTOCOL2_INST_FACTORY_RESET:
    case DYNAMIXEL_PROTOCOL2_INST_REBOOT:
    case DYNAMIXEL_PROTOCOL2_INST_CLEAR:
    case DYNAMIXEL_PROTOCOL2_INST_CONTROL_TABLE_BACKUP:
    case DYNAMIXEL_PROTOCOL2_INST_SYNC_READ:
    case DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE:
    // case DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ:
    case DYNAMIXEL_PROTOCOL2_INST_BULK_READ:
    case DYNAMIXEL_PROTOCOL2_INST_BULK_WRITE:
        // case DYNAMIXEL_PROTOCOL2_INST_FAST_BULK_READ:
        return true;
    default:
    case DYNAMIXEL_PROTOCOL2_INST_STATUS:
        return false;
    }
}

enum dynamixel_protocol2_status
dynamixel_protocol2_encode_request(uint8_t id, uint8_t instruction, const uint8_t *parameters,
                                   uint16_t parameters_length, uint8_t *packet, uint32_t capacity,
                                   uint32_t *packet_size)
{
    ULYSSES_ASSERT(id <= MAX_DEVICE_ID || id == DYNAMIXEL_PROTOCOL2_BROADCAST_ID,
                   "invalid dynamixel instruction ID");
    ULYSSES_ASSERT(is_valid_instruction(instruction), "invalid dynamixel instruction");
    ULYSSES_ASSERT(parameters != NULL || parameters_length == 0,
                   "dynamixel parameters must not be NULL for nonzero length");
    ULYSSES_ASSERT(packet != NULL, "dynamixel packet output must not be NULL");
    ULYSSES_ASSERT(packet_size != NULL, "dynamixel packet size output must not be NULL");

    if (capacity < PACKET_PREFIX_SIZE + 1 + CRC_LENGTH) {
        return DYNAMIXEL_PROTOCOL2_NO_SPACE;
    }

    memcpy(packet, header, sizeof(header));
    packet[4] = id;
    packet[7] = instruction;

    uint32_t ptr = 8;
    for (uint32_t i = 0; i < parameters_length; i++) {
        if (ptr + CRC_LENGTH >= capacity) {
            return DYNAMIXEL_PROTOCOL2_NO_SPACE;
        }
        packet[ptr++] = parameters[i];

        if (i >= 2 && is_header(&parameters[i - 2])) {
            if (ptr + CRC_LENGTH >= capacity) {
                return DYNAMIXEL_PROTOCOL2_NO_SPACE;
            }
            packet[ptr++] = 0xFD;
        }
    }

    uint32_t length = ptr - PACKET_PREFIX_SIZE + CRC_LENGTH;
    if (length > UINT16_MAX) {
        return DYNAMIXEL_PROTOCOL2_NO_SPACE;
    }

    write_u16le(&packet[5], length);
    write_u16le(&packet[ptr], crc16_umts(packet, ptr));
    *packet_size = ptr + CRC_LENGTH;

    return DYNAMIXEL_PROTOCOL2_OK;
}

enum dynamixel_protocol2_status
dynamixel_protocol2_decode_response(const uint8_t *packet, uint32_t packet_size, uint8_t *id,
                                    uint8_t *error, uint8_t *parameters, uint16_t capacity,
                                    uint16_t *parameters_length)
{
    ULYSSES_ASSERT(packet != NULL, "dynamixel packet must not be NULL");
    ULYSSES_ASSERT(id != NULL, "dynamixel ID output must not be NULL");
    ULYSSES_ASSERT(error != NULL, "dynamixel error output must not be NULL");
    ULYSSES_ASSERT(parameters != NULL || capacity == 0,
                   "dynamixel parameters output must not be NULL for nonzero capacity");
    ULYSSES_ASSERT(parameters_length != NULL,
                   "dynamixel parameters length output must not be NULL");

    if (packet_size < STATUS_PARAMETERS_OFFSET + CRC_LENGTH ||
        memcmp(packet, header, sizeof(header)) != 0 || packet[4] > MAX_DEVICE_ID ||
        read_u16le(&packet[5]) != packet_size - PACKET_PREFIX_SIZE ||
        packet[7] != DYNAMIXEL_PROTOCOL2_INST_STATUS) {
        return DYNAMIXEL_PROTOCOL2_INVALID_PACKET;
    }

    uint32_t crc_offset = packet_size - CRC_LENGTH;
    if (crc16_umts(packet, crc_offset) != read_u16le(&packet[crc_offset])) {
        return DYNAMIXEL_PROTOCOL2_INVALID_PACKET;
    }

    uint16_t length = 0;
    for (uint32_t i = STATUS_PARAMETERS_OFFSET; i < crc_offset; i++) {
        if (length >= capacity) {
            return DYNAMIXEL_PROTOCOL2_NO_SPACE;
        }
        parameters[length++] = packet[i];

        if (is_header(&packet[i - 2])) {
            if (i + 1 == crc_offset || packet[i + 1] != 0xFD) {
                return DYNAMIXEL_PROTOCOL2_INVALID_PACKET;
            }
            i++;
        }
    }

    *id = packet[4];
    *error = packet[ERROR_OFFSET];
    *parameters_length = length;

    return DYNAMIXEL_PROTOCOL2_OK;
}
