#include "ulysses/lib/checksum/crc.h"

#include <stddef.h>

#include "ulysses/lib/assert.h"

static inline uint16_t reflect16(uint16_t x)
{
    x = ((x & 0x5555) << 1) | ((x >> 1) & 0x5555);
    x = ((x & 0x3333) << 2) | ((x >> 2) & 0x3333);
    x = ((x & 0x0F0F) << 4) | ((x >> 4) & 0x0F0F);
    return (uint16_t)((x << 8) | (x >> 8));
}

static uint16_t crc16_msb(const uint8_t *data, uint32_t len, uint16_t crc, uint16_t poly)
{
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            uint16_t mask = -(uint16_t)(crc >> 15);
            crc = (crc << 1) ^ (poly & mask);
        }
    }
    return crc;
}

static uint16_t crc16_lsb(const uint8_t *data, uint32_t len, uint16_t crc, uint16_t poly)
{
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            uint16_t mask = -(uint16_t)(crc & 1);
            crc = (crc >> 1) ^ (poly & mask);
        }
    }
    return crc;
}

uint16_t checksum_crc16(const uint8_t *data, uint32_t length, uint16_t poly, uint16_t init,
                        bool ref_in, bool ref_out, uint16_t xor_out)
{
    ULYSSES_ASSERT(data != NULL || length == 0, "Data must be non NULL when length is nonzero");

    uint16_t crc;

    if (ref_in) {
        crc = crc16_lsb(data, length, reflect16(init), reflect16(poly));
    } else {
        crc = crc16_msb(data, length, init, poly);
    }

    if (ref_in != ref_out) {
        crc = reflect16(crc);
    }

    return crc ^ xor_out;
}

uint16_t checksum_crc16_umts(const uint8_t *data, uint32_t length)
{
    ULYSSES_ASSERT(data != NULL || length == 0, "Data must be non NULL when length is nonzero");

    uint16_t crc = 0;

    while (length--) {
        uint32_t feedback = (crc >> 8) ^ *data++;

        feedback ^= feedback >> 1;
        feedback ^= feedback >> 2;
        feedback ^= feedback >> 4;

        crc = (crc << 8) ^ (feedback << 15) ^ (feedback << 2) ^ feedback;
    }

    return crc;
}
