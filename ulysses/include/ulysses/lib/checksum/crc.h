#ifndef ULYSSES_LIB_CHECKSUM_CRC
#define ULYSSES_LIB_CHECKSUM_CRC

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Computes a CRC-16 checksum.
 *
 * @param data Input bytes; may be NULL when length is zero.
 * @param length Number of input bytes.
 * @param poly Polynomial in normal form, with the x^16 term omitted.
 * @param init Initial register value in normal form.
 * @param ref_in Reflect each input byte.
 * @param ref_out Reflect the final register before applying xor_out.
 * @param xor_out Final XOR value.
 *
 * @return Calculated checksum.
 */
uint16_t checksum_crc16(const uint8_t *data, uint32_t length, uint16_t poly, uint16_t init,
                        bool ref_in, bool ref_out, uint16_t xor_out);

/**
 * @brief Computes a CRC-16 UMTS checksum.
 *
 * @param data Input bytes; may be NULL when length is zero.
 * @param length Number of input bytes.
 *
 * @return Calculated checksum.
 */
uint16_t checksum_crc16_umts(const uint8_t *data, uint32_t length);

#endif // ULYSSES_LIB_CHECKSUM_CRC
