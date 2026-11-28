#ifndef ULYSSES_LIB_BUFFER
#define ULYSSES_LIB_BUFFER

#include <stdint.h>

/**
 * @brief Reads a unsigned 16-bit little-endian integer.
 *
 * @param data Pointer to 2 readable bytes.
 * @return The decoded value.
 */
static inline uint16_t buffer_read_uint16_le(const uint8_t *data)
{
    return data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief Reads a unsigned 16-bit big-endian integer.
 *
 * @param data Pointer to 2 readable bytes.
 * @return The decoded value.
 */
static inline uint16_t buffer_read_uint16_be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

/**
 * @brief Writes a unsigned 16-bit little-endian integer.
 *
 * @param[out] data Pointer to 2 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_uint16_le(uint8_t *data, uint16_t value)
{
    data[0] = value;
    data[1] = value >> 8;
}

/**
 * @brief Writes a unsigned 16-bit big-endian integer.
 *
 * @param[out] data Pointer to 2 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_uint16_be(uint8_t *data, uint16_t value)
{
    data[0] = value >> 8;
    data[1] = value;
}

/**
 * @brief Reads a unsigned 24-bit little-endian integer.
 *
 * @param data Pointer to 3 readable bytes.
 * @return The decoded value.
 */
static inline uint32_t buffer_read_uint24_le(const uint8_t *data)
{
    return data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
}

/**
 * @brief Reads a unsigned 24-bit big-endian integer.
 *
 * @param data Pointer to 3 readable bytes.
 * @return The decoded value.
 */
static inline uint32_t buffer_read_uint24_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

/**
 * @brief Writes a unsigned 24-bit little-endian integer.
 *
 * @param[out] data Pointer to 3 writable bytes.
 * @param value The value to encode. Only the low 24 bits are written.
 */
static inline void buffer_write_uint24_le(uint8_t *data, uint32_t value)
{
    data[0] = value;
    data[1] = value >> 8;
    data[2] = value >> 16;
}

/**
 * @brief Writes a unsigned 24-bit big-endian integer.
 *
 * @param[out] data Pointer to 3 writable bytes.
 * @param value The value to encode. Only the low 24 bits are written.
 */
static inline void buffer_write_uint24_be(uint8_t *data, uint32_t value)
{
    data[0] = value >> 16;
    data[1] = value >> 8;
    data[2] = value;
}

/**
 * @brief Reads a unsigned 32-bit little-endian integer.
 *
 * @param data Pointer to 4 readable bytes.
 * @return The decoded value.
 */
static inline uint32_t buffer_read_uint32_le(const uint8_t *data)
{
    return data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief Reads a unsigned 32-bit big-endian integer.
 *
 * @param data Pointer to 4 readable bytes.
 * @return The decoded value.
 */
static inline uint32_t buffer_read_uint32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) |
           data[3];
}

/**
 * @brief Writes a unsigned 32-bit little-endian integer.
 *
 * @param[out] data Pointer to 4 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_uint32_le(uint8_t *data, uint32_t value)
{
    data[0] = value;
    data[1] = value >> 8;
    data[2] = value >> 16;
    data[3] = value >> 24;
}

/**
 * @brief Writes a unsigned 32-bit big-endian integer.
 *
 * @param[out] data Pointer to 4 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_uint32_be(uint8_t *data, uint32_t value)
{
    data[0] = value >> 24;
    data[1] = value >> 16;
    data[2] = value >> 8;
    data[3] = value;
}

/**
 * @brief Reads a signed 16-bit little-endian integer.
 *
 * @param data Pointer to 2 readable bytes.
 * @return The decoded value.
 */
static inline int16_t buffer_read_int16_le(const uint8_t *data)
{
    uint16_t value = buffer_read_uint16_le(data);
    return *(int16_t *)&value;
}

/**
 * @brief Reads a signed 16-bit big-endian integer.
 *
 * @param data Pointer to 2 readable bytes.
 * @return The decoded value.
 */
static inline int16_t buffer_read_int16_be(const uint8_t *data)
{
    uint16_t value = buffer_read_uint16_be(data);
    return *(int16_t *)&value;
}

/**
 * @brief Writes a signed 16-bit little-endian integer.
 *
 * @param[out] data Pointer to 2 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_int16_le(uint8_t *data, int16_t value)
{
    buffer_write_uint16_le(data, *(uint16_t *)&value);
}

/**
 * @brief Writes a signed 16-bit big-endian integer.
 *
 * @param[out] data Pointer to 2 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_int16_be(uint8_t *data, int16_t value)
{
    buffer_write_uint16_be(data, *(uint16_t *)&value);
}

/**
 * @brief Reads a signed 24-bit little-endian integer.
 *
 * @param data Pointer to 3 readable bytes.
 * @return The value sign-extended to 32 bits.
 */
static inline int32_t buffer_read_int24_le(const uint8_t *data)
{
    uint32_t value = buffer_read_uint24_le(data);
    // HACK: technically signed >> is implementation-defined behavior, but
    // gcc emits an extra instruction for the more correct
    // return (int32_t)(value ^ 0x800000) - 0x800000;
    // and the tests will catch if your compiler doesn't sign-extend anyways
    return (int32_t)(value << 8) >> 8;
}

/**
 * @brief Reads a signed 24-bit big-endian integer.
 *
 * @param data Pointer to 3 readable bytes.
 * @return The value sign-extended to 32 bits.
 */
static inline int32_t buffer_read_int24_be(const uint8_t *data)
{
    uint32_t value = buffer_read_uint24_be(data);
    // HACK: see buffer_read_int24_le
    return (int32_t)(value << 8) >> 8;
}

/**
 * @brief Writes a signed 24-bit little-endian integer.
 *
 * @param[out] data Pointer to 3 writable bytes.
 * @param value The value to encode. Only the low 24 bits are written.
 */
static inline void buffer_write_int24_le(uint8_t *data, int32_t value)
{
    buffer_write_uint24_le(data, *(uint32_t *)&value);
}

/**
 * @brief Writes a signed 24-bit big-endian integer.
 *
 * @param[out] data Pointer to 3 writable bytes.
 * @param value The value to encode. Only the low 24 bits are written.
 */
static inline void buffer_write_int24_be(uint8_t *data, int32_t value)
{
    buffer_write_uint24_be(data, *(uint32_t *)&value);
}

/**
 * @brief Reads a signed 32-bit little-endian integer.
 *
 * @param data Pointer to 4 readable bytes.
 * @return The decoded value.
 */
static inline int32_t buffer_read_int32_le(const uint8_t *data)
{
    uint32_t value = buffer_read_uint32_le(data);
    return *(int32_t *)&value;
}

/**
 * @brief Reads a signed 32-bit big-endian integer.
 *
 * @param data Pointer to 4 readable bytes.
 * @return The decoded value.
 */
static inline int32_t buffer_read_int32_be(const uint8_t *data)
{
    uint32_t value = buffer_read_uint32_be(data);
    return *(int32_t *)&value;
}

/**
 * @brief Writes a signed 32-bit little-endian integer.
 *
 * @param[out] data Pointer to 4 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_int32_le(uint8_t *data, int32_t value)
{
    buffer_write_uint32_le(data, *(uint32_t *)&value);
}

/**
 * @brief Writes a signed 32-bit big-endian integer.
 *
 * @param[out] data Pointer to 4 writable bytes.
 * @param value The value to encode.
 */
static inline void buffer_write_int32_be(uint8_t *data, int32_t value)
{
    buffer_write_uint32_be(data, *(uint32_t *)&value);
}

#endif // ULYSSES_LIB_BUFFER
