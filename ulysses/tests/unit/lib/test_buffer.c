#include "unity.h"

#include <stdint.h>
#include <string.h>

#include "ulysses/lib/buffer.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_buffer_reads(void)
{
    const uint8_t small_bytes[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    const uint8_t large_bytes[] = {0xFB, 0xFC, 0xFD, 0xFE, 0xF7, 0xF8, 0xF9, 0xFA};

    TEST_ASSERT_EQUAL_HEX16(0x0201, buffer_read_uint16_le(small_bytes));
    TEST_ASSERT_EQUAL_HEX16(0x0102, buffer_read_uint16_be(small_bytes));
    TEST_ASSERT_EQUAL_HEX32(0x030201, buffer_read_uint24_le(small_bytes));
    TEST_ASSERT_EQUAL_HEX32(0x010203, buffer_read_uint24_be(small_bytes));
    TEST_ASSERT_EQUAL_HEX32(0x04030201, buffer_read_uint32_le(small_bytes));
    TEST_ASSERT_EQUAL_HEX32(0x01020304, buffer_read_uint32_be(small_bytes));

    TEST_ASSERT_EQUAL_INT16(0x0201, buffer_read_int16_le(small_bytes));
    TEST_ASSERT_EQUAL_INT16(0x0102, buffer_read_int16_be(small_bytes));
    TEST_ASSERT_EQUAL_INT32(0x030201, buffer_read_int24_le(small_bytes));
    TEST_ASSERT_EQUAL_INT32(0x010203, buffer_read_int24_be(small_bytes));
    TEST_ASSERT_EQUAL_INT32(0x04030201, buffer_read_int32_le(small_bytes));
    TEST_ASSERT_EQUAL_INT32(0x01020304, buffer_read_int32_be(small_bytes));

    TEST_ASSERT_EQUAL_HEX16(0xFCFB, buffer_read_uint16_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX16(0xFBFC, buffer_read_uint16_be(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFDFCFB, buffer_read_uint24_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFD, buffer_read_uint24_be(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFEFDFCFB, buffer_read_uint32_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFDFE, buffer_read_uint32_be(large_bytes));

    TEST_ASSERT_EQUAL_HEX16(0xFCFB, buffer_read_int16_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX16(0xFBFC, buffer_read_int16_be(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFFFDFCFB, buffer_read_int24_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFFFBFCFD, buffer_read_int24_be(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFEFDFCFB, buffer_read_int32_le(large_bytes));
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFDFE, buffer_read_int32_be(large_bytes));

    // HACK: Unity requires 64 bit support, this is done for better portability
    TEST_ASSERT_TRUE(buffer_read_uint64_le(small_bytes) == UINT64_C(0x0807060504030201));
    TEST_ASSERT_TRUE(buffer_read_uint64_be(small_bytes) == UINT64_C(0x0102030405060708));
    TEST_ASSERT_TRUE(buffer_read_int64_le(small_bytes) == INT64_C(0x0807060504030201));
    TEST_ASSERT_TRUE(buffer_read_int64_be(small_bytes) == INT64_C(0x0102030405060708));

    TEST_ASSERT_TRUE(buffer_read_uint64_le(large_bytes) == UINT64_C(0xFAF9F8F7FEFDFCFB));
    TEST_ASSERT_TRUE(buffer_read_uint64_be(large_bytes) == UINT64_C(0xFBFCFDFEF7F8F9FA));
    TEST_ASSERT_TRUE(buffer_read_int64_le(large_bytes) == -INT64_C(0x0506070801020305));
    TEST_ASSERT_TRUE(buffer_read_int64_be(large_bytes) == -INT64_C(0x0403020108070606));
}

void test_buffer_writes(void)
{
    const int16_t int16_value = -0x3F3E;
    const int32_t int24_value = -0x3F3E3D;
    const int32_t int32_value = -0x3F3E3D3C;
    uint8_t data[4] = {0};

    buffer_write_uint16_le(data, 0x7F7E);
    TEST_ASSERT_EQUAL_HEX16(0x7F7E, buffer_read_uint16_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint16_be(data, 0x7F7E);
    TEST_ASSERT_EQUAL_HEX16(0x7F7E, buffer_read_uint16_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint24_le(data, 0x7F7E7D);
    TEST_ASSERT_EQUAL_HEX32(0x7F7E7D, buffer_read_uint24_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint24_be(data, 0x7F7E7D);
    TEST_ASSERT_EQUAL_HEX32(0x7F7E7D, buffer_read_uint24_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint32_le(data, 0x7F7E7D7C);
    TEST_ASSERT_EQUAL_HEX32(0x7F7E7D7C, buffer_read_uint32_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint32_be(data, 0x7F7E7D7C);
    TEST_ASSERT_EQUAL_HEX32(0x7F7E7D7C, buffer_read_uint32_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_int16_le(data, int16_value);
    TEST_ASSERT_EQUAL_INT16(int16_value, buffer_read_int16_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_int16_be(data, int16_value);
    TEST_ASSERT_EQUAL_INT16(int16_value, buffer_read_int16_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_int24_le(data, int24_value);
    TEST_ASSERT_EQUAL_INT32(int24_value, buffer_read_int24_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_int24_be(data, int24_value);
    TEST_ASSERT_EQUAL_INT32(int24_value, buffer_read_int24_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_int32_le(data, int32_value);
    TEST_ASSERT_EQUAL_INT32(int32_value, buffer_read_int32_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_int32_be(data, int32_value);
    TEST_ASSERT_EQUAL_INT32(int32_value, buffer_read_int32_be(data));
}

void test_buffer_writes_64(void)
{
    const uint8_t expected_le[] = {0xA5, 0xC8, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xC0, 0x5A};
    const uint8_t expected_be[] = {0xA5, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC8, 0x5A};
    const uint64_t unsigned_value = UINT64_C(0xC0C1C2C3C4C5C6C8);
    const int64_t signed_value = -INT64_C(0x3F3E3D3C3B3A3938);
    uint8_t data[10] = {[0] = 0xA5, [9] = 0x5A};

    buffer_write_uint64_le(data + 1, unsigned_value);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_le, data, sizeof(expected_le));

    buffer_write_uint64_be(data + 1, unsigned_value);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_be, data, sizeof(expected_be));

    buffer_write_int64_le(data + 1, signed_value);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_le, data, sizeof(expected_le));

    buffer_write_int64_be(data + 1, signed_value);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_be, data, sizeof(expected_be));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_buffer_reads);
    RUN_TEST(test_buffer_writes);
    RUN_TEST(test_buffer_writes_64);

    return UNITY_END();
}
