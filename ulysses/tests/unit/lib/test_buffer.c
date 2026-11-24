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
    const uint8_t small_bytes[] = {0x01, 0x02, 0x03, 0x04};
    const uint8_t large_bytes[] = {0xFB, 0xFC, 0xFD, 0xFE};

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
}

void test_buffer_writes(void)
{
    const int16_t int16_value = 0xFBFC - ((int32_t)1 << 16);
    const int32_t int24_value = 0xFBFCFD - ((int32_t)1 << 24);
    const int32_t int32_value = 0xFBFCFDFE - ((int64_t)1 << 32);
    uint8_t data[4] = {0};

    buffer_write_uint16_le(data, 0xFBFC);
    TEST_ASSERT_EQUAL_HEX16(0xFBFC, buffer_read_uint16_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint16_be(data, 0xFBFC);
    TEST_ASSERT_EQUAL_HEX16(0xFBFC, buffer_read_uint16_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint24_le(data, 0xFBFCFD);
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFD, buffer_read_uint24_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint24_be(data, 0xFBFCFD);
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFD, buffer_read_uint24_be(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint32_le(data, 0xFBFCFDFE);
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFDFE, buffer_read_uint32_le(data));

    memset(data, 0, sizeof(data));
    buffer_write_uint32_be(data, 0xFBFCFDFE);
    TEST_ASSERT_EQUAL_HEX32(0xFBFCFDFE, buffer_read_uint32_be(data));

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

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_buffer_reads);
    RUN_TEST(test_buffer_writes);

    return UNITY_END();
}
