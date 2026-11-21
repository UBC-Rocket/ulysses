#include "unity.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/checksum/crc.h"

// Standard check values: https://reveng.sourceforge.io/crc-catalogue/16.htm
static const uint8_t CHECK_DATA[] = "123456789";

void setUp(void)
{
}

void tearDown(void)
{
}

void test_crc16_xmodem(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021, 0x0000, false, false, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0x31C3, actual_crc);
}

void test_crc16_ibm_3740(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021, 0xFFFF, false, false, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0x29B1, actual_crc);
}

void test_crc16_genibus(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021, 0xFFFF, false, false, 0xFFFF);

    TEST_ASSERT_EQUAL_HEX16(0xD64E, actual_crc);
}

void test_crc16_arc(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x8005, 0x0000, true, true, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0xBB3D, actual_crc);
}

void test_crc16_modbus(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x8005, 0xFFFF, true, true, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0x4B37, actual_crc);
}

void test_crc16_ibm_sdlc(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021, 0xFFFF, true, true, 0xFFFF);

    TEST_ASSERT_EQUAL_HEX16(0x906E, actual_crc);
}

void test_crc16_tms37157(void)
{
    // A non-symmetric init catches incorrect reflection of the initial register.
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021, 0x89EC, true, true, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0x26B1, actual_crc);
}

void test_crc16_dnp(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x3D65, 0x0000, true, true, 0xFFFF);

    TEST_ASSERT_EQUAL_HEX16(0xEA82, actual_crc);
}

void test_crc16_t10_dif(void)
{
    uint16_t actual_crc =
        checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x8BB7, 0x0000, false, false, 0x0000);

    TEST_ASSERT_EQUAL_HEX16(0xD0DB, actual_crc);
}

// Additional 0x1021 vectors use Python's binascii.crc_hqx, reflecting each input
// byte for ref_in and the 16-bit result for ref_out, then applying xor_out.
void test_crc16_all_reflection_combinations(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xBB93, checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021,
                                                   0x1234, false, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x81CF, checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021,
                                                   0x1234, false, true, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x1BD4, checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021,
                                                   0x1234, true, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x63CA, checksum_crc16(CHECK_DATA, sizeof(CHECK_DATA) - 1, 0x1021,
                                                   0x1234, true, true, 0x5678));
}

void test_crc16_empty_buffer(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x444C,
                            checksum_crc16(CHECK_DATA, 0, 0x1021, 0x1234, false, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x7A30,
                            checksum_crc16(CHECK_DATA, 0, 0x1021, 0x1234, false, true, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x444C,
                            checksum_crc16(CHECK_DATA, 0, 0x1021, 0x1234, true, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x7A30,
                            checksum_crc16(CHECK_DATA, 0, 0x1021, 0x1234, true, true, 0x5678));
}

void test_crc16_null_empty_buffer(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x444C, checksum_crc16(NULL, 0, 0x1021, 0x1234, false, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x7A30, checksum_crc16(NULL, 0, 0x1021, 0x1234, false, true, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x444C, checksum_crc16(NULL, 0, 0x1021, 0x1234, true, false, 0x5678));
    TEST_ASSERT_EQUAL_HEX16(0x7A30, checksum_crc16(NULL, 0, 0x1021, 0x1234, true, true, 0x5678));
}

void test_crc16_single_byte(void)
{
    const uint8_t data[] = {0x00, 0xFF, 0x80};

    TEST_ASSERT_EQUAL_HEX16(0xE1F0,
                            checksum_crc16(&data[0], 1, 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0xFF00,
                            checksum_crc16(&data[1], 1, 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0x7078,
                            checksum_crc16(&data[2], 1, 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0x0F87,
                            checksum_crc16(&data[0], 1, 0x1021, 0xFFFF, true, true, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0x00FF,
                            checksum_crc16(&data[1], 1, 0x1021, 0xFFFF, true, true, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0x8B8F,
                            checksum_crc16(&data[2], 1, 0x1021, 0xFFFF, true, true, 0x0000));
}

void test_crc16_binary_buffer(void)
{
    const uint8_t data[] = {0x00, 0x80, 0xFF, 0x01, 0x7F, 0x55, 0xAA};

    TEST_ASSERT_EQUAL_HEX16(
        0x1826, checksum_crc16(data, sizeof(data), 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0xF2CC,
                            checksum_crc16(data, sizeof(data), 0x1021, 0xFFFF, true, true, 0x0000));
}

void test_crc16_respects_buffer_length(void)
{
    const uint8_t data[] = "X123456789Y";

    TEST_ASSERT_EQUAL_HEX16(0x29B1,
                            checksum_crc16(&data[1], 9, 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0x906E,
                            checksum_crc16(&data[1], 9, 0x1021, 0xFFFF, true, true, 0xFFFF));
}

void test_crc16_buffer_longer_than_uint16(void)
{
    static uint8_t data[65537];

    for (uint32_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }

    TEST_ASSERT_EQUAL_HEX16(
        0xF9F0, checksum_crc16(data, sizeof(data), 0x1021, 0xFFFF, false, false, 0x0000));
    TEST_ASSERT_EQUAL_HEX16(0xB4C9,
                            checksum_crc16(data, sizeof(data), 0x1021, 0xFFFF, true, true, 0x0000));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_crc16_xmodem);
    RUN_TEST(test_crc16_ibm_3740);
    RUN_TEST(test_crc16_genibus);
    RUN_TEST(test_crc16_arc);
    RUN_TEST(test_crc16_modbus);
    RUN_TEST(test_crc16_ibm_sdlc);
    RUN_TEST(test_crc16_tms37157);
    RUN_TEST(test_crc16_dnp);
    RUN_TEST(test_crc16_t10_dif);
    RUN_TEST(test_crc16_all_reflection_combinations);
    RUN_TEST(test_crc16_empty_buffer);
    RUN_TEST(test_crc16_null_empty_buffer);
    RUN_TEST(test_crc16_single_byte);
    RUN_TEST(test_crc16_binary_buffer);
    RUN_TEST(test_crc16_respects_buffer_length);
    RUN_TEST(test_crc16_buffer_longer_than_uint16);

    return UNITY_END();
}
