#include "unity.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ulysses/lib/checksum/crc.h"

static const char test_strings[][64] = {
    "123456789",
    "UBCRocket",
    "The quick brown fox jumps over a lazy dog.",
};

void setUp(void)
{
}

void tearDown(void)
{
}

void test_crc16_umts(void)
{
    for (size_t i = 0; i < sizeof(test_strings) / sizeof(test_strings[0]); i++) {
        const uint8_t *data = (const uint8_t *)test_strings[i];
        uint32_t length = strlen(test_strings[i]);

        TEST_ASSERT_EQUAL_HEX16(checksum_crc16(data, length, 0x8005, 0, false, false, 0),
                                checksum_crc16_umts(data, length));

        TEST_ASSERT_EQUAL_HEX16(checksum_crc16(data, length - 1, 0x8005, 0, false, false, 0),
                                checksum_crc16_umts(data, length - 1));
    }
}

void test_crc16_umts_edge_cases(void)
{
    TEST_ASSERT_EQUAL_HEX16(checksum_crc16(NULL, 0, 0x8005, 0, false, false, 0),
                            checksum_crc16_umts(NULL, 0));

    for (uint32_t i = 0; i <= UINT8_MAX; i++) {
        uint8_t byte = i;
        TEST_ASSERT_EQUAL_HEX16(checksum_crc16(&byte, 1, 0x8005, 0, false, false, 0),
                                checksum_crc16_umts(&byte, 1));
    }
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_crc16_umts);
    RUN_TEST(test_crc16_umts_edge_cases);

    return UNITY_END();
}
