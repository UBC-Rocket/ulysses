#include "unity.h"

#include "stdint.h"

#include "ulysses/lib/bdshot/frame.h"
#include "unity_internals.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_bdshot_frame_checksum_all_zeroes(void)
{
    uint16_t frame = 0x0000;
    uint8_t expected_checksum = 0xF;

    uint8_t actual_checksum = bdshot_frame_checksum(frame);

    TEST_ASSERT_EQUAL(expected_checksum, actual_checksum);
}

void test_bdshot_frame_checksum_all_ones(void)
{
    uint16_t frame = 0xFFFF;
    uint8_t expected_checksum = 0x0;

    uint8_t actual_checksum = bdshot_frame_checksum(frame);

    TEST_ASSERT_EQUAL(expected_checksum, actual_checksum);
}

void test_bdshot_frame_checksum_random_frame(void)
{
    // Using example from https://betaflight.com/docs/development/API/Dshot#bidirectional-dshot
    uint16_t frame = 0x82C0;
    uint8_t expected_checksum = 0x9;

    uint8_t actual_checksum = bdshot_frame_checksum(frame);

    TEST_ASSERT_EQUAL(expected_checksum, actual_checksum);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_bdshot_frame_checksum_all_zeroes);
    RUN_TEST(test_bdshot_frame_checksum_all_ones);
    RUN_TEST(test_bdshot_frame_checksum_random_frame);

    return UNITY_END();
}
