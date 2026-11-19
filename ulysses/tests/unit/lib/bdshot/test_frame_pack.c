#include "unity.h"

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/lib/bdshot/frame.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_bdshot_frame_pack_throttle_without_telemetry(void)
{
    // Using example from https://betaflight.com/docs/development/API/Dshot#bidirectional-dshot
    uint16_t throttle = 1046 - BDSHOT_MIN_THROTTLE;
    uint16_t expected_frame = 0x82C9;
    uint16_t actual_frame = 0x0000;

    bool is_success = bdshot_frame_pack_throttle(&actual_frame, throttle, false);

    TEST_ASSERT_TRUE(is_success);
    TEST_ASSERT_EQUAL(expected_frame, actual_frame);
}

void test_bdshot_frame_pack_throttle_with_telemetry(void)
{
    uint16_t throttle = 1000;
    uint16_t expected_frame = 0x8315;
    uint16_t actual_frame = 0x0000;

    bool is_success = bdshot_frame_pack_throttle(&actual_frame, throttle, true);

    TEST_ASSERT_TRUE(is_success);
    TEST_ASSERT_EQUAL(expected_frame, actual_frame);
}

void test_bdshot_frame_pack_command_without_telemetry(void)
{
    enum bdshot_command command = BDSHOT_COMMAND_MOTOR_STOP;
    uint16_t expected_frame = 0x000F;
    uint16_t actual_frame = 0x0000;

    bool is_success = bdshot_frame_pack_command(&actual_frame, command, false);

    TEST_ASSERT_TRUE(is_success);
    TEST_ASSERT_EQUAL(expected_frame, actual_frame);
}

void test_bdshot_frame_pack_command_with_telemetry(void)
{
    enum bdshot_command command = BDSHOT_COMMAND_MOTOR_STOP;
    uint16_t expected_frame = 0x001E;
    uint16_t actual_frame = 0x0000;

    bool is_success = bdshot_frame_pack_command(&actual_frame, command, true);

    TEST_ASSERT_TRUE(is_success);
    TEST_ASSERT_EQUAL(expected_frame, actual_frame);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_bdshot_frame_pack_throttle_without_telemetry);
    RUN_TEST(test_bdshot_frame_pack_throttle_with_telemetry);
    RUN_TEST(test_bdshot_frame_pack_command_without_telemetry);
    RUN_TEST(test_bdshot_frame_pack_command_with_telemetry);

    return UNITY_END();
}
