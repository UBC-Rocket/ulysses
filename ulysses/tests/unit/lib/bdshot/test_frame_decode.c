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

void test_bdshot_frame_decode_from_wire_reference(void)
{
    // Using example from https://betaflight.com/docs/development/API/Dshot#erpm-transmission
    uint32_t wire_frame = 0x98D64;
    uint16_t expected_frame = 0x82C6;
    uint16_t actual_frame = 0x0000;

    bool is_success = bdshot_frame_decode_from_wire(&actual_frame, wire_frame);

    TEST_ASSERT_TRUE(is_success);
    TEST_ASSERT_EQUAL(expected_frame, actual_frame);
}

void test_bdshot_frame_is_edt_should_be_true(void)
{
    uint16_t frame = 0x82C6;

    bool is_edt = bdshot_frame_is_edt(frame);

    TEST_ASSERT_TRUE(is_edt);
}

void test_bdshot_frame_is_edt_should_be_false(void)
{
    uint16_t frame = 0x3003;

    bool is_edt = bdshot_frame_is_edt(frame);

    TEST_ASSERT_FALSE(is_edt);
}

void test_bdshot_frame_calculate_rpm(void)
{
    // 30000 eRPM
    uint16_t frame = 0x5F4E;
    uint8_t motor_pole_count = 12;
    float expected_rpm = 5000.0f;

    float actual_rpm = bdshot_frame_calculate_rpm(frame, motor_pole_count);

    TEST_ASSERT_EQUAL_FLOAT(expected_rpm, actual_rpm);
}

void test_bdshot_frame_calculate_rpm_zero_exponent(void)
{
    // 240000 eRPM
    uint16_t frame = 0x0FA5;
    uint8_t motor_pole_count = 8;
    float expected_rpm = 60000.0f;

    float actual_rpm = bdshot_frame_calculate_rpm(frame, motor_pole_count);

    TEST_ASSERT_EQUAL_FLOAT(expected_rpm, actual_rpm);
}

void test_bdshot_frame_calculate_rpm_zero_mantissa(void)
{
    uint16_t frame = 0x0000;
    uint8_t motor_pole_count = 12;
    float expected_rpm = 0.0f;

    float actual_rpm = bdshot_frame_calculate_rpm(frame, motor_pole_count);

    TEST_ASSERT_EQUAL_FLOAT(expected_rpm, actual_rpm);
}

void test_bdshot_frame_calculate_rpm_all_ones(void)
{
    uint16_t frame = 0xFFFF;
    uint8_t motor_pole_count = 12;
    float expected_rpm = 0.0f;

    float actual_rpm = bdshot_frame_calculate_rpm(frame, motor_pole_count);

    TEST_ASSERT_EQUAL_FLOAT(expected_rpm, actual_rpm);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_bdshot_frame_decode_from_wire_reference);
    RUN_TEST(test_bdshot_frame_is_edt_should_be_true);
    RUN_TEST(test_bdshot_frame_is_edt_should_be_false);
    RUN_TEST(test_bdshot_frame_calculate_rpm);
    RUN_TEST(test_bdshot_frame_calculate_rpm_zero_exponent);
    RUN_TEST(test_bdshot_frame_calculate_rpm_zero_mantissa);
    RUN_TEST(test_bdshot_frame_calculate_rpm_all_ones);

    return UNITY_END();
}
