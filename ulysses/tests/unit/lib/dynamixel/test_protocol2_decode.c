#include "unity.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ulysses/lib/dynamixel/protocol2.h"

// Reference packets: https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-details
static const uint8_t STATUS_OK[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04,
                                    0x00, 0x55, 0x00, 0xA1, 0x0C};

static const uint8_t PING_REPLY[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                     0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D};

void setUp(void)
{
}

void tearDown(void)
{
}

// Decode once, then invert the protected tail before repeating the call.
static void assert_ok(const uint8_t *packet, uint32_t packet_size, uint8_t expected_id,
                      uint8_t expected_error, const uint8_t *expected_parameters,
                      uint16_t expected_length)
{
    uint8_t id = 0;
    uint8_t error = 0;
    uint8_t parameters[2 * packet_size];
    memset(parameters, 0xAA, sizeof(parameters));
    uint16_t parameters_length = 0;
    uint16_t capacity = packet_size;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_decode_response(
        packet, packet_size, &id, &error, parameters, capacity, &parameters_length);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_OK, status);
    TEST_ASSERT_EQUAL_HEX8(expected_id, id);
    TEST_ASSERT_EQUAL_HEX8(expected_error, error);
    TEST_ASSERT_EQUAL_UINT16(expected_length, parameters_length);
    if (expected_length != 0) {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_parameters, parameters, expected_length);
    }

    for (size_t i = capacity; i < sizeof(parameters); i++) {
        parameters[i] = ~parameters[i];
    }
    uint8_t unchanged[sizeof(parameters)];
    memcpy(unchanged, parameters, sizeof(parameters));

    dynamixel_protocol2_decode_response(packet, packet_size, &id, &error, parameters, capacity,
                                        &parameters_length);

    TEST_ASSERT_EQUAL_HEX8_ARRAY(&unchanged[capacity], &parameters[capacity],
                                 sizeof(parameters) - capacity);
}

// Invert the scalar outputs from the first failure and ensure a retry preserves them.
static void assert_invalid_packet(const uint8_t *packet, uint32_t packet_size)
{
    uint8_t id = 0;
    uint8_t error = 0;
    uint8_t parameters[packet_size];
    memset(parameters, 0xAA, sizeof(parameters));
    uint16_t parameters_length = 0;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_decode_response(
        packet, packet_size, &id, &error, parameters, sizeof(parameters), &parameters_length);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_INVALID_PACKET, status);

    uint8_t unchanged_id = ~id;
    uint8_t unchanged_error = ~error;
    uint16_t unchanged_length = ~parameters_length;
    id = unchanged_id;
    error = unchanged_error;
    parameters_length = unchanged_length;

    dynamixel_protocol2_decode_response(packet, packet_size, &id, &error, parameters,
                                        sizeof(parameters), &parameters_length);

    TEST_ASSERT_EQUAL_HEX8(unchanged_id, id);
    TEST_ASSERT_EQUAL_HEX8(unchanged_error, error);
    TEST_ASSERT_EQUAL_UINT16(unchanged_length, parameters_length);
}

// Seed outputs with a successful call before retrying with insufficient parameter space.
static void assert_no_space(const uint8_t *packet, uint32_t packet_size, uint16_t capacity)
{
    uint8_t id = 0;
    uint8_t error = 0;
    uint8_t parameters[2 * packet_size];
    memset(parameters, 0xAA, sizeof(parameters));
    uint16_t parameters_length = 0;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_decode_response(
        packet, packet_size, &id, &error, parameters, sizeof(parameters), &parameters_length);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_OK, status);

    for (size_t i = capacity; i < sizeof(parameters); i++) {
        parameters[i] = ~parameters[i];
    }
    uint8_t unchanged[sizeof(parameters)];
    memcpy(unchanged, parameters, sizeof(parameters));
    uint8_t unchanged_id = ~id;
    uint8_t unchanged_error = ~error;
    uint16_t unchanged_length = ~parameters_length;
    id = unchanged_id;
    error = unchanged_error;
    parameters_length = unchanged_length;

    status = dynamixel_protocol2_decode_response(packet, packet_size, &id, &error,
                                                 capacity == 0 ? NULL : parameters, capacity,
                                                 &parameters_length);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_NO_SPACE, status);
    TEST_ASSERT_EQUAL_HEX8(unchanged_id, id);
    TEST_ASSERT_EQUAL_HEX8(unchanged_error, error);
    TEST_ASSERT_EQUAL_UINT16(unchanged_length, parameters_length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&unchanged[capacity], &parameters[capacity],
                                 sizeof(parameters) - capacity);
}

void test_protocol2_decode_parameters(void)
{
    const uint8_t expected_parameters[] = {0x06, 0x04, 0x26};

    assert_ok(PING_REPLY, sizeof(PING_REPLY), 1, 0, expected_parameters,
              sizeof(expected_parameters));
}

void test_protocol2_decode_empty_replies(void)
{
    const uint8_t error_packet[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04,
                                    0x00, 0x55, 0x81, 0xA7, 0x0F};

    assert_ok(STATUS_OK, sizeof(STATUS_OK), 1, 0, NULL, 0);
    assert_ok(error_packet, sizeof(error_packet), 1, 0x81, NULL, 0);
}

// Additional fixture CRCs use independent polynomial division with 0x18005.
void test_protocol2_decode_byte_stuffing(void)
{
    const uint8_t packet[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x55,
                              0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x01, 0xDD, 0x1C};
    const uint8_t expected_parameters[] = {0xFF, 0xFF, 0xFD, 0x01};

    assert_ok(packet, sizeof(packet), 1, 0x00, expected_parameters, sizeof(expected_parameters));
}

void test_protocol2_decode_invalid_crc(void)
{
    uint8_t packet[sizeof(PING_REPLY)];
    memcpy(packet, PING_REPLY, sizeof(packet));
    packet[sizeof(packet) - 1] ^= 0x01;

    assert_invalid_packet(packet, sizeof(packet));
}

void test_protocol2_decode_invalid_framing(void)
{
    // Valid CRCs isolate the reserved-byte, length, instruction and ID checks.
    const uint8_t packets[][sizeof(PING_REPLY)] = {
        {0xFF, 0xFF, 0xFD, 0x01, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x06, 0xDB},
        {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x76, 0xDC},
        {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x03, 0x00, 0x06, 0x04, 0x26, 0xF5, 0x45},
        {0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x45, 0xAF},
    };
    for (size_t i = 0; i < sizeof(packets) / sizeof(packets[0]); i++) {
        assert_invalid_packet(packets[i], sizeof(packets[i]));
    }

    assert_invalid_packet(STATUS_OK, sizeof(STATUS_OK) - 1);

    uint8_t trailing[sizeof(STATUS_OK) + 1] = {0};
    memcpy(trailing, STATUS_OK, sizeof(STATUS_OK));
    assert_invalid_packet(trailing, sizeof(trailing));
}

void test_protocol2_decode_missing_stuffing(void)
{
    // Both malformed packets retain valid CRCs.
    const uint8_t missing[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55,
                               0x00, 0xFF, 0xFF, 0xFD, 0x01, 0x92, 0x36};
    const uint8_t unterminated[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                    0x55, 0x00, 0xFF, 0xFF, 0xFD, 0xC7, 0x49};

    assert_invalid_packet(missing, sizeof(missing));
    assert_invalid_packet(unterminated, sizeof(unterminated));
}

void test_protocol2_decode_short_parameter_buffers(void)
{
    assert_no_space(PING_REPLY, sizeof(PING_REPLY), sizeof(PING_REPLY) - 1);
    assert_no_space(PING_REPLY, sizeof(PING_REPLY), 0);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_protocol2_decode_parameters);
    RUN_TEST(test_protocol2_decode_empty_replies);
    RUN_TEST(test_protocol2_decode_byte_stuffing);
    RUN_TEST(test_protocol2_decode_invalid_crc);
    RUN_TEST(test_protocol2_decode_invalid_framing);
    RUN_TEST(test_protocol2_decode_missing_stuffing);
    RUN_TEST(test_protocol2_decode_short_parameter_buffers);

    return UNITY_END();
}
