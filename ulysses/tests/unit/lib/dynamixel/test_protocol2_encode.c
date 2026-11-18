#include "unity.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ulysses/lib/dynamixel/protocol2.h"

// Reference packets: https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-details

void setUp(void)
{
}

void tearDown(void)
{
}

// Compare reference bytes, then invert the protected tail before repeating the call.
static void assert_ok(uint8_t id, enum dynamixel_protocol2_instruction instruction,
                      const uint8_t *parameters, uint16_t parameters_length,
                      const uint8_t *expected, uint32_t expected_size)
{
    uint32_t capacity = dynamixel_protocol2_required_packet_capacity(parameters_length);
    uint8_t packet[2 * capacity];
    memset(packet, 0xAA, sizeof(packet));
    uint32_t packet_size = 0;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_encode_request(
        id, instruction, parameters, parameters_length, packet, capacity, &packet_size);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_OK, status);
    TEST_ASSERT_EQUAL_UINT32(expected_size, packet_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, packet, expected_size);

    for (size_t i = capacity; i < sizeof(packet); i++) {
        packet[i] = ~packet[i];
    }
    uint8_t unchanged[sizeof(packet)];
    memcpy(unchanged, packet, sizeof(packet));

    dynamixel_protocol2_encode_request(id, instruction, parameters, parameters_length, packet,
                                       capacity, &packet_size);

    TEST_ASSERT_EQUAL_HEX8_ARRAY(&unchanged[capacity], &packet[capacity],
                                 sizeof(packet) - capacity);
}

// Seed outputs, then retry with one byte less than the required worst-case capacity.
static void assert_no_space(uint8_t id, enum dynamixel_protocol2_instruction instruction,
                            const uint8_t *parameters, uint16_t parameters_length)
{
    uint32_t required_capacity = dynamixel_protocol2_required_packet_capacity(parameters_length);
    uint8_t packet[2 * required_capacity];
    memset(packet, 0xAA, sizeof(packet));
    uint32_t packet_size = 0;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_encode_request(
        id, instruction, parameters, parameters_length, packet, required_capacity, &packet_size);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_OK, status);

    uint32_t capacity = required_capacity - 1;
    for (size_t i = capacity; i < sizeof(packet); i++) {
        packet[i] = ~packet[i];
    }
    uint8_t unchanged[sizeof(packet)];
    memcpy(unchanged, packet, sizeof(packet));
    uint32_t unchanged_size = ~packet_size;
    packet_size = unchanged_size;

    status = dynamixel_protocol2_encode_request(id, instruction, parameters, parameters_length,
                                                packet, capacity, &packet_size);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_NO_SPACE, status);
    TEST_ASSERT_EQUAL_UINT32(unchanged_size, packet_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&unchanged[capacity], &packet[capacity],
                                 sizeof(packet) - capacity);
}

void test_protocol2_encode_ping(void)
{
    const uint8_t expected[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};

    assert_ok(1, DYNAMIXEL_PROTOCOL2_INST_PING, NULL, 0, expected, sizeof(expected));
}

void test_protocol2_encode_write(void)
{
    const uint8_t parameters[] = {0x74, 0x00, 0x00, 0x02, 0x00, 0x00};
    const uint8_t expected[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03,
                                0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89};

    assert_ok(1, DYNAMIXEL_PROTOCOL2_INST_WRITE, parameters, sizeof(parameters), expected,
              sizeof(expected));
}

void test_protocol2_encode_broadcast(void)
{
    const uint8_t parameters[] = {0x74, 0x00, 0x04, 0x00, 0x01, 0x96, 0x00,
                                  0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00};
    const uint8_t expected[] = {0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83,
                                0x74, 0x00, 0x04, 0x00, 0x01, 0x96, 0x00, 0x00,
                                0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87};

    assert_ok(DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE, parameters,
              sizeof(parameters), expected, sizeof(expected));
}

void test_protocol2_encode_byte_stuffing(void)
{
    const uint8_t parameters[] = {0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD};
    const uint8_t expected[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x0E, 0x00, 0x03, 0x74, 0x00, 0xFF,
                                0xFF, 0xFD, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD, 0x40, 0xCE};

    assert_ok(1, DYNAMIXEL_PROTOCOL2_INST_WRITE, parameters, sizeof(parameters), expected,
              sizeof(expected));
}

void test_protocol2_encode_short_buffers(void)
{
    const uint8_t write_parameters[] = {0x74, 0x00, 0x00, 0x02, 0x00, 0x00};
    const uint8_t stuffed_parameters[] = {0x74, 0x00, 0xFF, 0xFF, 0xFD};

    assert_no_space(1, DYNAMIXEL_PROTOCOL2_INST_PING, NULL, 0);
    assert_no_space(1, DYNAMIXEL_PROTOCOL2_INST_WRITE, write_parameters, sizeof(write_parameters));
    assert_no_space(1, DYNAMIXEL_PROTOCOL2_INST_WRITE, stuffed_parameters,
                    sizeof(stuffed_parameters));
}

void test_protocol2_encode_parameter_limit(void)
{
    uint8_t parameters[DYNAMIXEL_PROTOCOL2_MAX_PARAMETERS_COUNT];
    static const uint8_t pattern[] = {0xFF, 0xFF, 0xFD};
    for (size_t i = 0; i < sizeof(parameters); i++) {
        parameters[i] = pattern[i % sizeof(pattern)];
    }
    uint8_t packet[dynamixel_protocol2_required_packet_capacity(sizeof(parameters))];
    uint32_t packet_size = 0;

    enum dynamixel_protocol2_status status = dynamixel_protocol2_encode_request(
        1, DYNAMIXEL_PROTOCOL2_INST_WRITE, parameters, sizeof(parameters), packet, sizeof(packet),
        &packet_size);

    TEST_ASSERT_EQUAL(DYNAMIXEL_PROTOCOL2_OK, status);
    TEST_ASSERT_EQUAL_UINT32(sizeof(packet), packet_size);

    uint16_t expected_length = sizeof(packet) - 7;
    TEST_ASSERT_EQUAL_HEX8(expected_length & 0xFF, packet[5]);
    TEST_ASSERT_EQUAL_HEX8(expected_length >> 8, packet[6]);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_protocol2_encode_ping);
    RUN_TEST(test_protocol2_encode_write);
    RUN_TEST(test_protocol2_encode_broadcast);
    RUN_TEST(test_protocol2_encode_byte_stuffing);
    RUN_TEST(test_protocol2_encode_short_buffers);
    RUN_TEST(test_protocol2_encode_parameter_limit);

    return UNITY_END();
}
