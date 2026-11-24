#include "unity.h"

#include <stddef.h>

#include "ulysses/lib/dynamixel/protocol2_instructions.h"

// Reference payloads: https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-details
// Reference payloads omit packet headers, opcodes, and final CRCs.

void setUp(void)
{
}

void tearDown(void)
{
}

void test_serialize_fast_sync_read_request(void)
{
    const uint8_t ids[] = {3, 7, 4};
    const struct dynamixel_protocol2_fast_sync_read_request request = {
        .address = 0x0084,
        .length = 4,
        .id_count = sizeof(ids),
        .ids = ids,
    };
    const uint8_t expected[] = {0x84, 0x00, 0x04, 0x00, 0x03, 0x07, 0x04};
    uint8_t parameters[sizeof(expected)] = {0};
    const uint8_t unchanged[sizeof(parameters)] = {0};
    uint16_t length = 1;

    TEST_ASSERT_EQUAL(ULS_STATUS_NO_SPACE,
                      dynamixel_protocol2_serialize_fast_sync_read_request(
                          &request, parameters, sizeof(parameters) - 1, &length));
    TEST_ASSERT_EQUAL_UINT16(1, length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(unchanged, parameters, sizeof(parameters));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_serialize_fast_sync_read_request(
                                         &request, parameters, sizeof(parameters), &length));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, parameters, length);
}

void test_serialize_sync_write_request(void)
{
    const uint8_t first[] = {0x96, 0x00, 0x00, 0x00};
    const uint8_t second[] = {0xAA, 0x00, 0x00, 0x00};
    const struct dynamixel_protocol2_sync_write_device devices[] = {
        {.id = 1, .data = first},
        {.id = 2, .data = second},
    };
    const struct dynamixel_protocol2_sync_write_request request = {
        .address = 0x0074,
        .length = sizeof(first),
        .device_count = 2,
        .devices = devices,
    };
    const uint8_t expected[] = {0x74, 0x00, 0x04, 0x00, 0x01, 0x96, 0x00,
                                0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00};
    uint8_t parameters[sizeof(expected)] = {0};
    const uint8_t unchanged[sizeof(parameters)] = {0};
    uint16_t length = 0;

    TEST_ASSERT_EQUAL(ULS_STATUS_NO_SPACE,
                      dynamixel_protocol2_serialize_sync_write_request(
                          &request, parameters, sizeof(parameters) - 1, &length));
    TEST_ASSERT_EQUAL_UINT16(0, length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(unchanged, parameters, sizeof(parameters));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_serialize_sync_write_request(
                                         &request, parameters, sizeof(parameters), &length));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, parameters, length);
}

void test_deserialize_ping_response(void)
{
    const uint8_t parameters[] = {0x00, 0x06, 0x04, 0x26};
    struct dynamixel_protocol2_ping_response ping = {0};

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_deserialize_ping_response(
                                         parameters, sizeof(parameters), &ping));
    TEST_ASSERT_EQUAL_HEX8(0x00, ping.error);
    TEST_ASSERT_EQUAL_HEX16(0x0406, ping.model);
    TEST_ASSERT_EQUAL_HEX8(0x26, ping.firmware);
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      dynamixel_protocol2_deserialize_ping_response(parameters, 3, &ping));
}

void test_deserialize_fast_sync_read_response(void)
{
    const uint8_t parameters[] = {0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, 0x00, 0x07, 0x1F,
                                  0x08, 0x00, 0x00, 0x16, 0xCA, 0x00, 0x04, 0xFF, 0x03, 0x00, 0x00};
    const struct {
        uint8_t id;
        uint16_t bytes_consumed;
    } expected[] = {{3, 8}, {7, 8}, {4, 6}};
    const uint16_t data_length = 4;
    uint16_t bytes_consumed = 1;
    struct dynamixel_protocol2_fast_sync_read_response response = {0};

    TEST_ASSERT_EQUAL(
        ULS_STATUS_INVALID_INPUT,
        dynamixel_protocol2_deserialize_fast_sync_read_response(
            parameters, sizeof(parameters) - 1, data_length, &response, &bytes_consumed));
    TEST_ASSERT_EQUAL_UINT16(1, bytes_consumed);
    TEST_ASSERT_NULL(response.data);

    uint16_t offset = 0;
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_deserialize_fast_sync_read_response(
                                             &parameters[offset], sizeof(parameters) - offset,
                                             data_length, &response, &bytes_consumed));
        TEST_ASSERT_EQUAL_HEX8(0x00, response.error);
        TEST_ASSERT_EQUAL_UINT8(expected[i].id, response.id);
        TEST_ASSERT_EQUAL_PTR(&parameters[offset + 2], response.data);
        TEST_ASSERT_EQUAL_UINT16(expected[i].bytes_consumed, bytes_consumed);
        offset += bytes_consumed;
    }
    TEST_ASSERT_EQUAL_UINT16(sizeof(parameters), offset);
}

// Non-ROBOTIS test cases.

void test_serialize_fast_sync_read_high_address(void)
{
    const uint8_t ids[] = {3};
    const struct dynamixel_protocol2_fast_sync_read_request request = {
        .address = 0x1234,
        .length = 4,
        .id_count = sizeof(ids),
        .ids = ids,
    };
    const uint8_t expected[] = {0x34, 0x12, 0x04, 0x00, 0x03};
    uint8_t parameters[sizeof(expected)] = {0};
    uint16_t length = 0;

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_serialize_fast_sync_read_request(
                                         &request, parameters, sizeof(parameters), &length));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, parameters, length);
}

void test_serialize_sync_write_high_address(void)
{
    const uint8_t data[] = {0x96};
    const struct dynamixel_protocol2_sync_write_device device = {.id = 1, .data = data};
    const struct dynamixel_protocol2_sync_write_request request = {
        .address = 0x1234,
        .length = sizeof(data),
        .device_count = 1,
        .devices = &device,
    };
    const uint8_t expected[] = {0x34, 0x12, 0x01, 0x00, 0x01, 0x96};
    uint8_t parameters[sizeof(expected)] = {0};
    uint16_t length = 0;

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_serialize_sync_write_request(
                                         &request, parameters, sizeof(parameters), &length));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, parameters, length);
}

void test_deserialize_ping_error(void)
{
    const uint8_t parameters[] = {0x81, 0x06, 0x04, 0x26};
    struct dynamixel_protocol2_ping_response response = {0};

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, dynamixel_protocol2_deserialize_ping_response(
                                         parameters, sizeof(parameters), &response));
    TEST_ASSERT_EQUAL_HEX8(0x81, response.error);
}

void test_deserialize_fast_sync_read_error(void)
{
    const uint8_t parameters[] = {0x81, 0x03, 0xA6, 0x00};
    struct dynamixel_protocol2_fast_sync_read_response response = {0};
    uint16_t bytes_consumed = 0;

    TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                      dynamixel_protocol2_deserialize_fast_sync_read_response(
                          parameters, sizeof(parameters), 2, &response, &bytes_consumed));
    TEST_ASSERT_EQUAL_HEX8(0x81, response.error);
    TEST_ASSERT_EQUAL_UINT16(sizeof(parameters), bytes_consumed);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_serialize_fast_sync_read_request);
    RUN_TEST(test_serialize_sync_write_request);
    RUN_TEST(test_deserialize_ping_response);
    RUN_TEST(test_deserialize_fast_sync_read_response);
    RUN_TEST(test_serialize_fast_sync_read_high_address);
    RUN_TEST(test_serialize_sync_write_high_address);
    RUN_TEST(test_deserialize_ping_error);
    RUN_TEST(test_deserialize_fast_sync_read_error);

    return UNITY_END();
}
