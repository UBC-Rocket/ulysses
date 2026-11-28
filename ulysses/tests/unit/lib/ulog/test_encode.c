#include "unity.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ulysses/lib/ulog.h"

#define MAX_RECORD_SIZE (65538)
#define GUARD (0xA5)

struct sink {
    uint8_t storage[MAX_RECORD_SIZE + 2];
    uint32_t length;
    uint32_t calls;
    enum uls_status error;
    uint32_t fail_after_bytes;
    bool failed;
};

static struct sink output;
static struct sink other_output;
static char long_text[65537];
static uint8_t long_data[65534];

static enum uls_status write_to_sink(const uint8_t *data, uint32_t length, void *user_data)
{
    struct sink *sink = user_data;
    TEST_ASSERT_TRUE(sink == &output || sink == &other_output);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_GREATER_THAN_UINT32(0, length);
    TEST_ASSERT_FALSE(sink->failed);
    sink->calls++;

    if (sink->error != ULS_STATUS_OK && sink->length >= sink->fail_after_bytes) {
        sink->failed = true;
        return sink->error;
    }

    if (length > MAX_RECORD_SIZE - sink->length) {
        sink->failed = true;
        return ULS_STATUS_NO_SPACE;
    }

    memcpy(sink->storage + 1 + sink->length, data, length);
    sink->length += length;
    return ULS_STATUS_OK;
}

static const struct ulog_writer writer = {write_to_sink, &output};

static inline void reset_sink(struct sink *sink)
{
    memset(sink->storage, GUARD, sizeof(sink->storage));
    sink->length = 0;
    sink->calls = 0;
    sink->error = ULS_STATUS_OK;
    sink->fail_after_bytes = 0;
    sink->failed = false;
}

void setUp(void)
{
    reset_sink(&output);
    reset_sink(&other_output);
}

static inline void assert_guards(const struct sink *sink)
{
    TEST_ASSERT_EQUAL_HEX8(GUARD, sink->storage[0]);
    TEST_ASSERT_EACH_EQUAL_HEX8(GUARD, sink->storage + 1 + sink->length,
                                MAX_RECORD_SIZE + 1 - sink->length);
}

void tearDown(void)
{
    assert_guards(&output);
    assert_guards(&other_output);
}

static inline void assert_output(const uint8_t *expected, uint32_t size)
{
    TEST_ASSERT_EQUAL_UINT32(size, output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, output.storage + 1, size);
    assert_guards(&output);
    reset_sink(&output);
}

void test_file_header_and_flag_bits(void)
{
    uint8_t expected[59] = {
        0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01, 0x08, 0x07,
        0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x28, 0x00, 0x42, 0x01,
    };
    uint64_t timestamp = UINT64_C(0x0102030405060708);

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_header(&writer, timestamp, true));
    assert_output(expected, sizeof(expected));

    expected[19] = 0;
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_header(&writer, timestamp, false));
    assert_output(expected, sizeof(expected));
}

void test_formats_and_information(void)
{
    const char schema[] = "sample:uint64_t timestamp;float value;";
    const uint8_t value[] = {'t', 'e', 's', 't'};
    const uint8_t expected[] = "\x26\x00"
                               "Fsample:uint64_t timestamp;float value;"
                               "\x11\x00I\x0C"
                               "char[4] nametest";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_format(&writer, schema));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                      ulog_write_info(&writer, "char[4] name", value, sizeof(value)));
    assert_output(expected, sizeof(expected) - 1);
}

void test_multi_information_records(void)
{
    const uint8_t first[] = {'a', 'b'};
    const uint8_t continuation[] = {'c', 'd', 'e'};
    const uint8_t expected[] = "\x12\x00M\x00\x0E"
                               "char[2] reportab"
                               "\x13\x00M\x01\x0E"
                               "char[3] reportcde"
                               "\x10\x00M\x00\x0E"
                               "char[0] report";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_info_multiple(&writer, false, "char[2] report",
                                                              first, sizeof(first)));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_info_multiple(&writer, true, "char[3] report",
                                                              continuation, sizeof(continuation)));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                      ulog_write_info_multiple(&writer, false, "char[0] report", NULL, 0));
    assert_output(expected, sizeof(expected) - 1);
}

void test_current_parameters(void)
{
    const uint8_t expected[] = "\x0E\x00P\x09int32_t X\xFE\xFF\xFF\xFF"
                               "\x0C\x00P\x07"
                               "float Y\x00\x00\x40\xBF";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_parameter_int32(&writer, "X", -2));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_parameter_float(&writer, "Y", -0.75f));
    assert_output(expected, sizeof(expected) - 1);
}

void test_default_parameters(void)
{
    const uint8_t expected[] = "\x0F\x00Q\x01\x09int32_t Y\x00\x00\x00\x80"
                               "\x0F\x00Q\x02\x09int32_t Y\x02\x01\x00\x00"
                               "\x0D\x00Q\x03\x07"
                               "float X\x00\x00\xC0\x3F";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_default_parameter_int32(
                                         &writer, ULOG_DEFAULT_TYPE_SYSTEM, "Y", INT32_MIN));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_default_parameter_int32(
                                         &writer, ULOG_DEFAULT_TYPE_CONFIGURATION, "Y", 258));
    TEST_ASSERT_EQUAL(
        ULS_STATUS_OK,
        ulog_write_default_parameter_float(
            &writer, ULOG_DEFAULT_TYPE_SYSTEM | ULOG_DEFAULT_TYPE_CONFIGURATION, "X", 1.5f));
    assert_output(expected, sizeof(expected) - 1);
}

void test_subscription_and_sample(void)
{
    const uint8_t subscription[] = "\x09\x00"
                                   "A\x02\x34\x12sample";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_subscription(&writer, 0x1234, 2, "sample"));
    assert_output(subscription, sizeof(subscription) - 1);

    const uint8_t sample[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F};
    const uint8_t expected[] = {0x0E, 0x00, 0x44, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F};

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_data(&writer, 0, sample, sizeof(sample)));
    assert_output(expected, sizeof(expected));
}

void test_logged_string_and_empty_text_at_each_severity(void)
{
    const uint8_t expected[] = {0x0B, 0x00, 0x4C, 0x36, 0x01, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x6B};

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_string(&writer, ULOG_LOG_LEVEL_INFO, 1, "ok"));
    assert_output(expected, sizeof(expected));

    const enum ulog_log_level levels[] = {
        ULOG_LOG_LEVEL_EMERG,   ULOG_LOG_LEVEL_ALERT,  ULOG_LOG_LEVEL_CRIT, ULOG_LOG_LEVEL_ERR,
        ULOG_LOG_LEVEL_WARNING, ULOG_LOG_LEVEL_NOTICE, ULOG_LOG_LEVEL_INFO, ULOG_LOG_LEVEL_DEBUG,
    };
    uint8_t empty[] = {0x09, 0x00, 0x4C, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};

    for (uint32_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
        empty[3] = 0x30 + i;
        TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                          ulog_write_string(&writer, levels[i], UINT64_C(0x0102030405060708), ""));
        assert_output(empty, sizeof(empty));
    }
}

void test_sync_dropout_and_independent_sinks(void)
{
    const struct ulog_writer other_writer = {write_to_sink, &other_output};
    const uint8_t expected[] = {0x08, 0x00, 0x53, 0x2F, 0x73, 0x13, 0x20, 0x25,
                                0x0C, 0xBB, 0x12, 0x02, 0x00, 0x4F, 0x02, 0x01};
    const uint8_t other_expected[] = {0x02, 0x00, 0x4F, 0xFF, 0xFF};

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_sync(&writer));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_dropout(&other_writer, UINT16_MAX));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_dropout(&writer, 258));

    assert_output(expected, sizeof(expected));
    TEST_ASSERT_EQUAL_UINT32(sizeof(other_expected), other_output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(other_expected, other_output.storage + 1, sizeof(other_expected));
}

static inline void assert_max_record(const uint8_t *prefix, uint32_t prefix_length,
                                     const void *tail)
{
    TEST_ASSERT_EQUAL_UINT32(MAX_RECORD_SIZE, output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(prefix, output.storage + 1, prefix_length);
    TEST_ASSERT_EQUAL_MEMORY(tail, output.storage + 1 + prefix_length,
                             MAX_RECORD_SIZE - prefix_length);
    assert_guards(&output);
    reset_sink(&output);
}

void test_maximum_format_subscription_and_message_lengths(void)
{
    memset(long_text, 'a', sizeof(long_text) - 1);
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_format(&writer, long_text));

    long_text[65535] = '\0';
    const uint8_t format_prefix[] = {0xFF, 0xFF, 0x46};
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_format(&writer, long_text));
    assert_max_record(format_prefix, sizeof(format_prefix), long_text);

    long_text[65533] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_subscription(&writer, 1, 2, long_text));

    long_text[65532] = '\0';
    const uint8_t subscription_prefix[] = {0xFF, 0xFF, 0x41, 0x02, 0x01, 0x00};
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_subscription(&writer, 1, 2, long_text));
    assert_max_record(subscription_prefix, sizeof(subscription_prefix), long_text);

    long_text[65527] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_string(&writer, ULOG_LOG_LEVEL_DEBUG, 0, long_text));

    long_text[65526] = '\0';
    const uint8_t message_prefix[12] = {0xFF, 0xFF, 0x4C, 0x37};
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                      ulog_write_string(&writer, ULOG_LOG_LEVEL_DEBUG, 0, long_text));
    assert_max_record(message_prefix, sizeof(message_prefix), long_text);
}

void test_maximum_information_and_sample_lengths(void)
{
    memset(long_data, 0x5A, sizeof(long_data));
    char key[257];
    memset(key, 'k', sizeof(key) - 1);
    key[256] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_info(&writer, key, long_data, 1));
    key[255] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_info(&writer, key, long_data, 65280));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_info(&writer, key, long_data, 65279));
    const uint8_t info_prefix[] = {0xFF, 0xFF, 0x49, 0xFF};
    TEST_ASSERT_EQUAL_UINT32(MAX_RECORD_SIZE, output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(info_prefix, output.storage + 1, sizeof(info_prefix));
    TEST_ASSERT_EQUAL_MEMORY(key, output.storage + 5, 255);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(long_data, output.storage + 260, 65279);
    assert_guards(&output);
    reset_sink(&output);

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_data(&writer, 0, long_data, 65534));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_data(&writer, 0xA1B2, long_data, 65533));
    const uint8_t sample_prefix[] = {0xFF, 0xFF, 0x44, 0xB2, 0xA1};
    assert_max_record(sample_prefix, sizeof(sample_prefix), long_data);
}

void test_multi_information_length_limits(void)
{
    const char key_type[] = "uint8_t[65278] ";
    char key[257];
    memset(key, 'k', sizeof(key) - 1);
    memcpy(key, key_type, sizeof(key_type) - 1);
    key[256] = '\0';
    memset(long_data, 0x5A, sizeof(long_data));

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_info_multiple(&writer, false, "", long_data, 1));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_info_multiple(&writer, false, key, long_data, 1));

    key[255] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_info_multiple(&writer, false, key, long_data, 65279));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_info_multiple(&writer, false, key, long_data, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);

    uint8_t expected_prefix[5 + 255] = {0xFF, 0xFF, 0x4D, 0x00, 0xFF};
    memcpy(expected_prefix + 5, key, 255);

    TEST_ASSERT_EQUAL(ULS_STATUS_OK,
                      ulog_write_info_multiple(&writer, false, key, long_data, 65278));
    assert_max_record(expected_prefix, sizeof(expected_prefix), long_data);
}

static inline void assert_max_parameter(const uint8_t *prefix, uint32_t prefix_length,
                                        const char *name, uint32_t name_length,
                                        const uint8_t *value)
{
    TEST_ASSERT_EQUAL_UINT32(prefix_length + name_length + 4, output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(prefix, output.storage + 1, prefix_length);
    TEST_ASSERT_EQUAL_MEMORY(name, output.storage + 1 + prefix_length, name_length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(value, output.storage + 1 + prefix_length + name_length, 4);
    assert_guards(&output);
    reset_sink(&output);
}

void test_maximum_typed_parameter_key_lengths(void)
{
    char name[251];
    memset(name, 'X', sizeof(name) - 1);
    name[250] = '\0';
    const uint8_t value[4] = {0};

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_parameter_float(&writer, name, 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_default_parameter_float(&writer, 1, name, 0));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);

    name[249] = '\0';
    const uint8_t float_prefix[] = "\x04\x01P\xFF"
                                   "float ";
    const uint8_t default_float_prefix[] = "\x05\x01Q\x01\xFF"
                                           "float ";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_parameter_float(&writer, name, 0));
    assert_max_parameter(float_prefix, sizeof(float_prefix) - 1, name, 249, value);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_default_parameter_float(&writer, 1, name, 0));
    assert_max_parameter(default_float_prefix, sizeof(default_float_prefix) - 1, name, 249, value);

    name[248] = '\0';
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_parameter_int32(&writer, name, 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_default_parameter_int32(&writer, 2, name, 0));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);

    name[247] = '\0';
    const uint8_t int_prefix[] = "\x04\x01P\xFFint32_t ";
    const uint8_t default_int_prefix[] = "\x05\x01Q\x02\xFFint32_t ";

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_parameter_int32(&writer, name, 0));
    assert_max_parameter(int_prefix, sizeof(int_prefix) - 1, name, 247, value);
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_default_parameter_int32(&writer, 2, name, 0));
    assert_max_parameter(default_int_prefix, sizeof(default_int_prefix) - 1, name, 247, value);
}

void test_bare_name_character_set(void)
{
    const char valid_name[] = "AZaz09_-/";
    const uint8_t expected[] = "\x0C\x00"
                               "A\x00\x00\x00"
                               "AZaz09_-/"
                               "\x16\x00P\x11int32_t AZaz09_-/\x00\x00\x00\x00";
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_subscription(&writer, 0, 0, valid_name));
    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_parameter_int32(&writer, valid_name, 0));
    assert_output(expected, sizeof(expected) - 1);

    const char *invalid_names[] = {"", "a b", "a.b", "a:b", "a\n", "\xC3\xA9"};
    for (uint32_t i = 0; i < sizeof(invalid_names) / sizeof(invalid_names[0]); i++) {
        TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                          ulog_write_subscription(&writer, 0, 0, invalid_names[i]));
        TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                          ulog_write_parameter_int32(&writer, invalid_names[i], 0));
        TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                          ulog_write_default_parameter_float(&writer, 1, invalid_names[i], 0));
    }
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);
}

void test_invalid_payloads_flags_and_severities_emit_nothing(void)
{
    const uint8_t data[8] = {0};

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_format(&writer, ""));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_info(&writer, "", data, 1));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_info(&writer, "uint8_t X", data, 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_info(&writer, "uint8_t X", data, UINT32_MAX));

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_data(&writer, 0, data, 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_data(&writer, 0, data, 7));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT, ulog_write_data(&writer, 0, data, UINT32_MAX));

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_default_parameter_int32(&writer, 0, "X", 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_default_parameter_int32(&writer, 4, "X", 0));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_default_parameter_float(&writer, 0x81, "X", 0));

    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_string(&writer, (enum ulog_log_level)0, 0, "ok"));
    TEST_ASSERT_EQUAL(ULS_STATUS_INVALID_INPUT,
                      ulog_write_string(&writer, (enum ulog_log_level)'8', 0, "ok"));
    TEST_ASSERT_EQUAL_UINT32(0, output.calls);

    TEST_ASSERT_EQUAL(ULS_STATUS_OK, ulog_write_data(&writer, 0, data, sizeof(data)));
    const uint8_t expected[13] = {0x0A, 0x00, 0x44};
    assert_output(expected, sizeof(expected));
}

static inline enum uls_status write_without_context(const uint8_t *data, uint32_t length,
                                                    void *user_data)
{
    TEST_ASSERT_NULL(user_data);
    return write_to_sink(data, length, &output);
}

void test_callback_error_before_any_bytes(void)
{
    const struct ulog_writer no_context = {write_without_context, NULL};
    output.error = ULS_STATUS_TIMEOUT;

    TEST_ASSERT_EQUAL(ULS_STATUS_TIMEOUT,
                      ulog_write_format(&no_context, "sample:uint64_t timestamp;"));
    TEST_ASSERT_TRUE(output.failed);
    TEST_ASSERT_EQUAL_UINT32(0, output.length);
}

void test_callback_error_after_emitting_bytes(void)
{
    const uint8_t value[] = {0x5A};
    const uint8_t expected[] = "\x0B\x00I\x09uint8_t XZ";
    output.error = ULS_STATUS_NO_SPACE;
    output.fail_after_bytes = 1;

    TEST_ASSERT_EQUAL(ULS_STATUS_NO_SPACE,
                      ulog_write_info(&writer, "uint8_t X", value, sizeof(value)));
    TEST_ASSERT_TRUE(output.failed);
    TEST_ASSERT_GREATER_THAN_UINT32(0, output.length);
    TEST_ASSERT_LESS_THAN_UINT32(sizeof(expected) - 1, output.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, output.storage + 1, output.length);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_file_header_and_flag_bits);
    RUN_TEST(test_formats_and_information);
    RUN_TEST(test_multi_information_records);
    RUN_TEST(test_current_parameters);
    RUN_TEST(test_default_parameters);
    RUN_TEST(test_subscription_and_sample);
    RUN_TEST(test_logged_string_and_empty_text_at_each_severity);
    RUN_TEST(test_sync_dropout_and_independent_sinks);
    RUN_TEST(test_maximum_format_subscription_and_message_lengths);
    RUN_TEST(test_maximum_information_and_sample_lengths);
    RUN_TEST(test_multi_information_length_limits);
    RUN_TEST(test_maximum_typed_parameter_key_lengths);
    RUN_TEST(test_bare_name_character_set);
    RUN_TEST(test_invalid_payloads_flags_and_severities_emit_nothing);
    RUN_TEST(test_callback_error_before_any_bytes);
    RUN_TEST(test_callback_error_after_emitting_bytes);

    return UNITY_END();
}
