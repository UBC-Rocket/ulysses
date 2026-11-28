#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "ulysses/lib/ulog.h"
#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"

#define RECORD_HEADER_SIZE (3)
#define FILE_HEADER_SIZE (16)
#define FLAG_BITS_SIZE (40)
#define PARAMETER_VALUE_SIZE (4)

static const char int32_type[] = "int32_t ";
static const char float_type[] = "float ";

static_assert(sizeof(float) == 4, "floats must be 4 bytes wide");

static inline void assert_writer(const struct ulog_writer *writer)
{
    ULYSSES_ASSERT(writer != NULL, "ulog writer must not be NULL");
    ULYSSES_ASSERT(writer->write != NULL, "ulog write callback must not be NULL");
}

static inline uint32_t float_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static inline void write_record_header(uint8_t *header, uint16_t payload_length, uint8_t type)
{
    buffer_write_uint16_le(header, payload_length);
    header[2] = type;
}

static inline enum uls_status write_prefixed_bytes(const struct ulog_writer *writer,
                                                   const uint8_t *prefix, uint32_t prefix_length,
                                                   const void *bytes, uint32_t length)
{
    enum uls_status status = writer->write(prefix, prefix_length, writer->user_data);
    if (status != ULS_STATUS_OK || length == 0) {
        return status;
    }

    return writer->write(bytes, length, writer->user_data);
}

static inline uint32_t bounded_string_length(const char *string, uint32_t max_length)
{
    uint32_t length = 0;
    while (length <= max_length && string[length] != '\0') {
        length++;
    }
    return length;
}

static inline bool is_name_character(char character)
{
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_' || character == '-' ||
           character == '/';
}

static inline uint32_t valid_name_length(const char *name, uint32_t max_length)
{
    uint32_t length = 0;
    while (length < max_length && is_name_character(name[length])) {
        length++;
    }
    return name[length] == '\0' ? length : 0;
}

static enum uls_status write_info_record(const struct ulog_writer *writer, uint8_t record_type,
                                         bool is_continued, const char *key, const uint8_t *value,
                                         uint32_t value_length)
{
    assert_writer(writer);
    ULYSSES_ASSERT(key != NULL, "ulog information key must not be NULL");

    bool is_multiple = record_type == 'M';
    ULYSSES_ASSERT(value != NULL || (is_multiple && value_length == 0),
                   "ulog information value must not be NULL");

    uint32_t fields_length = is_multiple ? 2 : 1;
    uint32_t key_length = bounded_string_length(key, UINT8_MAX);
    if (key_length == 0 || key_length > UINT8_MAX || (!is_multiple && value_length == 0) ||
        value_length > UINT16_MAX - fields_length - key_length) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t prefix[RECORD_HEADER_SIZE + 2];
    uint16_t payload_length = (uint16_t)(fields_length + key_length + value_length);
    write_record_header(prefix, payload_length, record_type);

    uint32_t prefix_length = RECORD_HEADER_SIZE;
    if (is_multiple) {
        prefix[prefix_length++] = is_continued;
    }
    prefix[prefix_length++] = (uint8_t)key_length;

    enum uls_status status = write_prefixed_bytes(writer, prefix, prefix_length, key, key_length);
    if (status != ULS_STATUS_OK || value_length == 0) {
        return status;
    }

    return writer->write(value, value_length, writer->user_data);
}

static enum uls_status write_parameter(const struct ulog_writer *writer, bool is_default,
                                       uint8_t default_types, const char *name, const char *type,
                                       uint32_t value)
{
    assert_writer(writer);
    ULYSSES_ASSERT(name != NULL, "ulog parameter name must not be NULL");

    if (is_default && (default_types == 0 || default_types > (ULOG_DEFAULT_TYPE_SYSTEM |
                                                              ULOG_DEFAULT_TYPE_CONFIGURATION))) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint32_t type_length = (uint32_t)strlen(type);
    uint32_t length = valid_name_length(name, UINT8_MAX - type_length);
    if (length == 0) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t key_length = (uint8_t)(type_length + length);
    uint8_t prefix[RECORD_HEADER_SIZE + 2 + sizeof(int32_type) - 1];
    uint16_t payload_length = is_default + 1 + key_length + PARAMETER_VALUE_SIZE;
    write_record_header(prefix, payload_length, is_default ? 'Q' : 'P');

    uint32_t offset = RECORD_HEADER_SIZE;
    if (is_default) {
        prefix[offset++] = default_types;
    }
    prefix[offset++] = key_length;
    memcpy(prefix + offset, type, type_length);

    enum uls_status status =
        write_prefixed_bytes(writer, prefix, offset + type_length, name, length);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    uint8_t value_bytes[PARAMETER_VALUE_SIZE];
    buffer_write_uint32_le(value_bytes, value);

    return writer->write(value_bytes, sizeof(value_bytes), writer->user_data);
}

enum uls_status ulog_write_header(const struct ulog_writer *writer, uint64_t timestamp_us,
                                  bool has_default_parameters)
{
    assert_writer(writer);

    uint8_t header[FILE_HEADER_SIZE + RECORD_HEADER_SIZE + FLAG_BITS_SIZE] = {
        'U', 'L', 'o', 'g', 0x01, 0x12, 0x35, 0x01,
    };
    buffer_write_uint64_le(header + 8, timestamp_us);

    write_record_header(header + FILE_HEADER_SIZE, FLAG_BITS_SIZE, 'B');
    header[FILE_HEADER_SIZE + RECORD_HEADER_SIZE] = has_default_parameters;

    return writer->write(header, sizeof(header), writer->user_data);
}

enum uls_status ulog_write_format(const struct ulog_writer *writer, const char *format)
{
    assert_writer(writer);
    ULYSSES_ASSERT(format != NULL, "ulog format must not be NULL");

    uint32_t length = bounded_string_length(format, UINT16_MAX);
    if (length == 0 || length > UINT16_MAX) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t header[RECORD_HEADER_SIZE];
    write_record_header(header, (uint16_t)length, 'F');

    return write_prefixed_bytes(writer, header, sizeof(header), format, length);
}

enum uls_status ulog_write_info(const struct ulog_writer *writer, const char *key,
                                const uint8_t *value, uint32_t value_length)
{
    return write_info_record(writer, 'I', false, key, value, value_length);
}

enum uls_status ulog_write_info_multiple(const struct ulog_writer *writer, bool is_continued,
                                         const char *key, const uint8_t *value,
                                         uint32_t value_length)
{
    return write_info_record(writer, 'M', is_continued, key, value, value_length);
}

enum uls_status ulog_write_parameter_int32(const struct ulog_writer *writer, const char *name,
                                           int32_t value)
{
    return write_parameter(writer, false, 0, name, int32_type, (uint32_t)value);
}

enum uls_status ulog_write_parameter_float(const struct ulog_writer *writer, const char *name,
                                           float value)
{
    return write_parameter(writer, false, 0, name, float_type, float_bits(value));
}

enum uls_status ulog_write_default_parameter_int32(const struct ulog_writer *writer,
                                                   uint8_t default_types, const char *name,
                                                   int32_t value)
{
    return write_parameter(writer, true, default_types, name, int32_type, (uint32_t)value);
}

enum uls_status ulog_write_default_parameter_float(const struct ulog_writer *writer,
                                                   uint8_t default_types, const char *name,
                                                   float value)
{
    return write_parameter(writer, true, default_types, name, float_type, float_bits(value));
}

enum uls_status ulog_write_subscription(const struct ulog_writer *writer, uint16_t msg_id,
                                        uint8_t multi_id, const char *message_name)
{
    assert_writer(writer);
    ULYSSES_ASSERT(message_name != NULL, "ulog message name must not be NULL");

    uint32_t length = valid_name_length(message_name, UINT16_MAX - 3);
    if (length == 0) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t prefix[RECORD_HEADER_SIZE + 3];
    write_record_header(prefix, (uint16_t)(3 + length), 'A');
    prefix[RECORD_HEADER_SIZE] = multi_id;
    buffer_write_uint16_le(prefix + RECORD_HEADER_SIZE + 1, msg_id);

    return write_prefixed_bytes(writer, prefix, sizeof(prefix), message_name, length);
}

enum uls_status ulog_write_data(const struct ulog_writer *writer, uint16_t msg_id,
                                const uint8_t *data, uint32_t data_length)
{
    assert_writer(writer);
    ULYSSES_ASSERT(data != NULL, "ulog sample data must not be NULL");

    if (data_length < 8 || data_length > UINT16_MAX - 2) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t prefix[RECORD_HEADER_SIZE + 2];
    write_record_header(prefix, (uint16_t)(2 + data_length), 'D');
    buffer_write_uint16_le(prefix + RECORD_HEADER_SIZE, msg_id);

    return write_prefixed_bytes(writer, prefix, sizeof(prefix), data, data_length);
}

enum uls_status ulog_write_string(const struct ulog_writer *writer, enum ulog_log_level level,
                                  uint64_t timestamp_us, const char *message)
{
    assert_writer(writer);
    ULYSSES_ASSERT(message != NULL, "ulog logged string must not be NULL");

    uint32_t length = bounded_string_length(message, UINT16_MAX - 9);
    if (level < ULOG_LOG_LEVEL_EMERG || level > ULOG_LOG_LEVEL_DEBUG || length > UINT16_MAX - 9) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint8_t prefix[RECORD_HEADER_SIZE + 9];
    write_record_header(prefix, (uint16_t)(9 + length), 'L');
    prefix[RECORD_HEADER_SIZE] = (uint8_t)level;
    buffer_write_uint64_le(prefix + RECORD_HEADER_SIZE + 1, timestamp_us);

    return write_prefixed_bytes(writer, prefix, sizeof(prefix), message, length);
}

enum uls_status ulog_write_sync(const struct ulog_writer *writer)
{
    assert_writer(writer);

    static const uint8_t sync_marker[] = {0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12};
    uint8_t record[RECORD_HEADER_SIZE + sizeof(sync_marker)];
    write_record_header(record, sizeof(sync_marker), 'S');
    memcpy(record + RECORD_HEADER_SIZE, sync_marker, sizeof(sync_marker));

    return writer->write(record, sizeof(record), writer->user_data);
}

enum uls_status ulog_write_dropout(const struct ulog_writer *writer, uint16_t duration_ms)
{
    assert_writer(writer);

    uint8_t record[RECORD_HEADER_SIZE + 2];
    write_record_header(record, 2, 'O');
    buffer_write_uint16_le(record + RECORD_HEADER_SIZE, duration_ms);

    return writer->write(record, sizeof(record), writer->user_data);
}
