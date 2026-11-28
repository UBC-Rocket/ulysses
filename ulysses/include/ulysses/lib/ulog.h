#ifndef ULYSSES_LIB_ULOG
#define ULYSSES_LIB_ULOG

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/status.h"

/**
 * @brief Output callback for a ULog stream
 *
 * Consume or copy the bytes during the callback. Do not call the encoder on the same stream from
 * the callback.
 *
 * @param data Bytes to append. Valid only during the callback.
 * @param length Nonzero byte count
 * @param user_data Context supplied by the writer. May be NULL.
 *
 * @retval ULS_STATUS_OK All bytes were accepted
 * @return A ULS_STATUS_* error code if the write failed
 */
typedef enum uls_status (*ulog_write_cb_t)(const uint8_t *data, uint32_t length, void *user_data);

/**
 * @brief Output configuration for a ULog stream
 *
 * Serialize writes to the same stream. Keep the writer configuration and record inputs valid
 * and unchanged until each call returns.
 *
 * The encoder may invoke the callback more than once. It returns the first callback error
 * unchanged, including ULS_STATUS_INVALID_INPUT if returned by the callback. A callback error
 * can leave a partial record. Recover to a known record boundary or start a new stream before
 * writing again. Local validation failures leave the stream untouched.
 */
struct ulog_writer {
    ulog_write_cb_t write; ///< Non-NULL output callback.
    void *user_data;       ///< Context passed to write.
};

/** @brief ULog diagnostic severity */
enum ulog_log_level {
    ULOG_LOG_LEVEL_EMERG = '0',   ///< System unusable
    ULOG_LOG_LEVEL_ALERT = '1',   ///< Immediate action needed
    ULOG_LOG_LEVEL_CRIT = '2',    ///< Critical condition
    ULOG_LOG_LEVEL_ERR = '3',     ///< Error condition
    ULOG_LOG_LEVEL_WARNING = '4', ///< Warning condition
    ULOG_LOG_LEVEL_NOTICE = '5',  ///< Significant normal condition
    ULOG_LOG_LEVEL_INFO = '6',    ///< Informational message
    ULOG_LOG_LEVEL_DEBUG = '7',   ///< Debug message
};

/** @brief Default parameter categories. Combine flags when the same default applies. */
enum ulog_default_type {
    ULOG_DEFAULT_TYPE_SYSTEM = (1 << 0),        ///< System-wide default
    ULOG_DEFAULT_TYPE_CONFIGURATION = (1 << 1), ///< Current configuration default
};

/**
 * @brief Starts a ULog v1 stream
 *
 * Call once on an empty stream.
 *
 * @param writer Configured ULog writer
 * @param timestamp_us Start time in microseconds in the same clock domain as samples
 * @param has_default_parameters True if the stream will contain default parameter records
 *
 * @retval ULS_STATUS_OK File header and flag bits record were written
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_header(const struct ulog_writer *writer, uint64_t timestamp_us,
                                  bool has_default_parameters);

/**
 * @brief Declares a format for subsequent subscriptions
 *
 * Write all format definitions before the first subscription or logged string.
 *
 * @param writer Configured ULog writer
 * @param format Complete NUL-terminated ULog schema, up to 65535 bytes before the terminator
 *
 * @retval ULS_STATUS_OK Format definition was written
 * @retval ULS_STATUS_INVALID_INPUT Format is empty or too long
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_format(const struct ulog_writer *writer, const char *format);

/**
 * @brief Writes a ULog information record
 *
 * @param writer Configured ULog writer
 * @param key NUL-terminated typed key
 * @param value Serialized value
 * @param value_length Value size in bytes
 *
 * @retval ULS_STATUS_OK Record was written
 * @retval ULS_STATUS_INVALID_INPUT Invalid key or value length
 *
 * @see [PX4 ULog Information Message](https://docs.px4.io/main/en/dev_log/ulog_file_format#i-information-message)
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_info(const struct ulog_writer *writer, const char *key,
                                const uint8_t *value, uint32_t value_length);

/**
 * @brief Writes a ULog Multi Information record
 *
 * @param writer Configured ULog writer
 * @param is_continued True to append to the preceding value for the same name
 * @param key NUL-terminated typed key for this chunk
 * @param value Serialized chunk. May be NULL when value_length is zero.
 * @param value_length Chunk size in bytes
 *
 * @retval ULS_STATUS_OK Record was written
 * @retval ULS_STATUS_INVALID_INPUT Invalid key or value length
 *
 * @see [PX4 ULog Multi Information Message](https://docs.px4.io/main/en/dev_log/ulog_file_format#m-multi-information-message)
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_info_multiple(const struct ulog_writer *writer, bool is_continued,
                                         const char *key, const uint8_t *value,
                                         uint32_t value_length);

/**
 * @brief Records an integer parameter's current value
 *
 * Use the definitions section for initial values and the data section for changes.
 *
 * @param writer Configured ULog writer
 * @param name Bare NUL-terminated name, up to 247 bytes. Use ASCII letters, digits, underscores,
 *             hyphens, or slashes.
 * @param value Current parameter value
 *
 * @retval ULS_STATUS_OK Parameter value was written
 * @retval ULS_STATUS_INVALID_INPUT Name is empty, too long, or contains an invalid character
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_parameter_int32(const struct ulog_writer *writer, const char *name,
                                           int32_t value);

/**
 * @brief Records a floating-point parameter's current value
 *
 * Like ulog_write_parameter_int32(), with an IEEE-754 binary32 value.
 *
 * @param writer Configured ULog writer
 * @param name Bare NUL-terminated name, up to 249 bytes. Uses the same characters as integer names.
 * @param value Current parameter value
 *
 * @retval ULS_STATUS_OK Parameter value was written
 * @retval ULS_STATUS_INVALID_INPUT Name is empty, too long, or contains an invalid character
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_parameter_float(const struct ulog_writer *writer, const char *name,
                                           float value);

/**
 * @brief Records an integer parameter's default value
 *
 * Set has_default_parameters when starting the log. A default can appear in either section.
 *
 * @param writer Configured ULog writer
 * @param default_types Nonzero combination of ULOG_DEFAULT_TYPE_* flags
 * @param name Same name requirements as ulog_write_parameter_int32()
 * @param value Default parameter value
 *
 * @retval ULS_STATUS_OK Default value was written
 * @retval ULS_STATUS_INVALID_INPUT Invalid parameter name or default category flags
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_default_parameter_int32(const struct ulog_writer *writer,
                                                   uint8_t default_types, const char *name,
                                                   int32_t value);

/**
 * @brief Records a floating-point parameter's default value
 *
 * Like ulog_write_default_parameter_int32(), with an IEEE-754 binary32 value.
 *
 * @param writer Configured ULog writer
 * @param default_types Nonzero combination of ULOG_DEFAULT_TYPE_* flags
 * @param name Same name requirements as ulog_write_parameter_float()
 * @param value Default parameter value
 *
 * @retval ULS_STATUS_OK Default value was written
 * @retval ULS_STATUS_INVALID_INPUT Invalid parameter name or default category flags
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_default_parameter_float(const struct ulog_writer *writer,
                                                   uint8_t default_types, const char *name,
                                                   float value);

/**
 * @brief Associates a previously defined format with a message ID
 *
 * Subscribe before writing data for the ID. Allocate IDs in increasing order starting at zero
 * and never reuse them within the stream.
 *
 * @param writer Configured ULog writer
 * @param msg_id Caller-assigned subscription identifier
 * @param multi_id Instance number, starting at zero for each format
 * @param message_name NUL-terminated format name, up to 65532 bytes. Uses the same characters as
 *                     parameter names.
 *
 * @retval ULS_STATUS_OK Subscription was written
 * @retval ULS_STATUS_INVALID_INPUT Name is empty, too long, or contains an invalid character
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_subscription(const struct ulog_writer *writer, uint16_t msg_id,
                                        uint8_t multi_id, const char *message_name);

/**
 * @brief Writes one sample for a subscription
 *
 * The subscribed format must include a uint64_t field named timestamp, in microseconds.
 * Timestamps must increase monotonically within each subscription.
 *
 * @param writer Configured ULog writer
 * @param msg_id Previously subscribed message ID
 * @param data Sample bytes in the subscribed format, including padding required by the format
 * @param data_length Sample size from 8 through 65533 bytes
 *
 * @retval ULS_STATUS_OK Sample was written
 * @retval ULS_STATUS_INVALID_INPUT Sample length is out of range
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_data(const struct ulog_writer *writer, uint16_t msg_id,
                                const uint8_t *data, uint32_t data_length);

/**
 * @brief Writes a diagnostic message in the data section
 *
 * @param writer Configured ULog writer
 * @param level Diagnostic severity
 * @param timestamp_us Time of the diagnostic in microseconds in the same clock domain as samples
 * @param message NUL-terminated text, up to 65526 bytes before the terminator. May be empty.
 *
 * @retval ULS_STATUS_OK Message was written
 * @retval ULS_STATUS_INVALID_INPUT Invalid severity or message is too long
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_string(const struct ulog_writer *writer, enum ulog_log_level level,
                                  uint64_t timestamp_us, const char *message);

/**
 * @brief Marks a point where readers can resynchronize in the data section
 *
 * @param writer Configured ULog writer
 *
 * @retval ULS_STATUS_OK Synchronization marker was written
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_sync(const struct ulog_writer *writer);

/**
 * @brief Records lost logging time in the data section
 *
 * @param writer Configured ULog writer
 * @param duration_ms Duration measured by the caller in milliseconds
 *
 * @retval ULS_STATUS_OK Dropout was recorded
 *
 * @see ulog_writer
 */
enum uls_status ulog_write_dropout(const struct ulog_writer *writer, uint16_t duration_ms);

#endif // ULYSSES_LIB_ULOG
