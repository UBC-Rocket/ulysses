#ifndef ULYSSES_SVC_GNSS
#define ULYSSES_SVC_GNSS

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/io/status.h"

/** @brief GNSS reciever backed by a vendor specific driver */
typedef struct dev_gnss dev_gnss_t;

/**
 * @brief Quality of the position solution
 *
 * Mapped from the GGA fix quality field, forced to NONE when RMC reports invalid
 */
enum dev_gnss_fix {
    DEV_GNSS_FIX_NONE,       ///< No usable position (GGA 0 or 6, or RMC status V)
    DEV_GNSS_FIX_STANDALONE, ///< Autonomous GNSS fix (GGA 1)
    DEV_GNSS_FIX_DGPS,       ///< Differential fix (GGA 2)
    DEV_GNSS_FIX_RTK_FLOAT,  ///< RTK with floating ambiguities (GGA 5)
    DEV_GNSS_FIX_RTK_FIXED,  ///< RTK with fixed ambiguities (GGA 4)
};

/**
 * @brief UTC time of the solution, as reported by the receiver
 */
struct dev_gnss_utc {
    uint16_t year;  ///< Full year, e.g. 2026
    uint8_t month;  ///< 1 to 12
    uint8_t day;    ///< 1 to 31
    uint8_t hour;   ///< 0 to 23
    uint8_t minute; ///< 0 to 59
    uint8_t second; ///< 0 to 60, whole seconds only
    bool valid;     ///< True if both time and date were reported
};

/**
 * @brief Latest navigation solution from the receiver (NMEA GGA + RMC)
 */
struct dev_gnss_data {
    double latitude;        ///< WGS84 latitude in degrees, positive north
    double longitude;       ///< WGS84 longitude in degrees, positive east
    float altitude_msl;     ///< Height above mean sea level in m
    float geoid_separation; ///< Ellipsoid minus geoid in m; ellipsoid height = msl + this

    float ground_speed; ///< Horizontal speed over ground in m/s
    float course;       ///< Course over ground in degrees from true north

    enum dev_gnss_fix fix;   ///< Quality of the position solution
    uint8_t satellite_count; ///< Satellites used in the solution
    float correction_age;    ///< Age of differential corrections in s, 0 if none

    struct dev_gnss_utc utc; ///< Receiver UTC time of the solution
};

/**
 * @brief Advances the reciver drivers
 *
 * Reads available receiver bytes and runs the driver state machine
 *
 * Call periodically from the same task that calls dev_gnss_get_data(),
 * and never from an interrupt. The driver is not safe for concurrent use
 *
 * @param gnss Non-NULL gnss intialized receiver
 *
 * @return Status returned by the backend
 */
enum io_status dev_gnss_update(dev_gnss_t *gnss);

/**
 * @brief Checks whether a soltuion newer than the last call is avilable
 *
 * @param gnss Non-NULL intialized receiver
 *
 * @retval true A new solution is available
 * @retval false A new solution is NOT available
 */
bool dev_gnss_is_data_ready(const dev_gnss_t *gnss);

/**
 * @brief Copies out the latest solution and clears the data ready state
 *
 * Call from the same task as dev_gnss_update()
 *
 * @param gnss Non-NULL intialized receiver
 * @param[out] data Non-NULL output for the latest solution, unchanged if no data is available
 *
 * @retval IO_STATUS_OK data was written
 * @retval IO_STATUS_ERROR no solution is newer than the last call; data is unchanged
 */
enum io_status dev_gnss_get_data(dev_gnss_t *gnss, struct dev_gnss_data *data);

/**
 * @brief Forwards RTCM correction bytes to the receiver
 *
 * @param gnss Non-NULL intialized receiver
 * @param data Non-NULL input buffer containing at least len bytes
 * @param len Number of bytes to send; must be greater than 0
 *
 * @retval IO_STATUS_ERROR The backend does not support corrections
 * @return Status returned by the backend otherwise
 */
enum io_status dev_gnss_send_corrections(dev_gnss_t *gnss, const uint8_t *data, size_t len);

/**
 * @brief Resets the receiver
 *
 * @param gnss Non-NULL intialized receiver
 *
 * @retval IO_STATUS_ERROR The backend does not support reset
 * @return Status returned by the backend otherwise
 */
enum io_status dev_gnss_reset(dev_gnss_t *gnss);

#endif // ULYSSES_SVC_GNSS
