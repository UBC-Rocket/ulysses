#ifndef ULYSSES_SVC_GNSS
#define ULYSSES_SVC_GNSS

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/svc/status.h"

/** @brief GNSS reciever backed by a vendor specific driver */
typedef struct svc_gnss svc_gnss_t;

/**
 * @brief Quality of the position solution
 *
 * Mapped from the GGA fix quality field, forced to NONE when RMC reports invalid
 */
enum svc_gnss_fix {
    SVC_GNSS_FIX_NONE,       ///< No usable position (GGA 0 or 6, or RMC status V)
    SVC_GNSS_FIX_STANDALONE, ///< Autonomous GNSS fix (GGA 1)
    SVC_GNSS_FIX_DGPS,       ///< Differential fix (GGA 2)
    SVC_GNSS_FIX_RTK_FLOAT,  ///< RTK with floating ambiguities (GGA 5)
    SVC_GNSS_FIX_RTK_FIXED,  ///< RTK with fixed ambiguities (GGA 4)
};

/**
 * @brief UTC time of the solution, as reported by the receiver
 */
struct svc_gnss_utc {
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
struct svc_gnss_data {
    double latitude;        ///< WGS84 latitude in degrees, positive north
    double longitude;       ///< WGS84 longitude in degrees, positive east
    float altitude_msl;     ///< Height above mean sea level in m
    float geoid_separation; ///< Ellipsoid minus geoid in m; ellipsoid height = msl + this

    float ground_speed; ///< Horizontal speed over ground in m/s
    float course;       ///< Course over ground in degrees from true north

    enum svc_gnss_fix fix;   ///< Quality of the position solution
    uint8_t satellite_count; ///< Satellites used in the solution
    float correction_age;    ///< Age of differential corrections in s, 0 if none

    struct svc_gnss_utc utc; ///< Receiver UTC time of the solution
};

/**
 * @brief Advances the reciver drivers
 *
 * Reads available receiver bytes and runs the driver state machine
 *
 * Call periodically from the same task that calls svc_gnss_get_data(),
 * and never from an interrupt. The driver is not safe for concurrent use
 *
 * @param gnss Non-NULL gnss intialized receiver
 *
 * @return Status returned by the backend
 */
enum svc_status svc_gnss_update(svc_gnss_t *gnss);

/**
 * @brief Checks whether a soltuion newer than the last call is avilable
 *
 * @param gnss Non-NULL intialized receiver
 *
 * @retval true A new solution is available
 * @retval false A new solution is NOT available
 */
bool svc_gnss_is_data_ready(const svc_gnss_t *gnss);

/**
 * @brief Copies out the latest solution and clears the data ready state
 *
 * Call from the same task as svc_gnss_update()
 *
 * @param gnss Non-NULL intialized receiver
 * @param[out] data Non-NULL output for the latest solution, unchanged if no data is available
 *
 * @retval SVC_STATUS_OK data was written
 * @retval SVC_STATUS_NO_DATA no solution is newer than the last call
 */
enum svc_status svc_gnss_get_data(svc_gnss_t *gnss, struct svc_gnss_data *data);

/**
 * @brief Forwards RTCM correction bytes to the receiver
 *
 * @param gnss Non-NULL intialized receiver
 * @param data Non-NULL input buffer containing at least len bytes
 * @param len Number of bytes to send; must be greater than 0
 *
 * @retval SVC_STATUS_UNSUPPORTED The backend does not support corrections
 * @return Status returned by the backend other
 */
enum svc_status svc_gnss_send_corrections(svc_gnss_t *gnss, const uint8_t *data, size_t len);

/**
 * @brief Resets the receiver
 *
 * @param gnss Non-NULL intialized receiver
 *
 * @retval SVC_STATUS_UNSUPPORTED The backend does not support reset
 * @return Status returned by the backend otherwise
 */
enum svc_status svc_gnss_reset(svc_gnss_t *gnss);

#endif // ULYSSES_SVC_GNSS
