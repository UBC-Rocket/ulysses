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
 * Derived from the receiver's fix type, fix valid flag and carrier solution state
 */
enum svc_gnss_fix_type {
    SVC_GNSS_FIX_NONE,      ///< No usable position
    SVC_GNSS_FIX_2D,        ///< Position without a valid altitude
    SVC_GNSS_FIX_3D,        ///< Standalone 3D position
    SVC_GNSS_FIX_RTK_FLOAT, ///< 3D position with a floating ambiguity RTK Solution
    SVC_GNSS_FIX_RTK_FIXED, ///< 3D position with a fixeed amviguity RTK solution
};

/**
 * @brief Latest navigation solution from the receiver (NAV-PVT)
 *
 * TODO: Member set, types and units are under review, UTC time with validity flags is not yet included.
 *
 * Fields keep the receiver's native units for now
 */
struct svc_gnss_data {
    int32_t latitude;           ///< WGS84 latitude in 1e-7 degrees, positive north
    int32_t longitude;          ///< WGS84 longitude in 1e-7 degrees, positive east
    int32_t altitude_ellipsoid; ///< Height above the WGS84 ellipsoid in mm
    int32_t altitude_msl;       ///< Height above mean sea level in mm

    uint32_t horizontal_accuracy; ///< Estimated horizontal accuracy in mm
    uint32_t vertical_accuracy;   ///< Estimated vertical accruracy in mm

    int32_t velocity_north; ///< North velocity in mm per s
    int32_t velocity_east;  ///< East velocity in mm per s
    int32_t velocity_down;  ///< Down velocity in mm per s

    uint32_t speed_accuracy; ///< Estimated speed accuracy in mm per s

    uint32_t time_of_week;      ///< GPS time of week of the solution in ms
    enum svc_gnss_fix_type fix; ///< Quality of the position solution
    uint32_t satellite_count;   ///< Number of satellites used in the solution
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
