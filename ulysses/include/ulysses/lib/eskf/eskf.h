#ifndef ULYSSES_LIB_ESKF_ESKF_H
#define ULYSSES_LIB_ESKF_ESKF_H

#define ESKF_ERROR_STATE_SIZE (15)

typedef enum {
    ESKF_STATUS_OK,
    ESKF_STATUS_INVALID_ARGUMENT,
    ESKF_STATUS_NUMERICAL_ERROR,
    ESKF_STATUS_MEASUREMENT_REJECTED,
} eskf_status_t;

// Nav-frame = North/East/Down
// Body-frame = forward/right/down
typedef struct {
    float position[3];           // Navigation frame, metres.
    float velocity[3];           // Navigation frame, metres/second.
    float orientation[4];    // Unit quaternion [w, x, y, z], body to navigation.
    float accelerometer_bias[3]; // Body frame, metres/second^2.
    float gyroscope_bias[3];     // Body frame, radians/second.
} eskf_state_t;

typedef struct {
    float gravity[3];      // Navigation frame, metres/second^2.
    float accelerometer_noise_density;    // metres/second^2/sqrt(Hz).
    float gyroscope_noise_density;        // radians/second/sqrt(Hz).
    float accelerometer_bias_random_walk; // metres/second^2/sqrt(second).
    float gyroscope_bias_random_walk;     // radians/second/sqrt(second).
    // Initial covariance diagonal: position, velocity, attitude error,
    // accelerometer bias, gyroscope bias (three components each).
    float initial_variance[ESKF_ERROR_STATE_SIZE];
} eskf_config_t;

typedef struct {
    float specific_force[3];   // Body frame, metres/second^2; includes bias.
    float angular_velocity[3]; // Body frame, radians/second; includes bias.
} eskf_imu_sample_t;

typedef struct {
    float position[3];
    float covariance[3][3]; // Navigation frame, metres^2.
} eskf_position_measurement_t;

typedef struct {
    eskf_state_t state;
    eskf_config_t config;
    float covariance[ESKF_ERROR_STATE_SIZE][ESKF_ERROR_STATE_SIZE];
} eskf_t;

// unit quat, covariance diagonal
eskf_status_t eskf_init(eskf_t *filter, const eskf_config_t *config,
                        const eskf_state_t *initial_state);

eskf_status_t eskf_reset(eskf_t *filter, const eskf_state_t *initial_state);

void eskf_get_state(const eskf_t *filter, eskf_state_t *state);

eskf_status_t eskf_predict(eskf_t *filter, const eskf_imu_sample_t *imu, float dt);

eskf_status_t eskf_update_position(eskf_t *filter, const eskf_position_measurement_t *measurement);

#endif // ULYSSES_LIB_ESKF_ESKF_H
