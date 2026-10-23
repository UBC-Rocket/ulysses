#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONFIG
#define ULYSSES_LIB_CONTROLS_FLIGHT_CONFIG


typedef struct fc_vehicle_properties {
    float mass_kg;
    float gimbal_pivot_body_m[3];
    float centre_of_mass_m[3];
    float inertia_kg_m2[3][3];

    float gimbal_max_x;
    float gimbal_max_y;
} fc_vehicle_properties_t;


typedef struct fc_environment_properties {
    float gravity_N_kg;
    float ref_pressure_pa;
    float ref_magnetic_m[3];
} fc_environment_properties_t;


typedef struct fc_torque_controller_config {
    float Kp[3][3];
    float Ki[3][3];
    float Kd[3][3];
} fc_torque_controller_config_t;


typedef struct fc_gimbal_controller_config {
    float kp_x;
    float ki_x;
    float kd_x;

    float kp_y;
    float ki_y;
    float kd_y;
} fc_gimbal_controller_config_t;


typedef struct fc_config {
    fc_vehicle_properties_t vehicle_cfg;
    fc_environment_properties_t environment_cfg;
    fc_torque_controller_config_t torque_gains;
    fc_gimbal_controller_config_t gimbal_gains;
} fc_config_t;

#endif /* ULYSSES_LIB_CONTROLS_FLIGHT_CONFIG */
