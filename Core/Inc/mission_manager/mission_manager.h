#include "state_estimation/state.h"

#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

// rocket state machine state of flight

typedef enum{
    IDLE,
    E_STOP,
    RISE,
    HOVER,
    LOWER
} flight_state_t;

static void handle_orientation_estop(const state_t *state, flight_state_t *flight_state);
static void log_flight_state_if_changed(flight_state_t flight_state, uint32_t timestamp);

#endif
