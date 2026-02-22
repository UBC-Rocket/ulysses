#ifndef STATE_EXCHANGE_H
#define STATE_EXCHANGE_H

#include <stdint.h>
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"

/**
 * @brief Initialise synchronization primitives for state sharing.
 *        Safe to call multiple times.
 */
void state_exchange_init(void);

/**
 * @brief Publish the latest fused state.
 * @param state Pointer to state structure.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_state(const state_t *state);

/**
 * @brief Copy the most recently published state.
 * @param state_out Destination pointer (optional).
 * @return Sequence number associated with the returned state.
 */
uint32_t state_exchange_get_state(state_t *state_out);

/**
 * @brief Publish current flight state.
 * @param flight_state New flight state.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_flight_state(flight_state_t flight_state);

/**
 * @brief Copy the latest flight state.
 * @param flight_state_out Destination pointer (optional).
 * @return Sequence number associated with the returned flight state.
 */
uint32_t state_exchange_get_flight_state(flight_state_t *flight_state_out);


#endif /* STATE_EXCHANGE_H */
