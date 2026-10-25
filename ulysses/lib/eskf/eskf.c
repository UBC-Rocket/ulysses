#include "ulysses/lib/eskf/eskf.h"

// current architecture has the firmware maintaining an eskf_t state and passing it in along with data
// we could use a static state and just have the firmware passing in sensor data since we're only running
// one eskf, will see 

eskf_status_t eskf_init(eskf_t *filter, const eskf_config_t *config, const eskf_state_t *initial_state) {

}

// helpers

void eskf_get_state(const eskf_t *filter, eskf_state_t *state) {

}

eskf_status_t eskf_reset(eskf_t *filter, const eskf_state_t *initial_state) {

}
