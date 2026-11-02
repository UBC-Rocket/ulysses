#ifndef ULYSSES_LIB_BDSHOT_FRAME
#define ULYSSES_LIB_BDSHOT_FRAME

#include <stdint.h>
#include <stdbool.h>

#include "ulysses/lib/bdshot/protocol.h"

bool bdshot_frame_pack_throttle(uint16_t *frame, uint16_t throttle, bool request_telemetry);
bool bdshot_frame_pack_command(uint16_t *frame, enum bdshot_command command, bool request_telemetry);

uint8_t bdshot_frame_checksum(uint16_t frame);

bool bdshot_frame_decode_from_wire(uint16_t *frame, uint32_t wire_frame);
bool bdshot_frame_is_edt(uint16_t frame);
float bdshot_frame_calculate_rpm(uint16_t frame, uint8_t motor_pole_count);

#endif // ULYSSES_LIB_BDSHOT_FRAME
