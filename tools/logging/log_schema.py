"""Auto-generated from Core/Inc/SD_logging/log_records.h
Do not edit manually. Run tools/logging/generate_log_schema.py instead.
"""

from __future__ import annotations

import struct

LOG_SCHEMA_VERSION = 1

TYPE_FORMATS = {'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h', 'uint32_t': 'I', 'int32_t': 'i', 'uint64_t': 'Q', 'int64_t': 'q', 'float': 'f'}

RECORDS = {

    "imu_sample": {
        "id": 1,
        "enum": "LOG_RECORD_TYPE_imu_sample",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("int16_t", "ax_milli_g"),
            ("int16_t", "ay_milli_g"),
            ("int16_t", "az_milli_g"),
            ("int16_t", "gx_mdps"),
            ("int16_t", "gy_mdps"),
            ("int16_t", "gz_mdps"),
        ],
        "format": "<Ihhhhhh",
        "struct": struct.Struct("<Ihhhhhh"),
    },
    "state_snapshot": {
        "id": 2,
        "enum": "LOG_RECORD_TYPE_state_snapshot",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float", "q_w"),
            ("float", "q_x"),
            ("float", "q_y"),
            ("float", "q_z"),
            ("float", "altitude_m"),
            ("float", "vel_n_mps"),
            ("float", "vel_e_mps"),
            ("float", "vel_d_mps"),
            ("uint8_t", "flight_state"),
            ("uint8_t", "estop_active"),
            ("uint16_t", "reserved"),
        ],
        "format": "<IffffffffBBH",
        "struct": struct.Struct("<IffffffffBBH"),
    },
    "event": {
        "id": 3,
        "enum": "LOG_RECORD_TYPE_event",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("uint16_t", "event_code"),
            ("uint16_t", "data_u16"),
        ],
        "format": "<IHH",
        "struct": struct.Struct("<IHH"),
    },
}

MAX_RECORD_SIZE = max(rec['struct'].size for rec in RECORDS.values())
