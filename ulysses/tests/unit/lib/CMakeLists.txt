add_unity_test(
    NAME "bdshot_frame_pack"
    SOURCES
        bdshot/test_frame_pack.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "bdshot_frame_checksum"
    SOURCES
        bdshot/test_frame_checksum.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "bdshot_frame_decode"
    SOURCES
        bdshot/test_frame_decode.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "checksum_crc16"
    SOURCES
        checksum/test_crc16.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "checksum_crc16_special"
    SOURCES
        checksum/test_crc16_special.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "buffer"
    SOURCES
        test_buffer.c
    LIBRARIES
        ulysses_public_headers
)

add_unity_test(
    NAME "dynamixel_protocol2_encode"
    SOURCES
        dynamixel/test_protocol2_encode.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "dynamixel_protocol2_decode"
    SOURCES
        dynamixel/test_protocol2_decode.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)

add_unity_test(
    NAME "dynamixel_protocol2_instructions"
    SOURCES
        dynamixel/test_protocol2_instructions.c
    LIBRARIES
        ulysses_lib
        ulysses_public_headers
)
