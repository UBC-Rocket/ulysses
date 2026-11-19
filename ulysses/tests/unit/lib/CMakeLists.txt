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
