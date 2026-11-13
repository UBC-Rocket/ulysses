include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG b6763fbd9cedfacaa89e2ad9fd00d615a234e355 # v2.7.0
    GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(unity)

function(add_unity_test)
    cmake_parse_arguments(arg
        ""
        "NAME"
        "SOURCES;LIBRARIES"
        ${ARGN}
    )

    set(test_target "test_${arg_NAME}")

    add_executable(${test_target})

    target_sources(${test_target}
        PRIVATE
            ${arg_SOURCES}
    )

    target_compile_options(${test_target}
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
    )

    target_compile_features(${test_target}
        PRIVATE
            c_std_11
    )

    target_link_libraries(${test_target}
        PRIVATE
            unity::framework
            ${arg_LIBRARIES}
    )

    add_test(
        NAME ${arg_NAME}
        COMMAND ${test_target}
    )
endfunction()

add_subdirectory(example)
add_subdirectory(lib)
