find_program(UV_EXECUTABLE NAMES uv REQUIRED)

file(GLOB_RECURSE codegen_inputs CONFIGURE_DEPENDS
    "${ULYSSES_CODEGEN_TOOLS_DIR}/eskf/*.py"
)

# generate codegen input list
set(codegen_input_list
    "${CMAKE_CURRENT_BINARY_DIR}/codegen-inputs.txt"
)

string(JOIN "\n" codegen_input_paths ${codegen_inputs})
file(GENERATE
    OUTPUT "${codegen_input_list}"
    CONTENT "${codegen_input_paths}\n"
)


# record the last successful codegen run
set(codegen_stamp "${CMAKE_CURRENT_BINARY_DIR}/codegen.stamp")

# run the codegen tool
set(eskf_header_dir "${ULYSSES_PROJECT_INCLUDE_DIR}/lib/eskf/generated")
set(eskf_source_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(eskf_generated_h
    "${eskf_header_dir}/eskf_model.h"
    "${eskf_header_dir}/eskf_indices.h"
    # "${eskf_header_dir}/eskf_measurement.h"
)

set(eskf_generated_c
    "${eskf_source_dir}/eskf_model.c"
    # "${eskf_source_dir}/eskf_measurement.h"
)

add_custom_command(
    OUTPUT "${codegen_stamp}"
    BYPRODUCTS
        "${eskf_generated_c}"
        "${eskf_generated_h}"

    COMMAND "${UV_EXECUTABLE}" run --locked
        --directory "${ULYSSES_CODEGEN_TOOLS_DIR}" python -m eskf.generate
        --header-dir "${eskf_header_dir}"
        --source-dir "${eskf_source_dir}"

    COMMAND "${CMAKE_COMMAND}" -E touch "${codegen_stamp}"

    DEPENDS
        ${codegen_inputs}
        "${codegen_input_list}"
        "${ULYSSES_REPO_ROOT}/pyproject.toml"
        "${ULYSSES_REPO_ROOT}/uv.lock"

    COMMENT "Running ESKF code generator"
    VERBATIM
)

add_custom_target(ulysses_eskf_codegen
    DEPENDS
        "${codegen_stamp}"
)

add_dependencies(ulysses_lib ulysses_eskf_codegen)


set_property(
    SOURCE "${eskf_generated_c}"
    TARGET_DIRECTORY ulysses_lib
    APPEND PROPERTY 
        COMPILE_OPTIONS 
            -Wno-unused-parameter 
            -fsingle-precision-constant
            -Wdouble-promotion
)

target_sources(ulysses_lib
    PRIVATE
        eskf.c
        "${eskf_generated_c}"
)

