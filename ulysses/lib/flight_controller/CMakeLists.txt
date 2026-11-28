find_program(UV_EXECUTABLE NAMES uv REQUIRED)

file(GLOB_RECURSE codegen_inputs CONFIGURE_DEPENDS
    "${ULYSSES_CODEGEN_TOOLS_DIR}/flight_controller/*.py"
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
set(flight_controller_header_dir "${ULYSSES_PROJECT_INCLUDE_DIR}/lib/flight_controller/generated")
set(flight_controller_source_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# set(flight_controller_generated_h
#     "${flight_controller_header_dir}/flight_controller_generated.h"
# )

# set(flight_controller_generated_c
#     "${flight_controller_source_dir}/flight_controller_codegen.c"
# )

add_custom_command(
    OUTPUT "${codegen_stamp}"
    # BYPRODUCTS 
        # "${flight_controller_generated_c}"
        # "${flight_controller_generated_h}"
    COMMAND "${UV_EXECUTABLE}" run --locked
        --directory "${ULYSSES_CODEGEN_TOOLS_DIR}" python -m flight_controller.generate
        --header-dir "${flight_controller_header_dir}"
        --source-dir "${flight_controller_source_dir}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${codegen_stamp}"
    DEPENDS 
        ${codegen_inputs} 
        "${codegen_input_list}"
        "${ULYSSES_REPO_ROOT}/pyproject.toml"
        "${ULYSSES_REPO_ROOT}/uv.lock"
    COMMENT "Running flight controller code generator"
    VERBATIM
)

add_custom_target(ulysses_flight_controller_codegen
    DEPENDS
        "${codegen_stamp}"
)

add_dependencies(ulysses_lib ulysses_flight_controller_codegen)

target_sources(ulysses_lib
    PRIVATE
        flight_controller.c
        # "${flight_controller_generated_c}"
)
