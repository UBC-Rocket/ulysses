find_program(UV_EXECUTABLE NAMES uv REQUIRED)

file(GLOB_RECURSE codegen_inputs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/codegen/*.py"
)

add_custom_target(ulysses_eskf_codegen
    COMMAND "${UV_EXECUTABLE}" run --locked
        --directory "${CMAKE_CURRENT_SOURCE_DIR}" python -m codegen.generate
    DEPENDS ${codegen_inputs}
    COMMENT "Running ESKF code generator"
    VERBATIM
)

add_dependencies(ulysses_lib ulysses_eskf_codegen)

target_sources(ulysses_lib 
    PRIVATE 
        eskf.c
)
