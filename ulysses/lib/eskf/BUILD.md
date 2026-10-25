# Building the ESKF library

ESKF is compiled into the `ulysses_lib` static library alongside the other library
modules. Its sources are listed in `ulysses/lib/CMakeLists.txt`; there is no separate
`eskf` CMake target or standalone CMake project in this directory.

Run the commands below from the repository root. Each parenthesized command block
changes directory only for that block.

## Prerequisites

For firmware builds, install CMake 3.22 or newer, Ninja, and the Arm GNU Toolchain
for `arm-none-eabi`. Follow the repository's
[development setup guide](../../../docs/development-setup.md) for installation.

Confirm the tools are available on your `PATH`:

```sh
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

## Build only the library for CM7

Configure the CM7 firmware project, then select the library target:

```sh
(
    cd ulysses/cfp/CM7
    cmake --preset Debug
    cmake --build --preset Debug --target ulysses_lib
)
```

The archive is written to:

```text
ulysses/cfp/CM7/build/Debug/ulysses/lib/libulysses_lib.a
```

This builds the library without linking the complete firmware. Replace `CM7` with
`CM4` to build for that core. Each core uses its own CPU/FPU settings and build
directory; use the archive built for the intended core.

For an optimized build, replace `Debug` with `Release` in both commands and in the
output path. After editing a source file, rerun the build command; rerun configure
after changing presets or toolchain settings.

## Build both firmware images

Use the parent project's presets to build CM4 and CM7 together:

```sh
(
    cd ulysses/cfp
    cmake --preset Debug
    cmake --build --preset Debug
)
```

This parent build uses separate output directories under
`ulysses/cfp/build/Debug/CM4/build` and `ulysses/cfp/build/Debug/CM7/build`.
The `ulysses_lib` target belongs to each core's project, so use the per-core
commands above when building only the library.

## Check C syntax on the host

For a quick check without the ARM toolchain, use a host C compiler:

```sh
cc -std=c11 -Wall -Wextra -Wpedantic \
    -Iulysses/include -fsyntax-only \
    ulysses/lib/eskf/eskf.c \
    ulysses/lib/eskf/predict.c \
    ulysses/lib/eskf/update.c \
    ulysses/lib/eskf/math.c
```

This checks parsing and declarations. It produces no archive, does not link, and
does not validate ARM compatibility or filter behavior. The current empty function
bodies produce unused-parameter warnings; compilation may also report missing
return values. Once implementations are complete, add `-Werror` to enforce a
warning-free check.

## Current limitations

The filter functions are stubs and do not yet compute an estimate or provide valid
status results. Building an archive successfully does not make the API functional.
The files under `ulysses/tests/eskf` are placeholders; there are no runnable ESKF
tests registered with CMake yet.

Include the public API using:

```c
#include "ulysses/lib/eskf/eskf.h"
```

New CMake consumers need both `ulysses_lib` and the public include path supplied by
`ulysses_public_headers`; the library's current private dependencies do not export
that include path to consumers.
