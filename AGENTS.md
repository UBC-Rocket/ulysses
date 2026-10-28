# AGENTS.md

Guidance for AI coding agents working in this repository. Human contributors should start with
[CONTRIBUTING.md](CONTRIBUTING.md); everything there applies to agents too.

## Project overview

Ulysses is the firmware for UBC Rocket's experimental VTOL platform (TVR, 2026-27). It runs on the
`TVR_FC_2026-27` flight computer, built around a dual-core STM32H745ZIT6 (Cortex-M7 + Cortex-M4).

- Both cores run FreeRTOS, and each core owns a different set of external chips.
- The CM7 is the primary core. It configures shared resources (HAL, system and peripheral clocks)
  during the CubeMX dual-core boot sync, then releases the CM4.
- Team reference docs: <https://github.com/UBC-Rocket/docs/tree/main/docs/2026-27-TVR/firmware>

## Repository layout

```
ulysses/
  include/ulysses/<layer>/   Public headers for each layer
  lib/                       Hardware-independent code such as math helpers and algorithms (e.g. the Kalman filter)
  io/                        Peripheral abstraction over the STM32 HAL
  svc/                       Higher-level (more abstract feature) that are buildt from lower layers (e.g. device drivers (sensors, ESCs, ...))
  app/cm7/, app/cm4/         Per-core application code and RTOS tasks
  cfp/                       STM32CubeMX project: generated code, HAL, FreeRTOS, linker scripts
docs/                        Setup, process, and style guides
tools/debugger/              pyOCD target fixups and SVD files
```

### Layering rules

- `lib` is pure: no HAL, no FreeRTOS, no `io` or `svc` includes. It must be buildable for any target.
- `io` is the layer that talks to the HAL. Code outside `io` (and `cfp`) must not include HAL headers.
- `svc` drivers use only the public `io` API from `include/ulysses/io/`, never `io` private headers.
- `app` composes `svc` and `lib` into tasks for its core.

### Module structure

A peripheral module in `io` is split into a public interface, a generic core, and a
platform-specific backend:

```
include/ulysses/io/i2c.h     Opaque typedefs for the device and bus, plus the public functions
io/i2c/device.{h,c}          Public API implementation; validates caller arguments
io/i2c/bus.{h,c}             Dispatches to the backend through an ops table
io/i2c/stm32/i2c_stm32.{h,c} STM32 HAL backend implementing the ops
```

- Public headers expose types only as opaque `typedef struct x x_t;`. Struct definitions live in
  private headers.
- HAL callbacks are forwarded to the IRQ controller in `io/irq/isr_hooks.c` with `io_irq_signal()`;
  backends register for events with `io_irq_register()`. Do not define HAL callbacks elsewhere.
- New source files must be added to the layer's `CMakeLists.txt`.

## Generated code (`cfp/`)

- **Do not edit anything under `ulysses/cfp/`**, including `USER CODE` blocks, CMake glue, and
  linker scripts. If a change seems to need it, stop and ask.
- The `.ioc` file is owned by the hardware team and maintained with STM32CubeMX. Never regenerate
  or edit it.
- When code depends on peripheral configuration (clock speeds, pin assignments, DMA channels,
  interrupt priorities, I2C timing), verify the actual values in the `.ioc` or generated code
  instead of assuming them.

## Building and verifying

Requires `arm-none-eabi-gcc` (CI uses 15.3.Rel1), CMake >= 3.22, and Ninja. See
[docs/development-setup.md](docs/development-setup.md).

Run from `ulysses/cfp`:

```shell
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
```

Both cores are built by each preset. Layer code builds with `-Wall -Wextra -Wpedantic -Werror` as
C11, so warnings are errors.

Before finishing a change:

1. Both presets build cleanly.
2. Changed files pass `clang-format` with the repo's `.clang-format` (`cfp/` and `third_party/`
   are excluded). CI enforces this.

There are no unit tests yet. Do not add a test framework.

Do not flash or debug hardware unless asked.

## Coding rules

Follow [docs/coding-style.md](docs/coding-style.md) and
[docs/documentation-style.md](docs/documentation-style.md). In short: lower snake case
everywhere, `_t` suffix on typedefs, no pointer typedefs, public functions prefixed with their
module (`io_i2c_...`), upper case macros and enum values, `static inline` over macros, and
parenthesized macro constants.

Additional rules:

- **No dynamic allocation.** `malloc`, `calloc`, `realloc`, and `free` are banned. Use static or
  caller-provided storage.
- **Errors:** use `ULYSSES_ASSERT` for caller bugs and violated preconditions (NULL pointers,
  invalid arguments, misuse of the API). Return `enum io_status` for runtime failures the caller
  can handle (bus errors, timeouts, busy peripherals).
- **Interrupt context:** async completion callbacks run in an ISR. Keep them short, never block,
  and use only the `FromISR` FreeRTOS APIs there. Mark ISR-shared state `volatile`.
- **Header guards** follow the path: `ULYSSES_IO_I2C` for public headers,
  `ULYSSES_IO_I2C_I2C_BUS_H` style for private ones.

## Keep it simple

Reviewers push back on the following. Check your diff for them before finishing:

- **Over-architecture:** no abstractions, hooks, or configuration for needs that don't exist yet.
  Build for the current board and the current use.
- **Safety nets:** no defensive checks for conditions that can't happen, and no fallback paths
  nobody asked for.
- **Redundant asserts:** check each precondition once, at the layer that owns it. If the public
  API already validated an argument, lower layers don't re-check it; say so in their docs instead
  (e.g. "Preconditions are checked by the device layer").
- **Over-complicated comments:** document public headers fully with Doxygen. Private and internal
  functions get a brief one-liner, or a "Like X(), but ..." reference to the function they mirror.
  Don't narrate what the code obviously does.

## Git workflow

See [docs/development-process.md](docs/development-process.md).

- Branch off an up-to-date `main` as `dev/<name>/<summary-of-change>`.
- Commit messages are `<scope>: <summary>`, where scope is a short path such as `io/i2c` or
  `io/i2c/stm32`. Check `git log` on the file for the usual scope.
- Keep commits and PRs small and reviewable. Split unrelated changes.
- PRs target `main` and are squash-merged.
