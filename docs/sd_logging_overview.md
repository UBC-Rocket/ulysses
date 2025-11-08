# SD Card Logging (WIP)

> **Status:** In development. The logger streams records to the SD card, but recovery tooling and mission-critical validation are still ongoing.

## Goals

Simplify post-flight analysis by capturing structured telemetry directly to an SD card without needing an external PC. Records follow the shared schema in `Core/Inc/SD_logging/log_records.h`, so firmware and host tools stay in sync.

## Architecture

1. **Detection & init** (`Core/Src/main.c`):
   * `MX_SDMMC1_SD_Init()` only runs when the card-detect GPIO reports a card present.
   * Init uses 1-bit bus width and <400 kHz clock to satisfy SDHC/UHS-I requirements, then switches to 4-bit / ~25 MHz once the card exits idle.
   * `g_sd_card_initialized` reflects whether the card is ready for logging.

2. **Log writer** (`Core/Src/SD_logging/log_writer.c`):
   * Double-buffered 512-byte blocks; each buffer fills with framed records and is flushed via `HAL_SD_WriteBlocks_DMA`.
   * Frames consist of a small header (`magic`, `type`, `payload length`, `CRC16`) followed by the payload, matching the structs in `log_records.h`.
   * A FreeRTOS mutex guards buffer access so any task can enqueue records safely.
   * DMA completion ISR releases a semaphore, allowing the next buffer to be written without blocking the mission manager.

3. **Mission manager integration** (`Core/Src/tasks/mission_manager.c`):
   * The task simply calls `log_service_try_init/log_state/log_event/log_service_periodic_flush`.
   * `log_service` (in `Core/Src/SD_logging/log_service.c`) is a façade around the writer: it handles init handshakes, formats `state_snapshot` and `event` records, and triggers periodic flushes.
   * The mission manager no longer knows about buffers or CRCs; it just forwards state data and E-STOP events to the service.

## Host tooling

* `tools/logging/generate_log_schema.py` parses `log_records.h` and produces `tools/logging/log_schema.py`.
* Ground scripts can import that schema to decode raw sector dumps into structured data (Python parsing scripts TBD).

## Small-print / Known limitations

* **WIP:** No filesystem. Data is appended as raw sectors, so you’ll need a custom decoder script to interpret the log.
* **Power loss:** The logger is append-only; unflushed data in the active buffer is lost if power drops. Flush frequency can be tuned in mission manager.
* **Card compatibility:** Initialization has been tested with standard SDHC and some UHS-I cards, but high-speed tuning, wear-level behaviour, and long-duration stress tests remain.
* **Tooling gap:** Scripts to parse the binary log into CSV/plots are planned but not included yet.

Despite the “WIP” status, the framework is designed so that once tooling is ready, the firmware can log arbitrarily complex records without further schema drift.
