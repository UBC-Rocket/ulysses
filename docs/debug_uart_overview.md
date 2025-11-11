# Debug UART Logging Overview

The debug UART provides a lightweight way to dump internal telemetry over the ST-LINK V3’s virtual COM port without interfering with flight-critical buses. This document covers how the firmware produces logs, how to view them on a PC, and how to capture them to disk.

## Hardware path

* **USART1** is wired to the ST-LINK V3 virtual COM channel. TX = PB6, RX = PB7.
* In Debug builds (`-DDEBUG`), `MX_USART1_UART_Init` and `HAL_UART_MspInit` enable the peripheral, set 115200 baud, and configure DMA channel `GPDMA2 Channel 0` for TX.
* Release builds skip this initialization entirely (guarded with `#ifndef DEBUG`), keeping USART1 free if ST-LINK is absent.

## Firmware components

`Core/Src/debug_uart.c` is compiled only when `DEBUG` is defined and consists of three layers:

1. **Queue** – a static FreeRTOS queue that buffers outgoing bytes (`DEBUG_UART_QUEUE_LENGTH = 512`). Any task can call `debug_uart_write` or `debug_uart_write_string`; the API copies data into the queue byte-by-byte.
2. **Worker task** – `debug_uart_task` runs at `tskIDLE_PRIORITY + 2`. It drains the queue, packs up to 128 bytes per chunk, and calls `debug_uart_flush_buffer`.
3. **DMA flush** – `debug_uart_flush_buffer` uses a mutex and binary semaphore to serialize DMA transfers. It starts `HAL_UART_Transmit_DMA(&huart1, …)`; when the DMA transfer completes, `HAL_UART_TxCpltCallback` gives the semaphore from ISR context, allowing the next chunk to start immediately.

This design ensures log writes are non-blocking for producers and still run at full UART line rate thanks to DMA.

## How to log from firmware

* Include the header guarded by `DEBUG`:
  ```c
  #ifdef DEBUG
  #include "debug_uart.h"
  #endif
  ```
* Initialize in `MX_FREERTOS_Init` (already done):
  ```c
  #ifdef DEBUG
  debug_uart_init();
  #endif
  ```
* Emit strings from any task:
  ```c
  #ifdef DEBUG
  debug_uart_write_string("MissionManager: entering E_STOP\r\n");
  #endif
  ```
* For binary data or formatted strings assembled elsewhere, call `debug_uart_write((const uint8_t*)buf, len);`.

Because the API is safe to call from multiple tasks, you can sprinkle logs across mission manager, controls, etc., without worrying about interleaving; the queue preserves ordering.

## Viewing logs on a terminal

1. Connect the ST-LINK V3 to your computer via USB.
2. On macOS or Linux, the interface typically appears as `/dev/tty.usbmodemXXXX`; on Windows, it shows up as `COMx`.
3. Use any serial terminal at 115200 8N1. Examples:
   * `screen /dev/tty.usbmodem1102 115200`
   * `picocom -b 115200 /dev/tty.usbmodem1102`
   * On Windows, PuTTY or `powershell> mode COM3 BAUD=115200` followed by `type COM3`.
4. You should immediately see lines such as `ACC t=123456 ax=0.123 ay=...` from the state-estimation task’s debug prints.

## Recording logs on your PC

The repository includes `tools/debug_logger.py`, a small Python utility that records UART output into timestamped JSONL files.

### Usage

```bash
python tools/debug_logger.py \
    --port /dev/tty.usbmodem1102 \
    --baud 115200 \
    --log-dir tools/logs
```

* Defaults:
  * Port: `/dev/tty.usbmodem1102` (Unix) or `COM3` (Windows).
  * Baud: 115200.
  * Log directory: `tools/logs/` (created automatically with a timestamped filename like `debug_log_20250312_153000.jsonl`).
* Every serial line is echoed to the console and also written to disk as a JSON object containing:
  ```json
  {"timestamp":"2025-03-12T15:30:00.123456+00:00","payload":"ACC t=..."}
  ```
* Ctrl‑C cleanly closes the file and reports its path so you can archive it.

To decipher logs later, load the JSONL file into Python or any log-analysis tool and parse the `payload` strings.

## Gotchas

* The debug UART exists only in Debug builds. If you flash a Release image, `debug_uart_init` compiles to a no-op and USART1 stays off.
* DMA and the UART IRQ are enabled regardless of whether you actually call `debug_uart_write`; an idle system produces no output and consumes negligible CPU.
* While the queue is large enough for typical bursts, avoid writing multi-kilobyte dumps from ISR context. Stick to concise lines or throttle the producer.

With this infrastructure, firmware developers can monitor mission logic, sensor health, and safety triggers in real time, while tooling on the host side makes it trivial to archive runs for later analysis.
