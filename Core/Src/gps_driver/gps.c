// gps.c — minimal RX + NMEA line reader (no time conversions)
#include "gps.h"
#include "stm32u5xx_hal.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>

// =========================== Config =======================================
// If you want raw lines to also print on a debug UART, define DEBUG_UART to
// your chosen handle (e.g., huart3) in your project or here:
//    #define DEBUG_UART huart3
// Comment the next line if you don't want auto-printing.
// #define DEBUG_UART huart3

#ifndef GPS_RING_CAP
#define GPS_RING_CAP 1024u
#endif

// =========================== State ========================================
static UART_HandleTypeDef *s_gps_huart = NULL;

// Single-byte RX buffer (interrupt-driven)
static volatile uint8_t s_rx_byte = 0;

// Lock-free ring buffer
static volatile uint8_t  s_ring[GPS_RING_CAP];
static volatile uint32_t s_r = 0;   // read index
static volatile uint32_t s_w = 0;   // write index

// Debug counters
static volatile uint32_t s_rx_isr_hits = 0;
volatile uint32_t gps_debug_err_hits   = 0;
volatile uint32_t gps_debug_last_err   = 0;

// =========================== Internals ====================================
static int  gps__has_complete_line(void);
static uint8_t gps__hex_nibble(char h);
static int  gps__nmea_checksum_ok(const uint8_t *line, size_t n);

// =========================== Public API ===================================
HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart_rx) {
    s_gps_huart = huart_rx;
    s_r = s_w = 0;
    return HAL_UART_Receive_IT(s_gps_huart, (uint8_t*)&s_rx_byte, 1);
}

// Non-blocking: returns bytes copied (including '\n') or 0 if no full line.
size_t gps_read_line(uint8_t *dst, size_t maxlen) {
    if (!gps__has_complete_line()) return 0;

    // Determine length up to '\n'
    uint32_t r = s_r;
    uint32_t w = s_w; // snapshot
    size_t len = 0;
    while (r != w) {
        uint8_t c = s_ring[r];
        len++;
        r = (r + 1u) % GPS_RING_CAP;
        if (c == '\n') break;
    }

    // Copy len bytes (truncate if needed)
    size_t to_copy = (len <= maxlen) ? len : maxlen;
    for (size_t i = 0; i < to_copy; ++i) {
        dst[i] = s_ring[s_r];
        s_r = (s_r + 1u) % GPS_RING_CAP;
    }
    // Drop remainder of the line if truncated
    for (size_t i = to_copy; i < len; ++i) {
        s_r = (s_r + 1u) % GPS_RING_CAP;
    }

    return to_copy;
}

// Poller: drains available lines; if checksum OK, dispatch to user callback.
void gps_poll(void) {
    static uint8_t line[128];

    for (;;) {
        size_t n = gps_read_line(line, sizeof(line));
        if (n == 0) break;

        // Optional: basic checksum gate (keeps noise out)
        int ok = gps__nmea_checksum_ok(line, n);
        if (!ok) continue;

#ifdef DEBUG_UART
        // Echo the valid raw NMEA to a debug port (optional)
        HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)line, n, 50);
#endif

        // User hook (weak) — do your parsing/printing in here
        gps_on_sentence((const char*)line, n);
    }
}

// =========================== Callbacks ====================================
// HAL calls this after each byte is received in interrupt mode.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == s_gps_huart) {
        s_rx_isr_hits++;

        // Push byte into ring
        uint32_t w = s_w;
        s_ring[w] = s_rx_byte;
        w = (w + 1u) % GPS_RING_CAP;

        // Overflow policy: drop oldest when full
        if (w == s_r) {
            s_r = (s_r + 1u) % GPS_RING_CAP;
        }
        s_w = w;

        // Re-arm RX for next byte
        HAL_UART_Receive_IT(s_gps_huart, (uint8_t*)&s_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == s_gps_huart) {
        gps_debug_err_hits++;
        gps_debug_last_err = huart->ErrorCode;

        // Clear common error flags that can stall RX (ORE/FE/PE/NE)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
#ifdef __HAL_UART_CLEAR_NEFLAG
        __HAL_UART_CLEAR_NEFLAG(huart);
#endif
        // Re-arm RX to avoid getting stuck after an error
        HAL_UART_Receive_IT(s_gps_huart, (uint8_t*)&s_rx_byte, 1);
    }
}

// =========================== Helpers ======================================
static int gps__has_complete_line(void) {
    uint32_t r = s_r;
    uint32_t w = s_w; // snapshot
    while (r != w) {
        if (s_ring[r] == '\n') return 1;
        r = (r + 1u) % GPS_RING_CAP;
    }
    return 0;
}

// hex char → 0..15 (used for NMEA checksum)
static uint8_t gps__hex_nibble(char h) {
    if (h >= '0' && h <= '9') return (uint8_t)(h - '0');
    h = (char)toupper((unsigned char)h);
    if (h >= 'A' && h <= 'F') return (uint8_t)(10 + h - 'A');
    return 0;
}

// Validate "$...*HH[\r]\n"
static int gps__nmea_checksum_ok(const uint8_t *line, size_t n) {
    if (n < 9 || line[0] != '$') return 0;

    // Find '*'
    const uint8_t *star = NULL;
    for (size_t i = 1; i + 3 < n; ++i) {
        if (line[i] == '*') { star = &line[i]; break; }
    }
    if (!star) return 0;

    // XOR between '$' (excluded) and '*' (excluded)
    uint8_t cs = 0;
    for (const uint8_t *p = line + 1; p < star; ++p) cs ^= *p;

    // Compare
    uint8_t hi = gps__hex_nibble((char)star[1]);
    uint8_t lo = gps__hex_nibble((char)star[2]);
    return cs == (uint8_t)((hi << 4) | lo);
}

// =========================== Weak hook ====================================
// Override this in your project to parse/print the sentence.
// Example implementation (in another .c):
//   void gps_on_sentence(const char* s, size_t n) {
//       extern UART_HandleTypeDef huart3;
//       HAL_UART_Transmit(&huart3, (uint8_t*)s, n, 50);
//   }
__attribute__((weak)) void gps_on_sentence(const char* sentence, size_t len) {
    (void)sentence; (void)len;
    // default: do nothing
}
