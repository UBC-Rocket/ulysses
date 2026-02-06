/**
 * @file gnss_radio_master.h
 * @brief GNSS Radio push mode driver for ulysses flight controller
 *
 * This driver handles communication with the ulysses-gnss-radio slave board
 * using push mode (slave-initiated via IRQ). It implements:
 * - Two-phase transaction reading (TYPE byte first, then PAYLOAD)
 * - Priority-based scheduling (GPS=HIGH, Radio=NORMAL)
 * - GPS fix and radio message queues
 *
 * Hardware:
 * - SPI1 bus (shared with motor control)
 * - EXT_INT_2 (PE7) for slave IRQ signal
 * - EXT_CS_2 (PE8) for chip select
 *
 * Usage:
 * 1. Call gnss_radio_init() after spi1_bus_init()
 * 2. Route EXT_INT_2 EXTI callback to gnss_radio_irq_handler()
 * 3. Poll queues or wait for task notifications for data
 *
 * UBC Rocket, Feb 2026
 */

#ifndef GNSS_RADIO_MASTER_H
#define GNSS_RADIO_MASTER_H

#include "stm32h5xx_hal.h"
#include "gnss_radio_protocol.h"
#include "spi1_bus.h"
#include "sync.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Hardware Configuration                                                     */
/* -------------------------------------------------------------------------- */

/** Chip select for GNSS Radio board */
#define GNSS_RADIO_CS_PORT        EXT_CS_2_GPIO_Port
#define GNSS_RADIO_CS_PIN         EXT_CS_2_Pin

/** IRQ input from GNSS Radio board (active low) */
#define GNSS_RADIO_IRQ_PORT       EXT_INT_2_GPIO_Port
#define GNSS_RADIO_IRQ_PIN        EXT_INT_2_Pin

/* -------------------------------------------------------------------------- */
/* Task Notification Flags                                                    */
/* -------------------------------------------------------------------------- */

/** Task notification flags for GNSS data availability */
#define GNSS_GPS_FIX_READY_FLAG    (1U << 4)
#define GNSS_RADIO_MSG_READY_FLAG  (1U << 5)

/* -------------------------------------------------------------------------- */
/* Push Mode State Machine                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Push mode state machine states
 *
 * The state machine handles two-phase reads:
 * Phase 1: Read TYPE byte to determine data type
 * Phase 2: Read PAYLOAD based on type (GPS or Radio)
 */
typedef enum {
    GNSS_STATE_IDLE,            /**< Waiting for IRQ from slave */
    GNSS_STATE_TYPE_PENDING,    /**< TYPE read job queued */
    GNSS_STATE_TYPE_READING,    /**< TYPE read in progress */
    GNSS_STATE_PAYLOAD_PENDING, /**< PAYLOAD read job queued */
    GNSS_STATE_PAYLOAD_READING, /**< PAYLOAD read in progress */
} gnss_push_state_t;

/* -------------------------------------------------------------------------- */
/* GPS NMEA Queue                                                             */
/* -------------------------------------------------------------------------- */

#define GNSS_GPS_NMEA_QUEUE_LEN    8

/**
 * @brief GPS NMEA queue for received raw NMEA sentences
 *
 * Stores raw NMEA sentences (up to 87 bytes each) from the GNSS Radio slave.
 * The master is responsible for parsing these into position data.
 */
typedef struct {
    uint8_t sentences[GNSS_GPS_NMEA_QUEUE_LEN][GNSS_NMEA_MAX_LEN];
    volatile uint8_t head;
    volatile uint8_t tail;
} gnss_gps_nmea_queue_t;

/* -------------------------------------------------------------------------- */
/* Radio Message Queue                                                        */
/* -------------------------------------------------------------------------- */

#define GNSS_RADIO_MSG_QUEUE_LEN  8

/**
 * @brief Radio message queue for received messages
 */
typedef struct {
    uint8_t messages[GNSS_RADIO_MSG_QUEUE_LEN][GNSS_RADIO_MESSAGE_MAX_LEN];
    volatile uint8_t head;
    volatile uint8_t tail;
} gnss_radio_msg_queue_t;

/* -------------------------------------------------------------------------- */
/* Driver Context                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Complete GNSS Radio driver context
 */
typedef struct {
    /* ── State machine ── */
    volatile gnss_push_state_t state;
    volatile uint8_t pending_type;       /**< TYPE byte read in phase 1 */
    volatile bool irq_pending;           /**< IRQ detected, awaiting processing */

    /* ── Hardware ── */
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *irq_port;
    uint16_t irq_pin;

    /* ── Data queues ── */
    gnss_gps_nmea_queue_t gps_queue;
    gnss_radio_msg_queue_t radio_queue;

    /* ── RX buffers for two-phase read ── */
    SYNC_CACHE_ALIGNED uint8_t type_rx_buf[4];   /**< Phase 1: TYPE byte + padding */
    SYNC_CACHE_ALIGNED uint8_t payload_rx_buf[GNSS_MAX_TRANSACTION_SIZE];

    /* ── TX buffer (dummy bytes for read operations) ── */
    SYNC_CACHE_ALIGNED uint8_t tx_dummy[GNSS_MAX_TRANSACTION_SIZE];

    /* ── Statistics ── */
    volatile uint32_t gps_fixes_received;
    volatile uint32_t radio_msgs_received;
    volatile uint32_t irq_count;
    volatile uint32_t transaction_errors;
    volatile uint32_t queue_overflows;

    /* ── Configuration ── */
    bool split_transactions;             /**< Enable two-phase reads (default: true) */
    bool initialized;
} gnss_radio_context_t;

/* -------------------------------------------------------------------------- */
/* Global Context                                                             */
/* -------------------------------------------------------------------------- */

/** Global GNSS Radio driver context - defined in gnss_radio_master.c */
extern gnss_radio_context_t gnss_radio_ctx;

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize GNSS Radio driver
 *
 * Sets up the driver context, queues, and configures CS pin.
 * Call this after spi1_bus_init().
 */
void gnss_radio_init(void);

/**
 * @brief Enable or disable two-phase (split) transactions
 *
 * When enabled (default), TYPE is read first, then PAYLOAD.
 * This allows priority re-evaluation between phases.
 *
 * @param enable true to enable split transactions
 */
void gnss_radio_set_split_mode(bool enable);

/* -------------------------------------------------------------------------- */
/* IRQ Handler                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Handle EXT_INT_2 rising edge (slave IRQ assertion)
 *
 * Call this from HAL_GPIO_EXTI_Rising_Callback when EXT_INT_2_Pin fires.
 * Initiates phase 1 of the push mode read sequence.
 */
void gnss_radio_irq_handler(void);

/* -------------------------------------------------------------------------- */
/* GPS NMEA Queue API                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if GPS NMEA queue is empty
 */
static inline bool gnss_gps_queue_empty(void) {
    return gnss_radio_ctx.gps_queue.head == gnss_radio_ctx.gps_queue.tail;
}

/**
 * @brief Check if GPS NMEA queue is full
 */
static inline bool gnss_gps_queue_full(void) {
    return ((gnss_radio_ctx.gps_queue.head + 1) % GNSS_GPS_NMEA_QUEUE_LEN) ==
           gnss_radio_ctx.gps_queue.tail;
}

/**
 * @brief Get number of GPS NMEA sentences in queue
 */
static inline uint8_t gnss_gps_queue_count(void) {
    return (gnss_radio_ctx.gps_queue.head - gnss_radio_ctx.gps_queue.tail +
            GNSS_GPS_NMEA_QUEUE_LEN) % GNSS_GPS_NMEA_QUEUE_LEN;
}

/**
 * @brief Dequeue a GPS NMEA sentence
 *
 * @param nmea Buffer to store the NMEA sentence (must be at least GNSS_NMEA_MAX_LEN bytes)
 * @return true if sentence was dequeued, false if queue empty
 */
bool gnss_gps_dequeue(uint8_t *nmea);

/**
 * @brief Peek at the oldest GPS NMEA sentence without removing it
 *
 * @param nmea Buffer to store the NMEA sentence
 * @return true if sentence is available, false if queue empty
 */
bool gnss_gps_peek(uint8_t *nmea);

/**
 * @brief Get pointer to newest GPS NMEA sentence (most recent)
 *
 * @return Pointer to sentence, or NULL if queue empty
 */
const uint8_t* gnss_gps_get_latest(void);

/* -------------------------------------------------------------------------- */
/* Radio Message Queue API                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if radio message queue is empty
 */
static inline bool gnss_radio_queue_empty(void) {
    return gnss_radio_ctx.radio_queue.head == gnss_radio_ctx.radio_queue.tail;
}

/**
 * @brief Check if radio message queue is full
 */
static inline bool gnss_radio_queue_full(void) {
    return ((gnss_radio_ctx.radio_queue.head + 1) % GNSS_RADIO_MSG_QUEUE_LEN) ==
           gnss_radio_ctx.radio_queue.tail;
}

/**
 * @brief Get number of radio messages in queue
 */
static inline uint8_t gnss_radio_queue_count(void) {
    return (gnss_radio_ctx.radio_queue.head - gnss_radio_ctx.radio_queue.tail +
            GNSS_RADIO_MSG_QUEUE_LEN) % GNSS_RADIO_MSG_QUEUE_LEN;
}

/**
 * @brief Dequeue a radio message
 *
 * @param msg Buffer to store the message (must be at least 256 bytes)
 * @return true if message was dequeued, false if queue empty
 */
bool gnss_radio_dequeue(uint8_t *msg);

/**
 * @brief Peek at the oldest radio message without removing it
 *
 * @param msg Buffer to store the message
 * @return true if message is available, false if queue empty
 */
bool gnss_radio_peek(uint8_t *msg);

/* -------------------------------------------------------------------------- */
/* Radio TX (Master to Slave)                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send a radio message to the slave for transmission
 *
 * Uses pull mode CMD_RADIO_TX to send data to the slave's radio.
 *
 * @param msg Message data (up to 256 bytes)
 * @param len Message length
 * @return true if message was queued for transmission
 */
bool gnss_radio_send(const uint8_t *msg, uint16_t len);

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get driver statistics
 *
 * @param gps_count Pointer to receive GPS fix count
 * @param radio_count Pointer to receive radio message count
 * @param errors Pointer to receive error count
 */
void gnss_radio_get_stats(uint32_t *gps_count,
                          uint32_t *radio_count,
                          uint32_t *errors);

/**
 * @brief Reset driver statistics
 */
void gnss_radio_reset_stats(void);

/**
 * @brief Get current state machine state (for debugging)
 */
gnss_push_state_t gnss_radio_get_state(void);

/* -------------------------------------------------------------------------- */
/* Internal Callbacks (for SPI1 bus driver)                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Phase 1 completion callback (TYPE read done)
 *
 * Called by SPI1 bus driver when TYPE byte read completes.
 * Parses type and submits phase 2 job with appropriate priority.
 */
void gnss_radio_phase1_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg);

/**
 * @brief Phase 2 completion callback (PAYLOAD read done)
 *
 * Called by SPI1 bus driver when PAYLOAD read completes.
 * Parses payload and enqueues to appropriate data queue.
 */
void gnss_radio_phase2_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg);

/**
 * @brief Single-phase completion callback (non-split mode)
 *
 * Called when split_transactions is false and entire TYPE+PAYLOAD
 * is read in one transaction.
 */
void gnss_radio_single_phase_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_RADIO_MASTER_H */
