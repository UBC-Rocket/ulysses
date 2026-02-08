/**
 * @file spi1_bus.h
 * @brief SPI1 bus driver with priority arbitration and window guard preemption
 *
 * This driver manages the SPI1 bus which is shared between:
 * - GNSS Radio slave board (push mode, GPS and radio messages)
 * - Motor control slave board (command/response)
 *
 * Key features:
 * - Priority-based job scheduling (CRITICAL > HIGH > NORMAL)
 * - Window guard preemption: prevents starting jobs too close to motor deadline
 * - DMA-based transfers with interrupt-driven completion
 * - Clean abstraction from device-specific drivers
 *
 * Thread Safety:
 * - spi1_submit_job() is safe to call from both ISR and task contexts
 * - Uses critical sections to protect queue state
 *
 * UBC Rocket, Feb 2026
 */

#ifndef SPI1_BUS_H
#define SPI1_BUS_H

#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "sync.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/** Number of jobs per priority level */
#define SPI1_QUEUE_SIZE_PER_PRIORITY  8

/** Total queue capacity across all priorities */
#define SPI1_TOTAL_QUEUE_SIZE         (SPI1_QUEUE_SIZE_PER_PRIORITY * 3)

/** RX staging buffer size (largest possible transaction) */
#define SPI1_RX_STAGING_SIZE          272

/** Default window guard margin (microseconds before deadline to stop starting jobs) */
#define SPI1_DEFAULT_GUARD_MARGIN_US  50

/* -------------------------------------------------------------------------- */
/* Priority Levels                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Priority levels for SPI1 jobs
 *
 * Lower numeric value = higher priority.
 * CRITICAL jobs (motor control) always run first when available.
 */
typedef enum {
    SPI1_PRIO_CRITICAL = 0,  /**< Motor control commands - highest priority */
    SPI1_PRIO_HIGH     = 1,  /**< GPS messages - small, time-sensitive */
    SPI1_PRIO_NORMAL   = 2,  /**< Radio messages - large, less urgent */
    SPI1_PRIO_COUNT    = 3
} spi1_priority_t;

/* -------------------------------------------------------------------------- */
/* Device Identifiers                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Device identifiers for SPI1 slaves
 */
typedef enum {
    SPI1_DEVICE_GNSS_RADIO = 0,  /**< GNSS Radio board */
    SPI1_DEVICE_MOTOR_CTRL = 1,  /**< Motor control board */
    SPI1_DEVICE_COUNT      = 2
} spi1_device_id_t;

/* -------------------------------------------------------------------------- */
/* Job Structure                                                              */
/* -------------------------------------------------------------------------- */

/** Forward declaration */
typedef struct spi1_job_t spi1_job_t;

/**
 * @brief Callback type for job completion
 * @param job    The job that just finished
 * @param rx_buf Pointer to received data buffer
 * @param arg    User argument pointer
 */
typedef void (*spi1_done_cb_t)(spi1_job_t *job,
                               const uint8_t *rx_buf,
                               void *arg);

/**
 * @brief SPI1 job descriptor with priority support
 *
 * Extends the base job concept with priority and timing information
 * for the window guard mechanism.
 */
struct spi1_job_t {
    /* ── Transfer parameters ── */
    GPIO_TypeDef *cs_port;       /**< Chip-select port */
    uint16_t cs_pin;             /**< Chip-select pin */
    uint8_t tx[SPI1_RX_STAGING_SIZE]; /**< TX buffer (embedded for simplicity) */
    uint16_t len;                /**< Transfer length in bytes */

    /* ── Timing ── */
    uint64_t t_submit;           /**< Timestamp when job was submitted (µs) */
    uint32_t max_duration_us;    /**< Estimated max transfer duration (µs) */

    /* ── Priority & scheduling ── */
    spi1_priority_t priority;    /**< Job priority level */
    spi1_device_id_t device;     /**< Originating device */
    bool preemptible;            /**< Can be deferred for higher priority */

    /* ── Transfer type ── */
    spi_xfer_type_t type;        /**< TX, RX, or TXRX */

    /* ── Completion handling ── */
    spi1_done_cb_t done;         /**< Completion callback (optional) */
    void *done_arg;              /**< User argument for callback */
    uint32_t task_notification_flag; /**< FreeRTOS notification flag */
};

/* -------------------------------------------------------------------------- */
/* Priority Queue Structure                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Single-priority job queue (circular buffer)
 */
typedef struct {
    spi1_job_t jobs[SPI1_QUEUE_SIZE_PER_PRIORITY];
    volatile uint8_t head;
    volatile uint8_t tail;
} spi1_prio_queue_t;

/* -------------------------------------------------------------------------- */
/* Bus Context Structure                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Complete SPI1 bus context
 *
 * Contains all state for priority arbitration, window guard, and DMA handling.
 */
typedef struct {
    /* ── HAL handle ── */
    SPI_HandleTypeDef *hspi;

    /* ── Priority queues ── */
    spi1_prio_queue_t queues[SPI1_PRIO_COUNT];

    /* ── Current job ── */
    spi1_job_t current_job;
    volatile bool bus_busy;

    /* ── Window guard ── */
    volatile uint64_t motor_deadline_us;  /**< Next motor control deadline */
    uint32_t guard_margin_us;             /**< Safety margin before deadline */
    volatile bool deadline_active;        /**< True when deadline is set */

    /* ── RX staging buffer ── */
    SYNC_CACHE_ALIGNED volatile uint8_t rx_staging[SPI1_RX_STAGING_SIZE];

    /* ── Statistics ── */
    volatile uint32_t jobs_completed[SPI1_PRIO_COUNT];
    volatile uint32_t jobs_deferred;      /**< Jobs delayed due to window guard */
    volatile uint32_t queue_overflows;    /**< Jobs dropped due to full queue */
    volatile HAL_StatusTypeDef last_status;
} spi1_bus_context_t;

/* -------------------------------------------------------------------------- */
/* Global Bus Context                                                         */
/* -------------------------------------------------------------------------- */

/** Global SPI1 bus context - defined in spi1_bus.c */
extern spi1_bus_context_t spi1_ctx;

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize SPI1 bus driver
 *
 * Sets up priority queues, window guard, and links to HAL SPI handle.
 * Call this after HAL_SPI_Init() for SPI1.
 *
 * @param hspi Pointer to HAL SPI handle for SPI1
 */
void spi1_bus_init(SPI_HandleTypeDef *hspi);

/* -------------------------------------------------------------------------- */
/* Job Submission                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to the SPI1 bus
 *
 * Safe to call from both ISR and task contexts.
 *
 * Behavior:
 * - If bus is idle and window guard allows, job starts immediately
 * - Otherwise, job is enqueued at its priority level
 * - CRITICAL priority jobs bypass window guard check
 *
 * @param job Job descriptor (copied into queue)
 * @return true if job was started immediately, false if queued or failed
 */
bool spi1_submit_job(const spi1_job_t *job);

/**
 * @brief Check if SPI1 bus is currently busy
 * @return true if a transaction is in progress
 */
static inline bool spi1_is_busy(void) {
    return spi1_ctx.bus_busy;
}

/**
 * @brief Check if any jobs are pending in the queues
 * @return true if at least one job is queued
 */
bool spi1_has_pending_jobs(void);

/* -------------------------------------------------------------------------- */
/* Window Guard API                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set the next motor control deadline
 *
 * Call this from the motor control timer to inform the bus driver
 * when the next motor command must be sent. The window guard will
 * prevent starting non-CRITICAL jobs too close to this deadline.
 *
 * @param deadline_us Absolute timestamp (µs) of next motor deadline
 */
void spi1_set_motor_deadline(uint64_t deadline_us);

/**
 * @brief Clear the motor deadline (no active deadline)
 *
 * Call when motor control is disabled or paused.
 */
void spi1_clear_motor_deadline(void);

/**
 * @brief Set the window guard margin
 *
 * @param margin_us Microseconds before deadline to stop starting jobs
 */
void spi1_set_guard_margin(uint32_t margin_us);

/**
 * @brief Check if a job can start within the window guard
 *
 * @param duration_us Estimated job duration in microseconds
 * @param priority Job priority level
 * @return true if job can start, false if too close to deadline
 */
bool spi1_window_guard_allows(uint32_t duration_us, spi1_priority_t priority);

/* -------------------------------------------------------------------------- */
/* Internal Functions (for DMA callbacks)                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Handle DMA completion for SPI1
 *
 * Called from HAL_SPI_TxRxCpltCallback when SPI1 transfer completes.
 * Invokes job callback, dequeues next job, and chains DMA.
 */
void spi1_dma_complete_handler(void);

/**
 * @brief Try to start the next pending job
 *
 * Called after job completion or when a new job might be startable.
 * Checks window guard and selects highest priority eligible job.
 */
void spi1_try_start_next(void);

/* -------------------------------------------------------------------------- */
/* Queue Helpers                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if a priority queue is empty
 */
static inline bool spi1_queue_empty(const spi1_prio_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if a priority queue is full
 */
static inline bool spi1_queue_full(const spi1_prio_queue_t *q) {
    return ((q->head + 1) % SPI1_QUEUE_SIZE_PER_PRIORITY) == q->tail;
}

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get bus statistics
 * @param completed Array of 3 elements to receive completion counts per priority
 * @param deferred Pointer to receive deferred job count
 * @param overflows Pointer to receive overflow count
 */
void spi1_get_stats(uint32_t completed[SPI1_PRIO_COUNT],
                    uint32_t *deferred,
                    uint32_t *overflows);

/**
 * @brief Reset bus statistics
 */
void spi1_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI1_BUS_H */
