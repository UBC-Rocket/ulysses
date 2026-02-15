/**
 * @file spi1_bus.c
 * @brief SPI1 bus driver implementation with priority arbitration
 *
 * This driver manages the SPI1 bus shared between GNSS Radio and Motor Control
 * slaves. It implements:
 * - Priority-based job scheduling
 * - Window guard preemption mechanism
 * - DMA-based transfers with interrupt-driven completion
 *
 * UBC Rocket, Feb 2026
 */

#include "spi1_bus.h"
#include "timestamp.h"
#include "sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Global Context                                                             */
/* -------------------------------------------------------------------------- */

spi1_bus_context_t spi1_ctx;

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static bool spi1_enqueue(spi1_prio_queue_t *q, const spi1_job_t *job);
static bool spi1_dequeue(spi1_prio_queue_t *q, spi1_job_t *job);
static void spi1_start_job(spi1_job_t *job);
static spi1_job_t* spi1_select_next_job(void);

/* -------------------------------------------------------------------------- */
/* Chip Select Macros                                                         */
/* -------------------------------------------------------------------------- */

#define CS_ASSERT(port, pin)   ((port)->BSRR = (uint32_t)(pin) << 16)
#define CS_DEASSERT(port, pin) ((port)->BSRR = (uint32_t)(pin))

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

void spi1_bus_init(SPI_HandleTypeDef *hspi)
{
    /* Clear entire context */
    memset(&spi1_ctx, 0, sizeof(spi1_ctx));

    /* Store HAL handle */
    spi1_ctx.hspi = hspi;

    /* Initialize all priority queues */
    for (int i = 0; i < SPI1_PRIO_COUNT; i++) {
        spi1_ctx.queues[i].head = 0;
        spi1_ctx.queues[i].tail = 0;
    }

    /* Set default window guard margin */
    spi1_ctx.guard_margin_us = SPI1_DEFAULT_GUARD_MARGIN_US;
    spi1_ctx.deadline_active = false;
    spi1_ctx.bus_busy = false;

    /* Initialize stats */
    spi1_ctx.last_status = HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Queue Operations                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enqueue a job into a priority queue
 * @return true if successful, false if queue full
 */
static bool spi1_enqueue(spi1_prio_queue_t *q, const spi1_job_t *job)
{
    if (spi1_queue_full(q)) {
        return false;
    }

    q->jobs[q->head] = *job;
    SYNC_DMB();  /* Ensure job data written before head update */
    q->head = (q->head + 1) % SPI1_QUEUE_SIZE_PER_PRIORITY;
    return true;
}

/**
 * @brief Dequeue a job from a priority queue
 * @return true if successful, false if queue empty
 */
static bool spi1_dequeue(spi1_prio_queue_t *q, spi1_job_t *job)
{
    if (spi1_queue_empty(q)) {
        return false;
    }

    SYNC_DMB();  /* Ensure we see job data before reading */
    *job = q->jobs[q->tail];
    SYNC_DMB();  /* Ensure job read completes before tail update */
    q->tail = (q->tail + 1) % SPI1_QUEUE_SIZE_PER_PRIORITY;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Window Guard                                                               */
/* -------------------------------------------------------------------------- */

void spi1_set_motor_deadline(uint64_t deadline_us)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    spi1_ctx.motor_deadline_us = deadline_us;
    spi1_ctx.deadline_active = true;

    SYNC_EXIT_CRITICAL_RAW(primask);

    /* After setting deadline, try to start any pending motor control jobs */
    if (!spi1_ctx.bus_busy) {
        spi1_try_start_next();
    }
}

void spi1_clear_motor_deadline(void)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    spi1_ctx.deadline_active = false;

    SYNC_EXIT_CRITICAL_RAW(primask);

    /* After clearing deadline, try to start any deferred jobs */
    if (!spi1_ctx.bus_busy) {
        spi1_try_start_next();
    }
}

void spi1_set_guard_margin(uint32_t margin_us)
{
    spi1_ctx.guard_margin_us = margin_us;
}

bool spi1_window_guard_allows(uint32_t duration_us, spi1_priority_t priority)
{
    /* CRITICAL priority always allowed */
    if (priority == SPI1_PRIO_CRITICAL) {
        return true;
    }

    /* If no deadline active, allow all */
    if (!spi1_ctx.deadline_active) {
        return true;
    }

    /* Check if job would complete before deadline minus margin */
    uint64_t now = timestamp_us();
    uint64_t deadline = spi1_ctx.motor_deadline_us;

    /* Handle deadline already passed (wrap-around safe) */
    if (now >= deadline) {
        /* Deadline passed, allow job - motor control should have run */
        return true;
    }

    uint64_t time_to_deadline = deadline - now;
    uint64_t required_time = duration_us + spi1_ctx.guard_margin_us;

    return time_to_deadline >= required_time;
}

/* -------------------------------------------------------------------------- */
/* Job Selection                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Select the highest priority job that can start now
 *
 * Checks queues from highest to lowest priority, respecting window guard.
 *
 * @return Pointer to job in queue (not removed), or NULL if none eligible
 */
static spi1_job_t* spi1_select_next_job(void)
{
    for (int prio = 0; prio < SPI1_PRIO_COUNT; prio++) {
        spi1_prio_queue_t *q = &spi1_ctx.queues[prio];

        if (!spi1_queue_empty(q)) {
            /* Peek at the head job */
            spi1_job_t *job = &q->jobs[q->tail];

            /* Check window guard for non-CRITICAL jobs */
            if (spi1_window_guard_allows(job->max_duration_us, job->priority)) {
                return job;
            }
            /* If this priority is blocked by window guard, don't check lower priorities
               unless it's preemptible */
            if (!job->preemptible) {
                return NULL;  /* Wait for deadline to pass */
            }
            /* Job is preemptible, continue checking lower priorities */
        }
    }

    return NULL;
}

/* -------------------------------------------------------------------------- */
/* Job Execution                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start a DMA transfer for a job
 *
 * Assumes bus is idle. Sets up CS, cache maintenance, and initiates DMA.
 */
static void spi1_start_job(spi1_job_t *job)
{
    spi1_ctx.bus_busy = true;

    /* Assert chip select */
    CS_ASSERT(job->cs_port, job->cs_pin);

    /* Unlock HAL SPI handle (may be locked from previous ISR context) */
    __HAL_UNLOCK(spi1_ctx.hspi);

    /* Cache maintenance: clean TX buffer before DMA reads it */
    if (job->type == SPI_XFER_TX || job->type == SPI_XFER_TXRX) {
        sync_dcache_clean((void *)job->tx, job->len);
    }

    HAL_StatusTypeDef status = HAL_ERROR;

    switch (job->type) {
    case SPI_XFER_TX:
        status = HAL_SPI_Transmit_DMA(spi1_ctx.hspi, job->tx, job->len);
        break;
    case SPI_XFER_RX:
        status = HAL_SPI_Receive_DMA(spi1_ctx.hspi,
                                     (uint8_t *)spi1_ctx.rx_staging,
                                     job->len);
        break;
    case SPI_XFER_TXRX:
    default:
        status = HAL_SPI_TransmitReceive_DMA(spi1_ctx.hspi,
                                             job->tx,
                                             (uint8_t *)spi1_ctx.rx_staging,
                                             job->len);
        break;
    }

    spi1_ctx.last_status = status;

    if (status != HAL_OK) {
        /* DMA start failed - release CS and mark bus idle */
        CS_DEASSERT(job->cs_port, job->cs_pin);
        spi1_ctx.bus_busy = false;
#if defined(DEBUG)
        __BKPT(0);
#endif
    }
}

/* -------------------------------------------------------------------------- */
/* Job Submission                                                             */
/* -------------------------------------------------------------------------- */

bool spi1_submit_job(const spi1_job_t *job)
{
    bool started = false;
    bool should_start = false;
    spi1_job_t *active_job = NULL;

    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    if (!spi1_ctx.bus_busy) {
        /* Bus is idle - check if we can start immediately */
        if (spi1_window_guard_allows(job->max_duration_us, job->priority)) {
            /* Can start now */
            spi1_ctx.bus_busy = true;
            spi1_ctx.current_job = *job;
            active_job = &spi1_ctx.current_job;
            should_start = true;
            started = true;
        } else {
            /* Window guard blocks - enqueue for later */
            spi1_prio_queue_t *q = &spi1_ctx.queues[job->priority];
            if (spi1_enqueue(q, job)) {
                spi1_ctx.jobs_deferred++;
                started = false;
            } else {
                spi1_ctx.queue_overflows++;
                started = false;
            }
        }
    } else {
        /* Bus is busy - enqueue at priority level */
        spi1_prio_queue_t *q = &spi1_ctx.queues[job->priority];
        if (spi1_enqueue(q, job)) {
            started = false;
        } else {
            spi1_ctx.queue_overflows++;
            started = false;
        }
    }

    SYNC_EXIT_CRITICAL_RAW(primask);

    /* Start DMA outside critical section */
    if (should_start && active_job != NULL) {
        spi1_start_job(active_job);
    }

    return started;
}

bool spi1_has_pending_jobs(void)
{
    for (int prio = 0; prio < SPI1_PRIO_COUNT; prio++) {
        if (!spi1_queue_empty(&spi1_ctx.queues[prio])) {
            return true;
        }
    }
    return false;
}

/* -------------------------------------------------------------------------- */
/* DMA Completion Handler                                                     */
/* -------------------------------------------------------------------------- */

void spi1_dma_complete_handler(void)
{
    BaseType_t xWoken = pdFALSE;
    bool should_start_next = false;
    spi1_job_t next_job;

    /* Deassert chip select for completed job */
    CS_DEASSERT(spi1_ctx.current_job.cs_port, spi1_ctx.current_job.cs_pin);

    /* Cache maintenance: invalidate RX buffer before CPU reads */
    sync_dcache_invalidate((void *)spi1_ctx.rx_staging, SPI1_RX_STAGING_SIZE);

    /* Copy current job info before potentially overwriting */
    spi1_job_t completed_job = spi1_ctx.current_job;

    /* Invoke completion callback with RX data */
    if (completed_job.done) {
        completed_job.done(&completed_job,
                          (const uint8_t *)spi1_ctx.rx_staging,
                          completed_job.done_arg);
    }

    /* Notify waiting task if requested */
    if (completed_job.task_notification_flag != 0) {
        xTaskNotifyFromISR(StateEstimationHandle,
                          completed_job.task_notification_flag,
                          eSetBits,
                          &xWoken);
    }

    /* Update statistics */
    spi1_ctx.jobs_completed[completed_job.priority]++;

    /* Critical section: dequeue next job and update state */
    UBaseType_t s = SYNC_ENTER_CRITICAL_FROM_ISR();

    /* Find and dequeue highest priority eligible job */
    for (int prio = 0; prio < SPI1_PRIO_COUNT; prio++) {
        spi1_prio_queue_t *q = &spi1_ctx.queues[prio];

        if (!spi1_queue_empty(q)) {
            /* Peek at job */
            spi1_job_t *pending = &q->jobs[q->tail];

            if (spi1_window_guard_allows(pending->max_duration_us, pending->priority)) {
                /* Can start this job - dequeue it */
                spi1_dequeue(q, &next_job);
                spi1_ctx.current_job = next_job;
                spi1_ctx.bus_busy = true;
                should_start_next = true;
                break;
            } else if (!pending->preemptible) {
                /* Non-preemptible job blocked - stop searching */
                spi1_ctx.bus_busy = false;
                break;
            }
            /* Preemptible job blocked - check lower priorities */
        }
    }

    if (!should_start_next) {
        spi1_ctx.bus_busy = false;
    }

    SYNC_EXIT_CRITICAL_FROM_ISR(s);

    /* Start next job outside critical section */
    if (should_start_next) {
        spi1_start_job(&spi1_ctx.current_job);
    }

    portYIELD_FROM_ISR(xWoken);
}

void spi1_try_start_next(void)
{
    uint32_t primask;
    bool should_start = false;
    spi1_job_t next_job;

    SYNC_ENTER_CRITICAL_RAW(primask);

    if (!spi1_ctx.bus_busy) {
        /* Find highest priority eligible job */
        for (int prio = 0; prio < SPI1_PRIO_COUNT; prio++) {
            spi1_prio_queue_t *q = &spi1_ctx.queues[prio];

            if (!spi1_queue_empty(q)) {
                spi1_job_t *pending = &q->jobs[q->tail];

                if (spi1_window_guard_allows(pending->max_duration_us, pending->priority)) {
                    spi1_dequeue(q, &next_job);
                    spi1_ctx.current_job = next_job;
                    spi1_ctx.bus_busy = true;
                    should_start = true;
                    break;
                } else if (!pending->preemptible) {
                    break;
                }
            }
        }
    }

    SYNC_EXIT_CRITICAL_RAW(primask);

    if (should_start) {
        spi1_start_job(&spi1_ctx.current_job);
    }
}

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

void spi1_get_stats(uint32_t completed[SPI1_PRIO_COUNT],
                    uint32_t *deferred,
                    uint32_t *overflows)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    for (int i = 0; i < SPI1_PRIO_COUNT; i++) {
        completed[i] = spi1_ctx.jobs_completed[i];
    }
    *deferred = spi1_ctx.jobs_deferred;
    *overflows = spi1_ctx.queue_overflows;

    SYNC_EXIT_CRITICAL_RAW(primask);
}

void spi1_reset_stats(void)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    for (int i = 0; i < SPI1_PRIO_COUNT; i++) {
        spi1_ctx.jobs_completed[i] = 0;
    }
    spi1_ctx.jobs_deferred = 0;
    spi1_ctx.queue_overflows = 0;

    SYNC_EXIT_CRITICAL_RAW(primask);
}
