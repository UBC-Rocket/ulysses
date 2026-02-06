/**
 * @file gnss_radio_master.c
 * @brief GNSS Radio push mode driver implementation
 *
 * Implements two-phase push mode reading from the GNSS Radio slave board:
 * - Phase 1: Read TYPE byte (1 byte) to determine message type
 * - Phase 2: Read PAYLOAD based on type (GPS: 48 bytes, Radio: 256 bytes)
 *
 * Priority assignment:
 * - GPS messages: SPI1_PRIO_HIGH (small, time-sensitive)
 * - Radio messages: SPI1_PRIO_NORMAL (large, less urgent)
 *
 * UBC Rocket, Feb 2026
 */

#include "gnss_radio_master.h"
#include "main.h"
#include "timestamp.h"
#include "sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Global Context                                                             */
/* -------------------------------------------------------------------------- */

gnss_radio_context_t gnss_radio_ctx;

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void submit_type_read(void);
static void submit_payload_read(uint8_t type);
static void submit_full_read(void);
static bool enqueue_gps_fix(const gnss_gps_fix_t *fix);
static bool enqueue_radio_msg(const uint8_t *msg);

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

void gnss_radio_init(void)
{
    /* Clear entire context */
    memset(&gnss_radio_ctx, 0, sizeof(gnss_radio_ctx));

    /* Set hardware configuration */
    gnss_radio_ctx.cs_port = GNSS_RADIO_CS_PORT;
    gnss_radio_ctx.cs_pin = GNSS_RADIO_CS_PIN;
    gnss_radio_ctx.irq_port = GNSS_RADIO_IRQ_PORT;
    gnss_radio_ctx.irq_pin = GNSS_RADIO_IRQ_PIN;

    /* Initialize state machine */
    gnss_radio_ctx.state = GNSS_STATE_IDLE;
    gnss_radio_ctx.irq_pending = false;

    /* Initialize queues */
    gnss_radio_ctx.gps_queue.head = 0;
    gnss_radio_ctx.gps_queue.tail = 0;
    gnss_radio_ctx.radio_queue.head = 0;
    gnss_radio_ctx.radio_queue.tail = 0;

    /* Initialize TX dummy buffer (zeros for read operations) */
    memset(gnss_radio_ctx.tx_dummy, 0x00, sizeof(gnss_radio_ctx.tx_dummy));

    /* Enable split transactions by default */
    gnss_radio_ctx.split_transactions = true;

    gnss_radio_ctx.initialized = true;
}

void gnss_radio_set_split_mode(bool enable)
{
    gnss_radio_ctx.split_transactions = enable;
}

/* -------------------------------------------------------------------------- */
/* IRQ Handler                                                                */
/* -------------------------------------------------------------------------- */

void gnss_radio_irq_handler(void)
{
    if (!gnss_radio_ctx.initialized) {
        return;
    }

    gnss_radio_ctx.irq_count++;

    /* Only process if we're idle */
    if (gnss_radio_ctx.state != GNSS_STATE_IDLE) {
        /* Already processing a push transaction - IRQ might be stale */
        gnss_radio_ctx.irq_pending = true;
        return;
    }

    /* Start push mode read sequence */
    if (gnss_radio_ctx.split_transactions) {
        /* Two-phase mode: read TYPE first */
        submit_type_read();
    } else {
        /* Single-phase mode: read full message (TYPE + max payload) */
        submit_full_read();
    }
}

/* -------------------------------------------------------------------------- */
/* Phase 1: TYPE Read                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to read just the TYPE byte
 */
static void submit_type_read(void)
{
    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    job.len = GNSS_PHASE1_TYPE_READ_SIZE;
    job.t_submit = timestamp_us();
    job.max_duration_us = GNSS_PHASE1_DURATION_US;
    job.priority = SPI1_PRIO_HIGH;  /* TYPE read is always high priority */
    job.device = SPI1_DEVICE_GNSS_RADIO;
    job.preemptible = true;         /* Can be deferred for motor control */
    job.type = SPI_XFER_TXRX;
    job.done = gnss_radio_phase1_done;
    job.done_arg = NULL;
    job.task_notification_flag = 0;

    /* TX buffer: dummy bytes (slave ignores during push) */
    memset(job.tx, 0x00, job.len);

    gnss_radio_ctx.state = GNSS_STATE_TYPE_PENDING;

    if (!spi1_submit_job(&job)) {
        /* Job queued (not started immediately) */
    }

    gnss_radio_ctx.state = GNSS_STATE_TYPE_READING;
}

/**
 * @brief Phase 1 completion callback
 *
 * Called when TYPE byte has been read. Examines the type and submits
 * phase 2 job with appropriate priority.
 */
void gnss_radio_phase1_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg)
{
    (void)job;
    (void)arg;

    /* Extract TYPE byte */
    uint8_t type = rx_buf[0];
    gnss_radio_ctx.pending_type = type;

    /* Validate type and submit phase 2 */
    if (type == GNSS_PUSH_TYPE_GPS || type == GNSS_PUSH_TYPE_RADIO) {
        submit_payload_read(type);
    } else {
        /* Unknown type - return to idle */
        gnss_radio_ctx.transaction_errors++;
        gnss_radio_ctx.state = GNSS_STATE_IDLE;

        /* Check for pending IRQ */
        if (gnss_radio_ctx.irq_pending) {
            gnss_radio_ctx.irq_pending = false;
            gnss_radio_irq_handler();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Phase 2: PAYLOAD Read                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to read the PAYLOAD based on type
 *
 * @param type The TYPE byte from phase 1
 */
static void submit_payload_read(uint8_t type)
{
    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    job.t_submit = timestamp_us();
    job.device = SPI1_DEVICE_GNSS_RADIO;
    job.type = SPI_XFER_TXRX;
    job.done = gnss_radio_phase2_done;
    job.done_arg = (void *)(uintptr_t)type;

    /* Set length and priority based on type */
    switch (type) {
    case GNSS_PUSH_TYPE_GPS:
        job.len = GNSS_PUSH_GPS_PAYLOAD;
        job.max_duration_us = GNSS_GPS_PAYLOAD_DURATION_US;
        job.priority = SPI1_PRIO_HIGH;
        job.preemptible = false;  /* GPS is short, let it complete */
        job.task_notification_flag = GNSS_GPS_FIX_READY_FLAG;
        break;

    case GNSS_PUSH_TYPE_RADIO:
        job.len = GNSS_PUSH_RADIO_PAYLOAD;
        job.max_duration_us = GNSS_RADIO_PAYLOAD_DURATION_US;
        job.priority = SPI1_PRIO_NORMAL;
        job.preemptible = true;   /* Radio is long, can be deferred */
        job.task_notification_flag = GNSS_RADIO_MSG_READY_FLAG;
        break;

    default:
        /* Should not reach here */
        gnss_radio_ctx.state = GNSS_STATE_IDLE;
        return;
    }

    /* TX buffer: dummy bytes */
    memset(job.tx, 0x00, job.len);

    gnss_radio_ctx.state = GNSS_STATE_PAYLOAD_PENDING;

    if (!spi1_submit_job(&job)) {
        /* Job queued */
    }

    gnss_radio_ctx.state = GNSS_STATE_PAYLOAD_READING;
}

/**
 * @brief Phase 2 completion callback
 *
 * Called when PAYLOAD has been read. Parses the data and enqueues
 * to the appropriate queue.
 */
void gnss_radio_phase2_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg)
{
    (void)job;
    uint8_t type = (uint8_t)(uintptr_t)arg;

    switch (type) {
    case GNSS_PUSH_TYPE_GPS: {
        /* Parse GPS fix from rx_buf */
        const gnss_gps_fix_t *fix = (const gnss_gps_fix_t *)rx_buf;

        if (enqueue_gps_fix(fix)) {
            gnss_radio_ctx.gps_fixes_received++;
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;
    }

    case GNSS_PUSH_TYPE_RADIO:
        if (enqueue_radio_msg(rx_buf)) {
            gnss_radio_ctx.radio_msgs_received++;
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;

    default:
        gnss_radio_ctx.transaction_errors++;
        break;
    }

    /* Return to idle state */
    gnss_radio_ctx.state = GNSS_STATE_IDLE;

    /* Check for pending IRQ */
    if (gnss_radio_ctx.irq_pending) {
        gnss_radio_ctx.irq_pending = false;
        /* Re-trigger IRQ handler to process next message */
        gnss_radio_irq_handler();
    }
}

/* -------------------------------------------------------------------------- */
/* Single-Phase Mode (Non-Split)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a full read (TYPE + max PAYLOAD) in one transaction
 */
static void submit_full_read(void)
{
    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    /* Read TYPE + max payload size (Radio is larger) */
    job.len = GNSS_PUSH_TYPE_BYTES + GNSS_PUSH_RADIO_PAYLOAD;
    job.t_submit = timestamp_us();
    job.max_duration_us = GNSS_RADIO_PAYLOAD_DURATION_US + GNSS_PHASE1_DURATION_US;
    job.priority = SPI1_PRIO_NORMAL;  /* Use normal since we don't know type yet */
    job.device = SPI1_DEVICE_GNSS_RADIO;
    job.preemptible = true;
    job.type = SPI_XFER_TXRX;
    job.done = gnss_radio_single_phase_done;
    job.done_arg = NULL;
    job.task_notification_flag = 0;

    /* TX buffer: dummy bytes */
    memset(job.tx, 0x00, job.len);

    gnss_radio_ctx.state = GNSS_STATE_TYPE_READING;

    spi1_submit_job(&job);
}

/**
 * @brief Single-phase completion callback
 */
void gnss_radio_single_phase_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg)
{
    (void)job;
    (void)arg;

    /* First byte is TYPE */
    uint8_t type = rx_buf[0];
    const uint8_t *payload = &rx_buf[GNSS_PUSH_TYPE_BYTES];

    switch (type) {
    case GNSS_PUSH_TYPE_GPS: {
        const gnss_gps_fix_t *fix = (const gnss_gps_fix_t *)payload;

        if (enqueue_gps_fix(fix)) {
            gnss_radio_ctx.gps_fixes_received++;
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;
    }

    case GNSS_PUSH_TYPE_RADIO:
        if (enqueue_radio_msg(payload)) {
            gnss_radio_ctx.radio_msgs_received++;
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;

    default:
        gnss_radio_ctx.transaction_errors++;
        break;
    }

    /* Return to idle state */
    gnss_radio_ctx.state = GNSS_STATE_IDLE;

    /* Check for pending IRQ */
    if (gnss_radio_ctx.irq_pending) {
        gnss_radio_ctx.irq_pending = false;
        gnss_radio_irq_handler();
    }
}

/* -------------------------------------------------------------------------- */
/* Queue Operations                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enqueue a GPS fix (called from ISR context)
 */
static bool enqueue_gps_fix(const gnss_gps_fix_t *fix)
{
    gnss_gps_fix_queue_t *q = &gnss_radio_ctx.gps_queue;

    if (gnss_gps_queue_full()) {
        return false;
    }

    memcpy(&q->fixes[q->head], fix, sizeof(gnss_gps_fix_t));
    SYNC_DMB();
    q->head = (q->head + 1) % GNSS_GPS_FIX_QUEUE_LEN;
    return true;
}

/**
 * @brief Enqueue a radio message (called from ISR context)
 */
static bool enqueue_radio_msg(const uint8_t *msg)
{
    gnss_radio_msg_queue_t *q = &gnss_radio_ctx.radio_queue;

    if (gnss_radio_queue_full()) {
        return false;
    }

    memcpy(q->messages[q->head], msg, GNSS_RADIO_MESSAGE_MAX_LEN);
    SYNC_DMB();
    q->head = (q->head + 1) % GNSS_RADIO_MSG_QUEUE_LEN;
    return true;
}

bool gnss_gps_dequeue(gnss_gps_fix_t *fix)
{
    gnss_gps_fix_queue_t *q = &gnss_radio_ctx.gps_queue;

    if (gnss_gps_queue_empty()) {
        return false;
    }

    SYNC_DMB();
    memcpy(fix, &q->fixes[q->tail], sizeof(gnss_gps_fix_t));
    SYNC_DMB();
    q->tail = (q->tail + 1) % GNSS_GPS_FIX_QUEUE_LEN;
    return true;
}

bool gnss_gps_peek(gnss_gps_fix_t *fix)
{
    gnss_gps_fix_queue_t *q = &gnss_radio_ctx.gps_queue;

    if (gnss_gps_queue_empty()) {
        return false;
    }

    SYNC_DMB();
    memcpy(fix, &q->fixes[q->tail], sizeof(gnss_gps_fix_t));
    return true;
}

const gnss_gps_fix_t* gnss_gps_get_latest(void)
{
    gnss_gps_fix_queue_t *q = &gnss_radio_ctx.gps_queue;

    if (gnss_gps_queue_empty()) {
        return NULL;
    }

    /* Get most recent (head - 1) */
    uint8_t latest_idx = (q->head == 0) ? (GNSS_GPS_FIX_QUEUE_LEN - 1) : (q->head - 1);
    return &q->fixes[latest_idx];
}

bool gnss_radio_dequeue(uint8_t *msg)
{
    gnss_radio_msg_queue_t *q = &gnss_radio_ctx.radio_queue;

    if (gnss_radio_queue_empty()) {
        return false;
    }

    SYNC_DMB();
    memcpy(msg, q->messages[q->tail], GNSS_RADIO_MESSAGE_MAX_LEN);
    SYNC_DMB();
    q->tail = (q->tail + 1) % GNSS_RADIO_MSG_QUEUE_LEN;
    return true;
}

bool gnss_radio_peek(uint8_t *msg)
{
    gnss_radio_msg_queue_t *q = &gnss_radio_ctx.radio_queue;

    if (gnss_radio_queue_empty()) {
        return false;
    }

    SYNC_DMB();
    memcpy(msg, q->messages[q->tail], GNSS_RADIO_MESSAGE_MAX_LEN);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Radio TX (Pull Mode)                                                       */
/* -------------------------------------------------------------------------- */

bool gnss_radio_send(const uint8_t *msg, uint16_t len)
{
    if (!gnss_radio_ctx.initialized) {
        return false;
    }

    if (len > GNSS_RADIO_MESSAGE_MAX_LEN) {
        return false;
    }

    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    /* Pull mode: CMD + DUMMY + PAYLOAD */
    job.len = GNSS_CMD_OVERHEAD + len;
    job.t_submit = timestamp_us();
    job.max_duration_us = job.len * GNSS_SPI_US_PER_BYTE + 20;
    job.priority = SPI1_PRIO_NORMAL;
    job.device = SPI1_DEVICE_GNSS_RADIO;
    job.preemptible = true;
    job.type = SPI_XFER_TXRX;
    job.done = NULL;  /* No callback needed for TX */
    job.done_arg = NULL;
    job.task_notification_flag = 0;

    /* Build TX buffer: [CMD][DUMMY x4][PAYLOAD] */
    job.tx[0] = GNSS_CMD_RADIO_TX;
    memset(&job.tx[1], 0x00, GNSS_PULL_DUMMY_BYTES);
    memcpy(&job.tx[GNSS_CMD_OVERHEAD], msg, len);

    return spi1_submit_job(&job);
}

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

void gnss_radio_get_stats(uint32_t *gps_count,
                          uint32_t *radio_count,
                          uint32_t *errors)
{
    if (gps_count) {
        *gps_count = gnss_radio_ctx.gps_fixes_received;
    }
    if (radio_count) {
        *radio_count = gnss_radio_ctx.radio_msgs_received;
    }
    if (errors) {
        *errors = gnss_radio_ctx.transaction_errors + gnss_radio_ctx.queue_overflows;
    }
}

void gnss_radio_reset_stats(void)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    gnss_radio_ctx.gps_fixes_received = 0;
    gnss_radio_ctx.radio_msgs_received = 0;
    gnss_radio_ctx.irq_count = 0;
    gnss_radio_ctx.transaction_errors = 0;
    gnss_radio_ctx.queue_overflows = 0;

    SYNC_EXIT_CRITICAL_RAW(primask);
}

gnss_push_state_t gnss_radio_get_state(void)
{
    return gnss_radio_ctx.state;
}
