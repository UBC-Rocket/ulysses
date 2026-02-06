/**
 * @file gnss_radio_protocol.h
 * @brief Protocol definitions for GNSS Radio slave communication
 *
 * This header defines the protocol shared between the ulysses flight controller
 * (master) and the ulysses-gnss-radio board (slave). It mirrors the protocol
 * definitions from the slave side (protocol_config.h).
 *
 * Protocol Modes:
 * - Pull mode: Master-initiated, command-based transactions
 * - Push mode: Slave-initiated via IRQ, data-driven transactions
 *
 * UBC Rocket, Feb 2026
 */

#ifndef GNSS_RADIO_PROTOCOL_H
#define GNSS_RADIO_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Protocol Mode Selection                                                    */
/* -------------------------------------------------------------------------- */

typedef enum {
    GNSS_PROTOCOL_MODE_PULL = 0x00,  /**< Master-initiated, command-based */
    GNSS_PROTOCOL_MODE_PUSH = 0x01,  /**< Slave-initiated via IRQ */
} gnss_protocol_mode_t;

/* -------------------------------------------------------------------------- */
/* Pull Mode Command Definitions                                              */
/* -------------------------------------------------------------------------- */

typedef enum {
    GNSS_CMD_RADIO_RX_LIFO   = 0x01,  /**< Read newest radio message (LIFO) */
    GNSS_CMD_RADIO_RX_FIFO   = 0x02,  /**< Read oldest radio message (FIFO) */
    GNSS_CMD_RADIO_RXBUF_LEN = 0x03,  /**< Read radio buffer message count */
    GNSS_CMD_RADIO_TX        = 0x04,  /**< Write radio message to transmit */
    GNSS_CMD_GPS_RX          = 0x05,  /**< Read raw NMEA sentence (pull mode) */
} gnss_pull_command_t;

/* -------------------------------------------------------------------------- */
/* Push Mode Data Type Identifiers                                            */
/* -------------------------------------------------------------------------- */

typedef enum {
    GNSS_PUSH_TYPE_RADIO = 0x01,  /**< Radio message available */
    GNSS_PUSH_TYPE_GPS   = 0x05,  /**< GPS fix available */
} gnss_push_type_t;

/* -------------------------------------------------------------------------- */
/* Protocol Transaction Sizes                                                 */
/* -------------------------------------------------------------------------- */

/** Pull mode transaction sizes (CMD + DUMMY + DATA) */
#define GNSS_PULL_CMD_BYTES           1
#define GNSS_PULL_DUMMY_BYTES         4    /**< CRITICAL: 4 bytes needed to avoid TX FIFO prefetch race */
#define GNSS_CMD_OVERHEAD             (GNSS_PULL_CMD_BYTES + GNSS_PULL_DUMMY_BYTES)  /* 5 bytes */
#define GNSS_PULL_RADIO_PAYLOAD       256
#define GNSS_PULL_GPS_PAYLOAD         87   /**< Max NMEA sentence length */
#define GNSS_PULL_BUFLEN_PAYLOAD      1    /**< Single byte count */

#define GNSS_PULL_RADIO_TOTAL         (GNSS_PULL_CMD_BYTES + GNSS_PULL_DUMMY_BYTES + GNSS_PULL_RADIO_PAYLOAD)  /* 261 */
#define GNSS_PULL_GPS_TOTAL           (GNSS_PULL_CMD_BYTES + GNSS_PULL_DUMMY_BYTES + GNSS_PULL_GPS_PAYLOAD)    /* 92 */
#define GNSS_PULL_BUFLEN_TOTAL        (GNSS_PULL_CMD_BYTES + GNSS_PULL_DUMMY_BYTES + GNSS_PULL_BUFLEN_PAYLOAD) /* 6 */

/** Push mode transaction sizes (TYPE + PAYLOAD) */
#define GNSS_PUSH_TYPE_BYTES          1
#define GNSS_PUSH_RADIO_PAYLOAD       256
#define GNSS_PUSH_GPS_PAYLOAD         48   /**< sizeof(gnss_gps_fix_t) */

#define GNSS_PUSH_RADIO_TOTAL         (GNSS_PUSH_TYPE_BYTES + GNSS_PUSH_RADIO_PAYLOAD)  /* 257 */
#define GNSS_PUSH_GPS_TOTAL           (GNSS_PUSH_TYPE_BYTES + GNSS_PUSH_GPS_PAYLOAD)    /* 49 */

/** Maximum transaction size (for buffer allocation) */
#define GNSS_MAX_TRANSACTION_SIZE     GNSS_PULL_RADIO_TOTAL  /* 261 bytes */

/** Two-phase read sizes */
#define GNSS_PHASE1_TYPE_READ_SIZE    GNSS_PUSH_TYPE_BYTES   /* 1 byte */

/* -------------------------------------------------------------------------- */
/* Timing Constants                                                           */
/* -------------------------------------------------------------------------- */

/**
 * Minimum time (microseconds) for master to respond to IRQ assertion.
 * If CS falls within this time after IRQ assertion, it's a collision.
 */
#define GNSS_T_RACE_US                15

/**
 * Estimated SPI transfer duration per byte at typical clock speeds.
 * Used for window guard calculations.
 * At 10 MHz SPI clock: 8 bits / 10 MHz = 0.8 us/byte, round up to 1 us
 */
#define GNSS_SPI_US_PER_BYTE          1

/** Estimated phase 1 duration (TYPE read) including overhead */
#define GNSS_PHASE1_DURATION_US       (GNSS_PHASE1_TYPE_READ_SIZE * GNSS_SPI_US_PER_BYTE + 10)

/** Estimated GPS payload read duration */
#define GNSS_GPS_PAYLOAD_DURATION_US  (GNSS_PUSH_GPS_PAYLOAD * GNSS_SPI_US_PER_BYTE + 10)

/** Estimated Radio payload read duration */
#define GNSS_RADIO_PAYLOAD_DURATION_US (GNSS_PUSH_RADIO_PAYLOAD * GNSS_SPI_US_PER_BYTE + 20)

/* -------------------------------------------------------------------------- */
/* GPS Fix Structure (Push Mode)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief GPS fix data structure for push mode
 *
 * This structure matches the slave-side gps_fix_t exactly.
 * Total size: 48 bytes
 */
typedef struct {
    /* Position */
    double   latitude;         /**< Degrees, signed (-90 to +90)          [8 bytes] */
    double   longitude;        /**< Degrees, signed (-180 to +180)        [8 bytes] */
    float    altitude_msl;     /**< Meters above sea level                [4 bytes] */

    /* Velocity & Heading */
    float    ground_speed;     /**< m/s                                   [4 bytes] */
    float    course;           /**< Degrees true north (0-360)            [4 bytes] */

    /* Quality indicators */
    uint8_t  fix_quality;      /**< 0=invalid, 1=GPS, 2=DGPS, etc.        [1 byte] */
    uint8_t  num_satellites;   /**< Number of satellites used             [1 byte] */
    uint8_t  padding1[2];      /**< Alignment padding                     [2 bytes] */
    float    hdop;             /**< Horizontal dilution of precision      [4 bytes] */

    /* Timestamp */
    uint32_t time_of_week_ms;  /**< GPS time of week in milliseconds      [4 bytes] */

    uint8_t  padding2[8];      /**< Padding to 48 bytes total             [8 bytes] */
} __attribute__((packed)) gnss_gps_fix_t;

/* Verify struct size matches GNSS_PUSH_GPS_PAYLOAD */
_Static_assert(sizeof(gnss_gps_fix_t) == GNSS_PUSH_GPS_PAYLOAD,
               "gnss_gps_fix_t size mismatch with protocol");

/* -------------------------------------------------------------------------- */
/* Radio Message Constants                                                    */
/* -------------------------------------------------------------------------- */

#define GNSS_RADIO_MESSAGE_MAX_LEN    256
#define GNSS_RADIO_QUEUE_DEPTH        10
#define GNSS_GPS_QUEUE_DEPTH          10

/* -------------------------------------------------------------------------- */
/* Configuration Frame Structure (Startup)                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration frame sent to slave at startup
 */
typedef struct {
    uint8_t mode;                 /**< 0x00 = Pull, 0x01 = Push */
    uint8_t gps_rate;             /**< GPS update rate (Hz) */
    uint8_t gps_constellation;    /**< GNSS constellation mask */
    uint8_t reserved[5];          /**< Future use */
} __attribute__((packed)) gnss_config_frame_t;

#define GNSS_CONFIG_FRAME_SIZE sizeof(gnss_config_frame_t)  /* 8 bytes */

#ifdef __cplusplus
}
#endif

#endif /* GNSS_RADIO_PROTOCOL_H */
