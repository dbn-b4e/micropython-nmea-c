/**
 * @file    nmea_vendor.h
 * @brief   Receiver configuration profiles (one per vendor command set).
 *
 * NMEA 0183 defines no configuration commands: every vendor has its own
 * proprietary sentences. A profile turns the generic requests below into the
 * vendor's commands and tells how to read the receiver's acknowledgement.
 *
 * Adding a vendor:
 *   1. write src/nmea_vendor_<name>.c implementing nmea_vendor_t;
 *   2. add it to nmea_vendors[] in nmea_vendor.c (and its id to nmea_vendor_id_t);
 *   3. add its documented examples to tests/test_core.c.
 * Only sentence-based (ASCII) command sets fit here; binary protocols such as
 * u-blox UBX need a different transport.
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#ifndef NMEA_VENDOR_H
#define NMEA_VENDOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Profiles compiled in. 0 = none (parse only, no configuration). */
typedef enum {
    NMEA_VENDOR_NONE = 0,
    NMEA_VENDOR_QUECTEL_PAIR = 1,   /**< Quectel LC26G / LC76G / LC86G (Airoha $PAIR commands) */
    NMEA_VENDOR_COUNT
} nmea_vendor_id_t;

/** Sentence types for set_output(), in the generic order of this library. */
typedef enum {
    NMEA_SEN_GGA = 0,
    NMEA_SEN_GLL,
    NMEA_SEN_GSA,
    NMEA_SEN_GSV,
    NMEA_SEN_RMC,
    NMEA_SEN_VTG,
    NMEA_SEN_ZDA,
    NMEA_SEN_GRS,
    NMEA_SEN_GST,
    NMEA_SEN_GNS,
    NMEA_SEN_COUNT
} nmea_sentence_t;

/** Result of a configuration request, as read from the acknowledgement. */
enum {
    NMEA_ACK_OK = 0,                /**< accepted */
    NMEA_ACK_PENDING = -1,          /**< "being processed": keep waiting */
    NMEA_ACK_TIMEOUT = -2,          /**< no acknowledgement in time */
    NMEA_ACK_UNSUPPORTED = -3,      /**< request not available in this profile */
    /* > 0: vendor error code, see the profile */
};

/** Maximum number of commands in one request (e.g. "save" sequences). */
#define NMEA_VENDOR_MAX_STEPS 4

/** One command to send and the prefix of the line that acknowledges it. */
typedef struct {
    char body[40];                  /**< text between '$' and '*' */
    char ack_prefix[24];            /**< e.g. "$PAIR001,050," */
} nmea_step_t;

typedef struct {
    const char* name;
    /** Fills steps[] for each request; returns the number of steps, 0 if unsupported. */
    int (*set_rate)(unsigned interval_ms, nmea_step_t* steps);
    int (*set_output)(nmea_sentence_t type, unsigned every_n_fixes, nmea_step_t* steps);
    int (*save)(nmea_step_t* steps);
    /** Reads an acknowledgement line: NMEA_ACK_OK, NMEA_ACK_PENDING or a vendor error code > 0. */
    int (*ack_result)(const char* line, size_t len);
} nmea_vendor_t;

/** Profile for an id, NULL for NMEA_VENDOR_NONE or an unknown id. */
const nmea_vendor_t* nmea_vendor_get(int id);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_VENDOR_H */
