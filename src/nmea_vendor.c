/**
 * @file    nmea_vendor.c
 * @brief   Profile table and the Quectel $PAIR profile.
 *
 * Quectel profile: LC26G&LC26G-T&LC76G&LC86G Series GNSS Protocol
 * Specification V1.4 (2.4.1 PAIR001, 2.4.10 PAIR050, 2.4.14 PAIR062,
 * 2.4.34 PAIR382, 2.4.3 PAIR003, 2.4.49 PAIR513, 2.4.2 PAIR002).
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#include "nmea_vendor.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ Quectel $PAIR */

static void pair_step(nmea_step_t* s, const char* body, unsigned cmd) {
    snprintf(s->body, sizeof(s->body), "%s", body);
    snprintf(s->ack_prefix, sizeof(s->ack_prefix), "$PAIR001,%03u,", cmd);
}

/* PAIR050: position fix interval, 100..1000 ms. Above 1 Hz the receiver
 * outputs only RMC, GGA and GNS at that rate, GSA and GSV at 1 Hz, and
 * stops VTG, GLL, ZDA, GRS and GST. */
static int pair_set_rate(unsigned interval_ms, nmea_step_t* steps) {
    if (interval_ms < 100 || interval_ms > 1000) {
        return 0;
    }
    char b[24];
    snprintf(b, sizeof(b), "PAIR050,%u", interval_ms);
    pair_step(&steps[0], b, 50);
    return 1;
}

/* PAIR062: output a sentence type once every N fixes (0 = off, N <= 20).
 * Its type numbering (GGA 0, GLL 1, GSA 2, GSV 3, RMC 4, VTG 5, ZDA 6, GRS 7,
 * GST 8, GNS 9) is the one of nmea_sentence_t. */
static int pair_set_output(nmea_sentence_t type, unsigned n, nmea_step_t* steps) {
    if ((unsigned)type >= NMEA_SEN_COUNT || n > 20) {
        return 0;
    }
    char b[24];
    snprintf(b, sizeof(b), "PAIR062,%u,%u", (unsigned)type, n);
    pair_step(&steps[0], b, 62);
    return 1;
}

/* PAIR513 saves the settings to flash. The specification requires, when the
 * fix rate exceeds 1 Hz, to stop the GNSS first (PAIR382,1 then PAIR003) and
 * to restart it afterwards (PAIR002); the sequence is harmless at 1 Hz. */
static int pair_save(nmea_step_t* steps) {
    pair_step(&steps[0], "PAIR382,1", 382);
    pair_step(&steps[1], "PAIR003", 3);
    pair_step(&steps[2], "PAIR513", 513);
    pair_step(&steps[3], "PAIR002", 2);
    return 4;
}

/* $PAIR001,<CommandID>,<Result>: 0 sent, 1 processing, 2 failed,
 * 3 not supported, 4 parameter error, 5 busy. */
static int pair_ack_result(const char* line, size_t len) {
    const char* end = memchr(line, '*', len);
    if (end == NULL) {
        return 2;
    }
    const char* comma = NULL;
    for (const char* c = line; c < end; c++) {
        if (*c == ',') {
            comma = c;
        }
    }
    if (comma == NULL || comma + 2 != end || comma[1] < '0' || comma[1] > '9') {
        return 2;
    }
    int r = comma[1] - '0';
    return r == 0 ? NMEA_ACK_OK : r == 1 ? NMEA_ACK_PENDING : r;
}

static const nmea_vendor_t quectel_pair = {
    .name = "Quectel PAIR",
    .set_rate = pair_set_rate,
    .set_output = pair_set_output,
    .save = pair_save,
    .ack_result = pair_ack_result,
};

/* ------------------------------------------------------------------ table */

static const nmea_vendor_t* const nmea_vendors[NMEA_VENDOR_COUNT] = {
    [NMEA_VENDOR_NONE] = NULL,
    [NMEA_VENDOR_QUECTEL_PAIR] = &quectel_pair,
};

const nmea_vendor_t* nmea_vendor_get(int id) {
    if (id <= 0 || id >= NMEA_VENDOR_COUNT) {
        return NULL;
    }
    return nmea_vendors[id];
}
