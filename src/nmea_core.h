/**
 * @file    nmea_core.h
 * @brief   Allocation-free, integer-only NMEA 0183 parser (pure C99).
 *
 * Decodes the sentences a navigation application needs - RMC, GGA, GSA and
 * the header of GSV - from any talker (GP, GL, GA, GB, GQ, GI, GN, BD...).
 * Everything else is checksum-verified and counted, then ignored.
 *
 * Design rules:
 *  - no heap, no floating point: every value is a scaled integer;
 *  - byte-oriented: feed whatever the UART delivered, in any chunk size;
 *  - a sentence is only used if its checksum is present and correct;
 *  - no dependency on MicroPython: the same file builds into the MicroPython
 *    module (modnmea.c), into host tests, or into any C/C++ firmware.
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#ifndef NMEA_CORE_H
#define NMEA_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest sentence accepted, '$' to checksum included (NMEA 0183 says 82). */
#define NMEA_LINE_MAX 120

/** Number of constellations tracked for the satellites-in-view count. */
#define NMEA_GSV_SYSTEMS 8

/** Fix type, from GSA field 2. */
typedef enum {
    NMEA_FIX_NONE = 1,          /**< no fix (also the value before any GSA) */
    NMEA_FIX_2D = 2,
    NMEA_FIX_3D = 3,
} nmea_fix_t;

/** Bits of nmea_t.have: which values have been received at least once. */
enum {
    NMEA_HAVE_TIME = 1u << 0,   /**< time_ms (RMC or GGA) */
    NMEA_HAVE_DATE = 1u << 1,   /**< date and therefore utc (RMC) */
    NMEA_HAVE_POS = 1u << 2,    /**< lat_e7 / lon_e7 (RMC status A or GGA quality > 0) */
    NMEA_HAVE_ALT = 1u << 3,    /**< alt_cm / geoid_cm (GGA) */
    NMEA_HAVE_SPEED = 1u << 4,  /**< speed_cms (RMC) */
    NMEA_HAVE_COURSE = 1u << 5, /**< course_e2 (RMC; empty when stationary on many receivers) */
    NMEA_HAVE_DOP = 1u << 6,    /**< hdop / pdop / vdop (GSA, hdop also GGA) */
    NMEA_HAVE_VIEW = 1u << 7,   /**< sats_in_view (GSV) */
};

/** Counters - never reset except by nmea_init() / nmea_reset_stats(). */
typedef struct {
    uint32_t sentences;         /**< checksum-valid sentences */
    uint32_t decoded;           /**< of which RMC / GGA / GSA / GSV decoded */
    uint32_t ignored;           /**< valid but not decoded (GLL, VTG, proprietary...) */
    uint32_t checksum_errors;   /**< wrong or missing checksum */
    uint32_t format_errors;     /**< valid checksum but a field could not be parsed */
    uint32_t overflows;         /**< line longer than NMEA_LINE_MAX, dropped */
} nmea_stats_t;

/** Parser state and decoded values. Treat as read-only outside nmea_core.c. */
typedef struct {
    /* Position - WGS84, 1e-7 degree (north / east positive). */
    int32_t lat_e7;
    int32_t lon_e7;
    /* Altitude above mean sea level and geoid separation, centimetres (GGA). */
    int32_t alt_cm;
    int32_t geoid_cm;
    /* Speed over ground, cm/s (RMC knots converted); course over ground, 0.01 degree. */
    uint32_t speed_cms;
    uint16_t course_e2;
    /* Time of day, milliseconds since 00:00 UTC; date as YYYYMMDD; Unix time, seconds. */
    uint32_t time_ms;
    uint32_t date;
    uint32_t utc;
    /* Fix. */
    uint8_t valid;              /**< RMC status: 1 = 'A' (valid), 0 = 'V' or not received */
    uint8_t fix;                /**< nmea_fix_t (GSA), NMEA_FIX_NONE until a GSA arrives */
    uint8_t quality;            /**< GGA fix quality: 0 none, 1 GPS, 2 DGPS/SBAS, 4 RTK, 5 float RTK, 6 dead reckoning... */
    char mode;                  /**< RMC mode indicator (NMEA 2.3+): 'A' autonomous, 'D' differential, 'E' estimated, 'N' not valid; 0 if absent */
    uint8_t sats_used;          /**< GGA field 7 */
    uint8_t sats_in_view;       /**< sum over constellations of the GSV "satellites in view" field */
    /* Dilution of precision, x100 (1.68 -> 168); 0 = not available. */
    uint16_t hdop;
    uint16_t pdop;
    uint16_t vdop;
    /* Bookkeeping. */
    uint16_t have;              /**< NMEA_HAVE_* bits */
    uint32_t seq;               /**< incremented on every decoded RMC (one per navigation epoch) */
    nmea_stats_t stats;

    /* Private. */
    char line[NMEA_LINE_MAX + 1];
    uint16_t len;
    uint8_t in_line;
    uint8_t gsv_view[NMEA_GSV_SYSTEMS];   /**< satellites in view per constellation */
    uint32_t gsv_seq[NMEA_GSV_SYSTEMS];   /**< seq value when that count was received */
} nmea_t;

/** Clears values, state and counters. */
void nmea_init(nmea_t* p);

/** Clears the counters only. */
void nmea_reset_stats(nmea_t* p);

/**
 * @brief Feeds received bytes, in any chunk size.
 * @return Number of checksum-valid sentences completed by this call.
 */
int nmea_feed(nmea_t* p, const uint8_t* data, size_t len);

/** Checksum of the characters between '$' and '*' (XOR), as used by NMEA. */
uint8_t nmea_checksum(const char* s, size_t len);

/** Days since 1970-01-01 for a proleptic Gregorian date (exposed for tests). */
int32_t nmea_days_from_civil(int32_t y, unsigned m, unsigned d);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_CORE_H */
