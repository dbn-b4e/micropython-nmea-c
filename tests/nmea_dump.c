/*
 * Host test harness: reads an NMEA byte stream on stdin, feeds it to the
 * parser and prints the decoded state as one CSV line.
 *
 *   nmea_dump lines    feed byte by byte, print the state after every '\n'
 *   nmea_dump chunks   feed in irregular chunks (1..23 bytes), print the final state only
 *
 * Used by tests/test_vs_pynmea2.py.
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#include <stdio.h>
#include <string.h>

#include "nmea_core.h"

static void dump(const nmea_t* p) {
    printf("%u,%u,%ld,%ld,%ld,%ld,%lu,%u,%lu,%lu,%lu,%u,%u,%d,%u,%u,%u,%u,%u,%u,"
           "%lu,%lu,%lu,%lu,%lu,%lu\n",
           (unsigned)p->seq, p->valid, (long)p->lat_e7, (long)p->lon_e7, (long)p->alt_cm, (long)p->geoid_cm,
           (unsigned long)p->speed_cms, p->course_e2, (unsigned long)p->time_ms, (unsigned long)p->date,
           (unsigned long)p->utc, p->fix, p->quality, p->mode, p->sats_used, p->sats_in_view, p->hdop, p->pdop,
           p->vdop, p->have, (unsigned long)p->stats.sentences, (unsigned long)p->stats.decoded,
           (unsigned long)p->stats.ignored, (unsigned long)p->stats.checksum_errors,
           (unsigned long)p->stats.format_errors, (unsigned long)p->stats.overflows);
}

int main(int argc, char** argv) {
    static uint8_t buf[1 << 20];
    size_t n = fread(buf, 1, sizeof(buf), stdin);
    static nmea_t p;
    nmea_init(&p);
    if (argc > 1 && strcmp(argv[1], "chunks") == 0) {
        size_t i = 0, k = 0;
        while (i < n) {
            size_t c = 1 + (k++ * 7) % 23;
            if (c > n - i) {
                c = n - i;
            }
            nmea_feed(&p, buf + i, c);
            i += c;
        }
        dump(&p);
        return 0;
    }
    for (size_t i = 0; i < n; i++) {
        nmea_feed(&p, buf + i, 1);
        if (buf[i] == '\n') {
            dump(&p);
        }
    }
    return 0;
}
