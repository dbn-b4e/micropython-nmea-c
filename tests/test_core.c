/*
 * Unit tests of the command helpers: nmea_build(), nmea_expect(), nmea_response().
 * Reference commands and replies are the examples printed in the Quectel
 * LC26G/LC76G/LC86G Series GNSS Protocol Specification V1.4.
 *
 *   cc -std=c99 -Isrc tests/test_core.c src/nmea_core.c src/nmea_vendor.c -o test_core && ./test_core
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#include <stdio.h>
#include <string.h>

#include "nmea_core.h"
#include "nmea_vendor.h"

static int failures;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            failures++;                                                \
        }                                                              \
    } while (0)

static void check_build(const char* body, const char* expected) {
    char out[64];
    size_t n = nmea_build(body, strlen(body), out, sizeof(out));
    CHECK(n == strlen(expected));
    CHECK(strcmp(out, expected) == 0);
    if (strcmp(out, expected) != 0) {
        printf("     got %s     expected %s", out, expected);
    }
}

static void feed_str(nmea_t* p, const char* s) {
    nmea_feed(p, (const uint8_t*)s, strlen(s));
}

int main(void) {
    /* nmea_build: examples from the specification */
    check_build("PAIR050,1000", "$PAIR050,1000*12\r\n");
    check_build("PAIR050,100", "$PAIR050,100*22\r\n");
    check_build("PAIR062,0,3", "$PAIR062,0,3*3D\r\n");
    check_build("PAIR513", "$PAIR513*3D\r\n");
    check_build("PAIR382,1", "$PAIR382,1*2E\r\n");
    check_build("PAIR003", "$PAIR003*39\r\n");
    check_build("PAIR002", "$PAIR002*38\r\n");
    {
        char small[8];
        CHECK(nmea_build("PAIR050,100", 11, small, sizeof(small)) == 0);     /* too small */
    }

    /* nmea_expect / nmea_response in the middle of the NMEA stream */
    static nmea_t p;
    nmea_init(&p);
    size_t n = 0;
    CHECK(nmea_response(&p, &n) == NULL);
    nmea_expect(&p, "$PAIR001,050,", 13);
    feed_str(&p,
             "$GNRMC,114249.000,A,5042.385152,N,00422.317906,E,0.27,10.85,011026,,,A,V*39\r\n"
             "$PAIR001,062,0*3F\r\n"                    /* another command's reply: ignored */
             "$PAIR001,050,1*3F\r\n");                  /* processing */
    const char* r = nmea_response(&p, &n);
    CHECK(r != NULL && strcmp(r, "$PAIR001,050,1*3F") == 0 && n == 17);
    feed_str(&p,
             "$PQTMANTENNASTATUS,3,1,2,1*52\r\n"
             "$PAIR001,050,0*3E\r\n"                    /* done */
             "$GNGGA,114249.000,5042.385152,N,00422.317906,E,1,7,1.68,119.254,M,47.366,M,,*4E\r\n");
    r = nmea_response(&p, &n);
    CHECK(r != NULL && strcmp(r, "$PAIR001,050,0*3E") == 0);
    CHECK(p.responses == 2);
    CHECK(p.valid == 1 && p.lat_e7 == 507064192 && p.sats_used == 7);     /* NMEA still decoded */

    /* a matching line with a wrong checksum is not a reply */
    nmea_expect(&p, "$PAIR001,513,", 13);
    feed_str(&p, "$PAIR001,513,0*00\r\n");
    CHECK(nmea_response(&p, &n) == NULL && p.responses == 0);
    feed_str(&p, "$PAIR001,513,0*3C\r\n");
    r = nmea_response(&p, &n);
    CHECK(r != NULL && strcmp(r, "$PAIR001,513,0*3C") == 0);

    /* empty prefix: stop watching */
    nmea_expect(&p, "", 0);
    feed_str(&p, "$PAIR001,050,0*3E\r\n");
    CHECK(nmea_response(&p, &n) == NULL);

    /* Quectel $PAIR profile */
    const nmea_vendor_t* q = nmea_vendor_get(NMEA_VENDOR_QUECTEL_PAIR);
    CHECK(q != NULL && nmea_vendor_get(NMEA_VENDOR_NONE) == NULL && nmea_vendor_get(99) == NULL);
    nmea_step_t st[NMEA_VENDOR_MAX_STEPS];
    CHECK(q->set_rate(100, st) == 1 && strcmp(st[0].body, "PAIR050,100") == 0 &&
          strcmp(st[0].ack_prefix, "$PAIR001,050,") == 0);
    CHECK(q->set_rate(1000, st) == 1 && strcmp(st[0].body, "PAIR050,1000") == 0);
    CHECK(q->set_rate(50, st) == 0 && q->set_rate(1001, st) == 0);          /* out of range */
    CHECK(q->set_output(NMEA_SEN_GGA, 3, st) == 1 && strcmp(st[0].body, "PAIR062,0,3") == 0 &&
          strcmp(st[0].ack_prefix, "$PAIR001,062,") == 0);
    CHECK(q->set_output(NMEA_SEN_GSV, 10, st) == 1 && strcmp(st[0].body, "PAIR062,3,10") == 0);
    CHECK(q->set_output(NMEA_SEN_VTG, 0, st) == 1 && strcmp(st[0].body, "PAIR062,5,0") == 0);
    CHECK(q->set_output(NMEA_SEN_GSV, 21, st) == 0);
    CHECK(q->save(st) == 4 && strcmp(st[0].body, "PAIR382,1") == 0 && strcmp(st[1].body, "PAIR003") == 0 &&
          strcmp(st[2].body, "PAIR513") == 0 && strcmp(st[3].body, "PAIR002") == 0 &&
          strcmp(st[0].ack_prefix, "$PAIR001,382,") == 0 && strcmp(st[1].ack_prefix, "$PAIR001,003,") == 0 &&
          strcmp(st[2].ack_prefix, "$PAIR001,513,") == 0 && strcmp(st[3].ack_prefix, "$PAIR001,002,") == 0);
    {
        char out[64];
        nmea_build(st[2].body, strlen(st[2].body), out, sizeof(out));
        CHECK(strcmp(out, "$PAIR513*3D\r\n") == 0);                     /* specification example */
    }
    CHECK(q->ack_result("$PAIR001,050,0*3E", 17) == NMEA_ACK_OK);
    CHECK(q->ack_result("$PAIR001,050,1*3F", 17) == NMEA_ACK_PENDING);
    CHECK(q->ack_result("$PAIR001,062,4*3B", 17) == 4);
    CHECK(q->ack_result("$PAIR001,062,5*3A", 17) == 5);
    CHECK(q->ack_result("$PAIR001,062*00", 15) > 0);                         /* malformed -> failure */

    if (failures) {
        printf("FAILED: %d check(s)\n", failures);
        return 1;
    }
    printf("OK: command helpers\n");
    return 0;
}
