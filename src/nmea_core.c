/**
 * @file    nmea_core.c
 * @brief   Allocation-free, integer-only NMEA 0183 parser (pure C99).
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#include "nmea_core.h"

#include <string.h>

#define MAX_FIELDS 32

/* ------------------------------------------------------------------ fields */

typedef struct {
    const char* s[MAX_FIELDS];
    uint8_t n[MAX_FIELDS];
    int count;
} fields_t;

/** Splits "$GPRMC,a,b*hh" (between '$' and '*') on commas. */
static void split(const char* body, size_t len, fields_t* f) {
    f->count = 0;
    size_t start = 0;
    for (size_t i = 0; i <= len && f->count < MAX_FIELDS; i++) {
        if (i == len || body[i] == ',') {
            f->s[f->count] = body + start;
            f->n[f->count] = (uint8_t)(i - start);
            f->count++;
            start = i + 1;
        }
    }
}

/** Field i, or an empty field when the sentence is shorter. */
static int field(const fields_t* f, int i, const char** s) {
    if (i >= f->count) {
        *s = "";
        return 0;
    }
    *s = f->s[i];
    return f->n[i];
}

static int is_digit(char c) {
    return c >= '0' && c <= '9';
}

/** Unsigned integer. Returns 1 if parsed, 0 if empty, -1 if malformed. */
static int parse_uint(const char* s, int n, uint32_t* out) {
    if (n == 0) {
        return 0;
    }
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        if (!is_digit(s[i]) || v > 429496728u) {
            return -1;
        }
        v = v * 10u + (uint32_t)(s[i] - '0');
    }
    *out = v;
    return 1;
}

/**
 * Decimal number scaled by 10^digits ("-12.345", digits 2 -> -1234; extra
 * decimals are truncated). Returns 1 if parsed, 0 if empty, -1 if malformed.
 */
static int parse_fixed(const char* s, int n, int digits, int64_t* out) {
    if (n == 0) {
        return 0;
    }
    int i = 0;
    int neg = 0;
    if (s[0] == '-' || s[0] == '+') {
        neg = s[0] == '-';
        i = 1;
    }
    int64_t v = 0;
    int seen = 0;
    int frac = -1;                              /* decimals consumed, -1 before the point */
    for (; i < n; i++) {
        char c = s[i];
        if (c == '.') {
            if (frac >= 0) {
                return -1;
            }
            frac = 0;
            continue;
        }
        if (!is_digit(c)) {
            return -1;
        }
        seen = 1;
        if (frac >= 0) {
            if (frac >= digits) {
                continue;                       /* truncate */
            }
            frac++;
        }
        if (v > 92233720368547758LL) {
            return -1;
        }
        v = v * 10 + (c - '0');
    }
    if (!seen) {
        return -1;
    }
    for (int k = frac < 0 ? 0 : frac; k < digits; k++) {
        v *= 10;
    }
    *out = neg ? -v : v;
    return 1;
}

/**
 * Latitude / longitude "dddmm.mmmm" + hemisphere -> 1e-7 degree.
 * The degree count is whatever precedes the two minute digits.
 */
static int parse_coord(const char* s, int n, const char* h, int hn, int max_deg, int32_t* out) {
    if (n == 0 || hn == 0) {
        return 0;
    }
    int dot = n;
    for (int i = 0; i < n; i++) {
        if (s[i] == '.') {
            dot = i;
            break;
        }
    }
    if (dot < 3) {
        return -1;
    }
    uint32_t deg;
    if (parse_uint(s, dot - 2, &deg) != 1 || deg > (uint32_t)max_deg) {
        return -1;
    }
    int64_t min_e7;
    if (parse_fixed(s + dot - 2, n - (dot - 2), 7, &min_e7) != 1 || min_e7 < 0 || min_e7 >= 600000000LL) {
        return -1;
    }
    int64_t v = (int64_t)deg * 10000000LL + (min_e7 + 30) / 60;     /* rounded */
    if (v > (int64_t)max_deg * 10000000LL) {
        return -1;
    }
    char c = h[0];
    if (c == 'S' || c == 'W') {
        v = -v;
    } else if (c != 'N' && c != 'E') {
        return -1;
    }
    *out = (int32_t)v;
    return 1;
}

/** "hhmmss.sss" -> milliseconds since midnight. */
static int parse_time(const char* s, int n, uint32_t* out) {
    if (n == 0) {
        return 0;
    }
    if (n < 6) {
        return -1;
    }
    uint32_t hh, mm;
    int64_t ss_ms;
    if (parse_uint(s, 2, &hh) != 1 || parse_uint(s + 2, 2, &mm) != 1 ||
        parse_fixed(s + 4, n - 4, 3, &ss_ms) != 1) {
        return -1;
    }
    if (hh > 23 || mm > 59 || ss_ms < 0 || ss_ms >= 61000) {       /* 60 = leap second */
        return -1;
    }
    *out = hh * 3600000u + mm * 60000u + (uint32_t)ss_ms;
    return 1;
}

/** "ddmmyy" -> YYYYMMDD (years 80..99 -> 19xx, otherwise 20xx). */
static int parse_date(const char* s, int n, uint32_t* out) {
    if (n == 0) {
        return 0;
    }
    uint32_t dd, mo, yy;
    if (n != 6 || parse_uint(s, 2, &dd) != 1 || parse_uint(s + 2, 2, &mo) != 1 || parse_uint(s + 4, 2, &yy) != 1) {
        return -1;
    }
    if (dd < 1 || dd > 31 || mo < 1 || mo > 12) {
        return -1;
    }
    uint32_t year = yy >= 80 ? 1900 + yy : 2000 + yy;
    *out = year * 10000u + mo * 100u + dd;
    return 1;
}

int32_t nmea_days_from_civil(int32_t y, unsigned m, unsigned d) {
    /* Howard Hinnant, "chrono-Compatible Low-Level Date Algorithms". */
    y -= m <= 2;
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

/* ------------------------------------------------------------------ sentences */

/** Index of a talker in gsv_view[]: GP, GL, GA, GB/BD, GQ, GI, GN, other. */
static int system_index(const char* talker) {
    static const char ids[] = "GPGLGAGBGQGIGN";
    if (talker[0] == 'B' && talker[1] == 'D') {
        return 3;
    }
    for (int i = 0; i < 7; i++) {
        if (talker[0] == ids[2 * i] && talker[1] == ids[2 * i + 1]) {
            return i;
        }
    }
    return 7;
}

#define CHECK(expr)      \
    do {                 \
        if ((expr) < 0) { \
            return -1;   \
        }                \
    } while (0)

static int decode_rmc(nmea_t* p, const fields_t* f) {
    const char *t, *st, *la, *ns, *lo, *ew, *sp, *co, *da, *mo;
    int tn = field(f, 1, &t), stn = field(f, 2, &st), lan = field(f, 3, &la), nsn = field(f, 4, &ns);
    int lon = field(f, 5, &lo), ewn = field(f, 6, &ew), spn = field(f, 7, &sp), con = field(f, 8, &co);
    int dan = field(f, 9, &da), mon = field(f, 12, &mo);

    uint32_t time_ms = 0, date = 0;
    int32_t lat = 0, lng = 0;
    int64_t kn_e3 = 0, cog_e2 = 0;
    int rt, rd, rla, rlo, rs, rc;
    CHECK(rt = parse_time(t, tn, &time_ms));
    CHECK(rd = parse_date(da, dan, &date));
    CHECK(rla = parse_coord(la, lan, ns, nsn, 90, &lat));
    CHECK(rlo = parse_coord(lo, lon, ew, ewn, 180, &lng));
    CHECK(rs = parse_fixed(sp, spn, 3, &kn_e3));
    CHECK(rc = parse_fixed(co, con, 2, &cog_e2));
    if (stn != 1 || (st[0] != 'A' && st[0] != 'V') || kn_e3 < 0 || cog_e2 < 0) {
        return -1;
    }

    /* Everything parsed: commit. */
    if (rt == 1) {
        p->time_ms = time_ms;
        p->have |= NMEA_HAVE_TIME;
    }
    if (rd == 1) {
        p->date = date;
        p->have |= NMEA_HAVE_DATE;
    }
    if (rt == 1 && rd == 1) {
        int32_t days = nmea_days_from_civil((int32_t)(date / 10000), (date / 100) % 100, date % 100);
        p->utc = (uint32_t)days * 86400u + time_ms / 1000u;
    }
    p->valid = st[0] == 'A';
    if (p->valid && rla == 1 && rlo == 1) {
        p->lat_e7 = lat;
        p->lon_e7 = lng;
        p->have |= NMEA_HAVE_POS;
    }
    if (rs == 1) {
        p->speed_cms = (uint32_t)((kn_e3 * 1852 + 18000) / 36000);   /* knot = 1852 m/h */
        p->have |= NMEA_HAVE_SPEED;
    }
    if (rc == 1) {
        p->course_e2 = (uint16_t)(cog_e2 % 36000);
        p->have |= NMEA_HAVE_COURSE;
    }
    p->mode = mon >= 1 ? mo[0] : 0;
    p->seq++;
    return 0;
}

static int decode_gga(nmea_t* p, const fields_t* f) {
    const char *t, *la, *ns, *lo, *ew, *q, *nsat, *hd, *al, *sep;
    int tn = field(f, 1, &t), lan = field(f, 2, &la), nsn = field(f, 3, &ns), lon = field(f, 4, &lo);
    int ewn = field(f, 5, &ew), qn = field(f, 6, &q), nn = field(f, 7, &nsat), hn = field(f, 8, &hd);
    int aln = field(f, 9, &al), sepn = field(f, 11, &sep);

    uint32_t time_ms = 0, quality = 0, sats = 0;
    int32_t lat = 0, lng = 0;
    int64_t hdop = 0, alt_cm = 0, sep_cm = 0;
    int rt, rla, rlo, rq, rn, rh, ra, rs;
    CHECK(rt = parse_time(t, tn, &time_ms));
    CHECK(rla = parse_coord(la, lan, ns, nsn, 90, &lat));
    CHECK(rlo = parse_coord(lo, lon, ew, ewn, 180, &lng));
    CHECK(rq = parse_uint(q, qn, &quality));
    CHECK(rn = parse_uint(nsat, nn, &sats));
    CHECK(rh = parse_fixed(hd, hn, 2, &hdop));
    CHECK(ra = parse_fixed(al, aln, 2, &alt_cm));
    CHECK(rs = parse_fixed(sep, sepn, 2, &sep_cm));
    if (quality > 9 || sats > 255 || hdop < 0 || hdop > 65535 || alt_cm < INT32_MIN || alt_cm > INT32_MAX) {
        return -1;
    }

    if (rt == 1) {
        p->time_ms = time_ms;
        p->have |= NMEA_HAVE_TIME;
    }
    p->quality = rq == 1 ? (uint8_t)quality : 0;
    if (rn == 1) {
        p->sats_used = (uint8_t)sats;
    }
    if (p->quality > 0 && rla == 1 && rlo == 1) {
        p->lat_e7 = lat;
        p->lon_e7 = lng;
        p->have |= NMEA_HAVE_POS;
    }
    if (rh == 1) {
        p->hdop = (uint16_t)hdop;
        p->have |= NMEA_HAVE_DOP;
    }
    if (p->quality > 0 && ra == 1) {
        p->alt_cm = (int32_t)alt_cm;
        p->geoid_cm = rs == 1 ? (int32_t)sep_cm : 0;
        p->have |= NMEA_HAVE_ALT;
    }
    return 0;
}

static int decode_gsa(nmea_t* p, const fields_t* f) {
    const char *fx, *pd, *hd, *vd;
    int fxn = field(f, 2, &fx), pdn = field(f, 15, &pd), hdn = field(f, 16, &hd), vdn = field(f, 17, &vd);
    uint32_t fix = 0;
    int64_t pdop = 0, hdop = 0, vdop = 0;
    int rf, rp, rh, rv;
    CHECK(rf = parse_uint(fx, fxn, &fix));
    CHECK(rp = parse_fixed(pd, pdn, 2, &pdop));
    CHECK(rh = parse_fixed(hd, hdn, 2, &hdop));
    CHECK(rv = parse_fixed(vd, vdn, 2, &vdop));
    if ((rf == 1 && (fix < 1 || fix > 3)) || pdop < 0 || pdop > 65535 || hdop < 0 || hdop > 65535 ||
        vdop < 0 || vdop > 65535) {
        return -1;
    }
    if (rf == 1) {
        p->fix = (uint8_t)fix;
    }
    if (rp == 1 && rh == 1 && rv == 1) {
        p->pdop = (uint16_t)pdop;
        p->hdop = (uint16_t)hdop;
        p->vdop = (uint16_t)vdop;
        p->have |= NMEA_HAVE_DOP;
    }
    return 0;
}

static int decode_gsv(nmea_t* p, const fields_t* f, const char* talker) {
    const char* v;
    int vn = field(f, 3, &v);
    uint32_t view = 0;
    int rv;
    CHECK(rv = parse_uint(v, vn, &view));
    if (rv == 0 || view > 255) {                /* the count is mandatory */
        return -1;
    }
    /*
     * One count per constellation and per epoch (seq). NMEA 4.10 receivers
     * may send one GSV group per signal (L1, L5...) for the same talker: the
     * largest count is kept so that a satellite seen on two signals counts once.
     */
    int k = system_index(talker);
    if (p->gsv_seq[k] != p->seq) {
        p->gsv_seq[k] = p->seq;
        p->gsv_view[k] = 0;
    }
    if (view > p->gsv_view[k]) {
        p->gsv_view[k] = (uint8_t)view;
    }
    unsigned sum = 0;
    for (int i = 0; i < NMEA_GSV_SYSTEMS; i++) {
        if ((uint32_t)(p->seq - p->gsv_seq[i]) <= 1u) {     /* this epoch or the previous one */
            sum += p->gsv_view[i];
        }
    }
    p->sats_in_view = (uint8_t)(sum > 255 ? 255 : sum);
    p->have |= NMEA_HAVE_VIEW;
    return 0;
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

uint8_t nmea_checksum(const char* s, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= (uint8_t)s[i];
    }
    return c;
}

/** One complete line, starting with '$'. Returns 1 if it was checksum-valid. */
static int process_line(nmea_t* p) {
    const char* l = p->line;
    size_t n = p->len;
    const char* star = memchr(l, '*', n);
    if (star == NULL || (size_t)(star - l) + 3 > n) {
        p->stats.checksum_errors++;
        return 0;
    }
    int hi = hexval(star[1]), lo = hexval(star[2]);
    size_t body_len = (size_t)(star - l) - 1;
    if (hi < 0 || lo < 0 || nmea_checksum(l + 1, body_len) != (uint8_t)(hi << 4 | lo)) {
        p->stats.checksum_errors++;
        return 0;
    }
    p->stats.sentences++;

    fields_t f;
    split(l + 1, body_len, &f);
    const char* addr = f.s[0];
    if (f.n[0] != 5 || addr[0] == 'P') {        /* proprietary ($P...) or unusual address */
        p->stats.ignored++;
        return 1;
    }
    const char* type = addr + 2;
    int r;
    if (memcmp(type, "RMC", 3) == 0) {
        r = decode_rmc(p, &f);
    } else if (memcmp(type, "GGA", 3) == 0) {
        r = decode_gga(p, &f);
    } else if (memcmp(type, "GSA", 3) == 0) {
        r = decode_gsa(p, &f);
    } else if (memcmp(type, "GSV", 3) == 0) {
        r = decode_gsv(p, &f, addr);
    } else {
        p->stats.ignored++;
        return 1;
    }
    if (r < 0) {
        p->stats.format_errors++;
    } else {
        p->stats.decoded++;
    }
    return 1;
}

/* ------------------------------------------------------------------ public */

void nmea_init(nmea_t* p) {
    memset(p, 0, sizeof(*p));
    p->fix = NMEA_FIX_NONE;
    for (int i = 0; i < NMEA_GSV_SYSTEMS; i++) {
        p->gsv_seq[i] = (uint32_t)-10;          /* never counted until received */
    }
}

void nmea_reset_stats(nmea_t* p) {
    memset(&p->stats, 0, sizeof(p->stats));
}

int nmea_feed(nmea_t* p, const uint8_t* data, size_t len) {
    int done = 0;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '$') {                         /* start of sentence, even mid-line */
            p->in_line = 1;
            p->len = 0;
        }
        if (!p->in_line) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            p->line[p->len] = 0;
            done += process_line(p);
            p->in_line = 0;
            continue;
        }
        if (p->len >= NMEA_LINE_MAX) {
            p->stats.overflows++;
            p->in_line = 0;
            continue;
        }
        p->line[p->len++] = c;
    }
    return done;
}
