/**
 * @file    modnmea.c
 * @brief   MicroPython binding of nmea_core: module `nmea`.
 *
 *   p = nmea.Parser()
 *   p.poll(uart)        read every byte the stream has ready (uart needs timeout=0), parse;
 *                       returns the number of checksum-valid sentences completed
 *   p.feed(buf)         same, from a bytes-like object
 *   p.read(arr)         copy every value into arr, an array('i') of nmea.NFIELDS items,
 *                       indexed by nmea.LAT_E7, nmea.LON_E7... - allocation-free
 *   p.lat_e7, p.utc...  the same values as attributes (large ints may allocate)
 *   p.stats()           (sentences, decoded, ignored, checksum_errors, format_errors, overflows)
 *   p.reset()           clear values, state and counters;  p.reset_stats(): counters only
 *
 * Copyright (c) 2026 B4E SRL - David Baldwin
 * PolyForm Noncommercial License 1.0.0 - see LICENSE
 */

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/binary.h"

#include "src/nmea_core.h"

/* Bytes taken from the stream per poll() call, so that one call stays short. */
#ifndef NMEA_POLL_MAX
#define NMEA_POLL_MAX 2048
#endif

/* Index of each value in read() and name of the matching attribute. */
enum {
    F_LAT_E7, F_LON_E7, F_ALT_CM, F_GEOID_CM, F_SPEED_CMS, F_COURSE_E2, F_TIME_MS, F_DATE, F_UTC,
    F_VALID, F_FIX, F_QUALITY, F_MODE, F_SATS_USED, F_SATS_IN_VIEW, F_HDOP, F_PDOP, F_VDOP, F_HAVE, F_SEQ,
    NFIELDS
};

static const qstr field_names[NFIELDS] = {
    MP_QSTR_lat_e7, MP_QSTR_lon_e7, MP_QSTR_alt_cm, MP_QSTR_geoid_cm, MP_QSTR_speed_cms, MP_QSTR_course_e2,
    MP_QSTR_time_ms, MP_QSTR_date, MP_QSTR_utc, MP_QSTR_valid, MP_QSTR_fix, MP_QSTR_quality, MP_QSTR_mode,
    MP_QSTR_sats_used, MP_QSTR_sats_in_view, MP_QSTR_hdop, MP_QSTR_pdop, MP_QSTR_vdop, MP_QSTR_have, MP_QSTR_seq,
};

typedef struct {
    mp_obj_base_t base;
    nmea_t p;
} nmea_parser_obj_t;

/** Value i as a 32-bit pattern (utc and seq are unsigned; see README, "Year 2038"). */
static int32_t field_value(const nmea_t* p, int i) {
    switch (i) {
        case F_LAT_E7: return p->lat_e7;
        case F_LON_E7: return p->lon_e7;
        case F_ALT_CM: return p->alt_cm;
        case F_GEOID_CM: return p->geoid_cm;
        case F_SPEED_CMS: return (int32_t)p->speed_cms;
        case F_COURSE_E2: return p->course_e2;
        case F_TIME_MS: return (int32_t)p->time_ms;
        case F_DATE: return (int32_t)p->date;
        case F_UTC: return (int32_t)p->utc;
        case F_VALID: return p->valid;
        case F_FIX: return p->fix;
        case F_QUALITY: return p->quality;
        case F_MODE: return (uint8_t)p->mode;
        case F_SATS_USED: return p->sats_used;
        case F_SATS_IN_VIEW: return p->sats_in_view;
        case F_HDOP: return p->hdop;
        case F_PDOP: return p->pdop;
        case F_VDOP: return p->vdop;
        case F_HAVE: return p->have;
        default: return (int32_t)p->seq;
    }
}

static mp_obj_t parser_make_new(const mp_obj_type_t* type, size_t n_args, size_t n_kw, const mp_obj_t* args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    nmea_parser_obj_t* self = mp_obj_malloc(nmea_parser_obj_t, type);
    nmea_init(&self->p);
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t parser_feed(mp_obj_t self_in, mp_obj_t buf_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t b;
    mp_get_buffer_raise(buf_in, &b, MP_BUFFER_READ);
    return MP_OBJ_NEW_SMALL_INT(nmea_feed(&self->p, b.buf, b.len));
}
static MP_DEFINE_CONST_FUN_OBJ_2(parser_feed_obj, parser_feed);

static mp_obj_t parser_poll(mp_obj_t self_in, mp_obj_t stream_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    const mp_stream_p_t* stream = mp_get_stream_raise(stream_in, MP_STREAM_OP_READ);
    uint8_t buf[64];
    int done = 0;
    for (size_t total = 0; total < NMEA_POLL_MAX;) {
        int err;
        mp_uint_t n = stream->read(stream_in, buf, sizeof(buf), &err);
        if (n == MP_STREAM_ERROR) {
            if (mp_is_nonblocking_error(err)) {
                break;                          /* nothing more ready */
            }
            mp_raise_OSError(err);
        }
        if (n == 0) {
            break;
        }
        done += nmea_feed(&self->p, buf, n);
        total += n;
        if (n < sizeof(buf)) {
            break;                              /* the stream is drained */
        }
    }
    return MP_OBJ_NEW_SMALL_INT(done);
}
static MP_DEFINE_CONST_FUN_OBJ_2(parser_poll_obj, parser_poll);

static mp_obj_t parser_read(mp_obj_t self_in, mp_obj_t arr_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t b;
    mp_get_buffer_raise(arr_in, &b, MP_BUFFER_WRITE);
    if ((b.typecode != 'i' && b.typecode != 'l') || mp_binary_get_size('@', b.typecode, NULL) != 4 ||
        b.len < NFIELDS * 4) {
        mp_raise_ValueError(MP_ERROR_TEXT("need array('i') of NFIELDS items"));
    }
    int32_t* out = b.buf;
    for (int i = 0; i < NFIELDS; i++) {
        out[i] = field_value(&self->p, i);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(parser_read_obj, parser_read);

static mp_obj_t parser_stats(mp_obj_t self_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    const nmea_stats_t* s = &self->p.stats;
    mp_obj_t t[6] = {
        mp_obj_new_int_from_uint(s->sentences), mp_obj_new_int_from_uint(s->decoded),
        mp_obj_new_int_from_uint(s->ignored), mp_obj_new_int_from_uint(s->checksum_errors),
        mp_obj_new_int_from_uint(s->format_errors), mp_obj_new_int_from_uint(s->overflows),
    };
    return mp_obj_new_tuple(6, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(parser_stats_obj, parser_stats);

static mp_obj_t parser_reset(mp_obj_t self_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    nmea_init(&self->p);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(parser_reset_obj, parser_reset);

static mp_obj_t parser_reset_stats(mp_obj_t self_in) {
    nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
    nmea_reset_stats(&self->p);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(parser_reset_stats_obj, parser_reset_stats);

static const mp_rom_map_elem_t parser_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_feed), MP_ROM_PTR(&parser_feed_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll), MP_ROM_PTR(&parser_poll_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&parser_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&parser_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&parser_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_stats), MP_ROM_PTR(&parser_reset_stats_obj) },
};
static MP_DEFINE_CONST_DICT(parser_locals, parser_locals_table);

/** Attribute access: values first, then methods. Values are read-only. */
static void parser_attr(mp_obj_t self_in, qstr attr, mp_obj_t* dest) {
    if (dest[0] == MP_OBJ_NULL) {
        nmea_parser_obj_t* self = MP_OBJ_TO_PTR(self_in);
        for (int i = 0; i < NFIELDS; i++) {
            if (field_names[i] == attr) {
                int32_t v = field_value(&self->p, i);
                dest[0] = (i == F_UTC || i == F_SEQ) ? mp_obj_new_int_from_uint((uint32_t)v) : mp_obj_new_int(v);
                return;
            }
        }
        dest[1] = MP_OBJ_SENTINEL;              /* continue lookup in locals_dict */
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    nmea_parser_type,
    MP_QSTR_Parser,
    MP_TYPE_FLAG_NONE,
    make_new, parser_make_new,
    attr, parser_attr,
    locals_dict, &parser_locals
    );

static const mp_rom_map_elem_t nmea_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_nmea) },
    { MP_ROM_QSTR(MP_QSTR_Parser), MP_ROM_PTR(&nmea_parser_type) },
    { MP_ROM_QSTR(MP_QSTR_NFIELDS), MP_ROM_INT(NFIELDS) },
    { MP_ROM_QSTR(MP_QSTR_LAT_E7), MP_ROM_INT(F_LAT_E7) },
    { MP_ROM_QSTR(MP_QSTR_LON_E7), MP_ROM_INT(F_LON_E7) },
    { MP_ROM_QSTR(MP_QSTR_ALT_CM), MP_ROM_INT(F_ALT_CM) },
    { MP_ROM_QSTR(MP_QSTR_GEOID_CM), MP_ROM_INT(F_GEOID_CM) },
    { MP_ROM_QSTR(MP_QSTR_SPEED_CMS), MP_ROM_INT(F_SPEED_CMS) },
    { MP_ROM_QSTR(MP_QSTR_COURSE_E2), MP_ROM_INT(F_COURSE_E2) },
    { MP_ROM_QSTR(MP_QSTR_TIME_MS), MP_ROM_INT(F_TIME_MS) },
    { MP_ROM_QSTR(MP_QSTR_DATE), MP_ROM_INT(F_DATE) },
    { MP_ROM_QSTR(MP_QSTR_UTC), MP_ROM_INT(F_UTC) },
    { MP_ROM_QSTR(MP_QSTR_VALID), MP_ROM_INT(F_VALID) },
    { MP_ROM_QSTR(MP_QSTR_FIX), MP_ROM_INT(F_FIX) },
    { MP_ROM_QSTR(MP_QSTR_QUALITY), MP_ROM_INT(F_QUALITY) },
    { MP_ROM_QSTR(MP_QSTR_MODE), MP_ROM_INT(F_MODE) },
    { MP_ROM_QSTR(MP_QSTR_SATS_USED), MP_ROM_INT(F_SATS_USED) },
    { MP_ROM_QSTR(MP_QSTR_SATS_IN_VIEW), MP_ROM_INT(F_SATS_IN_VIEW) },
    { MP_ROM_QSTR(MP_QSTR_HDOP), MP_ROM_INT(F_HDOP) },
    { MP_ROM_QSTR(MP_QSTR_PDOP), MP_ROM_INT(F_PDOP) },
    { MP_ROM_QSTR(MP_QSTR_VDOP), MP_ROM_INT(F_VDOP) },
    { MP_ROM_QSTR(MP_QSTR_HAVE), MP_ROM_INT(F_HAVE) },
    { MP_ROM_QSTR(MP_QSTR_SEQ), MP_ROM_INT(F_SEQ) },
    { MP_ROM_QSTR(MP_QSTR_FIX_NONE), MP_ROM_INT(NMEA_FIX_NONE) },
    { MP_ROM_QSTR(MP_QSTR_FIX_2D), MP_ROM_INT(NMEA_FIX_2D) },
    { MP_ROM_QSTR(MP_QSTR_FIX_3D), MP_ROM_INT(NMEA_FIX_3D) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_TIME), MP_ROM_INT(NMEA_HAVE_TIME) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_DATE), MP_ROM_INT(NMEA_HAVE_DATE) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_POS), MP_ROM_INT(NMEA_HAVE_POS) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_ALT), MP_ROM_INT(NMEA_HAVE_ALT) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_SPEED), MP_ROM_INT(NMEA_HAVE_SPEED) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_COURSE), MP_ROM_INT(NMEA_HAVE_COURSE) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_DOP), MP_ROM_INT(NMEA_HAVE_DOP) },
    { MP_ROM_QSTR(MP_QSTR_HAVE_VIEW), MP_ROM_INT(NMEA_HAVE_VIEW) },
};
static MP_DEFINE_CONST_DICT(nmea_module_globals, nmea_module_globals_table);

const mp_obj_module_t nmea_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&nmea_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_nmea, nmea_user_cmodule);
