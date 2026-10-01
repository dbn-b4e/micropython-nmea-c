#!/usr/bin/env python3
"""
Compares the C parser with pynmea2 (reference decoder) on:
  - real receiver output (tests/data/*.nmea),
  - synthetic edge cases (hemispheres, NMEA 2.x, bad / missing checksum,
    overlong line, garbage, proprietary sentences, malformed fields,
    multi-constellation satellites-in-view, ageing of a constellation).

For every line the expected parser state is rebuilt from the fields pynmea2
decodes, following the rules documented in README.md, and compared field by
field with the state printed by tests/nmea_dump (built from src/nmea_core.c).
A second run feeds the same stream in irregular chunks and must end in the
same state.

Requires: a C compiler (cc) and pynmea2 (pip install pynmea2).
Usage:    python3 tests/test_vs_pynmea2.py
"""

import calendar
import datetime
import functools
import subprocess
import sys
import tempfile
from pathlib import Path

import pynmea2

ROOT = Path(__file__).resolve().parents[1]
LINE_MAX = 120

FIELDS = ("seq valid lat_e7 lon_e7 alt_cm geoid_cm speed_cms course_e2 time_ms date utc fix quality mode "
          "sats_used sats_in_view hdop pdop vdop have sentences decoded ignored checksum_errors "
          "format_errors overflows").split()
# Values derived from a decimal string through float in pynmea2: allow 1 unit of the last digit.
TOLERANT = {"lat_e7", "lon_e7", "alt_cm", "geoid_cm", "speed_cms", "course_e2", "hdop", "pdop", "vdop"}

HAVE = dict(TIME=1, DATE=2, POS=4, ALT=8, SPEED=16, COURSE=32, DOP=64, VIEW=128)
SYSTEMS = ["GP", "GL", "GA", "GB", "GQ", "GI", "GN"]


def cs(body):
    return "%02X" % functools.reduce(lambda a, c: a ^ ord(c), body, 0)


def s(body):
    """Sentence with a correct checksum."""
    return f"${body}*{cs(body)}"


SYNTHETIC = [
    # southern / western hemisphere, 3-digit longitude, 1999 date, differential mode
    s("GPRMC,235959.999,A,3354.123456,S,15112.654321,W,12.5,359.99,311299,,,D"),
    s("GPGGA,235959.999,3354.123456,S,15112.654321,W,2,12,0.9,-12.34,M,-1.5,M,,"),
    s("GNGSA,A,3,01,02,03,,,,,,,,,,1.50,0.90,1.20,1"),
    # multi-constellation satellites in view
    s("GLGSV,2,1,07,65,40,083,46,66,10,120,40,72,30,300,35,80,05,010,20,1"),
    s("GLGSV,2,2,07,81,15,045,30,82,60,200,41,83,22,250,,1"),
    s("BDGSV,1,1,03,01,40,083,46,02,10,120,40,03,30,300,35"),
    s("GPGSV,1,1,04,10,40,083,46,12,10,120,40,15,30,300,35,18,05,010,20,1"),
    # NMEA 2.0 RMC (no mode field), time without fraction, course 360.0
    s("GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E"),
    # ageing: GL and BD stop reporting; after two RMC epochs only GP remains
    s("GPGSV,1,1,04,10,40,083,46,12,10,120,40,15,30,300,35,18,05,010,20,1"),
    s("GPRMC,081837,A,3751.66,S,14507.36,E,000.0,,130998,,"),
    s("GPGSV,1,1,04,10,40,083,46,12,10,120,40,15,30,300,35,18,05,010,20,1"),
    # leap second, invalid status: position must not change
    s("GNRMC,235960.000,V,1111.111111,N,02222.222222,E,0.00,,311216,,,N,V"),
    # lowercase checksum digits
    (lambda x: x[:-2] + x[-2:].lower())(s("GNGGA,000001.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,")),
    # bad checksum
    "$GNGGA,000001.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00",
    # missing checksum
    "$GNGGA,000001.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
    # overlong line
    s("GPTXT," + "X" * (LINE_MAX + 10)),
    # ignored but valid: GLL, VTG, proprietary
    s("GNGLL,4807.038,N,01131.000,E,000002.000,A,A"),
    s("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K,A"),
    s("PAIR001,062,0"),
    # malformed field with a valid checksum (letter in the time)
    s("GPGGA,12a456,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
]
# Expected outcome for lines the model cannot derive from pynmea2 alone.
OUTCOME = {SYNTHETIC[-1]: "format", SYNTHETIC[15]: "overflow"}
GARBAGE = "\x00\xff\x7fnoise$GPRMC,1234"           # partial sentence then garbage, cut by the next '$'


class Model:
    """Expected parser state, built from pynmea2's decoding."""

    def __init__(self):
        self.st = {k: 0 for k in FIELDS}
        self.st["fix"] = 1
        self.view = {}                             # system index -> (seq, count)

    def have(self, name):
        self.st["have"] |= HAVE[name]

    def line(self, raw):
        st = self.st
        if OUTCOME.get(raw) == "overflow" or len(raw) > LINE_MAX:
            st["overflows"] += 1
            return
        try:
            m = pynmea2.parse(raw, check=True)
        except pynmea2.ChecksumError:
            st["checksum_errors"] += 1
            return
        st["sentences"] += 1
        if OUTCOME.get(raw) == "format":
            st["format_errors"] += 1
            return
        if isinstance(m, pynmea2.ProprietarySentence) or m.sentence_type not in ("RMC", "GGA", "GSA", "GSV"):
            st["ignored"] += 1
            return
        getattr(self, m.sentence_type.lower())(m)
        st["decoded"] += 1

    @staticmethod
    def ms(t):
        if isinstance(t, str):                     # pynmea2 leaves a leap second (ss = 60) unparsed
            return (int(t[0:2]) * 3600 + int(t[2:4]) * 60) * 1000 + round(float(t[4:]) * 1000)
        return ((t.hour * 60 + t.minute) * 60 + t.second) * 1000 + t.microsecond // 1000

    def rmc(self, m):
        st = self.st
        if m.timestamp:
            st["time_ms"] = self.ms(m.timestamp)
            self.have("TIME")
        if m.datestamp:
            d = m.datestamp
            st["date"] = d.year * 10000 + d.month * 100 + d.day
            self.have("DATE")
        if m.timestamp and m.datestamp:
            st["utc"] = calendar.timegm(datetime.datetime.combine(m.datestamp, datetime.time()).timetuple()) \
                + st["time_ms"] // 1000
        st["valid"] = 1 if m.status == "A" else 0
        if st["valid"] and m.lat and m.lon:
            st["lat_e7"], st["lon_e7"] = round(m.latitude * 1e7), round(m.longitude * 1e7)
            self.have("POS")
        if m.spd_over_grnd not in (None, ""):
            st["speed_cms"] = round(float(m.spd_over_grnd) * 185200 / 3600)
            self.have("SPEED")
        if m.true_course not in (None, ""):
            st["course_e2"] = round(float(m.true_course) * 100) % 36000
            self.have("COURSE")
        mi = getattr(m, "mode_indicator", None)
        st["mode"] = ord(mi[0]) if mi else 0
        st["seq"] += 1

    def gga(self, m):
        st = self.st
        if m.timestamp:
            st["time_ms"] = self.ms(m.timestamp)
            self.have("TIME")
        st["quality"] = int(m.gps_qual) if m.gps_qual not in (None, "") else 0
        if m.num_sats:
            st["sats_used"] = int(m.num_sats)
        if st["quality"] > 0 and m.lat and m.lon:
            st["lat_e7"], st["lon_e7"] = round(m.latitude * 1e7), round(m.longitude * 1e7)
            self.have("POS")
        if m.horizontal_dil:
            st["hdop"] = round(float(m.horizontal_dil) * 100)
            self.have("DOP")
        if st["quality"] > 0 and m.altitude is not None:
            st["alt_cm"] = round(float(m.altitude) * 100)
            st["geoid_cm"] = round(float(m.geo_sep) * 100) if m.geo_sep else 0
            self.have("ALT")

    def gsa(self, m):
        st = self.st
        if m.mode_fix_type:
            st["fix"] = int(m.mode_fix_type)
        if m.pdop and m.hdop and m.vdop:
            st["pdop"], st["hdop"], st["vdop"] = (round(float(x) * 100) for x in (m.pdop, m.hdop, m.vdop))
            self.have("DOP")

    def gsv(self, m):
        st = self.st
        t = m.talker
        k = 3 if t == "BD" else (SYSTEMS.index(t) if t in SYSTEMS else 7)
        seq, cnt = self.view.get(k, (None, 0))
        if seq != st["seq"]:
            seq, cnt = st["seq"], 0
        self.view[k] = (seq, max(cnt, int(m.num_sv_in_view)))
        st["sats_in_view"] = min(255, sum(c for (q, c) in self.view.values() if st["seq"] - q <= 1))
        self.have("VIEW")


def main():
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / "nmea_dump"
        subprocess.run(["cc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "src"),
                        str(ROOT / "tests" / "nmea_dump.c"), str(ROOT / "src" / "nmea_core.c"), "-o", str(exe)],
                       check=True)
        real = []
        for f in sorted((ROOT / "tests" / "data").glob("*.nmea")):
            real += [l.strip() for l in f.read_text().splitlines() if l.strip()]
        lines = real + SYNTHETIC
        stream = "\r\n".join(lines[:len(real)]) + "\r\n" + GARBAGE + "\r\n" + "\r\n".join(SYNTHETIC) + "\r\n"

        out = subprocess.run([str(exe), "lines"], input=stream.encode("latin-1"), capture_output=True,
                             check=True).stdout.decode().splitlines()
        # one dump per '\n': real lines, the garbage line, then synthetic lines
        expected_lines = lines[:len(real)] + [GARBAGE] + SYNTHETIC
        assert len(out) == len(expected_lines), (len(out), len(expected_lines))

        model = Model()
        errors = 0
        for raw, got in zip(expected_lines, out):
            if raw is GARBAGE:
                model.st["checksum_errors"] += 1   # "$GPRMC,1234" cut by CR LF: no checksum
            else:
                model.line(raw)
            got = dict(zip(FIELDS, map(int, got.split(","))))
            for k in FIELDS:
                e, g = model.st[k], got[k]
                if e != g and not (k in TOLERANT and abs(e - g) <= 1):
                    errors += 1
                    print(f"DIFF {k}: expected {e}, got {g}\n     {raw!r}")
        final = out[-1]
        chunked = subprocess.run([str(exe), "chunks"], input=stream.encode("latin-1"), capture_output=True,
                                 check=True).stdout.decode().strip()
        if chunked != final:
            errors += 1
            print(f"DIFF chunked feeding:\n  byte by byte {final}\n  chunks       {chunked}")

    st = model.st
    print(f"{len(expected_lines)} lines ({len(real)} real, {len(SYNTHETIC)} synthetic, 1 garbage); "
          f"sentences {st['sentences']}, decoded {st['decoded']}, ignored {st['ignored']}, "
          f"checksum errors {st['checksum_errors']}, format errors {st['format_errors']}, overflows {st['overflows']}")
    if errors:
        print(f"FAILED: {errors} difference(s)")
        sys.exit(1)
    print("OK: C parser matches pynmea2 (byte by byte and chunked)")


if __name__ == "__main__":
    main()
