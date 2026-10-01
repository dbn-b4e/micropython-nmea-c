# Test of the `nmea` module, run by a MicroPython build that includes it
# (unix port in CI, or a board):  micropython tests/test_micropython.py
#
# Expected values come from tests/data/lc86g_20261001.nmea, checked against
# pynmea2 by tests/test_vs_pynmea2.py.

import io
import nmea
from array import array

DATA = "tests/data/lc86g_20261001.nmea"
with open(DATA, "rb") as f:
    raw = f.read()

# 1. feed() in one go
p = nmea.Parser()
n = p.feed(raw)
assert n == 42, n
assert p.valid == 1 and p.fix == nmea.FIX_NONE and p.quality == 1
assert p.lat_e7 == 507064192 and p.lon_e7 == 43719651, (p.lat_e7, p.lon_e7)
assert p.alt_cm == 11925 and p.geoid_cm == 4736
assert p.speed_cms == 14 and p.course_e2 == 1085
assert p.time_ms == 42169000 and p.date == 20261001 and p.utc == 1790854969
assert p.sats_used == 7 and p.hdop == 168
assert p.mode == ord("A") and p.seq == 4
assert p.have & nmea.HAVE_POS and p.have & nmea.HAVE_VIEW
assert p.sats_in_view == 12, p.sats_in_view
assert p.stats() == (42, 33, 9, 0, 0, 0), p.stats()

# 2. read() into array('i') - same values, no allocation
a = array("i", [0] * nmea.NFIELDS)
p.read(a)
assert a[nmea.LAT_E7] == p.lat_e7 and a[nmea.UTC] == p.utc and a[nmea.SEQ] == p.seq

# 3. poll() from a stream, byte stream identical result
q = nmea.Parser()
s = io.BytesIO(raw)
total = 0
while True:
    k = q.poll(s)
    total += k
    if s.tell() >= len(raw):
        break
b = array("i", [0] * nmea.NFIELDS)
q.read(b)
assert total == 42 and b == a, (total, list(b), list(a))

# 4. feed() byte by byte gives the same state
r = nmea.Parser()
for i in range(len(raw)):
    r.feed(raw[i:i + 1])
c = array("i", [0] * nmea.NFIELDS)
r.read(c)
assert c == a

# 5. errors and reset
p.reset_stats()
p.feed(b"$GPGGA,1*00\r\n$GPGGA,no checksum\r\n")
assert p.stats()[3] == 2
p.reset()
assert p.seq == 0 and p.have == 0 and p.stats() == (0, 0, 0, 0, 0, 0)
try:
    p.read(array("h", [0] * nmea.NFIELDS))
    raise AssertionError("read() must reject array('h')")
except ValueError:
    pass
try:
    p.lat_e7 = 1
    raise AssertionError("attributes are read-only")
except AttributeError:
    pass

print("OK")
