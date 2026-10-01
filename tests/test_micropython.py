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

# 6. command builder and reply catching
assert nmea.command("PAIR513") == b"$PAIR513*3D\r\n"
assert nmea.command(b"PAIR050,100") == b"$PAIR050,100*22\r\n"
p = nmea.Parser()
p.expect(b"$PAIR001,050,")
p.feed(b"$PAIR001,062,0*3F\r\n$PAIR001,050,0*3E\r\n")
assert p.response() == b"$PAIR001,050,0*3E"

# 7. Quectel $PAIR profile against a fake receiver
RMC = b"$GNRMC,114249.000,A,5042.385152,N,00422.317906,E,0.27,10.85,011026,,,A,V*39\r\n"


class FakeQuectel(io.IOBase):
    """Answers each $PAIRnnn command with $PAIR001,nnn,<r> for every r in replies[nnn]."""

    def __init__(self, replies=None):
        self.rx = bytearray()
        self.tx = []
        self.replies = replies or {}

    def write(self, b):
        b = bytes(b)
        self.tx.append(b)
        cid = b[5:8]                               # b"$PAIR050,100*22" -> b"050"
        for r in self.replies.get(cid, (b"1", b"0")):
            self.rx += nmea.command(b"PAIR001," + cid + b"," + r)
            self.rx += RMC                         # NMEA keeps flowing during the exchange
        return len(b)

    def readinto(self, buf):
        if not self.rx:
            return None                            # nothing ready: non-blocking
        n = min(len(buf), len(self.rx))
        buf[:n] = self.rx[:n]
        self.rx = self.rx[n:]
        return n


q = nmea.Parser(nmea.QUECTEL_PAIR)
u = FakeQuectel()
assert q.set_rate(u, 100) == 0
assert u.tx == [b"$PAIR050,100*22\r\n"]
assert q.valid == 1 and q.lat_e7 == 507064192            # decoded while waiting

u = FakeQuectel({b"062": (b"4",)})                         # parameter error
assert q.set_output(u, nmea.GSV, 10) == 4
assert u.tx == [nmea.command("PAIR062,3,10")], u.tx

u = FakeQuectel({b"050": ()})                              # silent receiver: 2 attempts, then None
assert q.set_rate(u, 100, 50) is None
assert len(u.tx) == 2

u = FakeQuectel()
assert q.configure(u, rate_ms=100, outputs=((nmea.GLL, 0), (nmea.VTG, 0)), save=True) == 0
assert u.tx == [nmea.command(c) for c in ("PAIR050,100", "PAIR062,1,0", "PAIR062,5,0",
                                          "PAIR382,1", "PAIR003", "PAIR513", "PAIR002")], u.tx

u = FakeQuectel({b"062": (b"3",)})                         # stops at the first failure
assert q.configure(u, rate_ms=200, outputs=((nmea.GSV, 5), (nmea.GLL, 0))) == 3
assert len(u.tx) == 2

for bad in (lambda: nmea.Parser().set_rate(FakeQuectel(), 100),     # no profile
            lambda: q.set_rate(FakeQuectel(), 50),                 # out of range
            lambda: q.set_output(FakeQuectel(), nmea.GSV, 21),
            lambda: nmea.Parser(99)):                              # unknown profile
    try:
        bad()
        raise AssertionError("ValueError expected")
    except ValueError:
        pass

print("OK")
