# micropython-nmea-c

Allocation-free, integer-only NMEA 0183 parser written in C, with a MicroPython
binding (module `nmea`), and receiver configuration through vendor profiles
(Quectel `$PAIR` included). The core (`src/nmea_core.c`, `src/nmea_vendor.c`)
is plain C99 with no dependency and can be used in any C/C++ firmware.

**License: PolyForm Noncommercial 1.0.0 — non-commercial use only. Personal,
educational and research use permitted. See [LICENSE](LICENSE).**

Required Notice: Copyright (c) 2026 B4E SRL - David Baldwin

## Why

Pure-Python NMEA parsers work, but on a microcontroller they cost:

- **one Python call per byte** (character-by-character state machines);
- **heap allocations**: every `float` is a heap object in MicroPython, so each
  decoded coordinate triggers allocations and, sooner or later, garbage
  collection pauses;
- **precision**: MicroPython ports built with single-precision floats lose
  about 0.2–0.5 m on a latitude around 50°.

This module parses in C, keeps every value as a **scaled integer**, never
allocates after `Parser()` is created, and lets you copy all values into an
`array('i')` without allocating either.

## What is decoded

Only checksum-valid sentences are used. Talker IDs are free: `GP`, `GL`, `GA`,
`GB`/`BD`, `GQ`, `GI`, `GN`...

| Sentence | Values |
|---|---|
| RMC | time, date, Unix time, status (valid), latitude, longitude, speed, course, mode indicator |
| GGA | time, fix quality, satellites used, HDOP, altitude (MSL), geoid separation, position when quality > 0 |
| GSA | fix type (none / 2D / 3D), PDOP, HDOP, VDOP |
| GSV | **header only**: number of satellites in view, summed over constellations |

Everything else (GLL, VTG, TXT, proprietary `$P...`) is checksum-verified,
counted as *ignored*, and skipped. GLL and VTG duplicate RMC.

| Value | Unit / meaning |
|---|---|
| `lat_e7`, `lon_e7` | 1e-7 degree, north / east positive (WGS84) |
| `alt_cm`, `geoid_cm` | centimetres (altitude above MSL, geoid separation) |
| `speed_cms` | cm/s (RMC knots × 1852 / 3600, rounded) |
| `course_e2` | 0.01 degree, 0..35999 |
| `time_ms` | milliseconds since 00:00 UTC |
| `date` | `YYYYMMDD` (two-digit years 80–99 → 19xx, else 20xx) |
| `utc` | Unix time, seconds |
| `valid` | 1 if the last RMC had status `A` |
| `fix` | `FIX_NONE` (1), `FIX_2D` (2), `FIX_3D` (3) — from GSA |
| `quality` | GGA fix quality (0 none, 1 GPS, 2 DGPS/SBAS, 4 RTK, 5 float RTK, 6 dead reckoning…) |
| `mode` | RMC mode indicator as a character code (`ord('A')`…), 0 if absent |
| `sats_used` | GGA satellites used |
| `sats_in_view` | GSV satellites in view, all constellations |
| `hdop`, `pdop`, `vdop` | × 100 (1.68 → 168), 0 = not received |
| `have` | `HAVE_*` bits: which values have been received at least once |
| `seq` | incremented on every decoded RMC (one per navigation epoch) |

## Using it in MicroPython

```python
import nmea
from machine import UART, Timer
from array import array

uart = UART(4, 115200, timeout=0, rxbuf=2048)   # timeout=0: poll() must not block
gps = nmea.Parser()
values = array("i", [0] * nmea.NFIELDS)

def tick(_):
    if gps.poll(uart):                # parse every byte available; returns sentences completed
        gps.read(values)              # copy all values, no allocation
        if values[nmea.VALID]:
            lat, lon = values[nmea.LAT_E7], values[nmea.LON_E7]

Timer(mode=Timer.PERIODIC, period=50, callback=tick)
```

### Configuring the receiver

NMEA defines no configuration commands: every vendor has its own. Pass the
receiver's profile to `Parser()`, then configure it at start-up:

```python
gps = nmea.Parser(nmea.QUECTEL_PAIR)        # Quectel LC26G / LC76G / LC86G
r = gps.configure(uart, rate_ms=100,         # 10 Hz
                  outputs=((nmea.GLL, 0), (nmea.VTG, 0), (nmea.GSV, 10)))
# r: 0 = every step accepted; > 0 = receiver error code; None = no acknowledgement
```

Each request is sent, then the call waits for the receiver's acknowledgement
(by default up to 1.5 s, one retry), **while still decoding the NMEA stream**;
interrupts and scheduled callbacks keep running (`mp_hal_delay_ms`). Configure
at start-up, not from a time-critical callback.

| Call | Quectel `$PAIR` command |
|---|---|
| `p.set_rate(uart, ms[, timeout_ms])` | `PAIR050` — position fix interval, 100–1000 ms |
| `p.set_output(uart, nmea.GSV, n[, timeout_ms])` | `PAIR062` — sentence once every `n` fixes (0 = off, ≤ 20) |
| `p.save(uart[, timeout_ms])` | `PAIR382,1` → `PAIR003` → `PAIR513` → `PAIR002` (save to flash, required sequence above 1 Hz) |
| `p.configure(uart, rate_ms=None, outputs=(), save=False, timeout_ms=1500)` | the above in order, stops at the first failure |

Quectel acknowledgement codes (`$PAIR001,<cmd>,<result>`): 0 accepted,
1 processing (waited for), 2 failed, 3 not supported, 4 parameter error,
5 busy. Source: Quectel *LC26G&LC26G-T&LC76G&LC86G Series GNSS Protocol
Specification* V1.4. Note from that document: above 1 Hz the receiver outputs
only RMC, GGA and GNS at the fix rate, GSA and GSV at 1 Hz, and no GLL / VTG.

Generic helpers, for any vendor or command not covered by a profile:

```python
uart.write(nmea.command("PAIR051"))   # b"$PAIR051*3E\r\n" (checksum added)
gps.expect(b"$PAIR051,")              # keep the last line starting with this prefix
...                                   # gps.poll(uart) as usual
gps.response()                        # b"$PAIR051,1000*13" (fix interval) or None
```

Sentence constants for `set_output()`: `GGA`, `GLL`, `GSA`, `GSV`, `RMC`,
`VTG`, `ZDA`, `GRS`, `GST`, `GNS`. Profiles: `VENDOR_NONE` (default, parse
only), `QUECTEL_PAIR`.

#### Adding a vendor profile

Profiles are C tables (`src/nmea_vendor.h`): a vendor provides functions that
build the command(s) for `set_rate`, `set_output` and `save`, and one that
reads its acknowledgement line. To add one:

1. write `src/nmea_vendor_<name>.c` implementing `nmea_vendor_t`, from the
   vendor's documentation (commands *and* acknowledgement format);
2. add its id to `nmea_vendor_id_t`, its entry to `nmea_vendors[]`, and a
   constant to the module table in `modnmea.c`;
3. add the examples printed in the vendor's documentation to `tests/test_core.c`.

Only sentence-based (ASCII) command sets fit this model; binary protocols such
as u-blox UBX need a different transport. Contributions welcome (MediaTek
`$PMTK`, SiRF, Unicore...).

### Values as attributes

Attributes give the same values for convenience (`gps.lat_e7`, `gps.utc`,
`gps.sats_in_view`…). Reading an attribute whose value does not fit a
MicroPython small integer (about ±2^30, for example `utc` or a longitude beyond
±107°) allocates an integer object; `read()` never allocates.

### API

| Call | Description |
|---|---|
| `nmea.Parser([vendor])` | new parser (about 400 bytes, allocated once); `vendor` = `nmea.QUECTEL_PAIR` to allow configuration |
| `p.poll(stream)` | reads every byte the stream has ready (up to 2048 per call, `NMEA_POLL_MAX`), parses them; returns the number of checksum-valid sentences completed. The stream must be non-blocking (`UART(..., timeout=0)`) |
| `p.feed(buf)` | same, from a bytes-like object, in any chunk size |
| `p.read(arr)` | copies every value into `arr`, an `array('i')` of `nmea.NFIELDS` items indexed by `nmea.LAT_E7`, `nmea.LON_E7`… |
| `p.<value>` | read-only attributes, see the table above |
| `p.stats()` | `(sentences, decoded, ignored, checksum_errors, format_errors, overflows)` |
| `p.reset()` | clears values, state and counters |
| `p.reset_stats()` | clears the counters only |
| `p.set_rate`, `p.set_output`, `p.save`, `p.configure` | receiver configuration, see above |
| `p.expect(prefix)`, `p.response()` | catch a reply line by its prefix |
| `nmea.command(body)` | `b"$" body "*hh\r\n"` |

Constants: `NFIELDS`, the field indexes (`LAT_E7` … `SEQ`), `FIX_NONE`,
`FIX_2D`, `FIX_3D`, `HAVE_TIME`, `HAVE_DATE`, `HAVE_POS`, `HAVE_ALT`,
`HAVE_SPEED`, `HAVE_COURSE`, `HAVE_DOP`, `HAVE_VIEW`.

## Building it into MicroPython

The module is a standard
[MicroPython user C module](https://docs.micropython.org/en/latest/develop/cmodules.html).

- **make-based ports** (stm32, unix…): `USER_C_MODULES` is the directory
  that *contains* this repository, for example
  ```
  make -C ports/stm32 BOARD=... USER_C_MODULES=/path/to/modules
  ```
  with this repository cloned (or added as a git submodule) as
  `/path/to/modules/micropython-nmea-c`.
- **CMake-based ports** (esp32, rp2): point `USER_C_MODULES` to
  `micropython-nmea-c/micropython.cmake`, or include it from your own
  `micropython.cmake`.

Flash cost on Cortex-M4 (GCC 14.3, `-Os`): 3.7 kB for the parser and 0.4 kB for the vendor
table (no static RAM); the binding and its names come on top (5.4 kB in total for version 0.1).

## Using the C core without MicroPython

```c
#include "nmea_core.h"

static nmea_t gps;
nmea_init(&gps);
...
nmea_feed(&gps, rx_bytes, rx_len);     /* any chunk size, e.g. from a UART ISR ring buffer */
if (gps.valid && (gps.have & NMEA_HAVE_POS)) { use(gps.lat_e7, gps.lon_e7); }
```

`nmea_t` is a plain struct (400 bytes); `nmea_feed()` is reentrant per
instance and does not lock: call it from one context only.

## Rules and limits

- **Checksum required.** A sentence without `*hh` or with a wrong checksum is
  counted in `checksum_errors` and ignored. Hex digits may be upper or lower case.
- **All or nothing per sentence.** If a decoded field is malformed, the whole
  sentence is rejected (`format_errors`) and no value changes.
- **Position** is updated by RMC only when its status is `A`, and by GGA only
  when its quality is above 0. Last known values are kept otherwise; check
  `valid` / `quality` / `fix` before using them.
- **Satellites in view**: the GSV count of each constellation is kept for the
  current and the previous epoch (an epoch ends with RMC), then dropped, so a
  constellation that stops reporting disappears from the total. Receivers that
  send one GSV group per signal (L1, L5…) for the same constellation: the
  largest count is kept, so a satellite seen on two signals counts once.
- **HDOP** comes from GSA when PDOP, HDOP and VDOP are all present, otherwise
  from GGA.
- **Course** is often empty while stationary: the last value is kept and
  `HAVE_COURSE` stays set.
- **Hemisphere letters** must be upper case (`N`, `S`, `E`, `W`).
- **Leap second** (`hhmm60`) is accepted.
- **Decimals beyond the stored resolution are truncated** (coordinates are
  rounded to 1e-7 degree after the minutes → degrees division).
- **Line length**: up to 120 characters from `$` to the checksum (NMEA allows
  82); longer lines are dropped and counted in `overflows`.
- **Year 2038**: `read()` stores `utc` and `seq` as their 32-bit pattern in an
  `array('i')`; after 2038-01-19 `values[nmea.UTC]` reads negative — use
  `values[nmea.UTC] & 0xFFFFFFFF`. The `utc` attribute is always correct.

## UART, interrupts and DMA

On the MicroPython stm32 port the UART driver receives with one interrupt per
byte into the `rxbuf` ring buffer; it does not use DMA. At 115 200 baud that is
at most about 11 500 interrupts per second, a few microseconds each on a
Cortex-M4 — negligible. This module does not touch the UART hardware: it reads
the ring buffer through the stream protocol, so it works with any port and any
UART driver. Size `rxbuf` for the longest gap between two `poll()` calls
(about 1.5 kB per second of default NMEA output at 1 Hz).

## Tests

```
make test                                  # host: C unit tests + C parser vs pynmea2 (needs cc, pip install pynmea2)
make test-unix MPY_DIR=/path/to/micropython   # builds the unix port with the module, runs tests/test_micropython.py
```

- `tests/test_vs_pynmea2.py` feeds real receiver output (`tests/data/`) and
  synthetic edge cases (southern / western hemispheres, NMEA 2.x, bad and
  missing checksums, overlong lines, garbage, proprietary sentences, malformed
  fields, several constellations, a constellation that stops reporting) to the
  C parser, byte by byte and in irregular chunks, and compares every value
  with what [pynmea2](https://github.com/Knio/pynmea2) decodes.
- `tests/test_core.c` checks the command helpers and the Quectel profile
  against the examples of the Quectel specification.
- `tests/test_micropython.py` checks the binding (`feed`, `poll`, `read`,
  attributes, counters, errors) and the configuration calls against a fake
  Quectel receiver (accepted, processing then accepted, error code, silent
  receiver with retry, full `configure()` sequence).

Tested with MicroPython v1.29.0:

- unix port (CI);
- STM32G474 (NUCLEO-G474RE) with a Quectel LC86GAAMD on UART4 at 115 200 baud, 1 Hz:
  3D fix, 16 satellites used / 35 in view, HDOP 0.84, PDOP 1.13, VDOP 0.76,
  63 sentences without a single error, **0 bytes allocated** over 200 `poll()` + `read()` calls.
- same board, version 0.2.0: `Parser(nmea.QUECTEL_PAIR).configure(uart, rate_ms=100,
  outputs=((nmea.GLL, 0), (nmea.VTG, 0)))` returns 0 (three commands acknowledged), then 10 fixes
  per second, 320 sentences without error.

## License

PolyForm Noncommercial License 1.0.0 — see [LICENSE](LICENSE).
Non-commercial use only. Personal, educational and research use permitted.

Required Notice: Copyright (c) 2026 B4E SRL - David Baldwin
