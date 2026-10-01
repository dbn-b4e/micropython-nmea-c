# Changelog

## 0.2.0 — 2026-10-01

- Receiver configuration through vendor profiles (`src/nmea_vendor.c`): `Parser(nmea.QUECTEL_PAIR)`,
  `set_rate`, `set_output`, `save`, `configure`; acknowledgement awaited while NMEA keeps being decoded.
- Quectel `$PAIR` profile (LC26G / LC76G / LC86G), from the Protocol Specification V1.4.
- Generic helpers: `nmea.command()`, `expect()` / `response()` (C: `nmea_build`, `nmea_expect`, `nmea_response`).
- `tests/test_core.c`; fake Quectel receiver in `tests/test_micropython.py`.
- `nmea_t` grows from 248 to 400 bytes (reply buffer).

## 0.1.0 — 2026-10-01

- C core: RMC, GGA, GSA, GSV header; any talker; integer-only, allocation-free.
- MicroPython binding `nmea`: `Parser`, `feed`, `poll`, `read`, attributes, `stats`, `reset`.
- Tests: C parser vs pynmea2 (real Quectel LC86G output + edge cases, byte by byte and chunked);
  MicroPython unix port test; CI for host, unix and stm32 builds.
- Verified on STM32G474 with a Quectel LC86GAAMD (live fix, no allocation).
