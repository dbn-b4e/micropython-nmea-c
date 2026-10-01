# Changelog

## 0.1.0 — 2026-10-01

- C core: RMC, GGA, GSA, GSV header; any talker; integer-only, allocation-free.
- MicroPython binding `nmea`: `Parser`, `feed`, `poll`, `read`, attributes, `stats`, `reset`.
- Tests: C parser vs pynmea2 (real Quectel LC86G output + edge cases, byte by byte and chunked);
  MicroPython unix port test; CI for host, unix and stm32 builds.
- Verified on STM32G474 with a Quectel LC86GAAMD (live fix, no allocation).
