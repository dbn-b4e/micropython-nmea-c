# Test data

`lc86g_20261001.nmea` — Quectel LC86GAAMD (firmware default: 115200 bauds, 1 Hz),
captured on 2026-10-01 near a window, on a STM32G474 running MicroPython.

- Lines 1–39: three complete epochs right after a cold start (no fix: RMC status `V`).
- Lines 40–42: the first epoch with a fix (RMC status `A`). This capture was filtered
  on the receiver side (only GGA, GBGSV and RMC kept), so that epoch has no GSA:
  the parser correctly keeps `fix = 1` (no GSA received yet) while `valid = 1`.

All checksums are original.
