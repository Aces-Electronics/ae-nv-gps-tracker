# SeaDoo CAN bus — what we know

Captured from a live ski on 2026-08-31 with the `can-sniffer` firmware
(`pio run -e can-sniffer`), tapped at the diagnostic port through the
ae-tracker-db daughter board.

Everything here was measured on one ski. Treat it as a starting point for that
model rather than a specification.

## Bus

| | |
|---|---|
| Bit rate | **500 kbit/s** |
| ECU | Bosch ME17.8.5 |
| Frame format | Standard 11-bit IDs, all 8 data bytes |
| Distinct IDs | 16 |
| Frame rate | ~982 frames/s |
| Bus load | ~25% |

**250 kbit/s hears nothing.** A J1939-derived powersports bus is the obvious
guess and it is wrong here, which is why the sniffer sweeps rates rather than
assuming one.

## Protocol overhead

Two bytes on almost every ID are not payload:

- **b6** is a 4-bit rolling counter, `00`-`0F`, on every ID that carries data.
- **b7** has a fixed high nibble per ID (`8_` for 0x010, `9_` for 0x012, `C_`
  for 0x102, `4_` for 0x408, `3_` for 0x410, `1_` for 0x110 and 0x122) over a
  varying low nibble. Counter plus checksum.

Ignore both when looking for signals.

## Message inventory

Rates measured over a 10 s window.

| ID | Rate | Carries engine data? |
|---|---|---|
| 0x102 | 100 Hz | **yes** — RPM |
| 0x103 | 100 Hz | yes |
| 0x122 | 100 Hz | **yes** — most active message on the bus |
| 0x010 | 91 Hz | no (static payload) |
| 0x230 | 90 Hz | yes (b5 only) |
| 0x110 | 50 Hz | yes |
| 0x300 | 50 Hz | yes |
| 0x308 | 50 Hz | yes |
| 0x320 | 50 Hz | yes |
| 0x342 | 50 Hz | **yes** — temperature |
| 0x516 | 50 Hz | no |
| 0x012 | 46 Hz | yes (b4) |
| 0x408 | 45 Hz | no |
| 0x410 | 45 Hz | yes |
| 0x514 | 45 Hz | no |
| 0x013 | 18 Hz | no |

Static even with the engine running: **0x010, 0x013, 0x408, 0x514, 0x516**.
Those are configuration or identity frames.

## Confirmed signals

### Engine RPM — 0x102, bytes 0-1

```
rpm = ((b0 << 8) | b1) / 4        // big-endian, 0.25 rpm per bit
```

**Verified against the tacho**, not inferred:

| | Decoded | Gauge |
|---|---|---|
| Idle | 1455 mean | 1400-1500 |
| Rev | 6554 peak | ~6500 |
| Engine off | 0 (`0000`) | — |

Both ends within 1%. Little-endian would have to claim 16,349 rpm at the same
peak, which is why the byte order is not in doubt either.

Implemented in `canDecodeRpm()` in `src/can_handler.cpp`.

## Strong candidates, not confirmed

Correlation coefficients are against the confirmed RPM over a 30-sample run
covering idle, a rev to 6500 and a return to idle.

### 0x342 b4 — a temperature

Range `8C`-`90` running. The evidence is not the +0.75 correlation with RPM but
what it did **after shutdown**: drifted `8C` → `83` over forty seconds, steadily,
with the engine stopped. That is a thermal mass cooling, and no RPM-derived
signal behaves that way.

Scaling unknown. Needs one reading against a gauge at a known temperature.

### 0x122 b1 — load or manifold vacuum

`r = -0.920`. Sits at `17` at idle and falls to `00` at wide-open throttle.
Strongly inversely related to engine load. Consistent with manifold vacuum.

### 0x122 b2 and b3 — track RPM

`r = +0.865` and `+0.893`. Both climb and fall with revs.

### 0x122 b0 and 0x110 b3 — one signal carried twice

Never differ by more than one count, never in direction — consistent with
sampling a 100 Hz message and a 50 Hz message at different instants. Weak RPM
correlation (`r ≈ +0.34`), but dips to `0B` on a snapped-shut throttle and peaks
at `18` at WOT. Behaves more like manifold pressure than anything RPM-derived.

## Not yet identifiable

**Speed.** On a trailer it reads zero for the whole session, which is
indistinguishable from a constant. This one needs the ski on water.

Published sources put SeaDoo speed on `0x208` (2008 Siemens ECU) or `0x268`
(2021 Spark, Bosch ME17.8.5), the latter with speed as a big-endian pair in
bytes 4-5 divided by 100.

**Neither ID exists on this bus.** The complete inventory is the sixteen IDs
above; the sniffer records up to 64 distinct IDs and never truncated, so that
list is not a sample. The same sources correctly identify `0x102` as carrying
RPM, which we confirmed independently, so they are not worthless — but the
speed mapping does not transfer to this ski, most likely a model-year
difference.

Two things about the `0x268` claim also look shaky on their own terms: a `/100`
scaling implies 0.01 km/h resolution on a jetski speedometer, and the `0x208`
description ("byte 0 for speeds above 25 km/h, byte 1 for below") is not a
scheme any ECU would plausibly use.

So speed has to be found the way RPM was found: by correlation against a known
reference. `pio run -e speed-survey` logs GPS speed alongside every byte of
every ID as CSV for exactly that.

```bash
pio run -e speed-survey -t upload && pio device monitor -b 115200 | tee survey.csv
```

Ride at several steady speeds, holding each for ~15 s, then correlate each byte
against the `gps_kmh` column. Bytes 6 and 7 can be ignored — they are the
counter and checksum.

## Reproducing

```bash
pio run -e can-sniffer -t upload && pio device monitor -b 115200
```

The build is **listen-only throughout** (`TWAI_MODE_LISTEN_ONLY`) — it does not
even send ACK bits, so a wrong bit-rate guess cannot disturb the vehicle. It
sweeps 250k/500k/125k, keeps whichever hears frames, prints one ID-and-range
discovery table, then streams a time series with RPM decoded.

`canSelfTest()` exists for bench use and **transmits** (`TWAI_MODE_NO_ACK`). It
is deliberately not called from any build that might be plugged into a ski.

## Hardware notes

- **JP1 (CAN termination) must be cut.** The vehicle bus is already terminated at
  both ends; a third 120R terminator drops the bus to ~40R.
- Wiring, from the diag port: CAN-H is the green wire, CAN-L white, ground red.
  See `hardware/ae-tracker-db/diag_pinout.png`.
