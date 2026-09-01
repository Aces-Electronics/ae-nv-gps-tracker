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

Nine of the sixteen IDs carry a counter and a checksum in their last two bytes:

- **b6** is a 4-bit rolling counter, `00`-`0F`.
- **b7 is the XOR of bytes 0-6.** `b7 = b0^b1^b2^b3^b4^b5^b6`.

The checksum is **confirmed on 20,019 frames with zero failures** across all
nine IDs. It is not a CRC8: a brute force over every 8-bit polynomial, init and
xorout, in both bit orders, fits only poly `0x01` — which is x^8+1, the
generator whose CRC *is* a byte-wise XOR. Nothing else fits.

It also explains an observation that predates it. b7 was noted as having a
"fixed high nibble per ID" (`C_` for 0x102, `8_` for 0x010, and so on). That
falls straight out: with a static payload `XOR(b0..b5)` is a constant K and b6
sweeps `00`-`0F`, so b7 covers exactly `K&0xF0 .. K|0x0F`. All nine observed
ranges reproduce exactly, nibble for nibble.

The nine: **0x010, 0x012, 0x102, 0x103, 0x110, 0x122, 0x230, 0x408, 0x410.**

### On the other seven, b6 and b7 are payload

**0x013, 0x300, 0x308, 0x320, 0x342, 0x514, 0x516** have no rolling counter
— b6 is a fixed byte — and no checksum. The earlier advice to ignore b6/b7
when looking for signals is wrong for these, and it cost us the engine hour
meter, which had been sitting in plain sight in `0x342` b6:b7 the whole time.

`canChecksumValid()` tells the two groups apart: it returns true only for frames
that actually run the scheme.

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

### 0x102 — the engine vitals frame

```
b0:b1  engine rpm, big-endian, 0.25 rpm/bit     CONFIRMED against the tacho
b2     manifold pressure                        strong candidate
b3     coolant temperature                      strong, scaling unconfirmed
b4     intake air temperature                   strong, scaling unconfirmed
b5     session-constant (CB, was CC)            unknown
b6     rolling counter
b7     XOR(b0..b6)
```

A published byte structure for this frame — 16-bit TPS at b2:b3, ECT at b4,
IAT at b5, CRC8 at b7 — was tested against the capture and is **shifted by
one**. Its field *list* is right; it assumes the throttle signal is 16 bits, and
it is 8. So ECT lands on b3, not b4, and IAT on b4, not b5.

b2:b3 cannot be a 16-bit pair in any case: b2 sweeps 132 counts while b3 stays
inside an 8-count band, and a big-endian pair whose MSB moves 132 counts must
carry its LSB through 132 full wraps.

**Engine rpm, b0:b1**

```
rpm = ((b0 << 8) | b1) / 4
```

Verified against the tacho: idle decoded to a 1455 mean against a gauge reading
1400-1500, a rev to 6554 against a reported ~6500. Implemented in
`canDecodeRpm()`.

**Coolant temperature, b3**

Identified by behaviour rather than by scaling, which is what makes it solid.
Over a 65 s run it climbed `62` -> `6E` one count at a time and **never once
stepped backwards**, then carried on climbing after shutdown (`6C` -> `6D` ->
`6E`). A monotonic rise while running plus a further rise once the engine stops
is heat soak with no circulation, and nothing else on the bus behaves that way.

Scaling is **not** confirmed. The standard `raw - 40` gives 58 -> 70 C, which is
believable, but 0.5 C/count fits the warm-up rate about as well. One gauge
reading settles it.

**Intake air temperature, b4**

The exact inverse of b3: `60` -> `5D` falling while running, back up to `5E`
after shutdown. A sensor cooled by airflow that heat-soaks when the airflow
stops.

**Manifold pressure, b2**

Cross-correlation against rpm peaks at **lag +120 ms** with a clean asymmetric
shape — b2 leads. Anything derived from rpm would peak at zero. On a snapped
throttle it collapses while rpm is still elevated.

What makes it pressure rather than throttle: with the engine stopped b2 reads
`1B`, *higher* than its idle value of `0A`. A throttle sensor cannot read higher
with the throttle shut than it does at idle. A MAP sensor must — atmospheric
against idle vacuum. Anchoring `1B` at atmospheric gives roughly 3.7-4 kPa per
count.

Ignore the cranking window and the single-sample spike at shutdown: both sit
around the ECU's reset frame (one all-zero 0x102 frame, at 13498 ms in the
Sep 1 capture) and are not calibrated.

### Engine hours — 0x342 b6:b7

```
minutes = (b6 << 8) | b7        // big-endian, minutes
```

**CONFIRMED to the minute.** `0x16E0` = 5856 minutes = 97h36m, which is exactly
what the dash read at the time of the capture. The odds of a random 16-bit value
matching both the hours and the minutes are about 1 in 5856.

Two more counters sit in the same range and are *not* the meter the dash shows.
Both moved between the Aug 31 and Sep 1 sessions while the engine meter did not,
which is consistent with a couple of minutes of key-on time and under a minute
of counted engine time:

| | Aug 31 | Sep 1, engine run | Sep 1, key-on only | Reads as |
|---|---|---|---|---|
| 0x342 b6:b7 | 5856 | 5856 (+0) | 5856 (+0) | 97h36m — **matches the dash** |
| 0x342 b0:b1 | 6116 | 6118 (+2) | 6119 (+1) | 101h59m — key-on minutes |
| 0x300 b2:b3 | 5779 | 5780 (+1) | 5780 (+0) | 96h20m — unknown |

The third session is the useful one: the key was on for a few minutes and **the
engine never started**. 0x342 b0:b1 advanced by one and the engine meter did not
move at all, which is what a key-on meter and an engine meter respectively must
do. It also rules the engine meter out of counting anything but running time.

## Strong candidates, not confirmed

Correlation coefficients are against the confirmed RPM over a 30-sample run
covering idle, a rev to 6500 and a return to idle. The first entry below is a
correction rather than a candidate.

### 0x342 b4 — not the coolant temperature

This was previously read as *the* engine temperature, on the strength of its
drift from `8C` to `83` over forty seconds after shutdown. **That inference was
wrong**, and the Sep 1 capture shows why: 0x342 b4 jitters between `8A`-`8F`
with rpm and *falls* after shutdown, while 0x102 b3 climbs monotonically and
keeps climbing. Coolant heat-soaks upward when you stop a hot engine; it does
not cool within the first minute.

So 0x342 b4 is a real signal with far less thermal mass than coolant —
exhaust is the obvious guess — but it is not what it was labelled.
`canReadEngine()` now takes coolant from 0x102 b3 and keeps this byte separately.

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

### iBR lever — 0x010 b1, and what is *not* broadcast

The lever is a switch, not a position sensor: it trips at or below quarter
travel, and quarter, half and full are indistinguishable on the bus.

**0x010 b1 pulses `25` -> `FF` -> `25` for 0.1-0.5 s at every state change.**
Confirmed across three sessions. In the cleanest one, an 8 s hold / 8 s release /
5 s hold gave transitions at 6.2, 14.1, 22.1 and 26.8 s and nothing in between,
which is what a switch must do and what a position sensor cannot.

**0x012 gains a third multiplexer slot once the lever is first used.** Before
the first squeeze the message alternates two slots (`b1` = `42`, `43`) with `b4`
fixed per slot. Within 0.35 s of the first squeeze a third slot appears
(`b1` = `46`, `b0` = `61`) and `b4` begins alternating in all three. It then
never stops, through releases and further squeezes alike. It latches on first
use; it does not track the lever.

**The held state is not broadcast with the engine off.** A 30 s full-rate record
of 0x010, 0x012, 0x408 and 0x410, taken while the lever was held for 8 s, then
released for 8 s, then held for 5 s, has **no byte whose value differs between
held and released**. Not one, across all four IDs. Combined with the transition
pulse, the ECU is announcing that the lever changed without publishing what it
changed to.

### iBR up/down — 0x408 b0 commands, 0x012 b2 is the position

The up/down buttons, as distinct from the brake/reverse lever above. **Both the
command and the resulting position are on the bus, and both work with the engine
stopped.**

```
0x408 b0   0x10 = step up, 0x20 = step down, 0x00 = idle   (one frame per step)
0x012 b2   position, 1..9                                   (9 discrete steps)
```

Confirmed by a 40 s record of the whole bus over two full up-and-down cycles:

| | |
|---|---|
| 0x408 b0 pulses | 32 (16x `0x10`, 16x `0x20`) |
| 0x012 b2 steps | 32 (16 up, 16 down) |
| Correspondence | 1:1, no missed or extra steps |
| Position lag behind command | **+24 ms mean** (range +7 to +41 ms) |

The lag is the command/feedback relationship: 0x408 b0 is what was asked for and
0x012 b2 is where the actuator got to. That is also the pattern that was expected
for the lever and its gate, and never appeared there.

Note 0x408 b0 asserts for a **single frame per step**, not for as long as the
button is held. Holding a button produces a train of one-frame pulses, roughly
every 100-300 ms. Nothing on the bus holds a level while the button is down. That
matches the lever exactly: this bus publishes iBR events and the resulting state,
not raw switch levels. Anything hoping to read "button currently pressed" has to
infer it from the pulse train.

**Retrospective confirmation.** The Aug 31 and Sep 1 captures differed by
`0x012 b2: 09 -> 01`, logged at the time as an unexplained -8. It was the trim
left fully up on one day and fully down on the other. A byte identified as a
1..9 position accounting exactly for a previously unexplained change between
sessions is about as good as an independent check gets.

### iBR gate (brake/reverse) — still not found

The gate driven by the *lever* remains unidentified, and it is a separate
question from the up/down position above. Nothing moved during any lever hold
beyond the transition pulse, which is consistent with a gate that is not driven
while the engine is stopped. `0x410 b0` is the remaining candidate for gate
state: it ranged `00`-`03` on Aug 31, the only session with the engine at real
rpm, and has been pinned at `03` in every engine-off capture since.

## Not yet identifiable

**Fuel level.** With the tank at about 40%, a scan of every byte and every
16-bit pair, both endiannesses, against eleven plausible full scales produced
**18 candidates** all reading 38-42%. The scan even flagged 0x102 b3 and b4,
which are known to be coolant and intake air. That is the method failing by
construction: "about 40%" against an unknown full scale is unidentifiable from a
single reading, and no amount of scanning fixes it.

It needs a **differential** measurement instead, and the cheapest one burns no
fuel at all:

> Ignition on, **engine off**, so nothing engine-driven is moving. Then rock the
> ski hard on the trailer for 30 s while capturing.

A float sender responds to slosh; a configuration byte does not. One byte will
swing and the other seventeen will not. Filling the tank and re-capturing works
too, and additionally gives the scale.


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

## Raw captures

Every claim above is backed by a capture in this directory, gzipped because the
uncompressed set is 5.8 MB of hex against 0.5 MB compressed. `gunzip -c file.gz`
or `zcat` reads one without unpacking it.

| File | What it establishes |
|---|---|
| `startup-idle-revto3500-idle-blip-blip-idle-off.csv.gz` | The engine decodes: checksum on 20,019 frames, coolant, intake air, MAP's 120 ms lead, engine hours |
| `ibr-lever-sweep.csv.gz` | First lever signal seen (0x010 b4) |
| `ibr-lever-sweep-2.csv.gz` | Lever fires on state change only, not on travel |
| `ibr-lever-3.csv.gz` | Same, reproduced against an 8/8/5 s sequence |
| `ibr-lever-4.csv.gz` | Full-rate record showing **no** byte differs between held and released |
| `ibr-flap.csv.gz` | iBR up/down: 32 command pulses against 32 position steps |

Three of these were cut short by the tooling rather than by the ski — the
recorder and the prompt lived in different phases, so a control was worked while
nothing was recording. They are kept because the lever timing across separate
sessions is what makes the switch reading reproducible rather than a single
observation.

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
