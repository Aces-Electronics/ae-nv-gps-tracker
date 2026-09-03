# AE-SIM7080G-S3 NB-IoT Tracker

LilyGo T-SIM7080G-S3 based GPS tracker with NB-IoT connectivity for the AE-NV ecosystem.

## Hardware

- **Board**: LilyGo T-SIM7080G-S3 (ESP32-S3 + SIM7080G modem)
- **GPS**: Integrated GNSS (GPS/GLONASS/Beidou/Galileo)
- **Connectivity**: NB-IoT / LTE Cat-M1
- **Battery**: LiPo with AXP2101 PMU
- **Daughter board**: `ae-tracker-db` — LIS3DH accelerometer, SN65HVD231 CAN
  transceiver, and a BQ25176J charger that tops the LiPo up from the jetski's
  supply through an MCU-switchable cutoff. Plugs onto the LilyGo's 16-pin
  expansion header; the J4 pin map is in [`src/utilities.h`](src/utilities.h).

## Features

- ✅ GPS tracking with satellite count and HDOP
- ✅ NB-IoT connectivity via Telstra network
- ✅ MQTT telemetry publishing to AE-NV backend
- ✅ BLE configuration interface (15s window on cold boot)
- ✅ Orientation detection (Flat/Vertical/Upside Down) — needs the daughter
  board; reports `Unknown` without it
- ✅ Configurable reporting intervals (1-60 minutes)
- ✅ Battery voltage monitoring
- ✅ Deep sleep between reports, with GPS-failure backoff
- ✅ **Daily heartbeat + wake on motion** — the timer is proof of life; movement
  is what actually triggers a report
- ✅ **Adaptive motion threshold** — climbs when the tracker keeps waking for
  movement GPS says didn't happen, drops straight back on real travel
- ✅ **Jetski engine data over CAN** — rpm, coolant, engine hours and iBR
  trim position decoded from the ECU (daughter board). See
  [`docs/can/seadoo-can.md`](docs/can/seadoo-can.md).

## Telemetry Fields

The tracker publishes the following data via MQTT:

```json
{
  "mac": "860016043157614",
  "model": "ae-sim7080g-tracker",
  "fix": true,               // Did this wake get a GPS fix at all
  "lat": -28.02575,          // lat/lon/alt/speed/sats/hdop are omitted if !fix
  "lon": 153.3879,
  "alt": -11.981,
  "speed": 0,
  "sats": 11,
  "supply_adc_raw": 9216,    // Ski supply, raw LIS3DH aux-ADC count
  "supply_voltage": 12.18,   // ...converted, once calibrated (absent if not)
  "voltage": 0.00,           // USB VBUS placeholder, debug frames only
  "device_voltage": 4.177,   // Internal battery voltage
  "battery_voltage": 4.177,  // Legacy alias
  "rssi": 19,
  "orientation": "Flat",
  "charge_state": "Charging",  // Charging | Idle | Fault | No Input
  "supply_enabled": true,      // Is the jetski allowed to feed the charger
  "wake_reason": "Motion",     // Cold Boot | Timer | Motion
  "accel_up": 1.00,            // Vehicle frame, g. Omitted if no IMU
  "accel_fwd": -0.02,
  "accel_stbd": 0.03,
  "engine_bus": true,          // Was the ski's CAN bus awake at all
  "ski_running": true,         // Drives the reporting cadence
  "rpm": 1455,                 // Omitted unless engine_bus
  "coolant_raw": 110,          // 0x102 b3, raw count — identity confirmed,
                               //   scaling not. raw-40 gives a believable 70C
  "engine_temp_raw": 140,      // 0x342 b4, raw count. NOT the coolant temp —
                               //   kept because it is a real signal, unidentified
  "engine_minutes": 5856,      // 0x342 b6:b7. Confirmed against the dash to the
                               //   minute: 5856 = 97h36m
  "ibr_trim": 5,               // 0x012 b2, iBR up/down position, 1..9
  "motion_sensitivity": "Medium", // Low | Medium | High -- the selected floor
  "motion_threshold_mg": 176,  // Where the adaptive ladder currently sits
  "motion_suppressed": 0,      // Motion wakes rate-limited away since last report
  "interval": 1440
}
```

## Development

### Prerequisites

- PlatformIO CLI or IDE
- USB-C cable for flashing

### Building & Flashing

```bash
# Build firmware
pio run -d /path/to/ae-sim7080g-tracker

# Flash to device
pio run -d /path/to/ae-sim7080g-tracker -t upload --upload-port /dev/ttyACM6

# Monitor serial output
pio device monitor --port /dev/ttyACM6 -b 115200
```

### CAN bring-up build

A separate environment boots straight into a listen-only CAN sniffer and never
reaches the tracker's normal cycle — no modem, no MQTT, no sleep:

```bash
pio run -e can-sniffer -t upload && pio device monitor -b 115200
```

It scans 250k → 500k → 125k, keeps whichever rate hears frames, prints a table
of distinct IDs with hit counts and per-byte ranges, then runs one probe: an
8-second learn phase, a single `*** GO ***`, and a 40-second window that records
every frame **and** narrates which bytes changed while you work a control.

The learn phase is the trick. It marks every byte that moves while the vehicle
is left alone — engine signals, counters, checksums — so afterwards it can
report only the bytes that held still and then moved. Operate one control and
the output is a short list naming that control's bytes.

Two things worth knowing before relying on it:

- **It prints its own blind spot.** A byte that moved during learning is hidden
  from the narration, which permanently hides both halves of any multiplexed
  signal. The excluded bytes are listed up front, and the CSV has every frame
  regardless — the narration is a convenience, the CSV is the evidence.
- **One prompt, then it halts.** Earlier versions recorded and narrated in
  separate phases with a `GO` for each, and the natural thing was to act on the
  second one, which was no longer recording. Three sessions were lost that way.

This is how the iBR up/down decode was found: the position byte and its command
byte both showed up in the narration within seconds of pressing a button.

**Listen-only never drives the bus**, not even the ACK bit, so this is safe to
plug into a running ski before anything is known about the bus or its rate. The
shipping firmware is unaffected — `default_envs` keeps this out of a bare
`pio run`, and the linker drops the sniffer and the TWAI driver from it.

Note that the daughter board's CAN termination jumper (JP1) is bridged by
default. A vehicle bus is already terminated at both ends, so JP1 most likely
wants cutting before connecting to a real ski.

### Configuration

The tracker can be configured via BLE during the 15-second window on cold boot:

- **Report Interval**: 1-60 minutes
- **Home Location**: Set via web UI
- **Admin Mode**: Forces 1-minute intervals for testing

## Debugging & Troubleshooting

### Serial Monitor

Monitor real-time debug output:

```bash
pio device monitor --port /dev/ttyACM0 -b 115200
```

### Debug Logging

The firmware includes comprehensive diagnostic logging:

**GPS Acquisition:**
```
[DEBUG] Raw CGNSINF: 1,1,20260206025753.000,-28.025740,153.387809,...
GPS FIX! Sats: 10 (lat: -28.02574, lon: 153.38780)
```

**Network Registration:**
```
[DEBUG] Checking network status...
[DEBUG] CEREG: +CEREG: 0,2  // 0,2 = registered on home network
[DEBUG] COPS: +COPS: 0,0,"Telstra Mobile",7  // Connected operator
[DEBUG] Signal Quality after registration: 20  // RSSI in dBm
```

**MQTT Publish:**
```
[DEBUG] Using Topic: ae-nv/tracker/860016043157614/up
[MQTT] Publishing Payload: {"mac":"860016043157614",...}
[MQTT] Publish OK
```

### Common Issues

**No GPS Fix:**
- Cold start can take 2-5 minutes
- Ensure clear view of sky
- Check antenna connection

**Network Registration Fails:**
- Verify SIM card is inserted and activated
- Check CEREG response: `0,0` = not registered, `0,2` = registered
- CSQ 99 = unknown signal (modem issue)
- CSQ 0-31 = valid signal strength

**MQTT Publish Fails:**
- Verify network registration succeeded first
- Check APN settings (default: `telstra.internet`)
- Ensure MQTT broker is reachable

**Device Not Appearing in UI:**
- Check MQTT payload in serial output
- Verify `mac` field matches device ID
- Check backend worker logs for ingestion errors
- Confirm device is claimed to your account

### Enable TinyGSM Debug

For detailed AT command logging, uncomment in `platformio.ini`:

```ini
build_flags = 
    -DTINY_GSM_DEBUG=Serial
```

This will show all modem AT commands and responses.

## Architecture

### GPS Parsing

The firmware uses robust satellite count parsing with fallback logic:
- Primary: "Satellites Used" from `+CGNSINF` response (index 15)
- Fallback: "Satellites in View" (index 14) when used count is unavailable

### Voltage Nomenclature

- `voltage`: External supply voltage (0.00V placeholder for future charging hardware)
- `device_voltage`: Internal PMU battery voltage (~4.2V when fully charged)
- `battery_voltage`: Legacy field for backward compatibility

### Power Management

- **Active Mode**: GPS acquisition + MQTT publish
- **BLE Window**: 90 seconds on boot for configuration
- **Charge cutoff**: the jetski feed is cut above **4.10V** and restored below
  **3.70V**, decided at report time against the PMU's battery reading and
  latched across sleep in RTC memory.

  4.10V sits *below* the BQ25176J's 4.2V regulation point, so the firmware —
  not the charger — is what ends a charge, opening the switch on the way up
  before the cell is ever held at full. The wide gap down to 3.70V is what makes
  that worth doing: a LiPo ages fastest sitting at full charge, so the pair
  cycles the cell through roughly 3.7–4.1 rather than floating it at 4.2. With
  only a sleeping tracker as a load, one traverse of that band takes a long time,
  so it costs very few cycles to buy. Both thresholds read the AXP2101's *cell*
  voltage — the jetski's own voltage is not available to decide on until the
  supply-sense divider is reworked.
- **Deep Sleep**: Cadence follows the ski, not the clock.
  - **Running** (CAN bus awake) → the configured `report_interval_mins`,
    5 minutes by default, settable over BLE. GPS-failure backoff applies
    (5/15/30/60/180 min).
  - **Parked** → a **1440-minute (daily) heartbeat**. GPS-failure backoff is not
    applied; stretching a day further on one missed fix trades away the only
    thing a heartbeat is for.

    **A parked heartbeat that fails to publish is retried sooner**, at 60 → 180
    → 360 minutes before falling back to the daily cadence. Without this, one
    bad modem session meant 48 hours of silence — and silence is
    indistinguishable from a stolen ski, a flat battery, or a dead tracker.
    Escalating rather than fixed because the usual cause is no coverage, and
    retrying hourly forever in a dead zone spends exactly the battery the daily
    cadence exists to protect.

    Note the daily timer is clocked by the ESP32's **internal RC oscillator**,
    not a crystal, so a 1440-minute sleep can legitimately land up to roughly an
    hour either side. A heartbeat that is late by tens of minutes is normal; one
    that is hours late is not.

  CAN traffic is the running signal because the ski's bus only wakes with the
  ignition. Supply voltage above ~13V would be the better test — it survives a
  CAN fault — but the supply-sense divider is mis-scaled and reads nothing
  usable yet (see `R15`/`R16` in [`src/utilities.h`](src/utilities.h)).
- **Wake on motion**: The LIS3DH interrupt is an `ext1` deep-sleep wake source.
  Motion reports are rate-limited to one per 120s, checked before the modem is
  powered, so a suppressed wake costs ~200ms rather than a GPS acquisition.

  **Tapping the board will not wake it, and that is the design.** `INT1_DURATION`
  is counted in ODR periods, so the default three samples at 50Hz require 60ms
  of *continuous* over-threshold motion, and normal mode band-limits to roughly
  ODR/2 = 25Hz before the comparator sees anything. A finger tap is a few
  milliseconds of mostly high-frequency energy — filtered down, and over long
  before the duration counter arrives. Shake the board instead.

  To check the path live rather than by waiting out a sleep cycle, the
  `db-bench` build watches INT1 for 15 seconds on startup and prints every
  latched event with the axes that caused it.

  On every wake from sleep the firmware reads the LIS3DH's interrupt block back
  *before* re-arming it and says whether it survived. Those registers are
  volatile, so retained values prove the part held its 3.3V rail through the
  sleep and reset defaults prove it did not — which settles the question a
  missed wake otherwise only raises. (It should always survive: the daughter
  board's 3V3 is J4.1 = AXP2101 DCDC1, the same rail that keeps the ESP32's own
  RTC domain alive. Nothing in this firmware touches DCDC1.)
- **Motion sensitivity**: three selectable levels, each setting the **floor** of
  the adaptive ladder rather than a fixed threshold — the ski still finds its own
  level from wherever it starts. Values are chosen against a *measured* noise
  floor of 24mg (high-pass filter settled, board at rest), so the margins are
  known rather than guessed:

  | Level | Floor | Margin | Ceiling | For |
  |---|---|---|---|---|
  | Low | 704mg | ~29× noise | 2032mg | A deliberate shove; a ski somewhere lively, where less would report the weather |
  | **Medium** | **352mg** | ~15× noise | **1408mg** | **Shipped default.** Registers ordinary handling |
  | High | 256mg | ~11× noise | 1024mg | A quiet mooring, where a nudge should count |

  These were **doubled** from an original 352/176/128 after the first real
  deployment: a unit on Medium riding in a car woke nine times in 65 minutes and
  spent the hour climbing away from a floor too low to start from. The old
  Medium is the new Low, which is the honest description of what 176mg proved
  to be worth.

  All three sit exactly on the `INT1_THS` grid (16mg steps at ±2g) — 44, 22 and
  16 counts — so none truncate. Worth checking when retuning: 56mg, for example,
  is not representable and would silently become 48mg, only 2× the noise floor.

  Settable over BLE on characteristic `beb5483e-36e1-4688-b7f5-ea07361b2051`
  (read/write, one byte: `0` Low, `1` Medium, `2` High). A write out of that
  range is rejected rather than clamped — clamping would silently land on
  Medium and the value read back afterwards would not be the one sent. The
  change takes effect **immediately**, re-arming the accelerometer rather than
  waiting for the next boot, which for a parked ski would be a day away.

  **TODO — web UI control.** The setting is stored in NVS (`mot_sens`), carried
  in `TrackerSettings.motion_sensitivity`, published as `motion_sensitivity`,
  and writable over BLE — but the web app has no control for it yet.

- **Adaptive threshold**, which moves in *both* directions:

  **Up.** After a motion wake, GPS arbitrates: a fix is *travel* if it shows
  2 km/h or more, **or** if it has moved 100m or more from the reference
  position. Anything else counts as unexplained, and three consecutive
  unexplained wakes raise the threshold one step. A wake with *no* fix changes
  nothing — a garage roof is not a false positive.

  Step and ceiling both **scale with the selected floor** (step = the floor,
  ceiling = 4× it), so three unexplained wakes cost the same fraction of the
  range at every setting. They used to be flat 176mg/1408mg — Medium's numbers
  applied to everything — which made the control a floor and nothing else: a
  unit on High could be walked to eleven times the sensitivity its owner asked
  for. `INT1_THS` caps the ceiling at 2032mg (7 bits, 16mg steps at ±2g), which
  is why Low's 2816mg is clamped.

  **Position is gated on PDOP**, not HDOP. Measured on a stationary bench, two
  fixes three seconds apart read 35m apart: one at PDOP 25.0, the other at PDOP
  2.0. HDOP on the bad one was a respectable 3.9 — the error was almost entirely
  vertical (VDOP 24.7), so only PDOP shows it. A DOP of `0` means the field was
  absent, not that the fix was perfect, and is refused everywhere.

  **Up, faster, on rate alone.** Motion wakes inside the 120s report limit are
  counted rather than discarded. Ten of them between two reports raises the
  threshold immediately, without waiting for GPS — the rate *is* the evidence,
  and those wakes are precisely the ones that never got a fix to be judged by.
  Previously they were invisible to this logic, which left a hole exactly where
  it was most needed: a hull working against a mooring produces mostly
  suppressed wakes.

  **Down.** Genuine travel resets straight to the floor — that is proof the
  tracker should be at its most sensitive, and it contradicts whatever evidence
  built the climb. Failing that, three consecutive reports with *no* motion
  activity at all (no wake, none suppressed) ease it down one step. Without
  this the ladder is a ratchet, and a ski moved to a calmer berth stays deaf
  forever — the worst failure available here, because it is self-sustaining:
  the deafer it gets, the less likely anything crosses the threshold to say
  otherwise, including a theft.

  The current value is persisted in NVS and published as `motion_threshold_mg`,
  next to `motion_sensitivity` so a false-wake report can tell what was chosen
  from what the ski decided, and `motion_suppressed` so the wake rate is visible
  at all.

  Changing the floor — by firmware retune or by the setting — discards a stored
  threshold derived from a different one, so a change actually reaches a unit
  that already has a value saved.

- **Wake LED**: the AXP2101 charge LED is lit while awake and switched off
  immediately before every deep sleep, so a dark unit is a sleeping one. A
  **4Hz blink instead of steady means the wake came from the accelerometer**,
  which is the only way to tell a motion wake from a timer wake on battery,
  where there is no serial port. Note this takes the LED away from the charger's
  own control; charge state is reported over MQTT instead.

  (`NEOPIXEL_PIN` in [`utilities.h`](src/utilities.h) is inherited from other S3
  boards and drives nothing — this board has no addressable RGB LED. The
  remaining `strip` calls in `main.cpp` are dead code.)

No migration is needed for already-provisioned trackers: a stored 5-minute
interval now describes the *running* case, which is what it was always for.

### Ski supply voltage

The daughter board divides the jetski supply into the LIS3DH's auxiliary ADC.
With **R16 swapped to 200k** the divider is 1.8M/200k = **1/10**, mapping 8–16V
onto 800–1600mV — which is the LIS3DH aux ADC's actual window (1200mV ±400mV).

**The divider taps `SUPPLY`, downstream of the TPS1H000 switch**, so it reads
nothing whenever the charge policy has the feed cut — which is most of the time
by design. `powerReadSupplyRaw()` therefore closes the switch briefly (200ms
settle, covering six time constants of 180k × 100nF as well as the switch's own
turn-on), takes the sample, and restores the switch to exactly the state it was
in. Leaving it closed because a measurement happened would quietly undo the
charge cutoff.

**Calibration is two-point and measured, not derived.** ST does not document
whether the aux-ADC code is inverted, and the real ratio depends on resistor
tolerance and an input impedance ST does not specify — so a formula from the
schematic would be a guess wearing a unit. Measured on hardware:

| point | meter | raw | 10-bit |
|---|---|---|---|
| low | 12.06V | 10188 | 159 |
| high | 14.33V | −7508 | −118 |

**The code is inverted** — 0V reads 32512, and the count falls as voltage rises.
That is exactly the ambiguity the datasheet leaves open, and it is why this is
measured. The slope works out at ~122 counts/V against ~128 predicted from the
design, i.e. within ~5% — comfortably inside resistor tolerance.

To calibrate a board, flash the `supply-cal` environment and drive it over
serial: `L <volts>` and `H <volts>` store the two points into the same NVS keys
the tracker reads (`sup_raw_lo`/`sup_v_lo`, `sup_raw_hi`/`sup_v_hi`). Readings
are averaged over 16 samples, because one sample of a 10-bit ADC on a 180k
source is noisy enough to bake that noise into the fit. `supply_voltage` is
published only once both points exist; until then only `supply_adc_raw` goes
out, so an uncalibrated unit reports nothing rather than something wrong.

**Useful range is ~8–16V.** Below about 8V the ADC floors and the reading is
optimistic — a flat jetski battery reads around 9V rather than as flat.

Note `charge_state: "Fault"` with **no LiPo attached** is expected: the
BQ25176J blinks STAT when it has nothing to charge.

### OTA downloads

Downloads used to die part-way with `+CAURC: buffer full` followed by
`Socket closed early` — 397468 of 680816 bytes on the observed failure.

The cause is arithmetic, not a bug. TinyGSM drains a socket **one byte at a
time** over the modem UART, and at 115200 baud that is ~11.5 KB/s. LTE Cat-M1
delivers up to ~37 KB/s, so the network fills the modem's socket buffer about
three times faster than it can be emptied — which is precisely what
`+CAURC: buffer full` reports. A 680 KB image also needs ≥59s of pure UART time
at 115200 before any AT overhead.

Three changes:

- **The modem UART is raised to 921600 for the transfer** (~92 KB/s), so the
  UART stops being the constraint rather than merely keeping pace. Restored to
  115200 afterwards: `AT+IPR` is not persisted without `AT&W`, so a modem left
  fast would be unreachable on the next boot when `Serial1` opens at 115200.
  The switch verifies the link at the new rate and reverts on failure — the only
  remote repair path for a fielded tracker is the OTA itself, so a half-applied
  baud change would be unrecoverable. Both 460800 and 921600 were confirmed on
  hardware with the `baud-test` environment.
- **`TINY_GSM_DEBUG` is no longer in the release build.** It prints a line per
  data URC — ~660 of them per OTA — onto the same serial port the download is
  competing with. Use the `modem-debug` environment when AT tracing is wanted.
- **The empty-poll delay is 2ms rather than 20ms**, and throughput is logged in
  KB/s, so a future failure can be read rather than guessed at.

## Known Issues

1. **BLE Connection**: Some devices experience connection stability issues (see agent prompt)
2. **Power Consumption**: No deep sleep implementation yet (see agent prompt)
3. **GPS Cold Start**: Can take 2-5 minutes for initial fix

## Web UI Integration

The tracker integrates with the AE-NV web dashboard:

- **Live Map**: Real-time GPS position with historical track
- **Battery Card**: Displays internal battery voltage
- **Supply Card**: Displays external supply (0V placeholder)
- **Satellite Count**: Live satellite visibility
- **Recent Locations**: Historical table with GPS, battery, and supply data

## Forensic Crash Reporting
The tracker includes a high-fidelity crash handling system for remote diagnostics:
- **Panic Interception**: Intercepts ESP32-S3 hardware panics to capture the exact state of failure.
- **Persistent Logs**: Saves register dumps and backtraces to NVS/RTC memory to survive reboots.
- **Cloud Delivery**: Automatically uploads the crash log to the cloud via MQTT on next boot.
- **Symbol Matching**: The CI pipeline preserves `.elf` files for every build, allowing the backend to translate raw addresses into source code lines (e.g., `main.cpp:142`).

## Next Steps

See the agent prompts in `/docs/agent-prompts/` for:
1. BLE connection debugging
2. Low-power mode implementation with power profiler analysis
