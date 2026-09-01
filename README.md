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
- ✅ **Jetski engine data over CAN** — RPM decoded from the ECU (daughter board)

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
  "voltage": 0.00,           // External supply (future hardware)
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
  "engine_temp_raw": 140,      // 0x342 b4, raw count — scaling unknown
  "motion_sensitivity": "Medium", // Low | Medium | High -- the selected floor
  "motion_threshold_mg": 176,  // Where the adaptive ladder currently sits
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

It scans 250k → 500k → 125k, keeps whichever rate hears frames, then prints a
table of distinct IDs with hit counts and the last payload for each, refreshed
every 10 seconds.

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
  - **Parked** → a **1440-minute (daily) heartbeat**. Backoff is not applied;
    stretching a day further on one missed fix trades away the only thing a
    heartbeat is for.

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

  | Level | Floor | Margin | For |
  |---|---|---|---|
  | Low | 352mg | ~15× noise | A deliberate shove; a ski somewhere lively, where less would report the weather |
  | **Medium** | **176mg** | ~7.3× noise | **Default.** Registers ordinary handling |
  | High | 128mg | ~5.3× noise | A quiet mooring, where a nudge should count |

  All three sit exactly on the `INT1_THS` grid (16mg steps at ±2g) — 22, 11 and
  8 counts — so none truncate. Worth checking when retuning: 56mg, for example,
  is not representable and would silently become 48mg, only 2× the noise floor.

  **TODO — expose this to the customer.** The setting is stored in NVS
  (`mot_sens`), carried in `TrackerSettings.motion_sensitivity`, and published
  as `motion_sensitivity`, but **nothing writes it yet**. It still needs a BLE
  characteristic in [`ble_handler.cpp`](src/ble_handler.cpp) alongside the
  existing settings, and a control in the web UI. Until then it can only be
  changed by reflashing.

- **Adaptive threshold**: After a motion wake, GPS decides whether it was real.
  A good fix showing under 2 km/h counts as unexplained; three consecutive
  unexplained wakes raise the threshold one step (176mg steps, 1408mg ceiling)
  from the selected floor. Genuine travel resets it to the floor immediately. A
  wake with *no* fix changes nothing — a garage roof is not a false positive. The
  current value is persisted in NVS and published as `motion_threshold_mg`, next
  to `motion_sensitivity` so a false-wake report can tell what was chosen from
  what the ski decided.

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
