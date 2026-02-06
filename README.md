# AE-SIM7080G-S3 NB-IoT Tracker

LilyGo T-SIM7080G-S3 based GPS tracker with NB-IoT connectivity for the AE-NV ecosystem.

## Hardware

- **Board**: LilyGo T-SIM7080G-S3 (ESP32-S3 + SIM7080G modem)
- **GPS**: Integrated GNSS (GPS/GLONASS/Beidou/Galileo)
- **Connectivity**: NB-IoT / LTE Cat-M1
- **Battery**: LiPo with AXP2101 PMU
- **Sensors**: MMA8452Q accelerometer for orientation detection

## Features

- ✅ GPS tracking with satellite count and HDOP
- ✅ NB-IoT connectivity via Telstra network
- ✅ MQTT telemetry publishing to AE-NV backend
- ✅ BLE configuration interface (90s window on boot)
- ✅ Orientation detection (Flat/Vertical/Upside Down)
- ✅ Configurable reporting intervals (1-60 minutes)
- ✅ Battery voltage monitoring
- ✅ Deep sleep support (planned)

## Telemetry Fields

The tracker publishes the following data via MQTT:

```json
{
  "mac": "860016043157614",
  "model": "ae-sim7080g-tracker",
  "lat": -28.02575,
  "lon": 153.3879,
  "alt": -11.981,
  "speed": 0,
  "sats": 11,
  "voltage": 0.00,           // External supply (future hardware)
  "device_voltage": 4.177,   // Internal battery voltage
  "battery_voltage": 4.177,  // Legacy alias
  "rssi": 19,
  "orientation": "Flat",
  "interval": 1
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

### Configuration

The tracker can be configured via BLE during the 90-second window on boot:

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
- **Deep Sleep**: Planned implementation for extended battery life

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

## Next Steps

See the agent prompts in `/docs/agent-prompts/` for:
1. BLE connection debugging
2. Low-power mode implementation with power profiler analysis
