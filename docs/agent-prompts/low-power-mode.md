# Agent Prompt: Implement Low-Power Mode with Power Profiler Analysis

## Context

The AE-SIM7080G-S3 tracker currently operates in a continuous active mode with configurable reporting intervals (1-60 minutes). To extend battery life, we need to implement deep sleep between GPS fixes and MQTT publishes.

## Current Power Consumption

Deep sleep is implemented -- `goToSleep()` powers the modem and GPS antenna
rail down and calls `esp_deep_sleep_start()` with a timer wake. The "STAY
AWAKE" development mode this document was written against is gone.

What has never been done is measuring any of it. Every number below is a target,
not a baseline, and the first job is still to put a profiler on a real unit.

Known remaining draws between reports:
- Nothing has verified the AXP2101 rails are all down in sleep
- The daughter board's CAN transceiver is parked in standby (~370uA typ) rather
  than fully unpowered

## Target Power Profile

- **Deep Sleep**: <100µA between reports
- **Active (GPS + MQTT)**: ~200-300mA for 30-60 seconds
- **Target Battery Life**: 7-14 days with 15-minute intervals

## Your Task

Implement deep sleep mode and use a power profiler to verify power consumption targets.

### Phase 1: Power Profiler Setup

1. **Hardware Setup**
   - Connect a power profiler (Nordic PPK2, Joulescope, or similar) to the tracker
   - Measure baseline current consumption in current "STAY AWAKE" mode
   - Document power consumption during each phase:
     - Boot and BLE advertising (90s)
     - GPS acquisition (cold start and warm start)
     - MQTT publish
     - Idle between reports

2. **Create Power Profile Baseline**
   - Generate graphs showing current consumption over a full report cycle
   - Identify power-hungry operations
   - Calculate estimated battery life with current firmware

### Phase 2: Deep Sleep Implementation

1. **Modify Main Loop** — DONE. `goToSleep()` sleeps on a timer derived from
   `settings.report_interval_mins`, and `power_handler.cpp` already keeps state
   across sleep in RTC memory and holds the supply-switch pad.

2. **Peripheral Management**
   - Power down GPS module before sleep
   - Put SIM7080G modem in minimum power mode or power off
   - Disable accelerometer or put in low-power mode
   - Turn off unnecessary peripherals (NeoPixel, etc.)

3. **Wake-Up Optimization**
   - Implement GPS warm start to reduce acquisition time
   - Use modem sleep modes instead of full power-down if faster
   - Consider keeping network registration if it saves power

4. **BLE Window Handling**
   - Ensure BLE window still works on cold boot
   - Skip BLE on wake-from-sleep to save power
   - Add manual wake trigger (button press) for configuration

### Phase 3: Power Profiler Verification

1. **Measure Deep Sleep Current**
   - Verify <100µA during sleep
   - Check for any unexpected wake-ups
   - Measure sleep entry/exit overhead

2. **Measure Active Cycle**
   - GPS acquisition time and current draw
   - MQTT publish duration and current
   - Total energy per report cycle

3. **Calculate Battery Life**
   - Use actual measurements to estimate battery life
   - Test with different reporting intervals (1min, 15min, 60min)
   - Document trade-offs between battery life and data freshness

### Phase 4: Optimization

1. **GPS Optimization**
   - Implement GPS warm start with almanac caching
   - Consider A-GPS if network-assisted location is available
   - Reduce GPS timeout if fix takes too long

2. **Modem Optimization**
   - Test PSM (Power Saving Mode) vs full power-down
   - Optimize MQTT keepalive and QoS settings
   - Consider eDRX (extended Discontinuous Reception) if supported

3. **Firmware Optimization**
   - Reduce boot time by minimizing initialization
   - Optimize MQTT payload size
   - Remove debug logging in production builds

## Reference Files

- `src/main.cpp` - Main firmware
- `src/power_handler.cpp` - Supply switch, charger status, deep-sleep pad holds
- `platformio.ini` - Build configuration
- ESP32-S3 Deep Sleep Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/sleep_modes.html
- SIM7080G AT Command Manual (for power modes)

## Success Criteria

- Deep sleep current <100µA verified with power profiler
- GPS acquisition time <60 seconds (warm start)
- Total active time per cycle <90 seconds
- Battery life >7 days with 15-minute intervals
- BLE configuration still functional on cold boot
- Power profiler graphs documenting before/after optimization

## Deliverables

1. Updated firmware with deep sleep implementation
2. Power profiler measurements (graphs/CSV data)
3. Battery life calculator spreadsheet based on actual measurements
4. Documentation of power optimization techniques used
5. Recommendations for further optimization

## Additional Notes

The AXP2101 PMU provides battery voltage monitoring. Make sure to read and log battery voltage before each sleep cycle to track battery drain over time. This data will be valuable for validating the power profiler measurements in real-world conditions.
