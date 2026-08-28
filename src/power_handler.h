#pragma once

#include <Arduino.h>

// The BQ25176J reports through two open-drain pins; between them they separate
// four situations worth telling apart in telemetry.
enum class ChargeState {
    NoInput,  // PG high: no jetski supply, or it is outside the charger's window
    Charging, // PG low, STAT low
    Idle,     // PG low, STAT high: charged, or we have cut the supply ourselves
    Fault,    // STAT blinking
};

struct PowerStatus {
    bool        supplyPresent = false;
    bool        charging      = false;
    bool        supplyEnabled = true; // what the switch was last asked for
    ChargeState state         = ChargeState::NoInput;
};

// Releases the previous cycle's deep-sleep pad hold and puts the three control
// pins into a defined state. Call once per wake, before powerRead().
void powerBegin();

// Samples PG, and watches STAT for long enough to catch it blinking.
PowerStatus powerRead();

// Decides whether the jetski should be feeding us and drives SUPPLY_EN to
// match, updating `status` with the outcome.
void powerApplyChargePolicy(float battVolts, PowerStatus& status);

// Latches SUPPLY_EN and the transceiver standby pin so they survive deep sleep.
// Call immediately before esp_deep_sleep_start().
void powerPrepareForSleep();

// Tests STAT and PG for the daughter board's own 10k pull-ups (R10/R11).
// Those hang off the board's 3.3V rail, so seeing them proves the board is both
// seated and powered -- which the I2C lines cannot tell you, since an
// unconnected pin and a healthy idle bus look identical from the ESP32 side.
void powerDumpPullups();

// Finds every GPIO carrying an external pull-up. Use when the daughter board is
// powered but its expected pins are silent -- it locates where the board is
// really connected.
void powerSweepPullups();

const char* chargeStateName(ChargeState s);
