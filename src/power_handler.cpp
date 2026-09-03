#include "power_handler.h"
#include "utilities.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include <Wire.h>
#include "imu_handler.h"

// Kept in RTC memory so the hysteresis band below has a previous decision to
// hold onto across a sleep. A cold boot initialises it to true, which matches
// what the 1.8M pull-up on SUPPLY_EN does on an unprogrammed board: charging
// allowed until something decides otherwise.
RTC_DATA_ATTR static bool s_supplyEnabled = true;

// A 1S LiPo is full at 4.2V. These thresholds are not a substitute for the
// charger's own termination -- the BQ25176J handles that -- they decide whether
// it is worth drawing from the jetski at all.
//
// 4.1V is deliberately BELOW the BQ25176J's 4.2V regulation point, which makes
// this firmware the thing that ends a charge rather than a backstop behind the
// charger's own termination. The switch opens on the way up, before the cell is
// ever held at full.
//
// The wide gap down to 3.7V is what makes that worth doing. A LiPo ages fastest
// sitting at full charge, so the pair cycles the cell through roughly 3.7-4.1
// instead of floating it at 4.2 -- and since the only load between reports is a
// sleeping tracker, one traverse of that band takes a long time and costs very
// few cycles to buy it. It also means a cut once made is not undone by the cell
// settling back, which a narrow band would do on every wake.
//
// Both thresholds are read against the AXP2101's cell voltage, not the jetski's:
// the supply-sense divider is still mis-scaled, so real input voltage is not
// available to decide on yet.
static const float CHARGE_STOP_V   = 4.10f;
static const float CHARGE_RESUME_V = 3.70f;

// STAT blinks to signal a fault, so a single read cannot tell a fault from a
// steady state. This window is long enough to catch an edge of a ~1Hz blink.
// Against a wake that can spend five minutes acquiring GPS, it is not worth
// shortening.
static const int STAT_SAMPLES  = 24;
static const int STAT_DELAY_MS = 30;

static void driveSupply(bool enabled) {
    pinMode(DB_SUPPLY_EN, OUTPUT);
    digitalWrite(DB_SUPPLY_EN, enabled ? HIGH : LOW);
    s_supplyEnabled = enabled;
}

void powerBegin() {
    // A pad latched before the last deep sleep is still latched now. Until the
    // hold is released, writes reach the output register but not the pin.
    gpio_hold_dis((gpio_num_t)DB_SUPPLY_EN);
    gpio_hold_dis((gpio_num_t)DB_CAN_RS);

    driveSupply(s_supplyEnabled);

    // Nothing on the board biases Rs, so it is undefined until something drives
    // it -- and an undefined Rs is an undefined transceiver. HIGH is standby.
    // CAN bring-up will take this pin over; until then standby is the honest
    // state, because nothing is listening to the bus.
    pinMode(DB_CAN_RS, OUTPUT);
    digitalWrite(DB_CAN_RS, HIGH);

    // Both status pins already carry 10k pull-ups on the daughter board. The
    // internal pull-up is asked for anyway so that with no daughter board
    // fitted they read HIGH -- "no supply" -- instead of floating.
    pinMode(DB_CHG_STAT, INPUT_PULLUP);
    pinMode(DB_CHG_PG, INPUT_PULLUP);
}

PowerStatus powerRead() {
    PowerStatus ps;
    ps.supplyEnabled = s_supplyEnabled;

    // PG is active low: LOW means the input is present and inside the charger's
    // window.
    ps.supplyPresent = (digitalRead(DB_CHG_PG) == LOW);

    int lows = 0;
    for (int i = 0; i < STAT_SAMPLES; i++) {
        if (digitalRead(DB_CHG_STAT) == LOW) lows++;
        delay(STAT_DELAY_MS);
    }

    const bool steadyLow  = (lows == STAT_SAMPLES);
    const bool steadyHigh = (lows == 0);
    ps.charging = steadyLow;

    // Steady either way is a state. Anything in between is STAT blinking, and
    // that is the only way this charger reports a fault -- so it outranks the
    // other readings rather than being averaged into them.
    if (!ps.supplyPresent) {
        // Checked before the blink test on purpose. With no input the charger
        // is not driving STAT at all, so a pin wandering between reads is noise
        // on an open-drain line with nothing on it -- reporting that as a
        // charger fault is how a parked, healthy ski ends up looking broken.
        ps.state = ChargeState::NoInput;
    } else if (!steadyLow && !steadyHigh) {
        ps.state = ChargeState::Fault;
    } else if (steadyLow) {
        ps.state = ChargeState::Charging;
    } else {
        ps.state = ChargeState::Idle;
    }

    Serial.printf("[Power] SUPPLY_EN pin reads %s\n",
                  digitalRead(DB_SUPPLY_EN) == HIGH ? "HIGH (switch on)" : "LOW (switch off)");
    Serial.printf("[Power] PG=%s STAT=%s -> %s (supply %s)\n",
                  ps.supplyPresent ? "present" : "absent",
                  ps.state == ChargeState::Fault ? "blinking" : (steadyLow ? "low" : "high"),
                  chargeStateName(ps.state),
                  ps.supplyEnabled ? "enabled" : "cut");
    return ps;
}

void powerApplyChargePolicy(float battVolts, PowerStatus& status) {
    bool enable = s_supplyEnabled;

    if (battVolts >= CHARGE_STOP_V) {
        enable = false;
    } else if (battVolts <= CHARGE_RESUME_V) {
        enable = true;
    }
    // Between the thresholds the previous decision stands, so a battery resting
    // near one of them does not flip the switch on every single wake.

    if (enable != s_supplyEnabled) {
        Serial.printf("[Power] Battery %.2fV -> %s jetski supply\n",
                      battVolts, enable ? "restoring" : "cutting");
        driveSupply(enable);
    }
    status.supplyEnabled = s_supplyEnabled;
}

// How long to let the divider settle once the switch closes.
//
// Source impedance is 1.8M || 200k = 180k. With C3 at 10nF that is a 1.8ms time
// constant; if C3 has been changed to 100nF it is 18ms. 200ms covers six time
// constants of the slower case and the TPS1H000's own turn-on, so it is correct
// for either board without needing to know which is fitted. The cost is 200ms of
// jetski supply per report, which is nothing against a report that runs the
// modem for a minute.
static const uint32_t SUPPLY_SETTLE_MS = 200;

bool powerReadSupplyRaw(int16_t& raw, bool& supplyEnabledBefore) {
    supplyEnabledBefore = s_supplyEnabled;
    if (!imuPresent()) return false;

    const bool needToClose = !s_supplyEnabled;
    if (needToClose) {
        Serial.println("[Power] Closing the switch briefly to read supply voltage.");
        driveSupply(true);
        delay(SUPPLY_SETTLE_MS);
    }

    const bool ok = imuReadAdc1(raw);

    // Back exactly as it was, including the case where the policy had it on.
    // Leaving the feed closed because a measurement happened would quietly undo
    // the charge cutoff, which is the one thing that stops the jetski battery
    // being drained by a parked tracker.
    if (needToClose) driveSupply(false);

    if (ok) {
        Serial.printf("[Power] Supply ADC raw=%d (switch was %s, %s to read)\n",
                      raw, supplyEnabledBefore ? "on" : "cut",
                      needToClose ? "closed briefly" : "already on");
    } else {
        Serial.println("[Power] Supply ADC read failed.");
    }
    return ok;
}

void powerPrepareForSleep() {
    // Without a pad hold these pins are released when deep sleep starts, and
    // the 1.8M pull-up would quietly turn the jetski feed back on for the whole
    // sleep. Since sleeping is essentially all this device does, cutting the
    // supply only while awake would save nothing at all.
    gpio_hold_en((gpio_num_t)DB_SUPPLY_EN);
    gpio_hold_en((gpio_num_t)DB_CAN_RS);
    gpio_deep_sleep_hold_en();

    Serial.printf("[Power] Sleeping with jetski supply %s\n",
                  s_supplyEnabled ? "enabled" : "cut");
}

// Deliberately a local copy rather than shared with imu_handler: the two
// modules have no other reason to know about each other, and this is eight
// lines.
// Probing a pin means taking it back from whatever peripheral owns it. For the
// two I2C buses that is not recoverable on its own: the controller keeps
// running, the pins no longer reach it, and every transfer afterwards fails
// with ESP_ERR_TIMEOUT. Both buses are therefore rebuilt after any sweep or dump
// that could have touched their pins.
//
// This is not theoretical. It broke Wire1 for the entire session after the first
// sweep, and read as a daughter board fault.
static void restoreI2CBuses() {
    // end() before begin(). begin() on a bus that is already initialised does
    // not re-attach the pins -- it returns having changed nothing -- so the
    // controller stays alive with its pins pointing elsewhere and every
    // transfer keeps timing out. Tearing it down first is what actually
    // reconnects them.
    Wire.end();
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire1.end();
    Wire1.begin(DB_IMU_SDA, DB_IMU_SCL, DB_I2C_HZ);
    delay(10);
}

static bool hasExternalPullup(uint8_t pin, int& mv) {
    pinMode(pin, INPUT_PULLDOWN);
    delayMicroseconds(1000);
    const bool stillHigh = digitalRead(pin) == HIGH;

    // On the S3 only GPIO1-20 reach an ADC. Asking any other pin for millivolts
    // logs an error and hands back a zero, which reads as a real measurement and
    // contradicts the digital verdict sitting next to it. -1 says "not measured".
    if (pin >= 1 && pin <= 20) {
        analogRead(pin);
        gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);
        delayMicroseconds(500);
        mv = analogReadMilliVolts(pin);
    } else {
        mv = -1;
    }
    pinMode(pin, INPUT_PULLUP);
    return stillHigh;
}

// Sweeps every GPIO that is safe to touch here, looking for the daughter
// board's 10k pull-ups (R10 on STAT, R11 on PG, R12 on CS). Those hang off its
// 3.3V rail, so wherever they turn up is where the board is actually wired --
// which is the question when the expected pins are silent but the board is
// demonstrably powered.
//
// Excluded deliberately: 19/20 are USB, 26-32 are SPI flash, 33-37 are the
// octal PSRAM this build uses, and touching any of those ends the session
// rather than informing it.
void powerSweepPullups() {
    static const uint8_t PINS[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        21, 38, 39, 40, 41, 42, 45, 46, 47, 48
    };
    Serial.println("[Power] Sweeping GPIOs for external pull-ups...");
    for (uint8_t pin : PINS) {
        int mv = 0;
        if (hasExternalPullup(pin, mv)) {
            const char* known = "";
            if (pin == I2C_SDA)      known = "  (PMU bus SDA)";
            else if (pin == I2C_SCL) known = "  (PMU bus SCL)";
            else if (pin == 0)       known = "  (boot button)";
            else if (pin >= 38 && pin <= 40) known = "  (LilyGo SD card)";
            if (mv >= 0) Serial.printf("[Power]   GPIO%-2u pulled up, %4d mV%s\n", pin, mv, known);
            else         Serial.printf("[Power]   GPIO%-2u pulled up (no ADC on this pin)%s\n", pin, known);
        }
    }
    restoreI2CBuses();
    Serial.println("[Power] Sweep done (I2C buses restored).");
}

void powerDumpPullups() {
    // Positive control. The AXP2101 sits on I2C_SDA/I2C_SCL and that bus
    // demonstrably works, so those lines must have real pull-ups. If this test
    // cannot see them, the test is broken and nothing else it says counts.
    int ctlSdaMv = 0, ctlSclMv = 0;
    const bool ctlSda = hasExternalPullup(I2C_SDA, ctlSdaMv);
    const bool ctlScl = hasExternalPullup(I2C_SCL, ctlSclMv);
    Serial.printf("[Power] CONTROL (PMU bus, known good): SDA=%s (%d mV)  SCL=%s (%d mV)%s\n",
                  ctlSda ? "FITTED" : "none", ctlSdaMv,
                  ctlScl ? "FITTED" : "none", ctlSclMv,
                  (ctlSda && ctlScl) ? "" : "   <-- CONTROL FAILED, distrust the rest");

    int statMv = 0, pgMv = 0;
    const bool stat = hasExternalPullup(DB_CHG_STAT, statMv);
    const bool pg   = hasExternalPullup(DB_CHG_PG, pgMv);
    Serial.printf("[Power] R10 on STAT: %s (%d mV)   R11 on PG: %s (%d mV)\n",
                  stat ? "FITTED" : "none", statMv, pg ? "FITTED" : "none", pgMv);
    if (stat || pg) {
        Serial.println("[Power]   -> daughter board is seated AND its 3.3V rail is up.");
    } else {
        Serial.println("[Power]   -> no pull-ups seen: board not seated, or its 3.3V rail is dead.");
    }
    restoreI2CBuses();
}

const char* chargeStateName(ChargeState s) {
    switch (s) {
        case ChargeState::Charging: return "Charging";
        case ChargeState::Idle:     return "Idle";
        case ChargeState::Fault:    return "Fault";
        default:                    return "No Input";
    }
}
