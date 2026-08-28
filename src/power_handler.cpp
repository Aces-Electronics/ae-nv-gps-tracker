#include "power_handler.h"
#include "utilities.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

// Kept in RTC memory so the hysteresis band below has a previous decision to
// hold onto across a sleep. A cold boot initialises it to true, which matches
// what the 1.8M pull-up on SUPPLY_EN does on an unprogrammed board: charging
// allowed until something decides otherwise.
RTC_DATA_ATTR static bool s_supplyEnabled = true;

// A 1S LiPo is full at 4.2V. These thresholds are not a substitute for the
// charger's own termination -- the BQ25176J handles that -- they decide whether
// it is worth drawing from the jetski at all.
static const float CHARGE_STOP_V   = 4.10f;
static const float CHARGE_RESUME_V = 3.90f;

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
    if (!steadyLow && !steadyHigh) {
        ps.state = ChargeState::Fault;
    } else if (!ps.supplyPresent) {
        ps.state = ChargeState::NoInput;
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
static bool hasExternalPullup(uint8_t pin, int& mv) {
    pinMode(pin, INPUT_PULLDOWN);
    delayMicroseconds(1000);
    const bool stillHigh = digitalRead(pin) == HIGH;
    analogRead(pin);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);
    delayMicroseconds(500);
    mv = analogReadMilliVolts(pin);
    pinMode(pin, INPUT_PULLUP);
    return stillHigh;
}

void powerDumpPullups() {
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
}

const char* chargeStateName(ChargeState s) {
    switch (s) {
        case ChargeState::Charging: return "Charging";
        case ChargeState::Idle:     return "Idle";
        case ChargeState::Fault:    return "Fault";
        default:                    return "No Input";
    }
}
