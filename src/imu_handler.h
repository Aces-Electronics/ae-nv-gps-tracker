#pragma once

#include <Arduino.h>

// The vocabulary is fixed by the telemetry contract: the README documents an
// "orientation" field carrying exactly these strings, and the web UI shows it
// as a label rather than parsing it. Unknown is what goes out when the IMU did
// not answer, so the field stays present and typed either way.
enum class Orientation {
    Unknown,
    Flat,
    Vertical,
    UpsideDown,
};

struct ImuReading {
    bool  valid = false;
    float x = 0, y = 0, z = 0;       // raw chip axes, g, gravity included

    // The same vector in the vehicle's frame, so nothing downstream has to know
    // how the board is mounted. At rest these read the direction that is UP:
    // sitting level, up = +1g and fwd = stbd = 0.
    float up = 0, fwd = 0, stbd = 0;

    Orientation orientation = Orientation::Unknown;
};

// Brings up the LIS3DH on the daughter board's I2C bus. Returns false when
// nothing answers, which is deliberately not fatal: this product's job is to
// report a position, and a missing or faulty daughter board must not be able to
// stop that happening.
bool imuBegin();

// True once imuBegin() has found a device.
bool imuPresent();

// The address imuBegin() settled on, or 0 if none. Worth logging: the board
// leaves SDO/SA0 floating, so which of the two it answers on is not fixed.
uint8_t imuAddress();

// INT1_SRC and INT1_CFG as they were on this boot, sampled before anything
// rewrote them. 0xFF means they were not captured (a cold boot, where there is
// nothing to retain). SRC bit 6 is IA: set means the part latched an event while
// the ESP32 was asleep, which is the difference between "the accelerometer never
// fired" and "it fired and the wake source did not act on it".
uint8_t imuWakeSrc();
uint8_t imuWakeCfg();

// Averages a short burst of samples and classifies the result. Safe to call
// with no IMU present -- returns a reading with valid == false.
ImuReading imuRead();

// Configures INT1 to assert on sustained movement, for use as a deep-sleep wake
// source. The high-pass filter is enabled on the interrupt path only, so the
// threshold is measured against change rather than against gravity -- without
// it a 1g DC bias would sit above any useful threshold forever.
//
// thresholdMg is rounded to the LIS3DH's 16mg step (at +/-2g). durationSamples
// is how long the condition must hold, counted in output-data-rate periods --
// at the 50Hz set by imuBegin(), one sample is 20ms.
//
// durationSamples is 1 rather than 3 because 3 was measured to be far too
// strict. The counter demands the condition hold CONTINUOUSLY and resets the
// moment it lapses, so 3 samples means 60ms unbroken. Motion is oscillatory --
// |axis| passes back through zero at every reversal -- so at around 10Hz each
// over-threshold excursion lasts 15-30ms and never reaches 60. Over fourteen
// 30-second sleeps with the board being shaken, DUR=3 latched exactly once.
//
// 0 in the end, not 1: at 1 (20ms) the same shaking woke it twice in five
// sleeps, better than 3 but still unreliable. At 10Hz an over-threshold
// excursion lasts 15-30ms, so even 20ms is marginal, and the debounce DUR
// provides is not needed here -- the measured noise floor with the filter
// settled is 24mg against a 352mg threshold, a margin of about 14x. The
// threshold does the discriminating; the duration was only losing events.
//
// Returns false if no IMU is present.
//
// Both values still want tuning against a ski on the water; a hull bobbing at a
// mooring is the case that decides them, and the adaptive ladder in main.cpp
// raises the threshold if this proves too eager.
bool imuEnableMotionWake(uint16_t thresholdMg = 352, uint8_t durationSamples = 0);

// Positive control for the ext1 wake source, independent of any motion: turns
// the high-pass filter off so gravity alone holds INT1 permanently HIGH. A board
// that then sleeps through its whole timer has a broken wake source, not a
// broken accelerometer. Returns whether INT1 was actually left high.
bool imuForceInterruptHigh();

// Reads INT1_SRC, which is what releases the latched interrupt and lets INT1
// fall again. Must happen before re-arming, or the pin is still high and the
// next sleep ends immediately.
void imuClearMotionInterrupt();

// Bench diagnostic: routes the high-pass filter into the output registers (FDS)
// so the filtered signal the interrupt generator compares can be read as
// numbers. Restores CTRL2 on the way out; still call imuEnableMotionWake() after.
void imuDumpFilteredData(uint32_t ms);

// Reads the LIS3DH's auxiliary ADC1 (U2.16), which the daughter board wires to
// the supply-sense divider. Returns the raw signed 16-bit register pair; the
// part is 10-bit and left-justified, so the meaningful value is (raw >> 6).
//
// Deliberately RAW. ST does not document whether the code is inverted, and the
// divider's real ratio depends on resistor tolerance and on an input impedance
// ST does not specify -- so a formula derived from the schematic would be a
// guess wearing a unit. Two known voltages turn this into volts; until then the
// raw count is the honest thing to publish.
//
// Returns false if no IMU answered.
bool imuReadAdc1(int16_t& raw);

// Bench diagnostic: static proof of the interrupt generator, using gravity as
// the stimulus so it needs nobody to shake anything. Leaves the interrupt block
// reconfigured -- call imuEnableMotionWake() afterwards to re-arm.
void imuInterruptSelfTest();

// What one watch window observed. peakMg is the largest ||a|-1g| seen, which is
// how a window that nobody shook is told apart from one that was shaken and
// produced nothing -- the distinction the whole test turns on.
struct MotionWatchResult {
    bool     latched     = false;  // the LIS3DH set IA
    bool     pinHigh     = false;  // INT1 reached GPIO12
    float    peakMg      = 0;
    uint16_t thresholdMg = 0;

    // A window is only worth drawing a conclusion from if something fired or the
    // motion actually cleared the threshold. Anything else is a window nobody
    // shook, and it says nothing about the configuration under test.
    bool conclusive() const { return latched || peakMg >= thresholdMg; }
};

// Bench diagnostic: watches INT1 for a while and reports what happened.
//
// The shipping arm is a sustained-motion detector, not a tap detector -- see the
// note on the definition -- so this exists to find a gesture that does trigger
// it, rather than concluding from a tap that nothing works.
MotionWatchResult imuWatchMotionInterrupt(uint32_t ms);

// Bench diagnostic: what the part says about itself, plus the bus levels.
// WHO_AM_I is the discriminator when readings go to zero -- 0x33 means a real
// LIS3DH that has lost its configuration, 0x00 means the bus is returning
// zeros and the part may not be talking at all.
void imuDumpState();

const char* orientationName(Orientation o);
