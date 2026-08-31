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
// Returns false if no IMU is present. Both defaults are starting points that
// want tuning against a ski on the water; a hull bobbing at a mooring is the
// case that decides them.
bool imuEnableMotionWake(uint16_t thresholdMg = 352, uint8_t durationSamples = 3);

// Reads INT1_SRC, which is what releases the latched interrupt and lets INT1
// fall again. Must happen before re-arming, or the pin is still high and the
// next sleep ends immediately.
void imuClearMotionInterrupt();

// Bench diagnostic: what the part says about itself, plus the bus levels.
// WHO_AM_I is the discriminator when readings go to zero -- 0x33 means a real
// LIS3DH that has lost its configuration, 0x00 means the bus is returning
// zeros and the part may not be talking at all.
void imuDumpState();

const char* orientationName(Orientation o);
