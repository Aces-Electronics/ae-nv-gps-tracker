#include "imu_handler.h"
#include "utilities.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "driver/gpio.h"

// The AXP2101 owns Wire (I2C_SDA/I2C_SCL). The daughter board lands the LIS3DH
// on a different pin pair entirely -- see the J4 map in utilities.h -- so this
// is the S3's second I2C controller, not a shared bus.
static Adafruit_LIS3DH lis(&Wire1);

static bool    s_present = false;
static uint8_t s_addr    = 0;

// Gravity is 1g, so 0.7g is a 45-degree cone around each pole. Outside both
// cones the board is closer to edge-on than to either face.
static const float FLAT_THRESHOLD_G = 0.7f;

// How the LIS3DH sits once the daughter board is mounted the way it goes into
// the ski, from a four-position tilt test on the bench. Held level it reads
// x=+0.04 y=-1.00 z=-0.03 at |v|=1.001g, and tilting gives:
//
//   roll right (stbd down)   x=+0.18 y=-0.79 z=-0.60   38 deg  -> -Z
//   roll left  (port down)   x=-0.01 y=-0.72 z=+0.72   44 deg  -> +Z
//   pitch fwd  (bow down)    x=+0.78 y=-0.67 z=+0.05   48 deg  -> +X
//   pitch back (bow up)      x=-0.64 y=-0.76 z=-0.05   40 deg  -> -X
//
// X moved at most 0.18 during the roll tests and Z at most 0.06 during the
// pitch tests, so the chip's axes line up with the hull's well enough to read
// one per motion. An accelerometer at rest reads +1g along whichever axis
// points up, so the measured vector points UP and the mapping is:
//
//   chip +X = aft        chip +Y = down      chip +Z = starboard
//
// These three functions are the whole of what this firmware knows about how the
// board is mounted. A change of mounting changes them and nothing else.
static float upComponent(float x, float y, float z)   { (void)x; (void)z; return -y; }
static float fwdComponent(float x, float y, float z)  { (void)y; (void)z; return -x; }
static float stbdComponent(float x, float y, float z) { (void)x; (void)y; return  z; }

// Enough samples to average out hull slap and engine vibration without holding
// up the wake. At 50 Hz a fresh sample lands every 20 ms, so this is a little
// over half a second of data -- set against a 3s downlink wait and a
// five-minute cycle, that is not worth optimising.
static const int SAMPLE_COUNT    = 32;
static const int SAMPLE_DELAY_MS = 20;

// The first transaction after Wire1.begin() does not reliably get an ACK on
// this board. With no pull-ups fitted the bus is held up by ~45k internals, and
// the very first START goes out before the line has properly settled. The proof
// is in the boot log that prompted this: the probe reported no ACK at 0x19, and
// the bus scan immediately afterwards -- same function, same address -- found
// the part at 0x19, because by then a dozen transactions had already run.
//
// So a single silent attempt is not evidence of absence. Retry before
// concluding anything.
static bool i2cAck(uint8_t addr);
static bool i2cAckRetry(uint8_t addr, int attempts = 4) {
    for (int i = 0; i < attempts; i++) {
        if (i2cAck(addr)) return true;
        delay(2);
    }
    return false;
}

// Probing for an ACK rather than calling lis.begin() twice: begin() failing
// part-way leaves the library holding a device object for the wrong address,
// and recovering from that is an implementation detail of a third-party lib
// rather than something this code should depend on.
static bool i2cAck(uint8_t addr) {
    Wire1.beginTransmission(addr);
    return Wire1.endTransmission() == 0;
}

// Voltage on a bus line with the internal pull-up engaged. digitalRead() only
// says "under VIH", which cannot separate a resistive divider from a diode
// clamp from something actively driving. analogRead() attaches the pad to the
// ADC and clears the pull doing so, so the pull is re-asserted afterwards.
//
// Self-checking by design: on an idle healthy line this has to come back near
// the 3.3V rail. If a line digitalRead() calls high reads near zero here, the
// pull is not surviving the ADC attach and every number from this is worthless.
static int lineMilliVoltsWith(uint8_t pin, gpio_pull_mode_t pull) {
    analogRead(pin);
    gpio_set_pull_mode((gpio_num_t)pin, pull);
    delayMicroseconds(500);
    const int mv = analogReadMilliVolts(pin);
    pinMode(pin, INPUT_PULLUP);
    return mv;
}

static int lineMilliVolts(uint8_t pin) {
    return lineMilliVoltsWith(pin, GPIO_PULLUP_ONLY);
}

// Is a real pull-up resistor fitted, or is the line only being held up by the
// ESP32's internal one?
//
// Turning the internal pull-up off is not enough to tell -- a floating line can
// read high by chance. So the internal PULL-DOWN is engaged instead and the line
// is asked to win against it. A fitted 4.7k against the internal ~45k holds the
// node near 3.0V and still reads high; nothing but the internal pull-up loses
// and the line goes low. That is a decisive test rather than a suggestive one.
//
// It matters because it is what decides the bus clock: a fitted pull-up can run
// 100kHz+, the internal one cannot.
static bool externalPullupPresent(uint8_t pin, int& mv) {
    pinMode(pin, INPUT_PULLDOWN);
    delayMicroseconds(1000);
    const bool stillHigh = digitalRead(pin) == HIGH;
    mv = lineMilliVoltsWith(pin, GPIO_PULLDOWN_ONLY); // pulldown held during the read
    return stillHigh;
}

// Standard I2C bus recovery. A master that resets mid-transaction leaves the
// slave part way through returning a byte: it is still holding SDA low, waiting
// for clocks that never arrive, and the bus is wedged until it gets them. Nine
// pulses on SCL let it finish the byte and release SDA, and a STOP leaves the
// bus in a known state.
//
// Not hypothetical on this board. The boot that prompted this read SDA at 0mV
// and could not drive it high -- indistinguishable from a short -- and then the
// part answered normally, because the address probe had clocked the bus free on
// its way past. Doing it deliberately is better than doing it by accident.
static void i2cBusRecover() {
    pinMode(DB_IMU_SDA, INPUT_PULLUP);
    pinMode(DB_IMU_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(DB_IMU_SCL, HIGH); delayMicroseconds(10);
        digitalWrite(DB_IMU_SCL, LOW);  delayMicroseconds(10);
    }
    // STOP condition: SDA released high while SCL is high.
    digitalWrite(DB_IMU_SCL, HIGH); delayMicroseconds(10);
    pinMode(DB_IMU_SDA, OUTPUT);
    digitalWrite(DB_IMU_SDA, LOW);  delayMicroseconds(10);
    pinMode(DB_IMU_SDA, INPUT_PULLUP); delayMicroseconds(10);
    pinMode(DB_IMU_SCL, INPUT_PULLUP);
}

// Only reached when a line is already stuck low, i.e. the bus is unusable
// either way. Briefly drives the line high to separate a resistor still pulling
// it down -- the S3 wins easily against 10k -- from a hard short to ground,
// which it cannot move. Arduino's OUTPUT is GPIO_MODE_INPUT_OUTPUT, so the
// read-back is the real pad level and not just the output register.
//
// Sourcing into a dead short for 200us is well inside what the pad tolerates,
// and this never runs on a healthy bus.
static void characteriseStuckLine(uint8_t pin, const char* name) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(200);
    const bool rose = digitalRead(pin) == HIGH;
    pinMode(pin, INPUT_PULLUP);
    // "Cannot be moved" is NOT proof of a short. A slave actively driving the
    // line wins against this pin just as a short does, and on SDA that is the
    // more likely of the two -- which is why recovery is attempted before any
    // of this is believed.
    Serial.printf("[IMU]   %s driven high -> %s: %s\n", name,
                  rose ? "rose" : "STILL LOW",
                  rose ? "a resistor is pulling it down, not a short"
                       : "a slave is holding it, or it is shorted to ground");
}

// Scans the whole 7-bit range. Only run when the LIS3DH did not answer, to
// separate "nothing on this bus" from "something is here but not where or what
// was expected".
static void imuScanBus() {
    Serial.println("[IMU] Scanning bus...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2cAck(addr)) {
            Serial.printf("[IMU]   device at 0x%02X\n", addr);
            found++;
        }
    }
    Serial.printf("[IMU] Scan found %d device(s).\n", found);
}

bool imuBegin() {
    // Checked before the pins are handed to the I2C peripheral. With the
    // internal pull-ups engaged both lines must rest high; if either does not,
    // nothing further can work and the cause is on the board rather than in
    // software. Worth distinguishing, because a stuck line and an absent board
    // look identical from the ACK alone.
    pinMode(DB_IMU_SDA, INPUT_PULLUP);
    pinMode(DB_IMU_SCL, INPUT_PULLUP);
    delayMicroseconds(500);
    const bool sdaIdleHigh = digitalRead(DB_IMU_SDA) == HIGH;
    const bool sclIdleHigh = digitalRead(DB_IMU_SCL) == HIGH;
    const int sdaMv = lineMilliVolts(DB_IMU_SDA);
    const int sclMv = lineMilliVolts(DB_IMU_SCL);
    Serial.printf("[IMU] Bus idle: SDA=%s (%d mV)  SCL=%s (%d mV)\n",
                  sdaIdleHigh ? "high" : "LOW", sdaMv,
                  sclIdleHigh ? "high" : "LOW", sclMv);
    if (!sdaIdleHigh || !sclIdleHigh) {
        Serial.println("[IMU] A line is stuck low - trying bus recovery first.");
        i2cBusRecover();
        delayMicroseconds(500);
        const bool sdaNow = digitalRead(DB_IMU_SDA) == HIGH;
        const bool sclNow = digitalRead(DB_IMU_SCL) == HIGH;
        Serial.printf("[IMU] After recovery: SDA=%s SCL=%s\n",
                      sdaNow ? "high" : "LOW", sclNow ? "high" : "LOW");
        // Only worth characterising what recovery could not fix; a line freed by
        // clocking was a wedged slave, which these tests would misreport.
        if (!sdaNow) characteriseStuckLine(DB_IMU_SDA, "SDA");
        if (!sclNow) characteriseStuckLine(DB_IMU_SCL, "SCL");
    }

    int sdaPdMv = 0, sclPdMv = 0;
    const bool sdaExt = externalPullupPresent(DB_IMU_SDA, sdaPdMv);
    const bool sclExt = externalPullupPresent(DB_IMU_SCL, sclPdMv);
    Serial.printf("[IMU] External pull-ups: SDA=%s (%d mV vs internal pulldown)  SCL=%s (%d mV)\n",
                  sdaExt ? "FITTED" : "none", sdaPdMv, sclExt ? "FITTED" : "none", sclPdMv);
    if (sdaExt && sclExt) {
        Serial.printf("[IMU] Real pull-ups on both lines - %u Hz is conservative, this bus could run faster.\n",
                      (unsigned)DB_I2C_HZ);
    }

    Wire1.begin(DB_IMU_SDA, DB_IMU_SCL, DB_I2C_HZ);
    // Weak pull-ups mean the lines take real time to reach idle after the
    // peripheral takes the pins. Cheap insurance against probing into a bus
    // that has not finished rising.
    delay(10);

    // SDO/SA0 floats on this board and the LIS3DH pulls that pin up internally,
    // so 0x19 is the expected answer -- see the note in utilities.h. 0x18 is
    // still tried, because a later board revision may strap it and this should
    // not need editing when that happens.
    const uint8_t candidates[] = { DB_IMU_ADDR_HIGH, DB_IMU_ADDR_LOW };
    uint8_t found = 0;
    for (uint8_t addr : candidates) {
        if (i2cAckRetry(addr)) {
            found = addr;
            break;
        }
    }

    if (!found) {
        Serial.println("[IMU] No ACK at 0x19 or 0x18.");
        imuScanBus();
        return false;
    }

    if (!lis.begin(found)) {
        Serial.printf("[IMU] 0x%02X ACKed but is not a LIS3DH (WHO_AM_I mismatch)\n", found);
        return false;
    }

    // +/-2g keeps the most resolution on the gravity vector, which is all this
    // is asked for today. Impact detection would want a wider range.
    lis.setRange(LIS3DH_RANGE_2_G);
    lis.setDataRate(LIS3DH_DATARATE_50_HZ);

    s_present = true;
    s_addr    = found;
    Serial.printf("[IMU] LIS3DH online at 0x%02X\n", s_addr);
    return true;
}

bool imuPresent() {
    return s_present;
}

uint8_t imuAddress() {
    return s_addr;
}

ImuReading imuRead() {
    ImuReading r;
    if (!s_present) return r;

    float sx = 0, sy = 0, sz = 0;
    int taken = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sensors_event_t e;
        if (lis.getEvent(&e)) {
            // getEvent() reports m/s^2. Gravity is the unit that makes
            // FLAT_THRESHOLD_G readable, so convert once here.
            sx += e.acceleration.x / SENSORS_GRAVITY_STANDARD;
            sy += e.acceleration.y / SENSORS_GRAVITY_STANDARD;
            sz += e.acceleration.z / SENSORS_GRAVITY_STANDARD;
            taken++;
        }
        delay(SAMPLE_DELAY_MS);
    }

    if (taken == 0) {
        Serial.println("[IMU] Answered at init but returned no samples.");
        return r;
    }

    r.valid = true;
    r.x = sx / taken;
    r.y = sy / taken;
    r.z = sz / taken;

    r.up   = upComponent(r.x, r.y, r.z);
    r.fwd  = fwdComponent(r.x, r.y, r.z);
    r.stbd = stbdComponent(r.x, r.y, r.z);

    const float up = r.up;
    if (up >= FLAT_THRESHOLD_G) {
        r.orientation = Orientation::Flat;
    } else if (up <= -FLAT_THRESHOLD_G) {
        r.orientation = Orientation::UpsideDown;
    } else {
        r.orientation = Orientation::Vertical;
    }

    Serial.printf("[IMU] up=%+.2f fwd=%+.2f stbd=%+.2f g  (raw %.2f/%.2f/%.2f, %d/%d) -> %s\n",
                  r.up, r.fwd, r.stbd, r.x, r.y, r.z,
                  taken, SAMPLE_COUNT, orientationName(r.orientation));
    return r;
}

// Adafruit's driver keeps writeRegister8/readRegister8 private and offers no
// activity-interrupt API -- only click detection, which is a different feature.
// The interrupt block is therefore driven straight over the bus this file
// already owns.
static void lisWrite8(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(s_addr);
    Wire1.write(reg);
    Wire1.write(value);
    Wire1.endTransmission();
}

static uint8_t lisRead8(uint8_t reg) {
    Wire1.beginTransmission(s_addr);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    if (Wire1.requestFrom((uint8_t)s_addr, (uint8_t)1) != 1) return 0;
    return Wire1.read();
}

// At +/-2g the INT1_THS step is 16mg.
static const uint16_t INT1_THS_STEP_MG = 16;

bool imuEnableMotionWake(uint16_t thresholdMg, uint8_t durationSamples) {
    if (!s_present) return false;

    // High-pass filter onto the interrupt path only. FDS stays clear on purpose:
    // setting it would filter the output registers too, and imuRead() needs the
    // gravity vector those still carry.
    lisWrite8(LIS3DH_REG_CTRL2, 0x01); // HPIS1
    (void)lisRead8(LIS3DH_REG_REFERENCE);

    lisWrite8(LIS3DH_REG_CTRL3, 0x40); // I1_IA1 -> INT1 pin
    lisWrite8(LIS3DH_REG_CTRL5, 0x08); // LIR_INT1: latch until INT1_SRC is read

    uint8_t ths = thresholdMg / INT1_THS_STEP_MG;
    if (ths == 0) ths = 1;      // 0 would fire on nothing at all
    if (ths > 0x7F) ths = 0x7F; // register is 7-bit
    lisWrite8(LIS3DH_REG_INT1THS, ths);
    lisWrite8(LIS3DH_REG_INT1DUR, durationSamples);

    // OR of the three high-event axes: movement along any axis counts. AOI and
    // 6D stay clear, which is what makes this OR rather than AND-of-all.
    lisWrite8(LIS3DH_REG_INT1CFG, 0x2A); // ZHIE | YHIE | XHIE

    imuClearMotionInterrupt();

    Serial.printf("[IMU] Motion wake armed: %umg (THS=%u), %u samples\n",
                  ths * INT1_THS_STEP_MG, ths, durationSamples);
    return true;
}

void imuClearMotionInterrupt() {
    if (!s_present) return;
    (void)lisRead8(LIS3DH_REG_INT1SRC);
}

void imuDumpState() {
    if (!s_present) {
        Serial.println("[IMU] not present");
    } else {
        const uint8_t wai = lisRead8(0x0F); // WHO_AM_I
        const uint8_t c1  = lisRead8(LIS3DH_REG_CTRL1);
        const uint8_t c4  = lisRead8(LIS3DH_REG_CTRL4);
        Serial.printf("[IMU] addr=0x%02X WHO_AM_I=0x%02X (expect 0x33) CTRL1=0x%02X CTRL4=0x%02X\n",
                      s_addr, wai, c1, c4);
        if (wai == 0x00) {
            Serial.println("[IMU]   WHO_AM_I is 0x00 - the bus is reading back zeros, not a configured part.");
        } else if (wai == 0x33 && (c1 & 0xF0) == 0x00) {
            Serial.println("[IMU]   ODR bits clear - the part is in power-down, so it has reset since init.");
        }
    }

    // Taking the pins for the ADC removes them from the I2C peripheral, so the
    // bus is reopened afterwards.
    Wire1.end();
    pinMode(DB_IMU_SDA, INPUT_PULLUP);
    pinMode(DB_IMU_SCL, INPUT_PULLUP);
    delayMicroseconds(500);
    const int sdaMv = lineMilliVolts(DB_IMU_SDA);
    const int sclMv = lineMilliVolts(DB_IMU_SCL);
    Serial.printf("[IMU] bus now: SDA=%d mV  SCL=%d mV\n", sdaMv, sclMv);
    Wire1.begin(DB_IMU_SDA, DB_IMU_SCL, DB_I2C_HZ);
    delay(10);
}

const char* orientationName(Orientation o) {
    switch (o) {
        case Orientation::Flat:       return "Flat";
        case Orientation::Vertical:   return "Vertical";
        case Orientation::UpsideDown: return "Upside Down";
        default:                      return "Unknown";
    }
}
