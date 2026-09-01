#include "imu_handler.h"
#include "utilities.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// The AXP2101 owns Wire (I2C_SDA/I2C_SCL). The daughter board lands the LIS3DH
// on a different pin pair entirely -- see the J4 map in utilities.h -- so this
// is the S3's second I2C controller, not a shared bus.
static Adafruit_LIS3DH lis(&Wire1);

static bool    s_present = false;
static uint8_t s_addr    = 0;

// Captured by reportRetainedConfig() on a wake, before anything in this file
// writes to the interrupt block. 0xFF means "not captured this boot".
//
// These exist because the evidence is destroyed almost immediately: imuBegin()
// runs imuEnableMotionWake(), which calls imuClearMotionInterrupt(), and reading
// INT1_SRC is exactly what releases the latch. By the time any caller thinks to
// ask, the answer is gone. So it is taken once, early, and kept.
static uint8_t s_wakeSrc = 0xFF;
static uint8_t s_wakeCfg = 0xFF;

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
static uint8_t lisRead8(uint8_t reg);
static void reportRetainedConfig();
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

// Is the daughter board's 3.3V rail actually driven, or is it floating up
// through the LIS3DH's ESD diodes off our own pull-ups?
//
// A meter cannot tell: a phantom rail charged through the ~45k internal
// pull-ups still reads near 3V with only a voltmeter on it. What gives it away
// is how it recovers. Pull both lines to ground to dump the charge on the
// board's decoupling (C4 10uF + C6 100nF), release, and time the rise. Charging
// 10uF through 45k is a ~0.5s time constant; a rail with a regulator behind it
// snaps back in microseconds.
//
// Read the result narrowly. A fast rise proves little: an unconnected pin rises
// fast because there is nothing to charge, and so does a phantom rail, because
// pulling SDA low reverse-biases the very ESD diode that would have to conduct
// to discharge it. Only a SLOW rise is conclusive, and it says the pull-up is
// charging real capacitance that can only be the daughter board's.
static uint32_t sdaRiseMicros() {
    pinMode(DB_IMU_SDA, OUTPUT); digitalWrite(DB_IMU_SDA, LOW);
    pinMode(DB_IMU_SCL, OUTPUT); digitalWrite(DB_IMU_SCL, LOW);
    delay(250);
    pinMode(DB_IMU_SCL, INPUT_PULLUP);
    const uint32_t t0 = micros();
    pinMode(DB_IMU_SDA, INPUT_PULLUP);
    while (digitalRead(DB_IMU_SDA) == LOW) {
        if (micros() - t0 > 2000000UL) return 0xFFFFFFFFUL;
    }
    return micros() - t0;
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

    const uint32_t riseUs = sdaRiseMicros();
    if (riseUs == 0xFFFFFFFFUL) {
        Serial.println("[IMU] SDA never rose after being pulled low - line held down.");
    } else {
        Serial.printf("[IMU] SDA rise after discharge: %lu us%s\n", (unsigned long)riseUs,
                      riseUs > 10000 ? "  <-- SLOW: charging the board's decoupling, so it IS connected"
                                     : "  (fast: proves little - see comment)");
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

    // Before lis.begin(), because Adafruit's begin() writes CTRL1, CTRL3 and
    // CTRL4 on its way past -- and CTRL3 is one of the registers that carries
    // the answer. s_addr is set early only so lisRead8() has an address; the
    // real assignment still happens below, once the part has identified itself.
    s_addr = found;
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
        reportRetainedConfig();
    }

    if (!lis.begin(found)) {
        Serial.printf("[IMU] 0x%02X ACKed but is not a LIS3DH (WHO_AM_I mismatch)\n", found);
        s_addr = 0; // imuAddress() promises 0 when nothing was found
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

uint8_t imuWakeSrc() { return s_wakeSrc; }
uint8_t imuWakeCfg() { return s_wakeCfg; }

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

// Did the LIS3DH keep its power through deep sleep?
//
// The argument says it must: the daughter board's 3.3V arrives on J4.1, which
// is the LilyGo header's 3V3 = AXP2101 DCDC1 -- the same rail that keeps the
// S3's own RTC domain alive while it sleeps. Nothing in this firmware touches
// DCDC1; modemPowerOff() disables DC3 and BLDO2 only. A rail the ESP32 needs to
// wake up at all cannot be off while it is asleep.
//
// But that is an argument, and "the accelerometer must have been unpowered" is
// a cheap conclusion to reach when a wake does not happen. These registers
// settle it as a measurement. They are volatile and they are not battery
// backed, so a part that lost its rail answers with reset defaults (all zero),
// and a part that did not answers with exactly what imuEnableMotionWake() wrote
// before the sleep. There is no third reading that means "powered but idle".
//
// Only meaningful on a wake from sleep. On a cold boot the defaults are the
// truthful answer and say nothing about the rail, which is why the caller
// checks the wake cause first.
static void reportRetainedConfig() {
    // After an ext1 wake the pad is still routed to the RTC IO mux, so the
    // digital GPIO the Arduino API reads is not what the pin is actually doing.
    // Handing it back first is what makes the level below a real measurement.
    rtc_gpio_deinit((gpio_num_t)DB_IMU_INT1);
    pinMode(DB_IMU_INT1, INPUT);

    const uint8_t c2  = lisRead8(LIS3DH_REG_CTRL2);
    const uint8_t c3  = lisRead8(LIS3DH_REG_CTRL3);
    const uint8_t c5  = lisRead8(LIS3DH_REG_CTRL5);
    const uint8_t cfg = lisRead8(LIS3DH_REG_INT1CFG);
    const uint8_t ths = lisRead8(LIS3DH_REG_INT1THS);
    const uint8_t dur = lisRead8(LIS3DH_REG_INT1DUR);
    const int     pin = digitalRead(DB_IMU_INT1);

    // Read last on purpose: reading INT1_SRC is what releases the latch, so
    // taking it earlier would clear the evidence the line above is reporting.
    const uint8_t src = lisRead8(LIS3DH_REG_INT1SRC);

    s_wakeSrc = src;
    s_wakeCfg = cfg;

    Serial.printf("[IMU] Retained: CTRL2=0x%02X CTRL3=0x%02X CTRL5=0x%02X INT1_CFG=0x%02X "
                  "THS=%u DUR=%u SRC=0x%02X  INT1 pin=%s\n",
                  c2, c3, c5, cfg, ths, dur, src, pin ? "HIGH" : "low");

    const bool armed = (cfg == 0x2A && c3 == 0x40 && c5 == 0x08);
    const bool blank = (cfg == 0x00 && c3 == 0x00 && c5 == 0x00 && ths == 0x00);

    if (armed) {
        Serial.printf("[IMU]   -> config survived the sleep, so the part held its 3.3V rail. "
                      "It was armed at %umg over %u samples (%ums).\n",
                      ths * INT1_THS_STEP_MG, dur, dur * 20);
        // IA is the interrupt-active flag; the low six bits say which axis and
        // direction. Set means this part did fire, whether or not that is what
        // ended the sleep.
        if (src & 0x40) Serial.printf("[IMU]   -> IA set: it latched a motion event (axes 0x%02X).\n", src & 0x3F);
        else            Serial.println("[IMU]   -> IA clear: it never saw motion past the threshold.");
    } else if (blank) {
        Serial.println("[IMU]   -> reset defaults: the part power-cycled during sleep, so nothing was armed.");
        Serial.println("[IMU]      That would be a real rail fault -- 3V3 on J4.1 should not drop while the S3 sleeps.");
    } else {
        Serial.println("[IMU]   -> neither armed nor blank: something else has written the interrupt block.");
    }
}

// One configuration, applied and then asked whether it fires, with the part
// left exactly as it is. Returns the IA flag; pinHigh reports what INT1 was
// doing at the same moment.
//
// The pin is sampled BEFORE INT1_SRC, because reading INT1_SRC is what releases
// the latch -- taking it first would drop the pin and then report it low.
static bool interruptFiresNow(uint8_t ctrl2, uint8_t thsCounts, uint8_t dur,
                              const char* what, bool& pinHigh) {
    lisWrite8(LIS3DH_REG_INT1CFG, 0x00);   // disabled while being reconfigured
    lisWrite8(LIS3DH_REG_CTRL2, ctrl2);
    (void)lisRead8(LIS3DH_REG_REFERENCE);  // HPM=00: reading this resets the filter
    lisWrite8(LIS3DH_REG_CTRL3, 0x40);     // IA1 -> INT1 pin
    lisWrite8(LIS3DH_REG_CTRL5, 0x08);     // latch
    lisWrite8(LIS3DH_REG_INT1THS, thsCounts);
    lisWrite8(LIS3DH_REG_INT1DUR, dur);
    // High events only. Enabling all six was a mistake worth recording: on this
    // part the LOW event is |axis| < THS -- that is what free-fall detection is
    // built from -- so once the filter correctly removes gravity, all three low
    // events become trivially TRUE and the interrupt fires for the very reason
    // it should be silent. The first run of this read a working filter as a
    // broken one on exactly that basis.
    lisWrite8(LIS3DH_REG_INT1CFG, 0x2A);   // ZHIE | YHIE | XHIE, OR'd
    // The filter has to settle BEFORE the latch is armed, not after. At
    // HPCF=00 and 50Hz the corner is ODR/50 = 1Hz, so the time constant is
    // about 160ms and gravity takes the better part of a second to decay out
    // of the filter. Clearing the latch first and then waiting 250ms measured
    // exactly that transient and called it a filter fault.
    delay(1000);                           // ~6 time constants
    (void)lisRead8(LIS3DH_REG_INT1SRC);    // arm only once settled
    delay(250);                            // >10 ODR periods at 50Hz

    pinHigh = (digitalRead(DB_IMU_INT1) == HIGH);
    const uint8_t src = lisRead8(LIS3DH_REG_INT1SRC);
    const bool ia = (src & 0x40) != 0;
    Serial.printf("[IMU]   %-30s CTRL2=0x%02X THS=%2u DUR=%u -> SRC=0x%02X IA=%d pin=%s\n",
                  what, ctrl2, thsCounts, dur, src, ia ? 1 : 0, pinHigh ? "HIGH" : "low");
    return ia;
}

// Deterministic proof of the interrupt generator, needing nothing from whoever
// is at the bench.
//
// Gravity is the stimulus. A part at rest still measures 1g along SOME
// direction, so with the high-pass filter OFF and all six direction events
// enabled, any threshold below 577mg -- the worst case for a 1g vector split
// evenly across three axes -- MUST be exceeded by at least one of them. That
// makes the first test a positive control that cannot legitimately fail, which
// is exactly what a shake test can never be: a shake that produces nothing is
// always ambiguous between a broken part and a feeble shake.
//
// The second test then switches the filter back on and asks the opposite
// question. At rest the high-pass filter should remove gravity entirely, so the
// same threshold must now NOT fire. Together the two bracket the filter: one
// says the generator works, the other says the filter is doing its job.
void imuInterruptSelfTest() {
    if (!s_present) {
        Serial.println("[IMU] No IMU - cannot self-test.");
        return;
    }
    rtc_gpio_deinit((gpio_num_t)DB_IMU_INT1);
    pinMode(DB_IMU_INT1, INPUT);

    Serial.println("\n[IMU] === interrupt self-test (static, no shaking needed) ===");

    bool pinA = false, pinB = false;
    const bool firesNoHpf = interruptFiresNow(0x00, 25, 0, "gravity, HPF off (MUST fire)", pinA);
    const bool firesHpf   = interruptFiresNow(0x01, 25, 0, "at rest, HPF on (must NOT)", pinB);

    if (!firesNoHpf) {
        Serial.println("[IMU] SELF-TEST FAIL: 1g of gravity did not trip a 400mg threshold.");
        Serial.println("[IMU]   The interrupt generator is not working at all. Nothing downstream");
        Serial.println("[IMU]   of it -- filter, duration, wake source -- can be diagnosed until it is.");
    } else if (!pinA) {
        Serial.println("[IMU] SELF-TEST: generator fires, but INT1 never reached GPIO12.");
        Serial.println("[IMU]   That is a wiring fault at J4.12, and no firmware change reaches past it.");
    } else if (firesHpf) {
        Serial.println("[IMU] SELF-TEST: generator and pin are good, but the HPF is NOT removing gravity");
        Serial.println("[IMU]   -- a high event still trips at rest, so the wake threshold is being");
        Serial.println("[IMU]   compared against a DC bias rather than against motion.");
    } else {
        Serial.println("[IMU] SELF-TEST PASS: generator fires on gravity, INT1 reaches GPIO12, and the");
        Serial.println("[IMU]   HPF removes gravity at rest. Generator, filter and wire are all good,");
        Serial.println("[IMU]   so a shake that does not wake it is the DURATION or the threshold.");
    }
}

// What does the interrupt generator actually see?
//
// Everything above infers the filtered signal from whether a comparator tripped,
// which is a one-bit answer to a question that deserves numbers. FDS routes the
// high-pass filter into the OUTPUT registers as well, so with it set the ordinary
// data path reports precisely the signal the interrupt path is working on.
//
// The shipping configuration deliberately leaves FDS clear, because imuRead()
// needs the gravity vector for orientation. That is the right default and the
// wrong thing for a diagnostic, so this sets it, measures, and puts it back.
//
// At rest every axis should read near zero: that IS the filter removing gravity.
// Anything near +/-1000mg means it is not, and the wake threshold is being
// compared against a DC bias rather than against motion.
void imuDumpFilteredData(uint32_t ms) {
    if (!s_present) {
        Serial.println("[IMU] No IMU - cannot dump filtered data.");
        return;
    }

    lisWrite8(LIS3DH_REG_INT1CFG, 0x00);   // nothing should latch during this
    lisWrite8(LIS3DH_REG_CTRL2, 0x09);     // FDS | HPIS1: filter the outputs too
    (void)lisRead8(LIS3DH_REG_REFERENCE);  // HPM=00: resets the filter

    Serial.println("\n[IMU] === filtered output (what the comparator compares) ===");
    Serial.println("[IMU] Settling the filter for 1s, then sampling. At rest this should be ~0/0/0.");
    delay(1000);

    float peak = 0;
    const uint32_t t0 = millis();
    uint32_t lastPrint = 0;
    while (millis() - t0 < ms) {
        sensors_event_t e;
        if (lis.getEvent(&e)) {
            const float fx = e.acceleration.x / SENSORS_GRAVITY_STANDARD * 1000.0f;
            const float fy = e.acceleration.y / SENSORS_GRAVITY_STANDARD * 1000.0f;
            const float fz = e.acceleration.z / SENSORS_GRAVITY_STANDARD * 1000.0f;
            const float mx = fmaxf(fabsf(fx), fmaxf(fabsf(fy), fabsf(fz)));
            if (mx > peak) peak = mx;
            const uint32_t el = millis() - t0;
            if (el - lastPrint >= 500) {
                lastPrint = el;
                Serial.printf("[IMU]   +%4lums  filtered x=%+6.0f y=%+6.0f z=%+6.0f mg   (peak |axis| %.0fmg)\n",
                              (unsigned long)el, fx, fy, fz, peak);
            }
        }
        delay(20);
    }

    Serial.printf("[IMU] Largest single-axis filtered value seen: %.0fmg\n", peak);
    Serial.println("[IMU]   Near zero at rest = the HPF is doing its job.");
    Serial.println("[IMU]   Near 1000mg at rest = it is not, and gravity is sitting on the wake path.");

    lisWrite8(LIS3DH_REG_CTRL2, 0x01);     // FDS back off; imuRead() needs gravity
    (void)lisRead8(LIS3DH_REG_REFERENCE);
}

// Live proof of the interrupt path, without waiting on a sleep cycle.
//
// Worth having because the shipping configuration deliberately rejects the
// obvious bench test. INT1_DURATION is counted in ODR periods, so the default
// three samples at 50Hz demand 60ms of continuous over-threshold motion, and
// normal mode band-limits to about ODR/2 = 25Hz before the comparator ever sees
// the signal. A finger tap is a few milliseconds of mostly high-frequency
// energy: it is filtered down and it is over long before the duration counter
// gets there.
//
// The reason this watches the PIN and the chip's own latch separately is that
// they fail independently, and a test that only watches one cannot tell which
// happened. LIR_INT1 is set, so a latched event is a LEVEL and not a pulse --
// once IA sets, INT1 stays high until INT1_SRC is read. That is what makes a
// slow poll sufficient and what makes the two readings decisive together:
//
//   pin went high              -> chip fired and the wire carried it. Path good.
//   pin low but IA latched     -> chip fired, GPIO12 never saw it. Wiring fault.
//   nothing, motion under THS  -> the test was too gentle to prove anything.
//   nothing, motion over THS   -> the interrupt block is not behaving as armed.
//
// INT1_SRC is read once, at the end, for exactly this reason: reading it is what
// releases the latch, so polling it during the watch would erase the evidence
// the pin is being watched for.
MotionWatchResult imuWatchMotionInterrupt(uint32_t ms) {
    MotionWatchResult result;
    if (!s_present) {
        Serial.println("[IMU] No IMU - nothing to watch.");
        return result;
    }

    // After an ext1 wake the pad is still routed to the RTC IO mux, so the
    // digital GPIO the Arduino API reads is not what the pin is actually doing.
    rtc_gpio_deinit((gpio_num_t)DB_IMU_INT1);
    pinMode(DB_IMU_INT1, INPUT);

    const uint8_t c1  = lisRead8(LIS3DH_REG_CTRL1);
    const uint8_t c2  = lisRead8(LIS3DH_REG_CTRL2);
    const uint8_t c3  = lisRead8(LIS3DH_REG_CTRL3);
    const uint8_t c5  = lisRead8(LIS3DH_REG_CTRL5);
    const uint8_t c6  = lisRead8(0x25); // CTRL_REG6, for INT polarity
    const uint8_t cfg = lisRead8(LIS3DH_REG_INT1CFG);
    const uint8_t ths = lisRead8(LIS3DH_REG_INT1THS);
    const uint8_t dur = lisRead8(LIS3DH_REG_INT1DUR);
    const uint16_t thsMg = ths * INT1_THS_STEP_MG;

    Serial.printf("[IMU] Armed: CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL5=0x%02X CTRL6=0x%02X\n",
                  c1, c2, c3, c5, c6);
    Serial.printf("[IMU]        INT1_CFG=0x%02X THS=%u (%umg) DUR=%u (%ums at 50Hz)\n",
                  cfg, ths, thsMg, dur, dur * 20);

    // Every one of these is a way for the arm to be silently wrong, and each
    // says which register to look at rather than just that something is off.
    if (c3 != 0x40) Serial.println("[IMU]   !! CTRL3 is not 0x40: IA1 is not routed to the INT1 pin.");
    if (cfg == 0x00) Serial.println("[IMU]   !! INT1_CFG is 0x00: no axis is enabled, so nothing can ever fire.");
    if (ths == 0x00) Serial.println("[IMU]   !! INT1_THS is 0: the threshold is degenerate.");
    if (c6 & 0x02)  Serial.println("[IMU]   !! CTRL6 INT_POLARITY set: INT1 is ACTIVE LOW, but ext1 waits for HIGH.");
    if ((c1 & 0xF0) == 0) Serial.println("[IMU]   !! CTRL1 ODR bits clear: the part is powered down and samples nothing.");

    imuClearMotionInterrupt();
    delay(5);
    Serial.printf("[IMU] INT1 pin after clearing the latch: %s\n",
                  digitalRead(DB_IMU_INT1) == HIGH ? "HIGH  <-- stuck; it should fall when the latch is released"
                                                   : "low (correct)");

    Serial.printf("\n[IMU] Shake the board hard for %lus -- a tap is too short to count.\n",
                  (unsigned long)(ms / 1000));

    bool  pinEverHigh  = false;
    float peakDevMg    = 0;
    float windowPeakMg = 0;
    uint32_t lastReport = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (!pinEverHigh && digitalRead(DB_IMU_INT1) == HIGH) {
            pinEverHigh = true;
            Serial.printf("[IMU]   +%.1fs  *** INT1 went HIGH ***\n", (millis() - t0) / 1000.0f);
        }
        // Deviation of the whole vector from 1g. The comparator actually works
        // per-axis on the high-passed signal, so this is a proxy rather than the
        // same number -- but it is measured from the same part through the same
        // bus, and it is enough to separate "too gentle" from "not working".
        sensors_event_t e;
        if (lis.getEvent(&e)) {
            const float gx = e.acceleration.x / SENSORS_GRAVITY_STANDARD;
            const float gy = e.acceleration.y / SENSORS_GRAVITY_STANDARD;
            const float gz = e.acceleration.z / SENSORS_GRAVITY_STANDARD;
            const float dev = fabsf(sqrtf(gx * gx + gy * gy + gz * gz) - 1.0f) * 1000.0f;
            if (dev > peakDevMg)    peakDevMg = dev;
            if (dev > windowPeakMg) windowPeakMg = dev;
        }

        // Live feedback, because the first run of this failed for the dullest
        // possible reason: the window opened and closed while nobody was
        // shaking, and 33mg of bench noise is indistinguishable in the summary
        // from a real attempt that was simply too gentle. Reporting as it goes
        // turns "shake harder" from advice into a number to aim at.
        const uint32_t elapsed = millis() - t0;
        if (elapsed - lastReport >= 2000) {
            lastReport = elapsed;
            Serial.printf("[IMU]   +%2lus  peak %4.0fmg  %s\n",
                          (unsigned long)(elapsed / 1000), windowPeakMg,
                          windowPeakMg >= thsMg ? "<-- OVER THRESHOLD" : "(need more)");
            windowPeakMg = 0;
        }
        delay(10);
    }

    const uint8_t src = lisRead8(LIS3DH_REG_INT1SRC); // reading it releases the latch
    result.latched     = (src & 0x40) != 0;
    result.pinHigh     = pinEverHigh;
    result.peakMg      = peakDevMg;
    result.thresholdMg = thsMg;

    Serial.printf("\n[IMU] Peak ||a|-1g| during the watch: %.0fmg (threshold %umg)\n", peakDevMg, thsMg);
    Serial.printf("[IMU] INT1_SRC=0x%02X  IA=%d  pin ever high: %s\n",
                  src, (src >> 6) & 1, pinEverHigh ? "YES" : "no");

    if (pinEverHigh) {
        Serial.println("[IMU] VERDICT: path works end to end -- the part fired and GPIO12 saw it.");
        Serial.println("[IMU]   A deep-sleep ext1 wake on this pin should work.");
    } else if (src & 0x40) {
        Serial.println("[IMU] VERDICT: the LIS3DH DID latch an event and GPIO12 never saw it.");
        Serial.println("[IMU]   The fault is between U2.11 and the ESP32 -- J4.12 open, or the pin held");
        Serial.println("[IMU]   by something else. No firmware change can make ext1 wake work through that.");
    } else if (peakDevMg < thsMg) {
        Serial.println("[IMU] VERDICT: nothing fired, but the motion never reached the threshold either.");
        Serial.println("[IMU]   Inconclusive -- shake harder and re-run before drawing any conclusion.");
    } else {
        Serial.println("[IMU] VERDICT: motion exceeded the threshold and nothing latched at all.");
        Serial.println("[IMU]   The interrupt block is not behaving as its registers claim; suspect config,");
        Serial.println("[IMU]   not wiring, and start from the CTRL values printed above.");
    }
    return result;
}

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

    // Let the filter settle BEFORE releasing the latch.
    //
    // Switching the high-pass filter on does not remove gravity instantly: at
    // HPCF=00 and 50Hz the corner is ODR/50 = 1Hz, so the time constant is about
    // 160ms and the full 1g takes the better part of a second to decay out. Clear
    // the latch during that window and the residual DC trips the threshold
    // immediately, leaving INT1 asserted on a board that is not moving.
    //
    // Measured, not theorised: a build that armed and then slept at once woke on
    // ext1 five times out of six while sitting still on the bench. It does not
    // bite the normal cycle, where arming happens in setup() and minutes of GPS
    // and modem work follow -- but updateMotionThreshold() re-arms immediately
    // before goToSleep(), and that path would spuriously wake every time the
    // threshold changed.
    delay(1000);
    imuClearMotionInterrupt();

    Serial.printf("[IMU] Motion wake armed: %umg (THS=%u), %u samples (filter settled)\n",
                  ths * INT1_THS_STEP_MG, ths, durationSamples);
    return true;
}

// Positive control for ext1 itself, needing no motion and nobody present.
//
// Switching the high-pass filter OFF puts the raw 1g gravity vector back on the
// interrupt path, and a threshold below 577mg is then exceeded by at least one
// axis no matter how the board is lying. With the latch enabled, INT1 goes HIGH
// and STAYS high -- so the board enters deep sleep with its ext1 source already
// asserted, and ESP_EXT1_WAKEUP_ANY_HIGH is a level trigger.
//
// That makes the outcome binary and free of the sensor entirely: the board must
// wake immediately. If it instead sleeps out its full timer with INT1 held high
// the whole while, ext1 on this pin does not work, and no amount of tuning the
// accelerometer will ever matter.
//
// Returns the INT1 pin level actually achieved, so a control that failed to set
// itself up is not mistaken for a wake-source fault.
bool imuForceInterruptHigh() {
    if (!s_present) return false;

    rtc_gpio_deinit((gpio_num_t)DB_IMU_INT1);
    pinMode(DB_IMU_INT1, INPUT);

    lisWrite8(LIS3DH_REG_INT1CFG, 0x00);   // disabled while reconfiguring
    lisWrite8(LIS3DH_REG_CTRL2, 0x00);     // HPF OFF: gravity back on the path
    lisWrite8(LIS3DH_REG_CTRL3, 0x40);     // IA1 -> INT1
    lisWrite8(LIS3DH_REG_CTRL5, 0x08);     // latch
    lisWrite8(LIS3DH_REG_INT1THS, 25);     // 400mg, under the 577mg worst case
    lisWrite8(LIS3DH_REG_INT1DUR, 0);
    lisWrite8(LIS3DH_REG_INT1CFG, 0x2A);   // high events only
    (void)lisRead8(LIS3DH_REG_INT1SRC);    // release; gravity re-latches at once
    delay(100);

    const bool high = (digitalRead(DB_IMU_INT1) == HIGH);
    const uint8_t src = lisRead8(LIS3DH_REG_INT1SRC);
    // Reading INT1_SRC just released the latch, so re-latch before sleeping.
    delay(50);
    const bool stillHigh = (digitalRead(DB_IMU_INT1) == HIGH);

    Serial.printf("[IMU] ext1 control: INT1=%s (SRC=0x%02X), after re-latch=%s\n",
                  high ? "HIGH" : "low", src, stillHigh ? "HIGH" : "low");
    if (!stillHigh) {
        Serial.println("[IMU]   Control did NOT hold INT1 high -- the control itself failed, ignore the wake.");
    }
    return stillHigh;
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
