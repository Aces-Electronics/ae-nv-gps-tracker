#include "imu_handler.h"
#include "utilities.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// The AXP2101 owns Wire (I2C_SDA/I2C_SCL). The daughter board lands the LIS3DH
// on a different pin pair entirely -- see the J4 map in utilities.h -- so this
// is the S3's second I2C controller, not a shared bus.
static Adafruit_LIS3DH lis(&Wire1);

static bool    s_present = false;
static uint8_t s_addr    = 0;

// Gravity is 1g, so 0.7g on Z is a 45-degree cone around each pole. Outside
// both cones the board is closer to edge-on than to either face.
static const float FLAT_THRESHOLD_G = 0.7f;

// Enough samples to average out hull slap and engine vibration without holding
// up the wake. At 50 Hz a fresh sample lands every 20 ms, so this is a little
// over half a second of data -- set against a 3s downlink wait and a
// five-minute cycle, that is not worth optimising.
static const int SAMPLE_COUNT    = 32;
static const int SAMPLE_DELAY_MS = 20;

// Probing for an ACK rather than calling lis.begin() twice: begin() failing
// part-way leaves the library holding a device object for the wrong address,
// and recovering from that is an implementation detail of a third-party lib
// rather than something this code should depend on.
static bool i2cAck(uint8_t addr) {
    Wire1.beginTransmission(addr);
    return Wire1.endTransmission() == 0;
}

bool imuBegin() {
    Wire1.begin(DB_IMU_SDA, DB_IMU_SCL);

    // SDO/SA0 is left floating on the daughter board, so the address is
    // whichever way that pin happens to sit. Try both rather than guessing.
    const uint8_t candidates[] = { DB_IMU_ADDR_LOW, DB_IMU_ADDR_HIGH };
    uint8_t found = 0;
    for (uint8_t addr : candidates) {
        if (i2cAck(addr)) {
            found = addr;
            break;
        }
    }

    if (!found) {
        // Both plausible addresses are silent. On this board revision the most
        // likely cause is not a missing daughter board: R8/R9 pull SDA/SCL down
        // to GND instead of up to 3.3V, so the bus never idles high and no
        // device can ACK. Says so here because it is not visible from the bus.
        Serial.println("[IMU] No ACK on 0x18 or 0x19 - board absent, or SDA/SCL not idling high.");
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

    if (r.z >= FLAT_THRESHOLD_G) {
        r.orientation = Orientation::Flat;
    } else if (r.z <= -FLAT_THRESHOLD_G) {
        r.orientation = Orientation::UpsideDown;
    } else {
        r.orientation = Orientation::Vertical;
    }

    Serial.printf("[IMU] x=%.2fg y=%.2fg z=%.2fg (%d/%d samples) -> %s\n",
                  r.x, r.y, r.z, taken, SAMPLE_COUNT, orientationName(r.orientation));
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

const char* orientationName(Orientation o) {
    switch (o) {
        case Orientation::Flat:       return "Flat";
        case Orientation::Vertical:   return "Vertical";
        case Orientation::UpsideDown: return "Upside Down";
        default:                      return "Unknown";
    }
}
