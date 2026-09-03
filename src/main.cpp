#define TINY_GSM_DEBUG Serial
#include <Arduino.h>
#include "utilities.h"
#include "esp_private/esp_clk.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <Preferences.h>
#include "ble_handler.h"
#include "crash_handler.h"
#include "ota_handler.h"
#include "imu_handler.h"
#include "power_handler.h"
#include "can_handler.h"

// --- Configuration ---
TrackerSettings settings;
TrackerStatus status;
Preferences prefs;

bool g_hasCrashLog = false;


// --- Globals ---
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
XPowersAXP2101 PMU;
BLEHandler ble;

String imei = "";
String mqtt_topic_up = "";
String mqtt_topic_dn = "";
String mqtt_topic_cfg;

// How long the MQTT session is held open after publishing, waiting for the
// retained downlink. The broker sends a retained message immediately on SUBACK,
// so this is one round trip's worth of slack, not a poll. It replaces the
// delay(1000) that used to sit here, so the worst case costs ~2s more modem-on
// time per wake -- on a five-minute cycle that is roughly 5 mAh/day at a
// registered-idle 25 mA, and the loop exits the moment something arrives.
static const unsigned long DOWNLINK_WAIT_MS = 3000;

// PubSubClient sizes one buffer for both directions, so this has to hold the
// outgoing telemetry frame and the incoming retained OTA command alike. Named
// because the publish path now checks against it -- see transmitData().
static const uint16_t MQTT_BUFFER_SIZE = 768;

// Cadence follows the ski, not the clock.
//
// A running engine is worth following closely, so that case uses the
// configured report_interval_mins (5 minutes by default, settable over BLE). A
// parked ski has nothing to say between one day and the next, so it gets a
// heartbeat -- proof of life, a battery reading, a position -- and relies on
// the accelerometer to wake it if anything actually happens.
//
// Splitting it this way also means a unit already provisioned with a 5-minute
// interval is correct as it stands: that value now describes the running case,
// which is what it was always meant for.
static const uint32_t HEARTBEAT_DEFAULT_MINS = 5;
// A day is right in the field and useless on a bench: one change takes a day to
// evaluate, and a heartbeat that fails leaves nothing to look at for another.
// Overridable at compile time so the shipping default cannot be shortened by
// accident. It IS currently shortened, deliberately: the default environment in
// platformio.ini overrides this to 60 while the fleet is one dev board. That
// line carries the note about removing it, and this value is what production
// goes back to.
#ifndef PARKED_INTERVAL_MINS_OVERRIDE
#define PARKED_INTERVAL_MINS_OVERRIDE 1440
#endif
static const uint32_t PARKED_INTERVAL_MINS   = PARKED_INTERVAL_MINS_OVERRIDE;

OtaCommand g_otaCmd;
bool g_otaPending = false;

// CI passes the release number in as OTA_VERSION; a bench build has none. An
// empty string is reported as-is rather than as a made-up number, because the
// backend matches an OTA command against exactly this value and a placeholder
// would close commands the device never applied.
const char* firmwareVersion() {
    return FIRMWARE_VERSION;
}

// Three selectable sensitivities, each the FLOOR of the adaptive ladder rather
// than a fixed threshold: the ski still finds its own level from wherever it
// starts. Chosen against a measured noise floor of 24mg (high-pass filter
// settled, board at rest), so the margins are known rather than guessed:
//
//   Low    704mg  ~29x noise  -- takes a deliberate shove. For a ski somewhere
//                               lively, where less would report the weather.
//   Medium 352mg  ~15x noise  -- the shipped default. Registers ordinary handling.
//   High   256mg  ~11x noise  -- for a quiet mooring where a nudge should count.
//
// Doubled from the original 352/176/128 after the first real deployment: a unit
// on Medium riding in a car woke nine times in 65 minutes, and the ladder spent
// the whole hour climbing away from a floor that was simply too low to start
// from. The old Medium is the new Low, which is the honest description of what
// 176mg turned out to be worth.
//
// All three sit exactly on the INT1_THS grid (16mg steps at +/-2g): 44, 22 and 16
// counts, so none of them truncate. That is worth checking when retuning -- 56mg,
// for instance, is not representable and would silently become 48mg, only 2x the
// noise floor and closer to chasing the filter's own residue than detecting
// anything.
static const uint16_t MOTION_SENS_LOW_MG    = 704;
static const uint16_t MOTION_SENS_MEDIUM_MG = 352;
static const uint16_t MOTION_SENS_HIGH_MG   = 256;

enum MotionSensitivity : uint8_t {
    MOTION_SENS_LOW = 0,
    MOTION_SENS_MEDIUM = 1,
    MOTION_SENS_HIGH = 2,
};

static uint16_t motionBaseFor(uint8_t sens) {
    switch (sens) {
        case MOTION_SENS_LOW:  return MOTION_SENS_LOW_MG;
        case MOTION_SENS_HIGH: return MOTION_SENS_HIGH_MG;
        default:               return MOTION_SENS_MEDIUM_MG;
    }
}

const char* motionSensitivityName(uint8_t sens) {
    switch (sens) {
        case MOTION_SENS_LOW:  return "Low";
        case MOTION_SENS_HIGH: return "High";
        default:               return "Medium";
    }
}

// The floor the ladder resets to, for whichever sensitivity is selected. Set by
// loadMotionThreshold() before anything arms the IMU.
uint16_t g_motionBaseMg = MOTION_SENS_MEDIUM_MG;


void loadSettings() {
    prefs.begin("tracker", false);
    settings.name = prefs.getString("name", "");
    settings.apn = prefs.getString("apn", "hologram");
    settings.mqtt_broker = prefs.getString("broker", "mqtt.aceselectronics.com.au");
    settings.mqtt_user = prefs.getString("user", "aesmartshunt");
    settings.mqtt_pass = prefs.getString("pass", "AERemoteAccess2024!");
    settings.report_interval_mins = prefs.getUInt("interval", HEARTBEAT_DEFAULT_MINS);
    if (settings.report_interval_mins < 5) settings.report_interval_mins = 5;

    // Read again here purely so the value is in settings for BLE and telemetry;
    // loadMotionThreshold() has already used it, because arming the IMU happens
    // long before this runs.
    settings.motion_sensitivity = prefs.getUChar("mot_sens", MOTION_SENS_MEDIUM);
    if (settings.motion_sensitivity > MOTION_SENS_HIGH) {
        settings.motion_sensitivity = MOTION_SENS_MEDIUM;
    }

    // getString()'s default only covers a *missing* key. A stored zero-length
    // value comes back as "", and an empty host fails DNS resolution instantly --
    // the uplink then dies with no obvious cause until the unit is re-provisioned.
    // Treat empty as absent so the device self-heals on the next boot.
    settings.mqtt_broker.trim();
    if (settings.mqtt_broker.length() == 0) {
        Serial.println("[Settings] WARN: Stored broker is empty. Falling back to default.");
        settings.mqtt_broker = "mqtt.aceselectronics.com.au";
    }

    // Override for Telstra SIM if default is hologram
    if (settings.apn == "hologram") {
        settings.apn = "telstra.internet";
    }
    
    settings.debug_payload = prefs.getBool("dbg", false);

    Serial.println("Settings Loaded from NVS.");
    Serial.printf("[Settings] Broker: %s\n", settings.mqtt_broker.c_str());
    Serial.printf("[Settings] Name: %s\n", settings.name.c_str());
    Serial.printf("[Settings] Interval: %d mins\n", settings.report_interval_mins);
    Serial.printf("[Settings] Motion sensitivity: %s (%umg floor)\n",
                  motionSensitivityName(settings.motion_sensitivity),
                  motionBaseFor(settings.motion_sensitivity));
    Serial.printf("[Settings] Payload: %s\n",
                  settings.debug_payload ? "DEBUG (verbose)" : "normal (essentials)");
    prefs.end();
}

void saveSettings() {
    prefs.begin("tracker", false);
    prefs.putString("name", settings.name);
    prefs.putString("apn", settings.apn);
    prefs.putString("broker", settings.mqtt_broker);
    prefs.putString("user", settings.mqtt_user);
    prefs.putString("pass", settings.mqtt_pass);
    prefs.putUInt("interval", settings.report_interval_mins);
    prefs.putUChar("mot_sens", settings.motion_sensitivity);
    prefs.putBool("dbg", settings.debug_payload);
    prefs.end();
    Serial.println("Settings Saved to NVS.");
}

void modemPowerOn() {
    Serial.println("[Modem] Init Power...");
    PMU.setDC3Voltage(3400); 
    PMU.enableDC3();
    PMU.setALDO4Voltage(3300);
    PMU.enableALDO4();
    
    // GPS Antenna Power
    PMU.setBLDO2Voltage(3300);
    PMU.enableBLDO2();
    
    // Drain any old data
    while(Serial1.available()) Serial1.read();

    // Recover a modem left at a non-default baud.
    //
    // The OTA raises the link to 921600 for the transfer and lowers it again
    // afterwards, but that restore only runs if the code keeps running. A reset
    // mid-download -- power loss, a crash, a watchdog, someone pulling USB --
    // leaves the modem fast while Serial1 comes back up at 115200, and AT+IPR
    // does not persist, so nothing else puts it right. The symptom is garbage on
    // the UART and "Power FAIL" from a modem that is perfectly healthy.
    //
    // On a fielded tracker that would be unrecoverable: the only remote repair
    // path is an OTA, and an OTA needs the modem. So this is checked on every
    // boot, before concluding anything about the modem's health.
    if (!modem.testAT(1000)) {
        static const uint32_t kRates[] = { 921600, 460800 };
        for (uint32_t rate : kRates) {
            Serial.printf("[Modem] No answer at 115200; trying %lu...\n", (unsigned long)rate);
            Serial1.updateBaudRate(rate);
            delay(100);
            while (Serial1.available()) Serial1.read();
            if (modem.testAT(1500)) {
                Serial.printf("[Modem] Found it at %lu -- restoring 115200.\n", (unsigned long)rate);
                modem.sendAT(GF("+IPR=115200"));
                modem.waitResponse(2000L);
                delay(100);
                Serial1.updateBaudRate(115200);
                delay(150);
                while (Serial1.available()) Serial1.read();
                Serial.printf("[Modem] Back at 115200: %s\n",
                              modem.testAT(2000) ? "OK" : "still no answer");
                break;
            }
        }
        // Whatever happened, carry on at the rate the rest of the firmware uses.
        Serial1.updateBaudRate(115200);
        delay(50);
        while (Serial1.available()) Serial1.read();
    }

    if (!modem.testAT(1000)) {
        Serial.println("[Modem] PWRKEY Pulse...");
        pinMode(MODEM_PWRKEY, OUTPUT);
        digitalWrite(MODEM_PWRKEY, LOW);
        delay(100);
        digitalWrite(MODEM_PWRKEY, HIGH);
        delay(1000);
        digitalWrite(MODEM_PWRKEY, LOW);
        
        unsigned long start = millis();
        while (millis() - start < 20000) {
            if (modem.testAT(500)) break;
            delay(500);
        }
    }
    
    if (modem.testAT(1000)) {
        Serial.println("[Modem] Online.");
        modem.sendAT("+CFUN=1");
        modem.waitResponse();
    } else {
        Serial.println("[Modem] Power FAIL");
    }
}

void modemPowerOff() {
    Serial.println("Powering down modem/GPS...");
    modem.sendAT("+CPOWD=1"); 
    modem.waitResponse(2000L);
    PMU.disableDC3();
    PMU.disableBLDO2(); // GPS Antenna
}

// Set after every successful publish. Lives in RTC memory because the whole
// point is to still be there on the far side of a deep sleep.
//
// time() rather than millis(): the RTC timer keeps counting through deep sleep
// and IDF restores the system clock from it on wake, so this stays monotonic
// across cycles. It is never set from the network, so it is elapsed-time-since-
// first-power-on, not a wall clock -- which is all a rate limit needs.
RTC_DATA_ATTR static time_t s_lastReportTime = 0;

// How soon after a report a motion wake is worth acting on. A hull bobbing at a
// mooring must not be able to spend the battery on repeating itself.
static const time_t MOTION_MIN_REPORT_S = 120;


// --- Adaptive motion threshold -------------------------------------------
//
// The LIS3DH wake threshold in milli-g. It starts sensitive and climbs only
// when the tracker keeps waking for movement that GPS says did not happen --
// wash from a passing boat, wind, someone leaning on the hull. Every step up
// costs sensitivity to real theft, so the ladder is short and it drops straight
// back to the floor the moment genuine travel is seen.
// The ladder's reach, as a multiple of whichever floor the sensitivity setting
// chose. Four, for two reasons that agree: it puts Medium's ceiling on 1408mg,
// which is where the ceiling has been all along, and it is the largest multiple
// that still fits inside the part.
//
// That second reason is not incidental. INT1_THS tops out at 2032mg (7 bits,
// 16mg steps at +/-2g), and once the floors doubled an eightfold reach ran every
// setting into that cap: High, Medium and Low would all have ceilinged at 2032,
// giving reaches of 7.9x, 5.8x and 2.9x. That is the same inversion this scaling
// exists to remove -- the MOST sensitive setting getting the MOST room to deafen
// itself -- reintroduced through the back door by a clamp rather than by a
// constant. Four fits, and the reach comes out even.
static const uint16_t MOTION_CEILING_MULTIPLE = 4;

// Step and ceiling both scale with the selected sensitivity. They used to be
// flat 176 and 1408 -- which are Medium's numbers -- and that made the setting
// a floor and nothing else: a unit on High (128mg floor) could still be walked
// to 1408mg, eleven times what its owner asked for, and a unit on Medium could
// pass 352mg and end up deafer than the LOW setting ever offers. Observed in
// the field at 528mg on Medium, which is what prompted this.
//
//   High    128mg floor ->  1024mg ceiling
//   Medium  176mg floor ->  1408mg ceiling   (unchanged from before)
//   Low     352mg floor ->  2816mg, clamped to 2032mg by the register
//
// Scaling the step too keeps the number of rungs the same at every setting, so
// "three unexplained wakes" costs the same fraction of the range wherever the
// owner put the floor.
static uint16_t motionStepMg() { return g_motionBaseMg; }
static uint16_t motionCeilingMg() {
    uint32_t c = (uint32_t)g_motionBaseMg * MOTION_CEILING_MULTIPLE;
    if (c > IMU_MAX_THRESHOLD_MG) c = IMU_MAX_THRESHOLD_MG;
    return (uint16_t)c;
}

// Consecutive unexplained motion wakes before the threshold goes up. Three,
// because one is noise and two is a coincidence.
static const int FALSE_WAKES_BEFORE_RAISE = 3;

// Above this PDOP a fix says nothing trustworthy about where anything is.
// Measured on this unit: a fix reporting PDOP 25.0 sat 35m from one taken three
// seconds later at PDOP 2.0, with the board stationary on a bench. HDOP on that
// bad fix was a respectable 3.9 -- the error was almost entirely vertical
// (VDOP 24.7) -- which is exactly why this gate is on PDOP and not on the HDOP
// the parser already carried. Believing that pair would have read as 35m of
// travel and reset the threshold to its floor.
static const float FIX_MAX_PDOP = 5.0f;

// How far the tracker must have moved from its reference before that counts as
// travel. Comfortably outside the scatter of a PDOP-gated fix, and well inside
// anything a ski does when it actually goes somewhere.
static const float TRAVEL_DISPLACEMENT_M = 100.0f;

// Where "parked" currently is. Deliberately NOT refreshed on every stationary
// report: holding it still is what lets a ski moved 40m at a time -- pushed up a
// driveway, winched onto a trailer -- add up to travel, instead of reading as
// four separate non-events. It is re-based only once travel is established.
RTC_DATA_ATTR static float s_refLat = 0.0f;
RTC_DATA_ATTR static float s_refLon = 0.0f;
RTC_DATA_ATTR static bool  s_haveRefFix = false;

// Under this, GPS is reporting its own noise rather than travel. A stationary
// receiver commonly shows a knot or two.
static const float STATIONARY_SPEED_KMH = 2.0f;

RTC_DATA_ATTR static int s_falseWakeCount = 0;

// Motion wakes that were rate-limited away before they could report.
//
// These used to be invisible to the ladder, which left a hole exactly where it
// was most needed: a hull working against a mooring wakes the board over and
// over, and every wake inside MOTION_MIN_REPORT_S bails out before
// updateMotionThreshold() ever runs. The threshold could only climb on the
// occasional wake that happened to fall outside the window, while the cheap
// ones -- the actual evidence that the threshold is too low -- were discarded.
//
// They are cheap, not free: each is a boot, an IMU bring-up and a sleep.
RTC_DATA_ATTR static int s_suppressedWakes = 0;

// Reports in a row with no motion activity at all. Drives the climb back down.
RTC_DATA_ATTR static int s_quietReports = 0;

// A rate this high is the environment talking, not a one-off. At best a
// suppressed wake costs a couple of hundred milliseconds, so ten of them between
// reports is a tracker being triggered continuously -- which is evidence about
// the threshold no GPS fix is needed to interpret.
static const int SUPPRESSED_WAKES_BEFORE_RAISE = 10;

// Quiet reports before the threshold steps back down.
//
// Without this the ladder is a ratchet: it only ever descends on confirmed
// travel, so a ski moved to a calmer berth, or one whose conditions simply
// settle, keeps a raised threshold indefinitely. That is the worst failure this
// logic can have, because it is self-sustaining -- the deafer it gets, the less
// likely anything crosses the threshold to tell it otherwise, including a theft.
static const int QUIET_REPORTS_BEFORE_LOWER = 3;

// Counts passes of the deep-sleep wake test so it can alternate the pad holds
// across sleeps instead of needing two builds.
RTC_DATA_ATTR static int s_sleepTestPass = 0;

// Survives deep sleep but not a reset, which is the point: if this keeps
// reading 0 the board is resetting rather than waking.
RTC_DATA_ATTR static int s_minimalBootCount = 0;

uint16_t g_motionThresholdMg = MOTION_SENS_MEDIUM_MG;
EngineData g_engine;

static uint16_t loadMotionThreshold() {
    prefs.begin("tracker", false);

    // Read straight from NVS rather than from settings: this runs before
    // loadSettings(), because the IMU has to be armed before the BLE window and
    // the GPS acquisition, and neither is worth reordering for one byte.
    const uint8_t sens = (uint8_t)prefs.getUChar("mot_sens", MOTION_SENS_MEDIUM);
    g_motionBaseMg = motionBaseFor(sens);

    uint16_t v = prefs.getUShort("mot_thr", g_motionBaseMg);

    // A stored value outlives both a retune and a sensitivity change, and the
    // clamp below only ever raises -- so lowering the floor would otherwise
    // change nothing on a unit that already had a value saved. Recording which
    // floor a value was derived from is what lets either actually reach the
    // field: a value from a different floor is discarded rather than kept.
    const uint16_t storedBase = prefs.getUShort("mot_base", 0);
    if (storedBase != g_motionBaseMg) {
        Serial.printf("[Motion] Floor changed %u -> %umg (%s); resetting stored %umg.\n",
                      storedBase, g_motionBaseMg, motionSensitivityName(sens), v);
        v = g_motionBaseMg;
        prefs.putUShort("mot_thr", v);
        prefs.putUShort("mot_base", g_motionBaseMg);
    }
    prefs.end();

    if (v < g_motionBaseMg)           v = g_motionBaseMg;
    if (v > motionCeilingMg())        v = motionCeilingMg();
    Serial.printf("[Motion] Sensitivity %s: floor %umg, current threshold %umg\n",
                  motionSensitivityName(sens), g_motionBaseMg, v);
    return v;
}

// Applies a sensitivity change made while the tracker is already running.
//
// The BLE window opens well after loadMotionThreshold() has chosen a floor and
// imuEnableMotionWake() has armed the part, so without this a change would sit
// in NVS doing nothing until the next boot -- and the next boot, for a parked
// ski, is a day away. Somebody who has just turned the sensitivity up expects
// the next nudge to wake it, not tomorrow's.
//
// Resets the ladder to the new floor rather than carrying the old position
// across: the climb is evidence gathered against the previous floor, and it
// does not transfer to a different one.
static void applyMotionSensitivity(uint8_t sens) {
    const uint16_t base = motionBaseFor(sens);
    if (base == g_motionBaseMg) return;

    Serial.printf("[Motion] Sensitivity now %s: floor %u -> %umg, re-arming.\n",
                  motionSensitivityName(sens), g_motionBaseMg, base);
    g_motionBaseMg = base;
    g_motionThresholdMg = base;
    s_falseWakeCount = 0;

    prefs.begin("tracker", false);
    prefs.putUShort("mot_thr", g_motionThresholdMg);
    prefs.putUShort("mot_base", g_motionBaseMg);
    prefs.end();

    imuEnableMotionWake(g_motionThresholdMg);
}

static void saveMotionThreshold(uint16_t mg) {
    prefs.begin("tracker", false);
    prefs.putUShort("mot_thr", mg);
    prefs.end();
}

static bool wokeOnMotion() {
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
}

static const char* wakeReasonName() {
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER:     return "Timer";
        case ESP_SLEEP_WAKEUP_EXT1:      return "Motion";
        case ESP_SLEEP_WAKEUP_UNDEFINED: return "Cold Boot";
        default:                         return "Other";
    }
}

// Run once per wake, after the GPS attempt, and only for wakes that came from
// the accelerometer. GPS is the arbiter: a good fix showing no speed means the
// interrupt was not a ski going anywhere.
// Steps the threshold one rung, in either direction, and re-arms the part.
// Returns false when it is already at the end of the ladder.
static bool stepMotionThreshold(bool up) {
    uint16_t next;
    if (up) {
        if (g_motionThresholdMg >= motionCeilingMg()) {
            Serial.printf("[Motion] Already at the %umg ceiling for this sensitivity; not going deafer.\n",
                          motionCeilingMg());
            return false;
        }
        next = g_motionThresholdMg + motionStepMg();
        if (next > motionCeilingMg()) next = motionCeilingMg();
    } else {
        if (g_motionThresholdMg <= g_motionBaseMg) return false;
        next = (g_motionThresholdMg > g_motionBaseMg + motionStepMg())
                 ? (uint16_t)(g_motionThresholdMg - motionStepMg())
                 : g_motionBaseMg;
    }

    Serial.printf("[Motion] Threshold %s: %u -> %umg\n",
                  up ? "raised" : "lowered", g_motionThresholdMg, next);
    g_motionThresholdMg = next;
    saveMotionThreshold(g_motionThresholdMg);
    // Re-arm now: the level in the part is whatever was set at boot, and the new
    // one has to be in place before the next sleep.
    imuEnableMotionWake(g_motionThresholdMg);
    return true;
}

// Great-circle distance in metres. Haversine rather than the cheap
// equirectangular approximation: this runs once per wake so the cost is
// irrelevant, and the cheap version's error grows with latitude silently.
static float haversineMetres(float lat1, float lon1, float lat2, float lon2) {
    const float R  = 6371000.0f;
    const float p1 = radians(lat1), p2 = radians(lat2);
    const float dp = p2 - p1, dl = radians(lon2 - lon1);
    float a = sinf(dp / 2) * sinf(dp / 2) +
              cosf(p1) * cosf(p2) * sinf(dl / 2) * sinf(dl / 2);
    if (a > 1.0f) a = 1.0f;
    return 2.0f * R * asinf(sqrtf(a));
}

static void updateMotionThreshold(bool gotFix, float speedKmh, float lat, float lon, float pdop) {
    const int suppressed = s_suppressedWakes;
    s_suppressedWakes = 0;

    if (!wokeOnMotion()) {
        // A scheduled report. If nothing has tripped the accelerometer since the
        // last one -- neither a wake that reported nor one that was rate-limited
        // away -- then whatever raised the threshold has stopped happening, and
        // holding a raised one costs sensitivity for no reason.
        if (suppressed > 0) {
            s_quietReports = 0;
            Serial.printf("[Motion] %d suppressed wake(s) since the last report; not a quiet period.\n",
                          suppressed);
            return;
        }
        if (g_motionThresholdMg <= g_motionBaseMg) return;

        if (++s_quietReports < QUIET_REPORTS_BEFORE_LOWER) {
            Serial.printf("[Motion] Quiet report %d/%d at %umg.\n",
                          s_quietReports, QUIET_REPORTS_BEFORE_LOWER, g_motionThresholdMg);
            return;
        }
        s_quietReports = 0;
        Serial.println("[Motion] Quiet for a while -- easing back toward the floor.");
        stepMotionThreshold(false);
        return;
    }

    s_quietReports = 0;

    // The rate is its own evidence, and it does not need GPS to interpret: this
    // many wakes between two reports means something is tripping the threshold
    // continuously. Checked before the fix test on purpose, because the wakes it
    // counts are exactly the ones that never got a fix to be judged by.
    if (suppressed >= SUPPRESSED_WAKES_BEFORE_RAISE) {
        Serial.printf("[Motion] %d suppressed wakes since the last report -- threshold is too low.\n",
                      suppressed);
        s_falseWakeCount = 0;
        stepMotionThreshold(true);
        return;
    }

    if (!gotFix) {
        // No fix is not evidence. Raising the threshold on a wake that could not
        // be classified would slowly deafen the tracker for reasons that have
        // nothing to do with motion -- a garage roof is not a false positive.
        Serial.println("[Motion] Motion wake with no fix; threshold unchanged.");
        return;
    }

    // Doppler speed is a single sample taken whenever the fix happened to land,
    // which makes it a poor witness on its own: a ski under tow stopped at a
    // light, or one being walked onto a trailer, both read as stationary.
    // Displacement does not care when the sample was taken -- but it is only
    // admissible when the geometry behind the fix is sound, hence the PDOP gate.
    const bool fixTrusted = (pdop > 0.0f && pdop <= FIX_MAX_PDOP);
    if (!fixTrusted) {
        Serial.printf("[Motion] PDOP %.1f exceeds %.1f; fix not trusted for position.\n",
                      pdop, FIX_MAX_PDOP);
    } else if (!s_haveRefFix) {
        s_refLat = lat; s_refLon = lon; s_haveRefFix = true;
        Serial.printf("[Motion] Reference position set (PDOP %.1f).\n", pdop);
    }

    bool  travelled = (speedKmh >= STATIONARY_SPEED_KMH);
    float movedM    = -1.0f;
    if (!travelled && fixTrusted && s_haveRefFix) {
        movedM = haversineMetres(s_refLat, s_refLon, lat, lon);
        if (movedM >= TRAVEL_DISPLACEMENT_M) travelled = true;
    }

    if (travelled) {
        s_falseWakeCount = 0;
        // Re-base: wherever it has stopped is what "parked" means from here.
        if (fixTrusted) { s_refLat = lat; s_refLon = lon; s_haveRefFix = true; }
        if (g_motionThresholdMg != g_motionBaseMg) {
            // All the way down, not one rung: travel is proof the tracker should
            // be at its most sensitive, and the climb was built on evidence that
            // has just been contradicted outright.
            if (movedM >= 0.0f) {
                Serial.printf("[Motion] Real travel: %.0fm from reference -> threshold back to %umg\n",
                              movedM, g_motionBaseMg);
            } else {
                Serial.printf("[Motion] Real travel at %.1f km/h -> threshold back to %umg\n",
                              speedKmh, g_motionBaseMg);
            }
            g_motionThresholdMg = g_motionBaseMg;
            saveMotionThreshold(g_motionThresholdMg);
            imuEnableMotionWake(g_motionThresholdMg);
        }
        return;
    }

    if (movedM >= 0.0f) {
        Serial.printf("[Motion] %.0fm from reference (needs %.0fm) - not travel.\n",
                      movedM, TRAVEL_DISPLACEMENT_M);
    }

    s_falseWakeCount++;
    Serial.printf("[Motion] Motion wake but GPS says %.1f km/h (%d/%d unexplained)\n",
                  speedKmh, s_falseWakeCount, FALSE_WAKES_BEFORE_RAISE);

    if (s_falseWakeCount < FALSE_WAKES_BEFORE_RAISE) return;
    s_falseWakeCount = 0;
    stepMotionThreshold(true);
}

// Arms the wake sources and goes. Shared by the end-of-cycle path and by the
// early bail-out on a motion wake that arrived too soon to be worth a report.
static void enterDeepSleep(int minutes) {
    uint64_t sleep_time = (uint64_t)minutes * 60 * 1000000ULL;
    if (sleep_time == 0) sleep_time = 60 * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_time);

    // Only ever armed when an IMU actually answered. With no daughter board
    // fitted GPIO12 floats, and an ext1 source on a floating pin is a flat
    // battery by morning.
    const bool motionArmed = imuPresent();
    if (motionArmed) {
        esp_sleep_enable_ext1_wakeup(1ULL << DB_IMU_INT1, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    powerPrepareForSleep();

    // Dark from here until the next wake. Paired with the switch-on in setup(),
    // this makes the LED a direct read-out of the awake/asleep cycle.
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);

    Serial.printf("Entering Deep Sleep for %d minutes%s...\n",
                  minutes, motionArmed ? " (or on motion)" : "");
    Serial.flush();

    // Clearing the latch is the last thing that happens. The interrupt latches,
    // so a source left uncleared holds INT1 high and the sleep ends the instant
    // it starts -- and every millisecond between the clear and the sleep is a
    // window where a knock can re-latch it and do exactly that. Doing this after
    // the logging and the flush shrinks that window from a serial write to an
    // I2C read, which is the smallest it can be made from here.
    if (motionArmed) imuClearMotionInterrupt();

    esp_deep_sleep_start();
}

// How long this wake will sleep for, from the inputs that decide it.
//
// Extracted because goToSleep() settles this *after* transmitData() has already
// built and sent the report, so the frame could never say when its own successor
// was due. Nothing else in the payload distinguishes a tracker on a 1440 minute
// interval from one that has stopped working, and telling those two apart cost
// an hour the first time it mattered.
//
// Pure: the NVS side effects (the gps_fail counter, the pub_fail ladder) stay in
// goToSleep(). Callers pass the fail count they expect to be acting on.
static int sleepIntervalMins(bool skiRunning, bool gotFix, int consecutiveFails) {
    int mins = skiRunning ? (int)settings.report_interval_mins
                          : (int)PARKED_INTERVAL_MINS;
    if (mins == 0) mins = 5;
    if (gotFix) return mins;

    if      (consecutiveFails >= 5) mins = 180;  // 3 hours
    else if (consecutiveFails == 4) mins = 60;
    else if (consecutiveFails == 3) mins = 30;
    else if (consecutiveFails == 2) mins = 15;
    else if (consecutiveFails == 1) mins = 5;

    // Backoff only means anything while the ski is running. Parked, the interval
    // is already long and stretching it further on a missed fix would trade away
    // the one thing a heartbeat is for.
    if (!skiRunning) {
        mins = PARKED_INTERVAL_MINS;
    } else if (mins < (int)settings.report_interval_mins) {
        mins = settings.report_interval_mins;
    }
    return mins;
}

void goToSleep(bool got_fix, bool published) {
    modemPowerOff();

    // CAN traffic is the signal that the ski is running: the bus only wakes
    // with the ignition. Supply voltage above ~13V would say the same thing and
    // is the better test -- it survives a CAN fault -- but the supply-sense
    // divider is mis-scaled and reads nothing usable yet (docs/can and the
    // R15/R16 note in utilities.h), so this is the one signal available.
    const bool skiRunning = g_engine.busAlive;

    prefs.begin("tracker", false);
    int fails = prefs.getUInt("gps_fail", 0);

    if (got_fix) {
        if (fails > 0) {
            Serial.printf("[Backoff] Fix obtained! Resetting fail count (was %d)\n", fails);
            prefs.putUInt("gps_fail", 0);
        }
        fails = 0;
    } else {
        fails++;
        prefs.putUInt("gps_fail", fails);
        Serial.printf("[Backoff] No Fix this session. Consecutive Fails: %d\n", fails);
    }

    int actual_interval = sleepIntervalMins(skiRunning, got_fix, fails);
    Serial.printf("[Lifecycle] Ski %s -> %d minute interval%s\n",
                  skiRunning ? "RUNNING" : "parked", actual_interval,
                  PARKED_INTERVAL_MINS == 1440 ? "" : "  [SHORTENED TEST BUILD]");
    if (!got_fix) {
        Serial.printf("[Backoff] Applying Backoff Sleep: %d minutes\n", actual_interval);
    }

    // A parked heartbeat that never reached the broker has to be retried sooner
    // than a day, or one bad modem session becomes 48 hours of silence -- and
    // silence is indistinguishable from a stolen ski, a flat battery, or a dead
    // tracker. This is the opposite case to the backoff above: that one refuses
    // to stretch the interval, this one refuses to leave it long.
    //
    // Escalating rather than fixed, because the common cause is no coverage, and
    // retrying hourly forever in a dead zone spends the battery that the daily
    // cadence exists to protect. After the ladder runs out it falls back to the
    // normal heartbeat and tries again then.
    if (!skiRunning && !published) {
        unsigned int pubFails = prefs.getUInt("pub_fail", 0) + 1;
        prefs.putUInt("pub_fail", pubFails);
        int retry;
        if (pubFails == 1)      retry = 60;
        else if (pubFails == 2) retry = 180;
        else if (pubFails == 3) retry = 360;
        else                    retry = PARKED_INTERVAL_MINS;
        if (retry < actual_interval) {
            Serial.printf("[Uplink] Parked heartbeat did not publish (%u in a row) -> retrying in %d minutes\n",
                          pubFails, retry);
            actual_interval = retry;
        }
    } else if (published && prefs.getUInt("pub_fail", 0) > 0) {
        Serial.println("[Uplink] Published again; clearing the retry ladder.");
        prefs.putUInt("pub_fail", 0);
    }
    prefs.end();

    enterDeepSleep(actual_interval);
}
// --- Custom CGNSINF Parser for SIM7080G ---
bool parseCGNSINF(String raw, float* lat, float* lon, float* speed, float* alt, int* sats, float* hdop, float* pdop) {
    // Expected: <run>,<fix>,<time>,<lat>,<lon>,<alt>,<speed>,<course>,<mode>,<reserved1>,<hdop>,<pdop>,<vdop>,<reserved2>,<sats_view>,<sats_used>,...
    // Raw Example: 1,,20260216002433.000,-28.025790,153.387616,-12.593,0.00,,1,,5.3,9.7,8.2,,5,,64.8,158.2
    
    // Check Run Status
    if (!raw.startsWith("1,")) return false;
    
    // Split by comma
    int idx = 0;
    int from = 0;
    String parts[25];
    int max_parts = 25;
    
    for (int i=0; i<max_parts; i++) {
        int comma = raw.indexOf(',', from);
        if (comma == -1) {
            parts[i] = raw.substring(from);
            from = raw.length();
            break;
        } else {
            parts[i] = raw.substring(from, comma);
            from = comma + 1;
        }
    }

    // Index 1: Fix Status (Must be '1'), Index 2: Time (Must not be empty)
    if (parts[1] != "1" || parts[2].length() == 0) return false;

    // Index 3: Lat, 4: Lon (Verify they are not empty)
    if (parts[3].length() == 0 || parts[4].length() == 0) return false;
    
    float lat_val = parts[3].toFloat();
    float lon_val = parts[4].toFloat();

    // Safety: Ignore modem default "Australia Center" placeholder (-27.000001, 133.0)
    if (abs(lat_val + 27.0) < 0.001 && abs(lon_val - 133.0) < 0.001) return false;

    *lat = lat_val;
    *lon = lon_val;
    *alt = parts[5].toFloat();
    *speed = parts[6].toFloat();
    *hdop = parts[10].toFloat();
    // Field 11. Verified against this modem's own output: a line reporting
    // HDOP 1.8 / PDOP 2.0 / VDOP 1.0 satisfies PDOP^2 = HDOP^2 + VDOP^2 to
    // within 0.06, as does 3.9 / 25.0 / 24.7. The layout is what it claims.
    *pdop = parts[11].toFloat();
    
    // Sats Logic: Use 'Used' (15) if present, else 'In View' (14)
    if (parts[15].length() > 0) {
        *sats = parts[15].toInt();
    } else if (parts[14].length() > 0) {
        *sats = parts[14].toInt();
    } else {
        *sats = 0;
    }
    
    return true; // Consider valid if we parsed Lat/Lon
}

// --- Lifecycle Functions ---

void checkPowerConfig() {
    float batt_volts = PMU.getBattVoltage() / 1000.0F;
    Serial.printf("[Lifecycle] Battery: %.2fV\n", batt_volts);
    
    // Survival Mode: < 3.4V (approx 10-15%)
    if (batt_volts < 3.40) {
        Serial.println("[Lifecycle] LOW BATTERY! Forcing 24h Interval.");
        settings.report_interval_mins = 1440; // 24 Hours
    } else {
        Serial.printf("[Lifecycle] Power OK. Interval: %d mins\n", settings.report_interval_mins);
    }
}

void initGNSS() {
    Serial.println("[Modem] Starting GNSS Engine...");
    modem.sendAT("+CGNSPWR=1");
    modem.waitResponse();
    modem.sendAT("+CGNSSEQ=\"gps;glonass;beidou;galileo\"");
    modem.waitResponse();
    modem.sendAT("+CGNSAN=1"); // Active Antenna
    modem.waitResponse();
}

// One field of a comma-separated CGNSINF line, or "" if the line is too short.
// Only the two logging paths use this; parseCGNSINF() splits the whole line
// itself because it needs most of the fields at once.
static String cgnsinfField(const String& raw, int index) {
    int from = 0;
    for (int i = 0; i <= index; i++) {
        int next = raw.indexOf(',', from);
        if (next == -1) {
            return (i == index) ? raw.substring(from) : String("");
        }
        if (i == index) return raw.substring(from, next);
        from = next + 1;
    }
    return String("");
}

// Fix status (field 1) and satellites in view (field 14) -- the two numbers
// worth watching while a fix is still coming in.
static void logGPSProgress(const char* tag, const String& raw) {
    String fix = cgnsinfField(raw, 1);
    String sv  = cgnsinfField(raw, 14);
    if (fix.length() == 0) fix = "0";
    if (sv.length() == 0)  sv  = "0";
    Serial.printf("[GPS] %s Fix=%s SatsView=%s\n", tag, fix.c_str(), sv.c_str());
}

void pollGPSDiagnostic() {
    modem.sendAT("+CGNSINF");
    if (modem.waitResponse(1000L, "+CGNSINF: ") == 1) {
        String res = modem.stream.readStringUntil('\n');
        res.trim();
        Serial.printf("[GPS-RAW] [%s]\n", res.c_str());
        logGPSProgress("Background...", res);
    }
}

// The BLE window is the only way in to change settings, but on a scheduled wake
// there is nobody standing there to use it -- and 15s of advertising is a large
// share of a cycle whose useful work is a GPS fix and one publish. So it opens
// on a cold boot, and on demand when the boot button is held down through reset.
// A timer wake goes straight to work.
//
// Requires pinMode(0, INPUT_PULLUP) to have run already.
static bool shouldRunBLEWindow() {
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
        Serial.println("[BLE] Cold boot - opening config window.");
        return true;
    }
    if (digitalRead(0) == LOW) {
        Serial.println("[BLE] Boot button held - opening config window.");
        return true;
    }
    Serial.println("[BLE] Timer wake - skipping config window.");
    return false;
}

void runBLEWindow(unsigned long duration_ms) {
    Serial.printf("\n=== BLE Window (%lu ms) ===\n", duration_ms);
    
    // Standardize Name
    String suffix = settings.name;
    if (suffix.length() == 0) {
        String mac = String((uint32_t)ESP.getEfuseMac(), HEX);
        mac.toUpperCase();
        if (mac.length() > 6) mac = mac.substring(mac.length() - 6);
        suffix = mac;
    }
    String bleName = "AE Tracker - " + suffix;
    
    // Init BLE
    BLEDevice::init(bleName.c_str());
    BLEDevice::setMTU(517);
    
    // Callback
    ble.setSettingsCallback([](const TrackerSettings& s) {
        settings = s;
        saveSettings();
        // Takes effect now rather than at the next boot, which for a parked ski
        // would be a day away.
        applyMotionSensitivity(settings.motion_sensitivity);
        Serial.println("[BLE] Settings Updated!");
    });
    
    ble.begin(bleName, settings, PMU.getBattVoltage() / 1000.0, PMU.getBatteryPercent());
    Serial.println("[BLE] Advertising...");

    unsigned long start = millis();
    unsigned long last_gps_poll = 0;
    strip.setPixelColor(0, 0, 0, 255); // Blue
    strip.show();

    while (millis() - start < duration_ms) {
        ble.loop();
        if (digitalRead(0) == LOW) {
            Serial.println("[BLE] Boot Button Pressed - Extending Window!");
            start = millis(); // Reset timer if interact
        }
        
        if (millis() - last_gps_poll > 5000) {
            pollGPSDiagnostic();
            last_gps_poll = millis();
        }

        if (ble.isConnected()) {
             strip.setPixelColor(0, 0, 255, 255); // Cyan
        } else {
             // Blink Blue
             if ((millis() / 500) % 2 == 0) strip.setPixelColor(0, 0, 0, 255);
             else strip.setPixelColor(0, 0, 0, 0);
        }
        strip.show();
        delay(10);
    }
    
    BLEDevice::deinit(); 
    Serial.println("[BLE] Window Closed.\n");
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
}

String getIMEIWithRetry() {
    Serial.println("[Modem] Getting IMEI...");
    for (int i = 0; i < 5; i++) {
        // Drain any pending data
        while (Serial1.available()) Serial1.read();
        
        String res = modem.getIMEI();
        res.trim();
        
        // Sometimes returns "OK" or empty if busy
        if (res.length() >= 14 && res != "OK") {
            // Validate numeric
            bool numeric = true;
            for (char c : res) {
                if (!isDigit(c)) { numeric = false; break; }
            }
            if (numeric) {
                Serial.printf("[Modem] IMEI Found: %s\n", res.c_str());
                return res;
            }
        }
        Serial.printf("[Modem] IMEI Retry %d (Got: '%s')\n", i+1, res.c_str());
        delay(1000);
    }
    
    // Fallback to MAC suffix if IMEI fails repeatedly
    String mac = String((uint32_t)ESP.getEfuseMac(), HEX);
    mac.toUpperCase();
    String fallback = "ESP32-" + mac;
    Serial.printf("[Modem] IMEI Failed. Using Fallback: %s\n", fallback.c_str());
    return fallback;
}

bool getPreciseLocation(float* lat, float* lon, float* speed, float* alt, int* sats, float* hdop, float* pdop) {
    Serial.println("[Lifecycle] Acquiring GPS Fix...");
    strip.setPixelColor(0, 255, 165, 0); // Orange
    strip.show();

    // Note: GNSS assumed to be POWERED ON by initGNSS() caller
    
    unsigned long start = millis();
    bool locked = false;

    while (millis() - start < 300000L) {
        modem.sendAT("+CGNSINF");
        if (modem.waitResponse(2000L, "+CGNSINF: ") == 1) {
            String res = modem.stream.readStringUntil('\n');
            res.trim();
            Serial.printf("[GPS-RAW] [%s]\n", res.c_str());
            
            float f_lat=0, f_lon=0, f_speed=0, f_alt=0, f_acc=0, f_pdop=0;
            int f_sats=0;
            
            if (parseCGNSINF(res, &f_lat, &f_lon, &f_speed, &f_alt, &f_sats, &f_acc, &f_pdop)) {
                Serial.printf("[GPS] Valid! Lat=%.4f Lon=%.4f Sats=%d HDOP=%.2f PDOP=%.2f\n", f_lat, f_lon, f_sats, f_acc, f_pdop);
                *lat = f_lat; *lon = f_lon; *speed = f_speed; *alt = f_alt; *sats = f_sats; *hdop = f_acc; *pdop = f_pdop;

                // A DOP of 0 is the field being absent, not a perfect fix.
                // It parses to 0.0 and sails through every "< 1.5" test here,
                // so an unqualifiable fix used to lock instantly and then be
                // published as the best kind there is. Unknown quality cannot
                // clear a quality bar. The cost of refusing it is that such a
                // fix keeps the acquisition loop running to its timeout and is
                // then reported as no fix, which is the honest answer.
                if (f_acc > 0.0f && (f_acc < 1.5f || (f_acc < 2.5f && f_sats >= 4))) {
                    locked = true;
                    break; 
                }
            } else {
                logGPSProgress("Wait...", res);
            }
        }
        delay(1000);
    }
    
    if (locked) {
        Serial.println("\n[Lifecycle] GPS Locked & Stable.");
        return true;
    } else {
        Serial.println("\n[Lifecycle] GPS Timeout!");
        return false;
    }
}

// Set when a CFG downlink changed something this wake. Published once so the
// backend can retire its retained command against a frame that proves the device
// actually applied it, rather than against a timeout.
static bool g_cfgApplied = false;

void onDownlink(char* topic, uint8_t* payload, unsigned int length) {
    Serial.printf("[MQTT] Downlink on %s (%u bytes)\n", topic, length);

    // Two topics are subscribed now, so the topic IS checked. It used to be safe
    // to skip -- there was only OTA -- and a config frame handed to
    // otaParseCommand() would simply be rejected, but silently, which is exactly
    // the failure that takes an afternoon to find.
    const String t(topic ? topic : "");

    if (t.endsWith("/CFG")) {
        // A zero-length payload is the backend withdrawing a retained command.
        // Nothing to apply, and importantly not an error.
        if (length == 0) {
            Serial.println("[CFG] Retained command withdrawn.");
            return;
        }
        JsonDocument cfg;
        if (deserializeJson(cfg, payload, length) != DeserializationError::Ok) {
            Serial.println("[CFG] Malformed command, ignored.");
            return;
        }
        if (cfg["debug"].is<bool>()) {
            const bool want = cfg["debug"].as<bool>();
            if (want != settings.debug_payload) {
                settings.debug_payload = want;
                saveSettings();
                Serial.printf("[CFG] Debug payload -> %s (persisted)\n", want ? "ON" : "OFF");
            } else {
                Serial.printf("[CFG] Debug payload already %s\n", want ? "ON" : "OFF");
            }
            // Applied on THIS wake: transmitData() has not built its document yet
            // when the downlink arrives on SUBACK, so the change shows up in the
            // very frame that confirms it rather than a whole sleep cycle later.
            g_cfgApplied = true;
        }
        return;
    }

    // Anything else is the OTA topic. A zero-length payload is the backend
    // withdrawing the retained command; otaParseCommand() rejects it and
    // g_otaPending stays false.
    if (otaParseCommand(payload, length, g_otaCmd)) {
        g_otaPending = true;
        Serial.printf("[OTA] Command: v%s %s\n", g_otaCmd.version.c_str(), g_otaCmd.url.c_str());
    }
}

// Returns whether the telemetry actually reached the broker. The caller needs
// to know: a parked ski sleeps for a day between reports, so a failure that is
// not noticed here is 48 hours of silence, not one missed report.
// Two-point calibration for the supply divider, held in NVS.
//
// A linear fit through two measured points rather than a formula from the
// schematic, because that absorbs everything unknown at once: whether the ADC
// code is inverted, the actual full-scale, the loading error against an input
// impedance ST does not specify, and resistor tolerance. Take one reading at a
// known low voltage and one at a known high voltage -- 12.00V and 14.40V span
// the useful range -- and store both.
//
// Returns false until both points exist, so supply_voltage simply does not
// appear rather than appearing wrong -- and false again when the answer is
// outside the window below, for the same reason.
//
// A two-point fit is a straight line, and a straight line has no idea where it
// stops being a measurement. Extrapolated far enough it will answer any
// question put to it, and the answer that matters is the open input: with
// nothing on the supply feed the top of the divider floats, the node sits at
// 0V, and the ADC -- which runs inverted -- pins near its positive rail. On
// this board that is raw 32512 against a possible 32704, and the fit turns it
// into 9.20V. Nothing about 9.20V looks like an error. It looks exactly like a
// jetski battery somebody should be told about, which is worse than no reading
// at all.
//
// Saturation can only push the code higher and so the volts lower, so an open
// input cannot report above ~9.2V no matter what. A floor of 10.0V therefore
// separates the two cases outright: a 12V lead-acid battery at 10.0V is flat
// past cranking, and is still reported. The ceiling is the same argument the
// other way -- above it the divider is faulty, not the alternator generous.
//
// supply_adc_raw is published either way, so a reading rejected here is still
// on the wire for anyone who wants to argue with the window.
static const float SUPPLY_MIN_PLAUSIBLE_V = 10.0f;
static const float SUPPLY_MAX_PLAUSIBLE_V = 16.0f;

static bool supplyRawToVolts(int16_t raw, float& volts) {
    prefs.begin("tracker", true);
    const int32_t rawLo  = prefs.getInt("sup_raw_lo", 0);
    const int32_t rawHi  = prefs.getInt("sup_raw_hi", 0);
    const float   voltLo = prefs.getFloat("sup_v_lo", 0.0f);
    const float   voltHi = prefs.getFloat("sup_v_hi", 0.0f);
    prefs.end();

    if (rawLo == rawHi || voltLo == voltHi) return false;   // not calibrated

    const float v = voltLo + (float)(raw - rawLo) * (voltHi - voltLo) / (float)(rawHi - rawLo);

    if (v < SUPPLY_MIN_PLAUSIBLE_V || v > SUPPLY_MAX_PLAUSIBLE_V) {
        Serial.printf("[Power] Supply reads %.2fV from raw %d, outside %.1f-%.1fV: "
                      "treating the feed as disconnected.\n",
                      v, raw, SUPPLY_MIN_PLAUSIBLE_V, SUPPLY_MAX_PLAUSIBLE_V);
        return false;
    }

    volts = v;
    return true;
}

bool transmitData(bool has_fix, float lat, float lon, float speed, float alt, int sats, float hdop) {
    bool published = false;
    Serial.println("[Lifecycle] Preparing Transmission...");

    // Sampled here rather than in setup() so the reported orientation is the one
    // at report time: GPS acquisition ahead of this can run for five minutes,
    // and a value from before that is not describing the same situation.
    ImuReading imu = imuRead();

    // Same reasoning as the IMU: read at report time, not at boot. The policy is
    // applied here too, so a decision to cut or restore the jetski feed is made
    // against the battery voltage that is about to be published alongside it.
    PowerStatus power = powerRead();

    // Read the ski's supply BEFORE the charge policy runs. The divider taps
    // downstream of the switch, so this has to close the switch briefly -- doing
    // it first keeps that momentary closure clearly separate from the policy's
    // own decision, and leaves the policy with the last word on what state the
    // switch is left in for the sleep.
    int16_t supplyRaw = 0;
    bool supplyWasEnabled = false;
    const bool haveSupply = powerReadSupplyRaw(supplyRaw, supplyWasEnabled);

    powerApplyChargePolicy(PMU.getBattVoltage() / 1000.0F, power);

    imei = getIMEIWithRetry();
    mqtt_topic_up = "ae-nv/tracker/" + imei + "/up";
    // The backend keys this device's devices row on the IMEI -- processTrackerData()
    // upserts with mac = imei -- and the command queue builds its topic from that
    // same column (ae/downlink/<device_mac>/OTA), so the IMEI is what a downlink is
    // addressed to. It has no parent, so it is never routed via a gateway.
    //
    // The catch is what happens when getIMEIWithRetry() does not get an IMEI: the
    // fallback identity is ESP32-<mac suffix>, and one enrolled row is already
    // stored as the literal 'OK' from an earlier version of that retry. Whatever
    // it publishes under is what it must subscribe under, so this is derived from
    // the same string rather than from the IMEI directly.
    mqtt_topic_dn = "ae/downlink/" + imei + "/OTA";
    mqtt_topic_cfg = "ae/downlink/" + imei + "/CFG";
    Serial.printf("[MQTT] MAC/IMEI: %s\n", imei.c_str());
    Serial.printf("[MQTT] Topic: %s\n", mqtt_topic_up.c_str());
    Serial.printf("[MQTT] Downlink: %s\n", mqtt_topic_dn.c_str());
    Serial.printf("[MQTT] Config:   %s\n", mqtt_topic_cfg.c_str());

    Serial.println("[Lifecycle] Connecting to Network...");
    
    modem.sendAT("+CFUN=1");
    modem.waitResponse(2000L);

    Serial.print("[Lifecycle] Waiting for Network...");
    if (!modem.waitForNetwork(180000L)) {
        Serial.println("Fail: Network Timeout");
        return published;
    }
    Serial.println(" OK");
    
    Serial.printf("[Lifecycle] Connecting to GPRS (APN: %s)...", settings.apn.c_str());
    if (modem.gprsConnect(settings.apn.c_str())) {
        Serial.println(" Connected");
        
        Serial.printf("[MQTT] Connecting to %s...", settings.mqtt_broker.c_str());
        mqtt.setServer(settings.mqtt_broker.c_str(), 1883);
        mqtt.setBufferSize(MQTT_BUFFER_SIZE); // Full JSON out, OTA command in, one frame each
        mqtt.setCallback(onDownlink);
        if (mqtt.connect(imei.c_str(), settings.mqtt_user.c_str(), settings.mqtt_pass.c_str())) {
            Serial.println(" Connected");

            // Subscribed before anything is published. The retained OTA arrives on
            // SUBACK, so subscribing first is what lets one session both report and
            // collect; doing it after the publish would still work, but it puts the
            // slowest step of the wake behind the one that must not be lost.
            //
            // qos 1: the broker only replays a retained message once, and a qos 0
            // copy dropped on a marginal LTE-M link is not repeated.
            if (!mqtt.subscribe(mqtt_topic_dn.c_str(), 1)) {
                Serial.println("[MQTT] Subscribe FAILED (no downlink this wake)");
            }
            // Same reasoning as the OTA topic: subscribed before publishing, so a
            // retained config change lands on SUBACK and is applied to the frame
            // this wake is about to send. A separate topic rather than a field on
            // the OTA one, because the two are retained independently -- clearing
            // a config command must not clear a pending firmware update.
            if (!mqtt.subscribe(mqtt_topic_cfg.c_str(), 1)) {
                Serial.println("[MQTT] Config subscribe FAILED (no config this wake)");
            }

            // Check for previous crash logs
            if (g_hasCrashLog) {
                String log = crash_handler_get_log();
                String crash_topic = "ae/crash/" + imei;
                Serial.println("[MQTT] Publishing Crash Log...");
                if (mqtt.publish(crash_topic.c_str(), log.c_str())) {
                    Serial.println("[MQTT] Crash Log Sent Successfully");
                    g_hasCrashLog = false;
                } else {
                    Serial.println("[MQTT] Crash Log Publish FAILED");
                }
            }

            
            JsonDocument doc;
            doc["mac"] = imei;
            // The key the worker reads for every other product
            // (backend-worker/src/index.ts:2154). Nothing has ever set it here, which
            // is why both enrolled trackers have an empty firmware_version column and
            // why an OTA command aimed at one could never be resolved.
            doc["fw_version"] = firmwareVersion();
            
            String suffix = settings.name;
            if (suffix.length() == 0) {
                suffix = imei;
                if (suffix.length() > 6) suffix = suffix.substring(suffix.length() - 6);
            }
            doc["model"] = "AE Tracker - " + suffix;
            
            // Absent rather than zero, on the same reasoning that keeps rpm
            // out of a parked ski's report. lat/lon default to 0,0 when the
            // acquisition times out, and 0,0 is not a null -- it is a real
            // coordinate in the Gulf of Guinea that a map will plot without
            // complaint, several thousand kilometres from anything this device
            // will ever see. A report with no position should say so and let
            // the backend keep the last known one, rather than assert a
            // position that is wrong.
            //
            // The flag goes out either way, so "no fix this wake" is a fact the
            // report carries rather than something inferred from missing keys.
            const bool dbg = settings.debug_payload;

            doc["fix"] = has_fix;
            if (has_fix) {
                doc["lat"] = lat;
                doc["lon"] = lon;
                doc["speed"] = speed;
                doc["sats"] = sats;
                // Altitude and HDOP describe the quality of the fix rather than
                // the fix. Kept for debug, where a suspect position needs them.
                if (dbg) {
                    doc["alt"] = alt;
                    doc["hdop"] = hdop;
                }
            }

            // Which frame this is. Published ALWAYS, and it is the single most
            // important field here: without it a missing coolant_raw is
            // ambiguous between "debug is off" and "the ski's bus was asleep",
            // and the wrong one of those sends somebody out to the ski.
            doc["debug"] = dbg;
            if (g_cfgApplied) doc["cfg_applied"] = true;

            doc["device_voltage"] = PMU.getBattVoltage() / 1000.0F;
            if (dbg) {
                // USB VBUS, not the ski's supply -- the divider cannot read the
                // ski yet. Debug-only until it means what its name suggests.
                doc["voltage"] = PMU.getVbusVoltage() / 1000.0F;
                // Legacy alias for device_voltage. Nothing new should read it.
                doc["battery_voltage"] = PMU.getBattVoltage() / 1000.0F;
            }
            // The AXP2101 keeps a fuel gauge and this reads it, but it answers
            // -1 when isBatteryConnect() is false rather than failing. Publishing
            // that verbatim put "-1%" on the dashboard, which reads as a
            // measurement rather than as an absent battery -- and a tracker
            // running from USB with no cell fitted is exactly the bench case
            // where it happens.
            const int socPct = PMU.getBatteryPercent();
            if (socPct >= 0) doc["soc"] = socPct;

            // Always emitted, "Unknown" included: the field has been in the
            // documented payload all along, and a key that appears only on
            // boards with a working IMU is harder for the backend to reason
            // about than one that is always there.
            doc["wake_reason"] = wakeReasonName();
            doc["charge_state"] = chargeStateName(power.state);
            if (dbg) {
                doc["orientation"] = orientationName(imu.orientation);
                doc["supply_enabled"] = power.supplyEnabled;
            }

            // Vehicle frame, so the backend never has to know how the board is
            // mounted. Two decimals is well inside what a +/-2g part resolves.
            if (dbg && imu.valid) {
                doc["accel_up"]   = roundf(imu.up   * 100) / 100.0f;
                doc["accel_fwd"]  = roundf(imu.fwd  * 100) / 100.0f;
                doc["accel_stbd"] = roundf(imu.stbd * 100) / 100.0f;
            }

            // Engine data, only when the ski's bus was actually awake. Sending
            // rpm=0 for a parked ski would be indistinguishable from a running
            // engine at a standstill, so absent means absent.
            doc["engine_bus"] = g_engine.busAlive;
            doc["ski_running"] = g_engine.busAlive;
            if (g_engine.rpmValid)  doc["rpm"] = g_engine.rpm;
            // Raw because the scaling is genuinely unknown -- see
            // docs/can/seadoo-can.md. Better an honest count than a fabricated
            // temperature nobody can check.
            // Engine hours ride in the normal frame. They are a confirmed decode,
            // they change slowly, and they are the one engine number an owner
            // actually wants between rides.
            if (g_engine.hoursValid) doc["engine_minutes"] = g_engine.engineMinutes;

            if (dbg) {
                if (g_engine.tempValid) doc["engine_temp_raw"] = g_engine.tempRaw;
                // Coolant is a confirmed signal but an unconfirmed scale, so it
                // goes out raw for the same reason as above.
                if (g_engine.coolantValid) doc["coolant_raw"] = g_engine.coolantRaw;
                // Also a confirmed decode, so it goes out as the step number it is.
                if (g_engine.trimValid) doc["ibr_trim"] = g_engine.trimPos;
            }

            // Published so the adaptive logic can be seen working from the
            // backend rather than only from a serial cable -- which is what the
            // dbg gate that used to be here prevented, defeating the sentence
            // above it. This board sat at 528mg on a 176mg floor for an hour and
            // nothing in the cloud said so.
            doc["motion_threshold_mg"] = g_motionThresholdMg;
            // The setting, alongside where the ladder currently sits. Both are
            // wanted: the first is what somebody chose, the second is what the
            // ski decided, and a support question about false wakes needs to
            // tell them apart.
            doc["motion_sensitivity"] = motionSensitivityName(settings.motion_sensitivity);
            // Wakes since the last report that were rate-limited away. Published
            // because it is the one number that explains a tracker reporting
            // often, or a battery going down faster than the interval suggests,
            // and it is invisible from the reports themselves. Read before
            // updateMotionThreshold() clears it -- transmitData() runs first.
            // Exception to the debug rule: a non-zero count is the only thing that
            // explains a tracker reporting more often than its interval or a
            // battery going down faster than it should. Sending it when it is
            // zero is what would be noise, so it goes out only when it is not.
            if (dbg || s_suppressedWakes > 0) doc["motion_suppressed"] = s_suppressedWakes;

            // Ski supply, read through the daughter board's divider by the
            // LIS3DH aux ADC. Published RAW, not as volts: ST does not document
            // whether the ADC code is inverted, and the real ratio depends on
            // resistor tolerance and on an input impedance ST does not specify,
            // so a figure derived from the schematic would be a guess wearing a
            // unit. Two bench readings at known voltages turn it into volts --
            // supplyRawToVolts() emits supply_voltage only once they exist,
            // and only when the result could describe a real 12V system -- an
            // open feed extrapolates to a plausible-looking 9.2V otherwise.
            //
            // Always sent when it could be read, including on a debug-off frame:
            // this is the field that the divider rework exists to produce, and a
            // raw count nobody can see is not worth taking.
            if (haveSupply) {
                doc["supply_adc_raw"] = supplyRaw;
                float supplyVolts;
                if (supplyRawToVolts(supplyRaw, supplyVolts)) {
                    doc["supply_voltage"] = supplyVolts;
                }
            }

            int csq = modem.getSignalQuality();
            int dbm = (csq == 99) ? -113 : (csq * 2) - 113;
            doc["rssi"] = dbm;
            
            doc["interval"] = settings.report_interval_mins;

            // When the next frame is actually due -- the running interval above
            // is only half the answer, and it is the half that misleads: a
            // parked ski ignores it entirely. Recomputed from the same inputs
            // goToSleep() will use rather than guessed.
            //
            // The failed-publish retry ladder in goToSleep() can shorten this,
            // but only when the publish failed -- and then no frame arrives to
            // carry the number, so what is delivered is always right.
            prefs.begin("tracker", true);
            const int gpsFails = has_fix ? 0 : ((int)prefs.getUInt("gps_fail", 0) + 1);
            prefs.end();
            doc["next_wake_mins"] = sleepIntervalMins(g_engine.busAlive, has_fix, gpsFails);
            
            String payload;
            serializeJson(doc, payload);
            Serial.println("[MQTT] Publishing: " + payload);

            // StaticJsonDocument<768> used to cap the document at the same size
            // as the buffer, so an over-long report came out truncated but went.
            // JsonDocument has no ceiling, so the ceiling moved here.
            //
            // PubSubClient refuses an oversized frame by returning false without
            // touching the wire, and the failure path below can only report
            // mqtt.state(), which still says "connected". A report that vanishes
            // for a reason nothing in the log names is the outcome worth
            // avoiding -- this is what makes the cause explicit. The formula is
            // PubSubClient's own: 5 bytes of fixed header, 2 for topic length,
            // then topic and payload.
            const size_t frameLen = 5 + 2 + mqtt_topic_up.length() + payload.length();
            if (frameLen > MQTT_BUFFER_SIZE) {
                Serial.printf("[MQTT] Frame is %u bytes, buffer is %u -- publish will be refused. Payload needs trimming.\n",
                              (unsigned)frameLen, (unsigned)MQTT_BUFFER_SIZE);
            }
            if (mqtt.publish(mqtt_topic_up.c_str(), payload.c_str())) {
                Serial.println("[MQTT] Publish Successful");
                published = true;
                // Only a delivered report starts the motion rate limit. A failed
                // publish leaves the previous timestamp standing, so the next
                // movement is still allowed to try.
                s_lastReportTime = time(NULL);
            } else {
                Serial.printf("[MQTT] Publish FAILED (State: %d)\n", mqtt.state());
            }
            
            // Hold the session just long enough for the retained downlink. This is
            // the only window this device has: setup() runs once per wake and loop()
            // is empty, so anything not collected here waits for the next wake.
            unsigned long wait_start = millis();
            while (!g_otaPending && millis() - wait_start < DOWNLINK_WAIT_MS) {
                mqtt.loop();
                delay(10);
            }

            mqtt.disconnect();
        } else {
             Serial.printf(" FAILED (State: %d)\n", mqtt.state());
        }

        // OTA runs on the same GPRS session, after MQTT is closed and before it is
        // torn down. On success this call does not return -- the device reboots into
        // the new slot -- so the telemetry publish above has to have happened first.
        if (g_otaPending) {
            String act;
            if (!otaShouldApply(g_otaCmd, firmwareVersion())) {
                // otaShouldApply logs the specific reason.
            } else if (!otaLinkIsCatM(modem, act)) {
                // otaLinkIsCatM logs the technology it actually saw. Deliberately not
                // a failure the device retries hard: it will be offered the same
                // retained command on the next wake, and may be on a better cell.
            } else if (otaDownloadAndApply(modem, g_otaCmd)) {
                modem.gprsDisconnect();
                modemPowerOff();
                Serial.flush();
                ESP.restart();
            }
        }

        modem.gprsDisconnect();
    } else {
        Serial.println(" GPRS FAILED");
    }
    return published;
}

void setup() {
    Serial.begin(115200);
    // process_on_boot() consumes the RTC magic, so it answers true exactly once --
    // on the boot straight after the panic. If the log is not delivered during
    // that one boot it is never looked at again, even though it is sitting in
    // NVS. A board that reboots before it can report is precisely the board whose
    // log is worth having, so ask the durable copy as well.
    g_hasCrashLog = crash_handler_process_on_boot() || crash_handler_has_log();
    delay(2000);

    
    esp_reset_reason_t reason = esp_reset_reason();
    // __DATE__/__TIME__ are stamped at compile time, so this is proof of which
    // build is actually running rather than which one was last built.
    Serial.printf("\n--- AE Tracker Boot (Reason: %d, Wake: %s, FW: '%s', built %s %s) ---\n",
                  reason, wakeReasonName(), firmwareVersion(), __DATE__, __TIME__);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("PMU FAIL");
    }
    // Awake indicator. The AXP2101's charge LED is the only one this board
    // actually has -- NEOPIXEL_PIN is a guess inherited from other S3 boards and
    // drives nothing here -- and it is the only way to see the unit's state on
    // battery, where USB (and therefore the serial log) is not available.
    //
    // Lit means awake. It is switched off immediately before every deep sleep,
    // so a dark unit is a sleeping one, and default charger control of the LED
    // is given up deliberately: charge state is already reported over MQTT,
    // whereas "is it awake right now" has no other channel at all.
    // Steady ON means awake; 4Hz blink means this wake came from the
    // accelerometer. Plain "on whenever awake" turned out to be useless for
    // telling whether motion wake works -- it lights identically for the timer,
    // so a unit waking on schedule looks exactly like one waking on a shake.
    PMU.setChargingLedMode(wokeOnMotion() ? XPOWERS_CHG_LED_BLINK_4HZ
                                          : XPOWERS_CHG_LED_ON);

    PMU.disableTSPinMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableBattDetection();
    PMU.enableCellbatteryCharge();

    // Daughter board. Absence is reported and then ignored -- see imuBegin().
    // powerBegin() comes first because it is what releases the pad hold from
    // the last sleep and gets the CAN transceiver out of an undefined state.
    powerBegin();
    g_motionThresholdMg = loadMotionThreshold();
    if (imuBegin()) {
        imuEnableMotionWake(g_motionThresholdMg);
    }

    pinMode(0, INPUT_PULLUP);
    strip.begin();
    strip.show();

#ifdef DB_BENCH_MODE
    // Bench build: charger and IMU only. No modem, no MQTT, no deep sleep --
    // deep sleep drops the USB CDC and makes every iteration a wait for
    // re-enumeration.
    Serial.println("\n=== DAUGHTER BOARD BENCH: continuous log ===");
    Serial.println("Columns: elapsed | battery | PG | STAT | charge state | supply switch\n");
    // First, before anything else has touched a pin. powerSweepPullups() walks
    // GPIO12 among the rest and leaves an internal pull-up on it, and INT1 is
    // the one signal here that must be measured exactly as the LIS3DH drives it.
    //
    // The static self-test runs ahead of the shake watch because it needs nobody
    // present and it cannot come back ambiguous -- and if it fails, the shake
    // test that follows has nothing left to tell anyone. It leaves the interrupt
    // block reconfigured, so the shipping arm has to be put back before the
    // watch, or the watch would be measuring the self-test's settings.
    imuDumpFilteredData(6000);
    imuInterruptSelfTest();

    // Duration sweep. Everything else has now been measured and works -- the
    // filter removes gravity to 24mg, the generator trips on gravity, and INT1
    // reaches GPIO12 -- which leaves INT1_DURATION as the only thing between a
    // 700mg shake and a wake that never came. It is counted in ODR periods, so
    // the shipping 3 demands 60ms of CONTINUOUS over-threshold motion, and the
    // counter resets the moment the condition lapses. Shaking is oscillatory:
    // |axis| passes back through zero at every reversal, so 60ms unbroken is a
    // far stronger demand than it looks. Running 0 against 3 over the same
    // gesture is what turns that from an argument into a measurement.
    // Each setting is retried until a window is actually shaken hard enough to
    // mean something. Two runs of this were wasted on fixed 15s windows that
    // opened and closed while nobody was at the bench, and a window nobody
    // shook produces the same silence as a configuration that does not work.
    // Waiting for the gesture instead of for the clock is what removes that
    // ambiguity -- and it lets whoever is holding the board take their time.
    const uint8_t DURATIONS[] = { 0, 3 };
    MotionWatchResult results[2];
    for (int i = 0; i < 2; i++) {
        const uint8_t d = DURATIONS[i];
        Serial.printf("\n[IMU] ===== duration sweep: INT1_DURATION=%u (%ums at 50Hz) =====\n",
                      d, d * 20);
        for (int attempt = 1; attempt <= 6; attempt++) {
            imuEnableMotionWake(g_motionThresholdMg, d);
            results[i] = imuWatchMotionInterrupt(10000);
            if (results[i].conclusive()) break;
            Serial.printf("[IMU] Window %d peaked at only %.0fmg -- not a real test. Shake during the next one.\n",
                          attempt, results[i].peakMg);
        }
    }

    Serial.println("\n[IMU] ===== duration sweep result =====");
    for (int i = 0; i < 2; i++) {
        Serial.printf("[IMU]   DUR=%u (%2ums): peak %4.0fmg  latched=%-3s  pin=%s\n",
                      DURATIONS[i], DURATIONS[i] * 20, results[i].peakMg,
                      results[i].latched ? "YES" : "no",
                      results[i].pinHigh ? "HIGH" : "low");
    }
    if (results[0].latched && !results[1].latched) {
        Serial.println("[IMU]   -> DUR is the blocker: the same shake wakes it at 0 and not at 3.");
        Serial.println("[IMU]      60ms of CONTINUOUS over-threshold motion is more than shaking sustains.");
    } else if (results[0].latched && results[1].latched) {
        Serial.println("[IMU]   -> Both fired, so DUR=3 is not what stopped the earlier attempts.");
    } else if (!results[0].latched && results[0].conclusive()) {
        Serial.println("[IMU]   -> Even DUR=0 did not fire on real motion. The threshold or the axis");
        Serial.println("[IMU]      enables are wrong, not the duration.");
    } else {
        Serial.println("[IMU]   -> Never shaken hard enough to conclude anything. Re-run.");
    }

    imuEnableMotionWake(g_motionThresholdMg);

    powerSweepPullups();
    // canSelfTest() is deliberately NOT called here. It uses TWAI_MODE_NO_ACK,
    // which transmits -- and this build gets flashed onto a tracker that may be
    // plugged into a ski. A bench convenience that drives a vehicle bus the
    // moment someone forgets where the board is plugged in is not worth having.
    // Run it explicitly from a bench build when you want it.

    const uint32_t t0 = millis();
    for (uint32_t n = 1; ; n++) {
        // powerRead() spends ~700ms watching STAT, because a blink is how this
        // charger reports a fault and one sample cannot tell a blink from a
        // level. That sets the cadence at roughly a line per second.
        PowerStatus ps = powerRead();
        const float vbat = PMU.getBattVoltage() / 1000.0F;

        Serial.printf("[Log %6.1fs] Vbat=%.3fV  PG=%-7s STAT=%-8s %-9s supply=%s\n",
                      (millis() - t0) / 1000.0f, vbat,
                      ps.supplyPresent ? "present" : "absent",
                      ps.charging ? "charging" : "idle",
                      chargeStateName(ps.state),
                      ps.supplyEnabled ? "ON" : "CUT");

        // Cutoff demonstration, once, ten seconds in. This is the one part of
        // the power design nothing else exercises: whether SUPPLY_EN actually
        // operates the TPS1H000. If it does, PG drops to absent within a
        // sample, because the charger stops seeing an input.
        static bool cutDone = false;
        if (!cutDone && (millis() - t0) > 10000) {
            cutDone = true;
            PowerStatus tmp;
            Serial.println("\n[TEST] === cutting the supply for 8s ===");
            powerApplyChargePolicy(4.50f, tmp);   // above CHARGE_STOP_V -> cut
            for (int i = 0; i < 8; i++) {
                PowerStatus c = powerRead();
                Serial.printf("[TEST]   cut+%ds  PG=%s  supply=%s\n", i,
                              c.supplyPresent ? "present" : "ABSENT",
                              c.supplyEnabled ? "ON" : "CUT");
            }
            Serial.println("[TEST] === restoring ===\n");
            powerApplyChargePolicy(3.50f, tmp);   // below CHARGE_RESUME_V -> on
        }

        // The IMU is not what is being watched here, and a read costs another
        // 640ms, so it goes in occasionally rather than in the way.
        if (imuPresent() && (n % 8 == 0)) imuRead();

        delay(200);
    }
#endif

#ifdef BAUD_TEST_MODE
    // Proves the OTA's baud switch on real hardware before it is trusted.
    //
    // Raising the modem UART is the fix for a download that cannot keep up, but
    // it is also the one change here that could strand a fielded unit: if the
    // modem takes +IPR and the host loses sync, the only remote repair path is
    // the OTA being repaired. So the switch, the link check and the restore all
    // get exercised here, where a failure costs a USB cable.
    {
        // setup() opens Serial1 well below this point, so without it here the
        // test talks to a UART that was never started -- which looks exactly
        // like a modem that will not answer.
        // 4KB rather than the 256-byte default, because the OTA raises this link
        // to 460800 for a transfer. At 256 bytes that holds only 5.6ms of data at
        // 460800 -- any scheduling delay longer than that drops bytes, corrupts
        // the AT framing, and TinyGSM then waits per byte for data that will
        // never arrive. 4KB is 89ms: a margin rather than a race. At 115200 the
        // default was fine, which is why this never mattered until now.
        Serial1.setRxBufferSize(4096);
        Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
        modemPowerOn();
        Serial.println("\n=== MODEM BAUD SWITCH TEST ===");
        if (!modem.testAT(5000)) {
            Serial.println("[BaudTest] Modem not responding at 115200; nothing to test.");
        } else {
            Serial.println("[BaudTest] OK at 115200.");
            for (uint32_t rate : { 460800UL, 921600UL }) {
                Serial.printf("\n[BaudTest] --- trying %lu ---\n", (unsigned long)rate);
                modem.sendAT(GF("+IPR="), rate);
                const int resp = modem.waitResponse(2000L);
                Serial.printf("[BaudTest] +IPR reply: %d\n", resp);
                delay(100);
                Serial1.updateBaudRate(rate);
                delay(150);
                const bool ok = modem.testAT(2000);
                Serial.printf("[BaudTest] link at %lu: %s\n",
                              (unsigned long)rate, ok ? "OK" : "DEAD");

                // Restore, and prove the restore works too -- an untested
                // restore is the half that actually strands a device.
                modem.sendAT(GF("+IPR="), 115200);
                modem.waitResponse(2000L);
                delay(100);
                Serial1.updateBaudRate(115200);
                delay(150);
                Serial.printf("[BaudTest] back at 115200: %s\n",
                              modem.testAT(2000) ? "OK" : "DEAD");
            }
        }
        Serial.println("\n[BaudTest] done; halting.");
        while (true) delay(1000);
    }
#endif

#ifdef SUPPLY_CAL_MODE
    // Two-point calibration for the supply divider.
    //
    // A linear fit through two measured points, not a formula from the
    // schematic: that absorbs whether the ADC code is inverted, the real
    // full-scale, the loading error against an input impedance ST does not
    // specify, and resistor tolerance -- all at once, and without needing to
    // know which of them is responsible for what.
    {
        Serial.println("\n=== SUPPLY DIVIDER CALIBRATION ===");
        Serial.println("Apply a known voltage to SUPPLY_IN, let the reading settle, then type:");
        Serial.println("  L <volts>   store as the LOW point    e.g.  L 12.00");
        Serial.println("  H <volts>   store as the HIGH point   e.g.  H 14.40");
        Serial.println("  S           show what is stored");
        Serial.println("  C           clear both points");
        Serial.println("Readings are an average of 16 samples, once a second.\n");

        String line;
        uint32_t lastRead = 0;
        int16_t lastRaw = 0;
        bool haveRaw = false;

        for (;;) {
            if (millis() - lastRead > 1000) {
                lastRead = millis();
                // Averaged: a single sample of a 10-bit ADC on a 180k source is
                // noisy enough that two calibration points taken from single
                // reads would bake that noise into the fit.
                int32_t sum = 0; int n = 0;
                for (int i = 0; i < 16; i++) {
                    int16_t r; bool was;
                    if (powerReadSupplyRaw(r, was)) { sum += r; n++; }
                    delay(5);
                }
                if (n) {
                    lastRaw = (int16_t)(sum / n);
                    haveRaw = true;
                    Serial.printf("[Cal] raw = %6d   (10-bit: %4d)\n", lastRaw, lastRaw >> 6);
                } else {
                    Serial.println("[Cal] ADC read failed -- is the daughter board fitted?");
                }
            }

            while (Serial.available()) {
                char c = (char)Serial.read();
                if (c == '\n' || c == '\r') {
                    line.trim();
                    if (line.length()) {
                        const char k = toupper(line[0]);
                        prefs.begin("tracker", false);
                        if ((k == 'L' || k == 'H') && haveRaw) {
                            const float v = line.substring(1).toFloat();
                            if (v <= 0) {
                                Serial.println("[Cal] Need a voltage, e.g. L 12.00");
                            } else {
                                prefs.putInt(k == 'L' ? "sup_raw_lo" : "sup_raw_hi", lastRaw);
                                prefs.putFloat(k == 'L' ? "sup_v_lo" : "sup_v_hi", v);
                                Serial.printf("[Cal] Stored %s point: raw %d = %.2fV\n",
                                              k == 'L' ? "LOW" : "HIGH", lastRaw, v);
                            }
                        } else if (k == 'S') {
                            Serial.printf("[Cal] LOW  raw %d = %.2fV\n",
                                          prefs.getInt("sup_raw_lo", 0), prefs.getFloat("sup_v_lo", 0.0f));
                            Serial.printf("[Cal] HIGH raw %d = %.2fV\n",
                                          prefs.getInt("sup_raw_hi", 0), prefs.getFloat("sup_v_hi", 0.0f));
                        } else if (k == 'C') {
                            prefs.remove("sup_raw_lo"); prefs.remove("sup_v_lo");
                            prefs.remove("sup_raw_hi"); prefs.remove("sup_v_hi");
                            Serial.println("[Cal] Cleared.");
                        } else {
                            Serial.println("[Cal] Commands: L <v>, H <v>, S, C");
                        }
                        prefs.end();
                    }
                    line = "";
                } else {
                    line += c;
                }
            }
            delay(5);
        }
    }
#endif

#ifdef OUR_SLEEP_TEST
    // Our own sleep path, timed, without the cycle in front of it.
    //
    // The vendor example sleeps its configured time to the second on this board,
    // so the hardware is not at fault. This calls the tracker's real
    // enterDeepSleep() -- powerPrepareForSleep(), the pad holds, the PMU state
    // this firmware leaves behind, all of it -- but at one minute and with no
    // BLE window, GPS acquisition or modem session ahead of it, so a result
    // takes a minute instead of eight.
    //
    // It is deliberately placed after powerBegin() and imuBegin() so the pins and
    // the PMU are in exactly the state the real cycle would leave them.
    {
        Serial.printf("\n=== OUR SLEEP PATH TEST === woke by: %s\n", wakeReasonName());
        Serial.println("[OurSleep] calling the tracker's own enterDeepSleep(1)");
        Serial.flush();
        enterDeepSleep(1);
    }
#endif

#ifdef MINIMAL_SLEEP_TEST
    // Does the RTC timer wake this board at all?
    //
    // Modelled on LilyGo's MinimalDeepSleepExample: modem off, every unused PMU
    // rail and measurement disabled, one timer wake source, nothing else. No
    // ext1, no GPS, no modem session, no pad holds -- so if this wakes on time
    // the timer is fine and the fault is something the tracker cycle does, and
    // if it does not the fault is under all of that.
    //
    // 60 seconds, because a test you have to wait an hour for is not a test.
    {
        Serial.printf("\n=== MINIMAL SLEEP TEST === boot #%d, woke by: %s\n",
                      ++s_minimalBootCount, wakeReasonName());

        // What the sleep timer is actually counting.
        //
        // esp_sleep_enable_timer_wakeup() converts microseconds into RTC ticks
        // using this calibration, so if the calibrated period does not match the
        // clock that is really running, every sleep is wrong by that ratio -- and
        // wrong in a way nothing in the sleep code itself would reveal.
        //
        // The value is the RTC slow clock period in microseconds, Q13.19 fixed
        // point. RC_SLOW is nominally ~136kHz; RC_FAST/256 is 31.25kHz and an
        // external 32k crystal is 32.768kHz. A period implying ~32kHz while the
        // code assumes ~136kHz is a 4.3x overshoot, which is the ratio the
        // 5-minute heartbeat actually showed.
        {
            const uint32_t cal = esp_clk_slowclk_cal_get();
            const double period_us = (double)cal / (1 << 19);
            const double hz = period_us > 0 ? 1e6 / period_us : 0;
            Serial.printf("[RTC] slow clock cal=%u -> period %.3f us -> %.0f Hz\n",
                          (unsigned)cal, period_us, hz);
            Serial.printf("[RTC] a 60s sleep therefore counts %.0f ticks\n",
                          60e6 / (period_us > 0 ? period_us : 1));
        }

        // Everything the LilyGo example turns off. DCDC1 is deliberately absent:
        // it is the 3.3V rail the ESP32 itself runs on, and the daughter board
        // with it.
        PMU.disableDC2(); PMU.disableDC3(); PMU.disableDC4(); PMU.disableDC5();
        PMU.disableALDO1(); PMU.disableALDO2(); PMU.disableALDO3(); PMU.disableALDO4();
        PMU.disableBLDO1(); PMU.disableBLDO2();
        PMU.disableCPUSLDO(); PMU.disableDLDO1(); PMU.disableDLDO2();
        PMU.disableVbusVoltageMeasure();
        PMU.disableBattVoltageMeasure();
        PMU.disableSystemVoltageMeasure();
        PMU.disableTemperatureMeasure();

        Serial.println("[Minimal] Sleeping 60s on the timer alone. No ext1, no holds.");
        Serial.flush();
        esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
#endif

#ifdef SLEEP_TEST_MODE
    // Isolates ONE variable: whether powerPrepareForSleep()'s pad holds are what
    // stop an ext1 wake on GPIO12.
    //
    // Everything else has been measured on this board and works. The LIS3DH
    // latches on a real shake at the shipping 352mg over 60ms, INT1 reaches
    // GPIO12, and the high-pass filter removes gravity to 24mg at rest. So a
    // tracker that still does not wake is failing between esp_sleep_enable_
    // ext1_wakeup() and the pad, and the only thing this firmware does in
    // between is gpio_hold_en() on two other RTC pads plus a blanket
    // gpio_deep_sleep_hold_en().
    //
    // Suspicion, and the reason this is worth a build of its own: the holds are
    // applied BEFORE esp_deep_sleep_start(), and it is inside that call that IDF
    // runs ext1_wakeup_prepare() and re-points GPIO12 at the RTC IO mux. An
    // autohold already in force is a plausible way for that configuration to be
    // frozen out -- and it would look exactly like this, with everything working
    // awake and nothing waking asleep.
    //
    // Alternating in RTC memory rather than by rebuilding means both cases run
    // against the same board, the same arm, and the same person shaking it.
    {
        // The log lives in NVS, not RTC memory, and that is the whole point.
        //
        // Opening the USB CDC port RESETS this board -- the S3's native USB
        // implements the auto-programming reset on the CDC control lines -- so
        // attaching to watch a wake destroys the evidence of it: RTC memory is
        // wiped and esp_sleep_get_wakeup_cause() reports UNDEFINED. Every boot
        // banner this session has claimed "Cold Boot" for exactly that reason.
        // Flash survives the reset, so the wake cause is written down at the
        // moment it is still true and read back later, when attaching no longer
        // costs anything.
        //
        // The procedure this build is built for: flash it, LEAVE THE PORT ALONE
        // for several cycles, shake it during some of the sleeps, and only then
        // attach to read the history back.
        Serial.println("\n=== DEEP SLEEP WAKE TEST ===");

        prefs.begin("sleeptest", false);
        String log = prefs.getString("log", "");
        const int pass = prefs.getUInt("pass", 0);

        // Recorded before anything else can disturb it.
        const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        const char* causeStr = (cause == ESP_SLEEP_WAKEUP_EXT1)  ? "MOTION"
                             : (cause == ESP_SLEEP_WAKEUP_TIMER) ? "timer"
                             : (cause == ESP_SLEEP_WAKEUP_UNDEFINED) ? "reset/cold"
                             : "other";
        // pass is the count of sleeps ALREADY taken, so the one being reported
        // here is pass-1, and its holds setting follows the same alternation.
        // Every pass is now the exact shipping arm. The control has served its
        // purpose: seven of seven control sleeps woke on ext1, so the wake
        // source, GPIO12 and the deep-sleep path are all proven good and there
        // is nothing left to compare against. What is unresolved is narrower --
        // whether the LIS3DH fires at all on real motion while the ESP32 sleeps
        // -- and IA answers exactly that.
        const bool prevWasControl = false;

        if (pass > 0) {
            // IA is the whole question now. ext1 did not wake this board with
            // holds on OR off, while the same part latches reliably on a shake
            // when awake. Those two facts only reconcile one of two ways, and
            // the latch tells them apart: LIR_INT1 holds INT1 high until
            // INT1_SRC is read, so a fire during sleep is still recorded at the
            // next wake, whatever ended it.
            //
            //   IA=1 on a timer wake -> the LIS3DH fired and ext1 ignored it.
            //   IA=0 on a timer wake -> the part never fired while asleep at all,
            //                           despite firing on the same gesture awake.
            const uint8_t wsrc = imuWakeSrc();
            const uint8_t wcfg = imuWakeCfg();
            char entry[96];
            snprintf(entry, sizeof(entry), "%d:%s:%s:IA%s:cfg%02X ", pass, causeStr,
                     prevWasControl ? "CONTROL" : "motion",
                     (wsrc == 0xFF) ? "?" : ((wsrc & 0x40) ? "1" : "0"),
                     wcfg);
            log += entry;
            if (log.length() > 400) log = log.substring(log.length() - 400);
            prefs.putString("log", log);
        }

        Serial.printf("[SleepTest] History (pass:wokeby:mode:IA:cfg): %s\n",
                      log.length() ? log.c_str() : "(none yet)");
        // ext1 is already proven good, so every reading here is about the sensor.
        if (log.indexOf("MOTION:motion:IA1") >= 0) {
            Serial.println("[SleepTest] >>> WORKS: the part fired on real motion while asleep and ext1 woke it.");
        } else if (log.indexOf("timer:motion:IA1") >= 0) {
            Serial.println("[SleepTest] >>> The part DID fire while asleep but the board slept on --");
            Serial.println("[SleepTest]     which contradicts the control, so look at the arm, not the wiring.");
        } else if (log.indexOf("timer:motion:IA0") >= 0) {
            Serial.println("[SleepTest] >>> So far every sleep ended on the timer with nothing latched:");
            Serial.println("[SleepTest]     the LIS3DH is not detecting the motion at 352mg/60ms while asleep.");
        }

        const bool runControl = (pass % 2) == 0;
        prefs.putUInt("pass", pass + 1);
        prefs.end();

        // Attaching a terminal to this board resets it -- the S3's USB-Serial-JTAG
        // controller issues a chip reset when the host drives DTR/RTS, and macOS
        // does that on open. Fighting that turned out to be the wrong approach:
        // NVS survives a reset, so the reset is harmless as long as the board is
        // still awake afterwards to be read from and flashed.
        //
        // So a reset buys a grace window and a real sleep wake does not. Every
        // attach lands in this branch (a reset reports UNDEFINED), which is what
        // makes the port reliably catchable instead of a 2-second lottery -- and
        // it is why this stopped needing a BOOT+RST between every flash.
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            Serial.println("[SleepTest] Reset detected -- holding awake 20s so this can be read and reflashed.");
            for (int i = 0; i < 4; i++) {
                delay(5000);
                Serial.printf("[SleepTest] (awake %ds) History: %s\n", (i + 1) * 5, log.c_str());
            }
        }

        (void)runControl;
        Serial.printf("[SleepTest] Mode: shipping arm (%umg, DUR=3, HPF on, settled).\n",
                      g_motionThresholdMg);
        imuEnableMotionWake(g_motionThresholdMg);   // settles the filter itself now

        // If this is not low, the arm left INT1 asserted and the sleep will end
        // instantly for reasons that have nothing to do with being shaken --
        // which is precisely the artifact that made the last run unreadable.
        const bool int1Low = (digitalRead(DB_IMU_INT1) == LOW);
        Serial.printf("[SleepTest] Sleeping 30s. INT1 reads %s going in%s\n",
                      int1Low ? "low (correct)" : "HIGH",
                      int1Low ? "." : " -- expect an immediate, meaningless wake.");
        Serial.println("[SleepTest] Run this on BATTERY -- with USB attached every wake is reported");
        Serial.println("[SleepTest]   as a cold boot and the result is meaningless.");
        Serial.flush();

        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);

        esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
        esp_sleep_enable_ext1_wakeup(1ULL << DB_IMU_INT1, ESP_EXT1_WAKEUP_ANY_HIGH);
        esp_deep_sleep_start();
    }
#endif

#ifdef SPEED_SURVEY_MODE
    // Finding the speed signal needs CAN bytes and a speed reference in the
    // same log. GPS is that reference: independent of the bus, already aboard,
    // and honest about its own quality via sats and HDOP. No MQTT, no sleep --
    // ride, then read the CSV back and correlate offline exactly as RPM was
    // correlated against the tacho.
    //
    // Published sources put SeaDoo speed on 0x208 or 0x268. Neither ID exists
    // on this bus (see docs/can/seadoo-can.md), so it has to be found rather
    // than looked up, and this emits every byte of every ID rather than
    // guessing which to watch.
    modemPowerOn();
    initGNSS();
    if (!canBeginListenOnly(CanBitrate::Rate500k)) {
        Serial.println("[SURVEY] Could not open the CAN bus. Halting.");
        while (true) delay(1000);
    }
    canResetSeen();

    Serial.println("\n=== SPEED SURVEY ===");
    Serial.println("Ride at a few steady speeds and hold each for ~15s.");
    Serial.println("---8<--- BEGIN SURVEY CSV ---8<---");
    Serial.println("t_ms,gps_kmh,sats,hdop,id,d0,d1,d2,d3,d4,d5,d6,d7");

    const uint32_t t0 = millis();
    float gpsKmh = -1; int gpsSats = 0; float gpsHdop = 99;
    while (true) {
        canDrain(400);

        modem.sendAT("+CGNSINF");
        if (modem.waitResponse(800L, "+CGNSINF: ") == 1) {
            String res = modem.stream.readStringUntil('\n');
            res.trim();
            float la, lo, sp, al, hd, pd; int st;
            if (parseCGNSINF(res, &la, &lo, &sp, &al, &st, &hd, &pd)) {
                gpsKmh = sp; gpsSats = st; gpsHdop = hd;
            }
        }

        const uint32_t ms = millis() - t0;
        for (size_t i = 0; i < canSeenCount(); i++) {
            const CanFrameSummary* f = canSeen(i);
            if (!f) continue;
            Serial.printf("%lu,%.2f,%d,%.2f,%03X", (unsigned long)ms, gpsKmh, gpsSats, gpsHdop, f->id);
            for (int b = 0; b < 8; b++) Serial.printf(",%02X", f->lastData[b]);
            Serial.println();
        }
    }
#endif

#ifdef CAN_SNIFFER_MODE
    // Bring-up build: never returns. Deliberately after powerBegin() so the
    // transceiver starts from a defined state, and before anything touches the
    // modem -- none of that is wanted with a laptop on the diag port.
    canSnifferLoop();
#endif
    
    // 4KB rather than the 256-byte default, because the OTA raises this link
    // to 460800 for a transfer. At 256 bytes that holds only 5.6ms of data at
    // 460800 -- any scheduling delay longer than that drops bytes, corrupts
    // the AT framing, and TinyGSM then waits per byte for data that will
    // never arrive. 4KB is 89ms: a margin rather than a race. At 115200 the
    // default was fine, which is why this never mattered until now.
    Serial1.setRxBufferSize(4096);
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    loadSettings();
    checkPowerConfig();

    // A motion wake this soon after a report has nothing new to say, and saying
    // it would cost a GPS acquisition and a modem session. Bail before
    // modemPowerOn(), which is where the expense starts.
    if (wokeOnMotion()) {
        time_t since = time(NULL) - s_lastReportTime;
        if (since < MOTION_MIN_REPORT_S) {
            // Counted before sleeping, because this wake is evidence even though
            // it is not worth a report: the ladder needs to know how often the
            // threshold is being tripped, not just how often it reports.
            s_suppressedWakes++;
            Serial.printf("[Motion] Wake %llds after last report (min %llds) - back to sleep. (%d suppressed)\n",
                          (long long)since, (long long)MOTION_MIN_REPORT_S, s_suppressedWakes);
            enterDeepSleep(settings.report_interval_mins);
        }
        Serial.printf("[Motion] Wake %llds after last report - reporting.\n", (long long)since);
    }

    modemPowerOn(); 
    initGNSS(); // Start GPS early
    
    if (shouldRunBLEWindow()) {
        runBLEWindow(15000);
    }
    
    float lat=0, lon=0, speed=0, alt=0, hdop=99, pdop=99; 
    int sats=0;
    bool has_fix = getPreciseLocation(&lat, &lon, &speed, &alt, &sats, &hdop, &pdop);
    
    modem.sendAT("+CGNSPWR=0");
    modem.waitResponse();

    // A second on the ski's bus. Cheap at ~980 frames/s -- everything of
    // interest repeats at 50-100Hz -- and a quiet bus just means ignition off,
    // which is the normal state for a parked ski.
    g_engine = canReadEngine(1000);

    const bool published = transmitData(has_fix, lat, lon, speed, alt, sats, hdop);

    // After the report, so a threshold change is decided on the same fix that
    // was just published and re-arms before this wake ends.
    updateMotionThreshold(has_fix, speed, lat, lon, pdop);

    goToSleep(has_fix, published);
}

void loop() {
    // Empty
}

