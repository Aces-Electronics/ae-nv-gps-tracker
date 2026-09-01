#define TINY_GSM_DEBUG Serial
#include <Arduino.h>
#include "utilities.h"
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
static const uint32_t PARKED_INTERVAL_MINS   = 1440;

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
//   Low    704mg  ~29x noise  -- a real knock. For a ski somewhere lively, where
//                               anything less would report the weather.
//   Medium 176mg  ~7.3x noise -- the default. Registers ordinary handling.
//   High    64mg  ~2.7x noise -- for a quiet mooring where a nudge should count.
//
// All three sit exactly on the INT1_THS grid (16mg steps at +/-2g): 44, 11 and 4
// counts, so none of them truncate. That is worth checking when retuning -- 56mg,
// for instance, is not representable and would silently become 48mg, only 2x the
// noise floor and closer to chasing the filter's own residue than detecting
// anything.
static const uint16_t MOTION_SENS_LOW_MG    = 704;
static const uint16_t MOTION_SENS_MEDIUM_MG = 176;
static const uint16_t MOTION_SENS_HIGH_MG   = 64;

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
    
    Serial.println("Settings Loaded from NVS.");
    Serial.printf("[Settings] Broker: %s\n", settings.mqtt_broker.c_str());
    Serial.printf("[Settings] Name: %s\n", settings.name.c_str());
    Serial.printf("[Settings] Interval: %d mins\n", settings.report_interval_mins);
    Serial.printf("[Settings] Motion sensitivity: %s (%umg floor)\n",
                  motionSensitivityName(settings.motion_sensitivity),
                  motionBaseFor(settings.motion_sensitivity));
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
static const uint16_t MOTION_THRESHOLD_STEP_MG = 176;
static const uint16_t MOTION_THRESHOLD_MAX_MG  = 1408;

// Consecutive unexplained motion wakes before the threshold goes up. Three,
// because one is noise and two is a coincidence.
static const int FALSE_WAKES_BEFORE_RAISE = 3;

// Under this, GPS is reporting its own noise rather than travel. A stationary
// receiver commonly shows a knot or two.
static const float STATIONARY_SPEED_KMH = 2.0f;

RTC_DATA_ATTR static int s_falseWakeCount = 0;

// Counts passes of the deep-sleep wake test so it can alternate the pad holds
// across sleeps instead of needing two builds.
RTC_DATA_ATTR static int s_sleepTestPass = 0;

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
    if (v > MOTION_THRESHOLD_MAX_MG)  v = MOTION_THRESHOLD_MAX_MG;
    Serial.printf("[Motion] Sensitivity %s: floor %umg, current threshold %umg\n",
                  motionSensitivityName(sens), g_motionBaseMg, v);
    return v;
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
static void updateMotionThreshold(bool gotFix, float speedKmh) {
    if (!wokeOnMotion()) return;

    if (!gotFix) {
        // No fix is not evidence. Raising the threshold on a wake that could not
        // be classified would slowly deafen the tracker for reasons that have
        // nothing to do with motion -- a garage roof is not a false positive.
        Serial.println("[Motion] Motion wake with no fix; threshold unchanged.");
        return;
    }

    if (speedKmh >= STATIONARY_SPEED_KMH) {
        s_falseWakeCount = 0;
        if (g_motionThresholdMg != g_motionBaseMg) {
            Serial.printf("[Motion] Real travel at %.1f km/h -> threshold back to %umg\n",
                          speedKmh, g_motionBaseMg);
            g_motionThresholdMg = g_motionBaseMg;
            saveMotionThreshold(g_motionThresholdMg);
            imuEnableMotionWake(g_motionThresholdMg);
        }
        return;
    }

    s_falseWakeCount++;
    Serial.printf("[Motion] Motion wake but GPS says %.1f km/h (%d/%d unexplained)\n",
                  speedKmh, s_falseWakeCount, FALSE_WAKES_BEFORE_RAISE);

    if (s_falseWakeCount < FALSE_WAKES_BEFORE_RAISE) return;
    s_falseWakeCount = 0;

    if (g_motionThresholdMg >= MOTION_THRESHOLD_MAX_MG) {
        Serial.printf("[Motion] Already at the %umg ceiling; not going deafer than this.\n",
                      MOTION_THRESHOLD_MAX_MG);
        return;
    }

    uint16_t next = g_motionThresholdMg + MOTION_THRESHOLD_STEP_MG;
    if (next > MOTION_THRESHOLD_MAX_MG) next = MOTION_THRESHOLD_MAX_MG;
    g_motionThresholdMg = next;
    saveMotionThreshold(g_motionThresholdMg);
    // Re-arm now: the level in the part is whatever was set at boot, and the
    // new one has to be in place before the next sleep.
    imuEnableMotionWake(g_motionThresholdMg);
    Serial.printf("[Motion] Threshold raised to %umg\n", g_motionThresholdMg);
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

void goToSleep(bool got_fix) {
    modemPowerOff();

    // CAN traffic is the signal that the ski is running: the bus only wakes
    // with the ignition. Supply voltage above ~13V would say the same thing and
    // is the better test -- it survives a CAN fault -- but the supply-sense
    // divider is mis-scaled and reads nothing usable yet (docs/can and the
    // R15/R16 note in utilities.h), so this is the one signal available.
    const bool skiRunning = g_engine.busAlive;

    prefs.begin("tracker", false);
    int fails = prefs.getUInt("gps_fail", 0);
    int actual_interval = skiRunning ? (int)settings.report_interval_mins
                                     : (int)PARKED_INTERVAL_MINS;
    if (actual_interval == 0) actual_interval = 5;
    Serial.printf("[Lifecycle] Ski %s -> %d minute interval\n",
                  skiRunning ? "RUNNING" : "parked", actual_interval);

    if (got_fix) {
        if (fails > 0) {
            Serial.printf("[Backoff] Fix obtained! Resetting fail count (was %d)\n", fails);
            prefs.putUInt("gps_fail", 0);
        }
    } else {
        fails++;
        prefs.putUInt("gps_fail", fails);
        Serial.printf("[Backoff] No Fix this session. Consecutive Fails: %d\n", fails);
        
        // Backoff Strategy (Exponential-ish if failing)
        if (fails >= 5) actual_interval = 180;      // 3 hours
        else if (fails == 4) actual_interval = 60;  // 1 hour
        else if (fails == 3) actual_interval = 30;  // 30 mins
        else if (fails == 2) actual_interval = 15;  // 15 mins
        else if (fails == 1) actual_interval = 5;   // 5 mins
        
        // Backoff only means anything while the ski is running. Parked, the
        // interval is already a day and stretching it further on a missed fix
        // would trade away the one thing a heartbeat is for.
        if (!skiRunning) {
            actual_interval = PARKED_INTERVAL_MINS;
        } else if (actual_interval < (int)settings.report_interval_mins) {
            actual_interval = settings.report_interval_mins;
        }
        Serial.printf("[Backoff] Applying Backoff Sleep: %d minutes\n", actual_interval);
    }
    prefs.end();

    enterDeepSleep(actual_interval);
}
// --- Custom CGNSINF Parser for SIM7080G ---
bool parseCGNSINF(String raw, float* lat, float* lon, float* speed, float* alt, int* sats, float* hdop) {
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

bool getPreciseLocation(float* lat, float* lon, float* speed, float* alt, int* sats, float* hdop) {
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
            
            float f_lat=0, f_lon=0, f_speed=0, f_alt=0, f_acc=0;
            int f_sats=0;
            
            if (parseCGNSINF(res, &f_lat, &f_lon, &f_speed, &f_alt, &f_sats, &f_acc)) {
                Serial.printf("[GPS] Valid! Lat=%.4f Lon=%.4f Sats=%d HDOP=%.2f\n", f_lat, f_lon, f_sats, f_acc);
                *lat = f_lat; *lon = f_lon; *speed = f_speed; *alt = f_alt; *sats = f_sats; *hdop = f_acc;

                if (f_acc < 1.5 || (f_acc < 2.5 && f_sats >= 4)) {
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

void onDownlink(char* topic, uint8_t* payload, unsigned int length) {
    Serial.printf("[MQTT] Downlink on %s (%u bytes)\n", topic, length);

    // Only OTA is subscribed, so the topic is not re-checked here. A zero-length
    // payload is the backend withdrawing the retained command; otaParseCommand()
    // rejects it and g_otaPending stays false.
    if (otaParseCommand(payload, length, g_otaCmd)) {
        g_otaPending = true;
        Serial.printf("[OTA] Command: v%s %s\n", g_otaCmd.version.c_str(), g_otaCmd.url.c_str());
    }
}

void transmitData(bool has_fix, float lat, float lon, float speed, float alt, int sats, float hdop) {
    Serial.println("[Lifecycle] Preparing Transmission...");

    // Sampled here rather than in setup() so the reported orientation is the one
    // at report time: GPS acquisition ahead of this can run for five minutes,
    // and a value from before that is not describing the same situation.
    ImuReading imu = imuRead();

    // Same reasoning as the IMU: read at report time, not at boot. The policy is
    // applied here too, so a decision to cut or restore the jetski feed is made
    // against the battery voltage that is about to be published alongside it.
    PowerStatus power = powerRead();
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
    Serial.printf("[MQTT] MAC/IMEI: %s\n", imei.c_str());
    Serial.printf("[MQTT] Topic: %s\n", mqtt_topic_up.c_str());
    Serial.printf("[MQTT] Downlink: %s\n", mqtt_topic_dn.c_str());

    Serial.println("[Lifecycle] Connecting to Network...");
    
    modem.sendAT("+CFUN=1");
    modem.waitResponse(2000L);

    Serial.print("[Lifecycle] Waiting for Network...");
    if (!modem.waitForNetwork(180000L)) {
        Serial.println("Fail: Network Timeout");
        return;
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
            doc["fix"] = has_fix;
            if (has_fix) {
                doc["lat"] = lat;
                doc["lon"] = lon;
                doc["alt"] = alt;
                doc["speed"] = speed;
                doc["sats"] = sats;
                doc["hdop"] = hdop;
            }
            
            doc["voltage"] = PMU.getVbusVoltage() / 1000.0F; 
            doc["device_voltage"] = PMU.getBattVoltage() / 1000.0F; 
            doc["battery_voltage"] = PMU.getBattVoltage() / 1000.0F; 
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
            doc["orientation"] = orientationName(imu.orientation);
            doc["charge_state"] = chargeStateName(power.state);
            doc["supply_enabled"] = power.supplyEnabled;
            doc["wake_reason"] = wakeReasonName();

            // Vehicle frame, so the backend never has to know how the board is
            // mounted. Two decimals is well inside what a +/-2g part resolves.
            if (imu.valid) {
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
            if (g_engine.tempValid) doc["engine_temp_raw"] = g_engine.tempRaw;

            // Published so the adaptive logic can be seen working from the
            // backend rather than only from a serial cable.
            doc["motion_threshold_mg"] = g_motionThresholdMg;
            // The setting, alongside where the ladder currently sits. Both are
            // wanted: the first is what somebody chose, the second is what the
            // ski decided, and a support question about false wakes needs to
            // tell them apart.
            doc["motion_sensitivity"] = motionSensitivityName(settings.motion_sensitivity);

            int csq = modem.getSignalQuality();
            int dbm = (csq == 99) ? -113 : (csq * 2) - 113;
            doc["rssi"] = dbm;
            
            doc["interval"] = settings.report_interval_mins;
            
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
            float la, lo, sp, al, hd; int st;
            if (parseCGNSINF(res, &la, &lo, &sp, &al, &st, &hd)) {
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
    
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    loadSettings();
    checkPowerConfig();

    // A motion wake this soon after a report has nothing new to say, and saying
    // it would cost a GPS acquisition and a modem session. Bail before
    // modemPowerOn(), which is where the expense starts.
    if (wokeOnMotion()) {
        time_t since = time(NULL) - s_lastReportTime;
        if (since < MOTION_MIN_REPORT_S) {
            Serial.printf("[Motion] Wake %llds after last report (min %llds) - back to sleep.\n",
                          (long long)since, (long long)MOTION_MIN_REPORT_S);
            enterDeepSleep(settings.report_interval_mins);
        }
        Serial.printf("[Motion] Wake %llds after last report - reporting.\n", (long long)since);
    }

    modemPowerOn(); 
    initGNSS(); // Start GPS early
    
    if (shouldRunBLEWindow()) {
        runBLEWindow(15000);
    }
    
    float lat=0, lon=0, speed=0, alt=0, hdop=99; 
    int sats=0;
    bool has_fix = getPreciseLocation(&lat, &lon, &speed, &alt, &sats, &hdop);
    
    modem.sendAT("+CGNSPWR=0");
    modem.waitResponse();

    // A second on the ski's bus. Cheap at ~980 frames/s -- everything of
    // interest repeats at 50-100Hz -- and a quiet bus just means ignition off,
    // which is the normal state for a parked ski.
    g_engine = canReadEngine(1000);

    transmitData(has_fix, lat, lon, speed, alt, sats, hdop);

    // After the report, so a threshold change is decided on the same fix that
    // was just published and re-arms before this wake ends.
    updateMotionThreshold(has_fix, speed);

    goToSleep(has_fix);
}

void loop() {
    // Empty
}

