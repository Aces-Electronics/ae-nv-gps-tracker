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

OtaCommand g_otaCmd;
bool g_otaPending = false;

// CI passes the release number in as OTA_VERSION; a bench build has none. An
// empty string is reported as-is rather than as a made-up number, because the
// backend matches an OTA command against exactly this value and a placeholder
// would close commands the device never applied.
const char* firmwareVersion() {
    return FIRMWARE_VERSION;
}

void loadSettings() {
    prefs.begin("tracker", false);
    settings.name = prefs.getString("name", "");
    settings.apn = prefs.getString("apn", "hologram");
    settings.mqtt_broker = prefs.getString("broker", "mqtt.aceselectronics.com.au");
    settings.mqtt_user = prefs.getString("user", "aesmartshunt");
    settings.mqtt_pass = prefs.getString("pass", "AERemoteAccess2024!");
    settings.report_interval_mins = prefs.getUInt("interval", 5);
    if (settings.report_interval_mins < 5) settings.report_interval_mins = 5;

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
        // The interrupt latches, so a source left uncleared holds INT1 high and
        // the next sleep would end the instant it started.
        imuClearMotionInterrupt();
        esp_sleep_enable_ext1_wakeup(1ULL << DB_IMU_INT1, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    powerPrepareForSleep();
    Serial.printf("Entering Deep Sleep for %d minutes%s...\n",
                  minutes, motionArmed ? " (or on motion)" : "");
    Serial.flush();
    esp_deep_sleep_start();
}

void goToSleep(bool got_fix) {
    modemPowerOff();

    prefs.begin("tracker", false);
    int fails = prefs.getUInt("gps_fail", 0);
    int actual_interval = settings.report_interval_mins;
    if (actual_interval == 0) actual_interval = 5;

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
        
        if (actual_interval < settings.report_interval_mins) {
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

void transmitData(float lat, float lon, float speed, float alt, int sats, float hdop) {
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
        mqtt.setBufferSize(768); // Send the full JSON, and receive an OTA command in one frame
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

            
            StaticJsonDocument<768> doc;
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
            
            doc["lat"] = lat;
            doc["lon"] = lon;
            doc["alt"] = alt;
            doc["speed"] = speed;
            doc["sats"] = sats;
            doc["hdop"] = hdop;
            
            doc["voltage"] = PMU.getVbusVoltage() / 1000.0F; 
            doc["device_voltage"] = PMU.getBattVoltage() / 1000.0F; 
            doc["battery_voltage"] = PMU.getBattVoltage() / 1000.0F; 
            doc["soc"] = PMU.getBatteryPercent();

            // Always emitted, "Unknown" included: the field has been in the
            // documented payload all along, and a key that appears only on
            // boards with a working IMU is harder for the backend to reason
            // about than one that is always there.
            doc["orientation"] = orientationName(imu.orientation);
            doc["charge_state"] = chargeStateName(power.state);
            doc["supply_enabled"] = power.supplyEnabled;
            doc["wake_reason"] = wakeReasonName();

            int csq = modem.getSignalQuality();
            int dbm = (csq == 99) ? -113 : (csq * 2) - 113;
            doc["rssi"] = dbm;
            
            doc["interval"] = settings.report_interval_mins;
            
            String payload;
            serializeJson(doc, payload);
            Serial.println("[MQTT] Publishing: " + payload);
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
    Serial.printf("\n--- AE Tracker Boot (Reason: %d, Wake: %s, FW: '%s') ---\n",
                  reason, wakeReasonName(), firmwareVersion());

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("PMU FAIL");
    }
    PMU.disableTSPinMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableBattDetection();
    PMU.enableCellbatteryCharge();

    // Daughter board. Absence is reported and then ignored -- see imuBegin().
    // powerBegin() comes first because it is what releases the pad hold from
    // the last sleep and gets the CAN transceiver out of an undefined state.
    powerBegin();
    if (imuBegin()) {
        imuEnableMotionWake();
    }

    pinMode(0, INPUT_PULLUP);
    strip.begin();
    strip.show();
    
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
    
    transmitData(lat, lon, speed, alt, sats, hdop);
    goToSleep(has_fix);
}

void loop() {
    // Empty
}

