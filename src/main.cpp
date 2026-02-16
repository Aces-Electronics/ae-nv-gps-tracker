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

// --- Configuration ---
TrackerSettings settings;
TrackerStatus status;
Preferences prefs;

// --- Globals ---
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
XPowersAXP2101 PMU;
BLEHandler ble;

String imei = "";
String mqtt_topic_up = "";
bool stay_awake = false; // Enable Sleep for Backoff Strategy

void loadSettings() {
    prefs.begin("tracker", false);
    settings.name = prefs.getString("name", "");
    settings.apn = prefs.getString("apn", "hologram");
    settings.mqtt_broker = prefs.getString("broker", "mqtt.aceselectronics.com.au");
    settings.mqtt_user = prefs.getString("user", "aesmartshunt");
    settings.mqtt_pass = prefs.getString("pass", "AERemoteAccess2024!");
    settings.report_interval_mins = prefs.getUInt("interval", 1); 
    
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

void goToSleep(bool got_fix) {
    modemPowerOff();

    prefs.begin("tracker", false);
    int fails = prefs.getUInt("gps_fail", 0);
    int actual_interval = settings.report_interval_mins;

    if (got_fix) {
        if (fails > 0) {
            Serial.printf("[Backoff] Fix obtained! Resetting fail count (was %d)\n", fails);
            prefs.putUInt("gps_fail", 0);
        }
    } else {
        fails++;
        prefs.putUInt("gps_fail", fails);
        Serial.printf("[Backoff] No Fix this session. Consecutive Fails: %d\n", fails);
        
        // Backoff Strategy
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

    uint64_t sleep_time = (uint64_t)actual_interval * 60 * 1000000ULL;
    if (sleep_time == 0) sleep_time = 60 * 1000000ULL;

    Serial.printf("Entering Deep Sleep for %d minutes...\n", actual_interval);
    esp_sleep_enable_timer_wakeup(sleep_time);
    esp_deep_sleep_start();
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

void pollGPSDiagnostic() {
    modem.sendAT("+CGNSINF");
    if (modem.waitResponse(1000L, "+CGNSINF: ") == 1) {
        String res = modem.stream.readStringUntil('\n');
        res.trim();
        Serial.printf("[GPS-RAW] [%s]\n", res.c_str());
        
        // Quick parse for logs: <run>,<fix>,...
        int firstComma = res.indexOf(',');
        int secondComma = res.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
            String fixStr = res.substring(firstComma + 1, secondComma);
            if (fixStr.length() == 0) fixStr = "0";
            
            // Extract SatsView (index 14)
            int commaCount = 0;
            String satsView = "0";
            int from = 0;
            for (int i=0; i<15; i++) {
                int next = res.indexOf(',', from);
                if (next == -1) {
                    if (i == 14) satsView = res.substring(from);
                    break;
                }
                if (i == 14) {
                    satsView = res.substring(from, next);
                }
                from = next + 1;
            }
            if (satsView.length() == 0) satsView = "0";
            Serial.printf("[GPS] Background... Fix=%s SatsView=%s\n", fixStr.c_str(), satsView.c_str());
        }
    }
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
                // Diagnostic Logging
                int firstComma = res.indexOf(',');
                int secondComma = res.indexOf(',', firstComma + 1);
                if (firstComma != -1 && secondComma != -1) {
                    String fixStatus = res.substring(firstComma + 1, secondComma);
                    if (fixStatus.length() == 0) fixStatus = "0";
                    
                    // Sats View is at index 14
                    int commaCount = 0;
                    String sv = "0";
                    int from = 0;
                    for (int i=0; i<15; i++) {
                        int next = res.indexOf(',', from);
                        if (next == -1) {
                            if (i == 14) sv = res.substring(from);
                            break;
                        }
                        if (i == 14) {
                            sv = res.substring(from, next);
                        }
                        from = next + 1;
                    }
                    if (sv.length() == 0) sv = "0";
                    Serial.printf("[GPS] Wait... FixStatus=%s SatsView=%s\n", fixStatus.c_str(), sv.c_str());
                }
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

void transmitData(float lat, float lon, float speed, float alt, int sats, float hdop) {
    Serial.println("[Lifecycle] Preparing Transmission...");
    
    imei = getIMEIWithRetry();
    mqtt_topic_up = "ae-nv/tracker/" + imei + "/up";
    Serial.printf("[MQTT] MAC/IMEI: %s\n", imei.c_str());
    Serial.printf("[MQTT] Topic: %s\n", mqtt_topic_up.c_str());

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
        mqtt.setBufferSize(512); // Ensure we can send full JSON
        if (mqtt.connect(imei.c_str(), settings.mqtt_user.c_str(), settings.mqtt_pass.c_str())) {
            Serial.println(" Connected");
            
            StaticJsonDocument<512> doc;
            doc["mac"] = imei;
            
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
            
            int csq = modem.getSignalQuality();
            int dbm = (csq == 99) ? -113 : (csq * 2) - 113;
            doc["rssi"] = dbm;
            
            doc["interval"] = settings.report_interval_mins;
            
            String payload;
            serializeJson(doc, payload);
            Serial.println("[MQTT] Publishing: " + payload);
            if (mqtt.publish(mqtt_topic_up.c_str(), payload.c_str())) {
                Serial.println("[MQTT] Publish Successful");
            } else {
                Serial.printf("[MQTT] Publish FAILED (State: %d)\n", mqtt.state());
            }
            
            delay(1000);
            mqtt.disconnect();
        } else {
             Serial.printf(" FAILED (State: %d)\n", mqtt.state());
        }
        modem.gprsDisconnect();
    } else {
        Serial.println(" GPRS FAILED");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("\n--- AE Tracker Boot (Reason: %d) ---\n", reason);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("PMU FAIL");
    }
    PMU.disableTSPinMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableBattDetection();
    PMU.enableCellbatteryCharge();
    
    pinMode(0, INPUT_PULLUP);
    strip.begin();
    strip.show();
    
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    loadSettings();
    checkPowerConfig();
    
    modemPowerOn(); 
    initGNSS(); // Start GPS early
    
    runBLEWindow(30000); 
    
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

