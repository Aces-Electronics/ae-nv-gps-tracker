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
    
    prefs.end();
    Serial.println("Settings Loaded from NVS.");
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
    // DC3: Modem Power (3.0V per LilyGo Examples for T-SIM7080G-S3)
    // Range is 2700~3400mV. 3000mV is safe default.
    PMU.setDC3Voltage(3000); 
    PMU.enableDC3();
    
    // ALDOs for level shifters
    PMU.enableALDO1();
    PMU.enableALDO2();
    PMU.enableALDO3();
    PMU.enableALDO4();

    // BLDO2: GPS Antenna
    PMU.setBLDO2Voltage(3300);
    PMU.enableBLDO2();

    digitalWrite(MODEM_PWRKEY, LOW);
    delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    
    int retry = 0;
    while (!modem.testAT(1000) && retry < 10) {
        Serial.print(".");
        retry++;
    }
    if (modem.testAT()) {
        Serial.println("\nModem ON");
        modem.sendAT("+CFUN=0"); // Minimum functionality
        modem.waitResponse(2000L);
        modem.sendAT("+CFUN=1"); // Full functionality
        modem.waitResponse(2000L);
    } else {
        Serial.println("\nModem Power FAIL");
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

    // Index 3: Lat, 4: Lon (Verify they are not empty)
    if (parts[3].length() == 0 || parts[4].length() == 0) return false;
    
    *lat = parts[3].toFloat();
    *lon = parts[4].toFloat();
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
    strip.setPixelColor(0, 0, 0, 255); // Blue
    strip.show();

    while (millis() - start < duration_ms) {
        ble.loop();
        if (digitalRead(0) == LOW) {
            Serial.println("[BLE] Boot Button Pressed - Extending Window!");
            start = millis(); // Reset timer if interact
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
    
    BLEDevice::deinit(); // Save RAM/Power? Or just let it die with deep sleep
    Serial.println("[BLE] Window Closed.\n");
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
}

bool getPreciseLocation(float* lat, float* lon, float* speed, float* alt, int* sats, float* hdop) {
    Serial.println("[Lifecycle] Acquiring GPS Fix...");
    strip.setPixelColor(0, 255, 165, 0); // Orange
    strip.show();

    // Power ON GNSS
    modem.sendAT("+CGNSPWR=1");
    modem.waitResponse();
    modem.sendAT("+CGNSSEQ=\"gps;glonass;beidou;galileo\"");
    modem.waitResponse();
    modem.sendAT("+CGNSAN=1"); // Active Antenna
    modem.waitResponse();

    unsigned long start = millis();
    // Timeout: 120s max for fix
    unsigned long timeout = 120000; 
    
    bool locked = false;
    float last_pdop = 99.9;
    unsigned long stable_start = 0;

    while (millis() - start < timeout) {
        modem.sendAT("+CGNSINF");
        String res = "";
        if (modem.waitResponse(1000L, "+CGNSINF: ") == 1) {
            res = modem.stream.readStringUntil('\n');
            res.trim();
            Serial.printf("[GPS-RAW] %s\n", res.c_str());
        }

        float f_lat=0, f_lon=0, f_speed=0, f_alt=0, f_acc=0;
        int f_sats=0;
        
        if (parseCGNSINF(res, &f_lat, &f_lon, &f_speed, &f_alt, &f_sats, &f_acc)) {
            Serial.printf("[GPS] Fix! Sats=%d HDOP=%.2f\n", f_sats, f_acc);
            
            // Always update coordinates if we have a somewhat valid fix (HDOP < 10)
            if (f_acc < 10.0 && f_lat != 0.0) {
                *lat = f_lat; *lon = f_lon; *speed = f_speed; *alt = f_alt; *sats = f_sats; *hdop = f_acc;
            }

            // Lock Criteria: HDOP < 2.5 and Sats >= 4 OR HDOP < 2.0 (Strong Fix)
            if (f_acc < 2.0 || (f_acc < 2.5 && f_sats >= 4)) {
                locked = true;
                break; 
            }
        } else {
            Serial.print(".");
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
    Serial.println("[Lifecycle] Connecting to Network...");
    
    // Ensure Modem RF is ON (if we turned it off previously)
    modem.sendAT("+CFUN=1");
    modem.waitResponse(2000L);

    Serial.print("[Lifecycle] Waiting for Network...");
    if (!modem.waitForNetwork(180000L)) {
        Serial.println("Fail: Network Timeout");
        return;
    }
    Serial.println(" OK");
    
    if (modem.gprsConnect(settings.apn.c_str())) {
        Serial.println("[Lifecycle] GPRS Connected");
        
        if (imei == "") imei = modem.getIMEI();
        mqtt_topic_up = "ae-nv/tracker/" + imei + "/up";
        
        mqtt.setServer(settings.mqtt_broker.c_str(), 1883);
        if (mqtt.connect(imei.c_str(), settings.mqtt_user.c_str(), settings.mqtt_pass.c_str())) {
            
            StaticJsonDocument<512> doc;
            doc["mac"] = imei;
            
            String suffix = settings.name;
            if (suffix.length() == 0) {
                String mac = String((uint32_t)ESP.getEfuseMac(), HEX);
                mac.toUpperCase();
                if (mac.length() > 6) mac = mac.substring(mac.length() - 6);
                suffix = mac;
            }
            doc["model"] = "AE Tracker - " + suffix;
            
            doc["lat"] = lat;
            doc["lon"] = lon;
            doc["alt"] = alt;
            doc["speed"] = speed;
            doc["sats"] = sats;
            doc["hdop"] = hdop;
            
            doc["voltage"] = PMU.getVbusVoltage() / 1000.0F; 
            doc["device_voltage"] = PMU.getBattVoltage() / 1000.0F; // Legacy field
            doc["battery_voltage"] = PMU.getBattVoltage() / 1000.0F; 
            doc["soc"] = PMU.getBatteryPercent();
            doc["rssi"] = modem.getSignalQuality();
            doc["interval"] = settings.report_interval_mins;
            
            String payload;
            serializeJson(doc, payload);
            Serial.println("[MQTT] Publishing: " + payload);
            mqtt.publish(mqtt_topic_up.c_str(), payload.c_str());
            
            delay(500);
            mqtt.disconnect();
        } else {
             Serial.println("[Lifecycle] MQTT Connect Fail");
        }
        modem.gprsDisconnect();
    } else {
        Serial.println("[Lifecycle] GPRS Fail");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- AE Tracker Boot (Low Power Mode) ---");

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("PMU FAIL");
    }
    PMU.disableTSPinMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableBattDetection();
    PMU.enableCellbatteryCharge();
    
    // Hardware Init
    pinMode(0, INPUT_PULLUP);
    strip.begin();
    strip.show();
    
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    // Load Config
    loadSettings();
    
    // 1. Power Config Check
    checkPowerConfig();
    
    // 2. BLE Window (30s)
    modemPowerOn(); // Need modem on for some reason? No, independent.
    // Actually, keep modem OFF during BLE to save power? 
    // User flow said: "1. Power on... 2. Get GPS".
    // Let's power modem ON here anyway to warm it up OR just for BLE manuf data?
    // Manuf data needs Voltage.
    runBLEWindow(30000); 

    // 3. Acquire GPS
    // Ensure Modem/GPS is powered
    modemPowerOn(); 
    
    float lat=0, lon=0, speed=0, alt=0, hdop=99; 
    int sats=0;
    bool has_fix = getPreciseLocation(&lat, &lon, &speed, &alt, &sats, &hdop);
    
    // 4. Transmit (if we have fix OR if we want to report heartbeat?)
    // User said: "Once GPS attained... power down GPS... send payloads"
    // If NO FIX, do we send? 
    // Backoff logic handles the "No Fix" case usually.
    // For now, let's try to send even if no fix (heartbeat) or maybe just fail?
    // We'll send what we have (0,0 if fail) but the Backoff Logic is key here.
    
    // Stop GPS to save power before TX
    modem.sendAT("+CGNSPWR=0");
    modem.waitResponse();
    
    transmitData(lat, lon, speed, alt, sats, hdop);
    
    // 5. Sleep
    goToSleep(has_fix);
}

void loop() {
    // Empty
}

