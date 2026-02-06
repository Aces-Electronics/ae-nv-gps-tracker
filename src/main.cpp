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
bool stay_awake = false;

void loadSettings() {
    prefs.begin("tracker", false);
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
    prefs.putString("apn", settings.apn);
    prefs.putString("broker", settings.mqtt_broker);
    prefs.putString("user", settings.mqtt_user);
    prefs.putString("pass", settings.mqtt_pass);
    prefs.putUInt("interval", settings.report_interval_mins);
    prefs.end();
    Serial.println("Settings Saved to NVS.");
}

void modemPowerOn() {
    // DC3: Modem Power
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

void goToSleep() {
    modemPowerOff();
    uint64_t sleep_time = (uint64_t)settings.report_interval_mins * 60 * 1000000ULL;
    if (sleep_time == 0) sleep_time = 60 * 1000000ULL; // Default 1 min if 0

    Serial.printf("Entering Deep Sleep for %u minutes...\n", settings.report_interval_mins);
    esp_sleep_enable_timer_wakeup(sleep_time);
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- AE Tracker Boot ---");

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("PMU FAIL");
    }
    PMU.disableTSPinMeasure();
    
    loadSettings();
    settings.report_interval_mins = 1; // FORCE 1m for testing
    Serial.println("Forcing 1-minute test interval (ADMIN MODE).");

    pinMode(0, INPUT_PULLUP); // Boot Button
    
    strip.begin();
    strip.setPixelColor(0, 0, 0, 255); // Blue (BLE Mode)
    strip.show();

    // BLE Window
    ble.begin("AE-Tracker-" + String((uint32_t)ESP.getEfuseMac(), HEX), settings);
    Serial.println("BLE Advertising (90s window)...");
    
    unsigned long ble_start = millis();
    while (millis() - ble_start < 90000 || ble.isConnected()) {
        if (digitalRead(0) == LOW) {
            stay_awake = true;
            Serial.println("\n[DEBUG] Stay Awake Triggered by Button Press!");
        }
        
        if (ble.isConnected()) {
            strip.setPixelColor(0, 0, 255, 255); // Cyan (Connected)
            strip.show();
            // Optional: Handle real-time telemetry updates to BLE app here
        }
        delay(100);
    }
    
    saveSettings(); // Save any changes from BLE
    
    // One-Shot Sequence
    strip.setPixelColor(0, 255, 165, 0); // Orange (Modem/GPS)
    strip.show();

    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    modemPowerOn();
    
    imei = modem.getIMEI();
    mqtt_topic_up = "ae-nv/tracker/" + imei + "/up";

    // GPS Acquisition
    Serial.println("Waiting for GPS Fix (5 min max)...");
    modem.sendAT("+CGNSPWR=1");
    modem.waitResponse();
    
    unsigned long gps_start = millis();
    bool got_fix = false;
    float lat=0, lon=0, speed=0, alt=0, acc=0;
    int usat=0;

    while (millis() - gps_start < 300000) { // 5 min timeout
        // Try TinyGsm first for core coordinates
        if (modem.getGPS(&lat, &lon, &speed, &alt, nullptr, &usat, &acc)) {
            got_fix = true;
            // Now get raw info for accurate satellite count
            modem.sendAT("+CGNSINF");
            if (modem.waitResponse(1000L, "+CGNSINF:") == 1) {
                String res = modem.stream.readStringUntil('\n');
                res.trim();
                // Response: run,fix,date,lat,lon,alt,speed,course,mode,res,hdop,pdop,vdop,res,sats_view,sats_used,...
                // Sats used is at 15th index (0-based)
                int commaIndex = -1;
                for(int i=0; i<15; i++) {
                    commaIndex = res.indexOf(',', commaIndex + 1);
                }
                if (commaIndex != -1) {
                    int nextComma = res.indexOf(',', commaIndex + 1);
                    if (nextComma != -1) {
                        usat = res.substring(commaIndex + 1, nextComma).toInt();
                    }
                }
            }
            Serial.printf("GPS FIX! Sats: %d\n", usat);
            break;
        }
        Serial.print(".");
        delay(5000);
    }

    // Network & Send
    int csq = modem.getSignalQuality();
    Serial.printf("Connecting to Network (CSQ: %d)...\n", csq);
    
    // Diagnostic: Check registration status
    modem.sendAT("+CEREG?");
    modem.waitResponse();
    modem.sendAT("+COPS?");
    modem.waitResponse();
    modem.sendAT("+CGNAPN"); // Check if network provides an APN
    modem.waitResponse();

    modem.sendAT("+CMNB=1"); // Set to Cat-M only for Telstra preference
    modem.waitResponse();
    modem.sendAT("+COPS=0"); // Force automatic registration
    modem.waitResponse();
    modem.setNetworkMode(2); // Automatic mode
    modem.setPreferredMode(3); // CAT-M/NB-IoT
    
    if (modem.waitForNetwork(180000L)) {
        Serial.println("Network Registered OK");
        
        bool connected = false;
        Serial.print("Connecting to GPRS (APN: " + settings.apn + ")...");
        if (modem.gprsConnect(settings.apn.c_str())) {
            connected = true;
            Serial.println(" OK");
        } else {
            Serial.println(" fail");
            Serial.println("Attempting Auto-APN Discovery (+CGNAPN)...");
            modem.sendAT("+CGNAPN");
            if (modem.waitResponse(10000L, "+CGNAPN:") == 1) {
                String res = modem.stream.readStringUntil('\n');
                res.trim();
                int firstQuote = res.indexOf('"');
                int lastQuote = res.lastIndexOf('"');
                if (firstQuote != -1 && lastQuote != -1 && lastQuote > firstQuote) {
                    String discoveredAPN = res.substring(firstQuote + 1, lastQuote);
                    Serial.println("Discovered APN: " + discoveredAPN);
                    if (discoveredAPN.length() > 0 && modem.gprsConnect(discoveredAPN.c_str())) {
                        connected = true;
                        settings.apn = discoveredAPN;
                        Serial.println("Connected via Discovered APN!");
                    }
                }
            }
            if (!connected) {
                Serial.println("Trying Empty APN...");
                if (modem.gprsConnect("")) {
                    connected = true;
                    Serial.println("Connected with Empty APN!");
                }
            }
        }

        if (connected) {
            mqtt.setServer(settings.mqtt_broker.c_str(), 1883);
            if (mqtt.connect(imei.c_str(), settings.mqtt_user.c_str(), settings.mqtt_pass.c_str())) {
                StaticJsonDocument<512> doc;
                doc["mac"] = imei;
                doc["model"] = "ae-sim7080g-tracker";
                doc["lat"] = lat;
                doc["lon"] = lon;
                doc["alt"] = alt;
                doc["speed"] = speed;
                doc["sats"] = usat;
                doc["battery_voltage"] = PMU.getBattVoltage() / 1000.0F;
                doc["rssi"] = modem.getSignalQuality();
                doc["interval"] = settings.report_interval_mins;
                
                String payload;
                serializeJson(doc, payload);
                Serial.println("[MQTT] Publishing Payload: " + payload);
                if (mqtt.publish(mqtt_topic_up.c_str(), payload.c_str())) {
                    Serial.println("[MQTT] Publish OK");
                } else {
                    Serial.println("[MQTT] Publish FAIL");
                }
                delay(1000);
                mqtt.disconnect();
            } else {
                Serial.println("MQTT FAIL");
            }
            modem.gprsDisconnect();
        } else {
            Serial.println("GPRS FAIL");
        }
    } else {
        Serial.println("NET FAIL");
    }

    if (stay_awake) {
        Serial.println("STAY AWAKE: Skipping Deep Sleep. Re-running setup in 10s...");
        delay(10000);
        ESP.restart();
    } else {
        goToSleep();
    }
}

void loop() {
    // Empty (Deep Sleep handles re-boot)
}

