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
    settings.report_interval_mins = prefs.getUInt("interval", 2); // Shorten for testing
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
    Serial.println(modem.testAT() ? "\nModem ON" : "\nModem Power FAIL");
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
    
    strip.begin();
    strip.setPixelColor(0, 0, 0, 255); // Blue (BLE Mode)
    strip.show();

    // BLE Window
    ble.begin("AE-Tracker-" + String((uint32_t)ESP.getEfuseMac(), HEX), settings);
    Serial.println("BLE Advertising (60s window)...");
    
    unsigned long ble_start = millis();
    while (millis() - ble_start < 60000 || ble.isConnected()) {
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
    int vsat=0, usat=0;

    while (millis() - gps_start < 300000) { // 5 min timeout
        if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &acc)) {
            got_fix = true;
            Serial.printf("GPS FIX! Sats: %d\n", usat);
            break;
        }
        Serial.print(".");
        delay(5000);
    }

    // Network & Send
    Serial.print("Connecting to Network...");
    modem.setNetworkMode(38); // LTE Only
    modem.setPreferredMode(3); // CAT-M/NB-IoT
    
    if (modem.waitForNetwork(60000L)) {
        Serial.println(" OK");
        if (modem.gprsConnect(settings.apn.c_str())) {
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
                mqtt.publish(mqtt_topic_up.c_str(), payload.c_str());
                Serial.println("Published: " + payload);
                delay(1000);
                mqtt.disconnect();
            } else {
                Serial.println("MQTT FAIL");
            }
            modem.gprsDisconnect();
        } else {
            Serial.println("GPRS FAIL (APN: " + settings.apn + ")");
        }
    } else {
        Serial.println("NET FAIL");
    }

    goToSleep();
}

void loop() {
    // Empty (Deep Sleep handles re-boot)
}

