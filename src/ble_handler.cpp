#include "ble_handler.h"
#include <Arduino.h>
#include "esp_mac.h"

// Correct UUIDs matching ae-ble-app/lib/models/tracker.dart
const char* BLEHandler::SERVICE_UUID        = "4fafc203-1fb5-459e-8fcc-c5c9c331914b"; // Updated Service UUID

const char* BLEHandler::GPS_DATA_CHAR_UUID  = "beb5483e-36e1-4688-b7f5-ea07361b2030";
const char* BLEHandler::STATUS_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b2031";

const char* BLEHandler::WIFI_SSID_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b2640"; // Reusing WiFi SSID for potential future usage or fallback
const char* BLEHandler::BROKER_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b2645";
const char* BLEHandler::USER_CHAR_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b2646";
const char* BLEHandler::PASS_CHAR_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b2647";

// Legacy/Custom UUIDs
const char* BLEHandler::APN_CHAR_UUID       = "ae000101-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::INTERVAL_CHAR_UUID  = "ae000105-1fb5-459e-8fcc-c5c9c331914b";


class TrackerBLECallbacks : public BLECharacteristicCallbacks {
    BLEHandler* _handler;
    TrackerSettings* _settings;
public:
    TrackerBLECallbacks(BLEHandler* handler, TrackerSettings* settings) : _handler(handler), _settings(settings) {}

    void onWrite(BLECharacteristic* pChar) {
        std::string val = pChar->getValue();
        String uuid = pChar->getUUID().toString().c_str();

        if (uuid == BLEHandler::APN_CHAR_UUID) _settings->apn = val.c_str();
        else if (uuid == BLEHandler::BROKER_CHAR_UUID) _settings->mqtt_broker = val.c_str();
        else if (uuid == BLEHandler::USER_CHAR_UUID) _settings->mqtt_user = val.c_str();
        else if (uuid == BLEHandler::PASS_CHAR_UUID) _settings->mqtt_pass = val.c_str();
        else if (uuid == BLEHandler::INTERVAL_CHAR_UUID) {
            if (val.length() >= 4) {
               memcpy(&_settings->report_interval_mins, val.data(), 4);
            }
        }
        // Handle WiFi SSID write (even if not used by SIM7080G directly yet)
        else if (uuid == BLEHandler::WIFI_SSID_CHAR_UUID) {
             // _settings->wifi_ssid = val.c_str(); // Add to settings if needed
             Serial.printf("[BLE] WiFi SSID set: %s\n", val.c_str());
        }

        Serial.printf("[BLE] Write to %s\n", uuid.c_str());
    }
};

class ServerCallbacks: public BLEServerCallbacks {
    BLEHandler* pHandler;
public:
    ServerCallbacks(BLEHandler* handler) : pHandler(handler) {}

    void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.printf("BLE client connected (ID: %d). Scheduling Params Update (Delayed)...\n", desc->conn_handle);
        if(pHandler) pHandler->scheduleConnParamsUpdate(desc->conn_handle);
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("BLE client disconnected");
        if (pHandler) {
            pHandler->scheduleConnParamsUpdate(0);
        }
    }
    
    void onMtuChanged(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU changed to: %d\n", MTU);
    }
};

BLEHandler::BLEHandler() : pServer(nullptr), pService(nullptr), _settings(nullptr) {
    _pendingConnHandle = 0;
    _connTime = 0;
}

void BLEHandler::begin(const String& deviceName, TrackerSettings& settings, float batteryVoltage, int batterySoc) {
    _settings = &settings;
    
    // Security DISABLED for verification
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));
    pService = pServer->createService(SERVICE_UUID);

    TrackerBLECallbacks* cb = new TrackerBLECallbacks(this, _settings);

    // -- App Compatible Characteristics --

    // GPS Data (Notify | Read)
    pGpsChar = pService->createCharacteristic(GPS_DATA_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Status (Notify | Read)
    pStatusChar = pService->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Broker (Read | Write)
    pBrokerChar = pService->createCharacteristic(BROKER_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pBrokerChar->setCallbacks(cb);
    pBrokerChar->setValue(_settings->mqtt_broker.c_str());

    // User (Read | Write)
    pUserChar = pService->createCharacteristic(USER_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pUserChar->setCallbacks(cb);
    pUserChar->setValue(_settings->mqtt_user.c_str());
    
    // Pass (Write Only)
    pPassChar = pService->createCharacteristic(PASS_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pPassChar->setCallbacks(cb);
    
    // WiFi SSID (Read | Write)
    pWifiSsidChar = pService->createCharacteristic(WIFI_SSID_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pWifiSsidChar->setCallbacks(cb);
    pWifiSsidChar->setValue("N/A"); // Default

    // -- Custom/Legacy Characteristics --

    // APN (Read | Write) - Crucial for SIM7080G
    pApnChar = pService->createCharacteristic(APN_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pApnChar->setCallbacks(cb);
    pApnChar->setValue(_settings->apn.c_str());

    // Interval (Read | Write)
    pIntervalChar = pService->createCharacteristic(INTERVAL_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pIntervalChar->setCallbacks(cb);
    pIntervalChar->setValue((uint8_t*)&_settings->report_interval_mins, 4);


    pService->start();
    
    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    
    // Optimized Params
    pAdv->setMinPreferred(0x06);
    pAdv->setMaxPreferred(0x0C);
    
    // Add Manufacturer Data for Battery Scan (Mv as uint16 LE)
    String mfgData = "";
    uint16_t voltMv = (uint16_t)(batteryVoltage * 1000);
    // Company ID (Espressif - 0x02E5) - reversed? No, NimBLE usually expects correct endianness or byte string
    // App expects 0x02E5 key. setManufacturerData takes a string.
    // We must manually construct the string where the internal key is not part of the data if using setManufacturerData? 
    // NimBLE syntax: setManufacturerData(std::string data) sets the WHOLE field including ID? 
    // Wait, NimBLEArduino's setManufacturerData takes string data. 
    // But usually we set ID and Data. 
    // Let's verify App parsing: it looks for key 0x02E5. 
    // In NimBLE-Arduino: pAdv->setManufacturerData(data) sets the whole Manufacturer Specific Data AD Type (0xFF) payload?
    // No, standard BLE library often separates ID.
    // Checking NimBLE documentation/examples... usually just put bytes.
    // Let's assume standard format: [ID LSB] [ID MSB] [Data...]
    // ID: 0xE5 0x02 (0x02E5 in Little Endian)
    // Data: [Volt LSB] [Volt MSB] [0x00] [0x00]
    
    char mfgBuf[6];
    mfgBuf[0] = 0xE5; 
    mfgBuf[1] = 0x02;
    mfgBuf[2] = (uint8_t)(voltMv & 0xFF);
    mfgBuf[3] = (uint8_t)((voltMv >> 8) & 0xFF);
    mfgBuf[4] = (uint8_t)batterySoc; 
    mfgBuf[5] = 0x00; 
    
    pAdv->setManufacturerData(std::string(mfgBuf, 6));

    pAdv->start();
    Serial.println("[BLE] Advertising started (App Compatible UUIDs + Battery Data)");
}

bool BLEHandler::isConnected() {
    return pServer->getConnectedCount() > 0;
}

void BLEHandler::updateStatus(const TrackerStatus& status) {
    if (pStatusChar) {
        // App expects "volts,rssi,status" or now "volts,soc,rssi,status"
        char buf[128]; 
        snprintf(buf, sizeof(buf), "%.2f,%d,%d,%s", status.battery_voltage, status.battery_soc, status.rssi, status.gsm_status.c_str());
        
        size_t len = strlen(buf);
        pStatusChar->setValue((uint8_t*)buf, len); 
        pStatusChar->notify();
    }
}

void BLEHandler::updateGps(const TrackerStatus& status) {
    if (pGpsChar) {
        // App expects "lat,lng,speed,sats"
        char buf[128];
        snprintf(buf, sizeof(buf), "%.6f,%.6f,%.2f,%d", status.lat, status.lon, status.speed, status.sats);
        
        size_t len = strlen(buf);
        Serial.printf("[BLE-DEBUG] GPS Payload (%d bytes): %s [HEX: ", len, buf);
        for(size_t i=0; i<len; i++) Serial.printf("%02X ", buf[i]);
        Serial.println("]");
        
        pGpsChar->setValue((uint8_t*)buf, len); // Explicit length
        pGpsChar->notify();
    }
}

void BLEHandler::scheduleConnParamsUpdate(uint16_t connHandle) {
    if (connHandle == 0) {
        _pendingConnHandle = 0;
        _connTime = 0;
    } else {
        _pendingConnHandle = connHandle;
        _connTime = millis();
    }
}

void BLEHandler::loop() {
    if (_pendingConnHandle != 0 && _connTime != 0) {
        if (millis() - _connTime > 2000) {
            Serial.printf("[BLE] Updating Conn Params for Handle %d (Delayed)\n", _pendingConnHandle);
            if (pServer) {
                pServer->updateConnParams(_pendingConnHandle, 24, 40, 4, 300);
            }
            _pendingConnHandle = 0;
            _connTime = 0;
        }
    }
}
