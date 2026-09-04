#include "ble_handler.h"
#include <Arduino.h>
#include "esp_mac.h"

// Mirrors NimBLEServer.cpp's own guarded include: the short path only exists on
// ESP-IDF component builds, and these are PlatformIO/Arduino builds.
#if defined(CONFIG_NIMBLE_CPP_IDF)
#include "services/gatt/ble_svc_gatt.h"
#elif __has_include("nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h")
#include "nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h"
#endif

// Correct UUIDs matching ae-ble-app/lib/models/tracker.dart
const char* BLEHandler::SERVICE_UUID        = "4fafc203-1fb5-459e-8fcc-c5c9c331914b"; // Updated Service UUID

const char* BLEHandler::GPS_DATA_CHAR_UUID  = "beb5483e-36e1-4688-b7f5-ea07361b2030";
const char* BLEHandler::STATUS_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b2031";
const char* BLEHandler::NAME_CHAR_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b2040";

const char* BLEHandler::WIFI_SSID_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b2640"; // Reusing WiFi SSID for potential future usage or fallback
const char* BLEHandler::BROKER_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b2645";
const char* BLEHandler::USER_CHAR_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b2646";
const char* BLEHandler::PASS_CHAR_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b2647";

// Legacy/Custom UUIDs
const char* BLEHandler::APN_CHAR_UUID       = "ae000101-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::INTERVAL_CHAR_UUID  = "beb5483e-36e1-4688-b7f5-ea07361b2050";
const char* BLEHandler::MOTION_SENS_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b2051";
const char* BLEHandler::DEBUG_PAYLOAD_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b2052";

const char* BLEHandler::PAIRING_CHAR_UUID = "ACDC1234-5678-90AB-CDEF-1234567890CB";

uint32_t generatePinFromMac() {
    uint8_t mac[6];
    // esp_read_mac rather than the BLE address: this reads the Wi-Fi base MAC,
    // which is what the app derives its displayed PIN from. The BLE address is
    // that value plus two in the last byte, so deriving from it would produce a
    // PIN that never matches the one on screen.
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    const uint32_t val = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    return val % 1000000;
}

// Indexed by DiagSection, so the order here IS the enum's order.
const char* BLEHandler::DIAG_CHAR_UUIDS[(size_t)DiagSection::Count] = {
    "beb5483e-36e1-4688-b7f5-ea07361b2060", // Device
    "beb5483e-36e1-4688-b7f5-ea07361b2061", // Position
    "beb5483e-36e1-4688-b7f5-ea07361b2062", // Power
    "beb5483e-36e1-4688-b7f5-ea07361b2063", // Motion
    "beb5483e-36e1-4688-b7f5-ea07361b2064", // Engine
};


class TrackerBLECallbacks : public BLECharacteristicCallbacks {
    BLEHandler* _handler;
    TrackerSettings* _settings;
public:
    TrackerBLECallbacks(BLEHandler* handler, TrackerSettings* settings) : _handler(handler), _settings(settings) {}

    void onWrite(BLECharacteristic* pChar) {
        std::string val = pChar->getValue();
        String uuid = pChar->getUUID().toString().c_str();

        if (uuid == BLEHandler::NAME_CHAR_UUID) {
            _settings->name = val.c_str();
            Serial.printf("[BLE] Device Name set: %s\n", _settings->name.c_str());
        }
        else if (uuid == BLEHandler::APN_CHAR_UUID) _settings->apn = val.c_str();
        else if (uuid == BLEHandler::BROKER_CHAR_UUID) {
            // Ignore an empty write rather than saving it: an empty host bricks the
            // uplink until the tracker is re-provisioned, and the write is almost
            // always a settings form saved with the broker field never populated.
            String broker = String(val.c_str());
            broker.trim();
            if (broker.length() == 0) {
                Serial.println("[BLE] Rejected empty broker write; keeping stored value");
            } else {
                _settings->mqtt_broker = broker;
            }
        }
        else if (uuid == BLEHandler::USER_CHAR_UUID) _settings->mqtt_user = val.c_str();
        else if (uuid == BLEHandler::PASS_CHAR_UUID) _settings->mqtt_pass = val.c_str();
        else if (uuid == BLEHandler::INTERVAL_CHAR_UUID) {
            if (val.length() >= 4) {
               memcpy(&_settings->report_interval_mins, val.data(), 4);
            }
        }
        else if (uuid == BLEHandler::MOTION_SENS_CHAR_UUID) {
            // Validated rather than trusted. An out-of-range byte here would
            // otherwise reach motionBaseFor(), whose default arm is Medium --
            // so a typo in an app would silently look like it worked, and the
            // setting reported back afterwards would not be what was sent.
            if (val.length() >= 1) {
                const uint8_t v = (uint8_t)val[0];
                if (v <= 2) {
                    _settings->motion_sensitivity = v;
                    Serial.printf("[BLE] Motion sensitivity set to %u\n", v);
                } else {
                    Serial.printf("[BLE] Rejected motion sensitivity %u (expected 0-2)\n", v);
                }
            }
        }
        else if (uuid == BLEHandler::DEBUG_PAYLOAD_CHAR_UUID) {
            if (val.length() >= 1) {
                _settings->debug_payload = (val[0] != 0);
                Serial.printf("[BLE] Debug payload %s\n",
                              _settings->debug_payload ? "ON (verbose)" : "OFF (essentials only)");
            }
        }
        // Handle WiFi SSID write (even if not used by SIM7080G directly yet)
        else if (uuid == BLEHandler::WIFI_SSID_CHAR_UUID) {
             // _settings->wifi_ssid = val.c_str(); // Add to settings if needed
             Serial.printf("[BLE] WiFi SSID set: %s\n", val.c_str());
        }

        Serial.printf("[BLE] Write to %s\n", uuid.c_str());
        if (_handler->getSettingsCallback()) {
            _handler->getSettingsCallback()(*_settings);
        }
    }
};

class ServerCallbacks: public BLEServerCallbacks {
    BLEHandler* pHandler;
public:
    ServerCallbacks(BLEHandler* handler) : pHandler(handler) {}

    void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.printf("BLE client connected (ID: %d). Scheduling Params Update (Delayed)...\n", desc->conn_handle);
        // Printed on every connect, including ones that reuse an existing bond.
        // Worded explicitly because reading it as "pairing happened" made a bond
        // that was working look like it was re-pairing every time.
        Serial.printf("[BLE SEC] PIN (only needed if pairing): %06u\n",
                      (unsigned)generatePinFromMac());

        // Tell the client its cached attribute table may be stale.
        //
        // This firmware added six characteristics to the service -- five
        // diagnostics sections and the pairing trigger -- and a phone that has
        // connected to this tracker before may still be holding the old table.
        // Without this it would discover neither the diagnostics it is meant to
        // subscribe to nor the pairing characteristic that encrypts the link,
        // and the failure would be silent: an empty debug screen and config
        // writes that fail with an encryption error.
        //
        // Sent unconditionally rather than gated on a stored schema version the
        // way the shunt does it. The shunt gates it because it is connected to
        // constantly and a re-discovery on every connect is real cost; this
        // radio is only up during a config window on a cold boot, so the
        // bookkeeping would cost more than it saves.
        Serial.println("[BLE] Indicating Service Changed (attribute table may be cached)");
        ble_svc_gatt_changed(0x0001, 0xffff);
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
    for (size_t i = 0; i < (size_t)DiagSection::Count; i++) pDiagChars[i] = nullptr;
    pPairingChar = nullptr;
}

void BLEHandler::begin(const String& deviceName, TrackerSettings& settings, float batteryVoltage, int batterySoc) {
    _settings = &settings;

    // Bonding, MITM and Secure Connections, with a passkey the device displays
    // and the phone types. Matches every other AE product.
    //
    // This was previously off entirely -- the source said "Security DISABLED for
    // verification" -- which meant the PIN the app printed on the tracker screen
    // was decoration: nothing ever asked for it, and anyone within radio range
    // during a config window could rewrite the APN, the broker and the MQTT
    // credentials. DISPLAY_ONLY is what makes the phone prompt for it.
    const uint32_t passkey = generatePinFromMac();
    BLEDevice::setSecurityAuth(true, true, true);
    BLEDevice::setSecurityPasskey(passkey);
    BLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    Serial.printf("[BLE SEC] Pairing PIN: %06u\n", (unsigned)passkey);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));
    pService = pServer->createService(SERVICE_UUID);

    TrackerBLECallbacks* cb = new TrackerBLECallbacks(this, _settings);

    // -- App Compatible Characteristics --

    // GPS Data (Notify | Read)
    pGpsChar = pService->createCharacteristic(GPS_DATA_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Status (Notify | Read)
    pStatusChar = pService->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    
    // Name (Read | Write)
    pNameChar = pService->createCharacteristic(NAME_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pNameChar->setCallbacks(cb);
    pNameChar->setValue(_settings->name.c_str());

    // Broker (Read | Write)
    pBrokerChar = pService->createCharacteristic(BROKER_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pBrokerChar->setCallbacks(cb);
    pBrokerChar->setValue(_settings->mqtt_broker.c_str());

    // User (Read | Write)
    pUserChar = pService->createCharacteristic(USER_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pUserChar->setCallbacks(cb);
    pUserChar->setValue(_settings->mqtt_user.c_str());
    
    // Pass (Write Only)
    pPassChar = pService->createCharacteristic(PASS_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pPassChar->setCallbacks(cb);
    
    // WiFi SSID (Read | Write)
    pWifiSsidChar = pService->createCharacteristic(WIFI_SSID_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pWifiSsidChar->setCallbacks(cb);
    pWifiSsidChar->setValue("N/A"); // Default

    // -- Custom/Legacy Characteristics --

    // APN (Read | Write) - Crucial for SIM7080G
    pApnChar = pService->createCharacteristic(APN_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pApnChar->setCallbacks(cb);
    pApnChar->setValue(_settings->apn.c_str());

    // Interval (Read | Write)
    pIntervalChar = pService->createCharacteristic(INTERVAL_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pIntervalChar->setCallbacks(cb);
    pIntervalChar->setValue((uint8_t*)&_settings->report_interval_mins, 4);

    // Motion sensitivity (Read | Write), one byte: 0 Low, 1 Medium, 2 High.
    pMotionSensChar = pService->createCharacteristic(MOTION_SENS_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pMotionSensChar->setCallbacks(cb);
    pMotionSensChar->setValue((uint8_t*)&_settings->motion_sensitivity, 1);


    // Debug payload (Read | Write), one byte: 0 essentials only, non-zero verbose.
    pDebugPayloadChar = pService->createCharacteristic(DEBUG_PAYLOAD_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    pDebugPayloadChar->setCallbacks(cb);
    {
        uint8_t dbg = _settings->debug_payload ? 1 : 0;
        pDebugPayloadChar->setValue(&dbg, 1);
    }

    // Pairing. Never read for its contents -- the app reads it on connect
    // purely to force the link to encrypt before it subscribes to anything,
    // because a subscribe on an encrypted characteristic across an unencrypted
    // link fails, and it fails quietly.
    pPairingChar = pService->createCharacteristic(
        PAIRING_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC);
    pPairingChar->setValue("AE");

    // Diagnostics. Seeded with an empty object rather than left unset: an
    // unwritten characteristic reads back as zero bytes, which the app cannot
    // tell from a device that has nothing to say, and "{}" parses.
    for (size_t i = 0; i < (size_t)DiagSection::Count; i++) {
        pDiagChars[i] = pService->createCharacteristic(
            DIAG_CHAR_UUIDS[i],
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
        pDiagChars[i]->setValue("{}");
    }

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
    if (!pStatusChar) return;

    // "volts,soc,rssi,status". The status field is last and is allowed to
    // contain commas -- the app rejoins everything from the fourth field on --
    // so nothing may be appended after it. Anything new goes in the diagnostics
    // characteristic instead.
    char buf[128];
    snprintf(buf, sizeof(buf), "%.2f,%d,%d,%s",
             status.battery_voltage, status.battery_soc, status.rssi,
             status.gsm_status.c_str());

    pStatusChar->setValue((uint8_t*)buf, strlen(buf));
    pStatusChar->notify();
}

void BLEHandler::updateGps(const TrackerStatus& status) {
    if (!pGpsChar) return;

    // "lat,lon,speed,sats,hdop,sats_in_view,fix".
    //
    // The first five are the original contract and keep their meaning. The last
    // two are appended rather than inserted so an app built against the old
    // format is unaffected: it splits on commas and reads the first five, and
    // extra fields fall off the end. Without them an unlocked tracker is
    // indistinguishable from a broken one -- every field reads zero either way.
    char buf[128];
    snprintf(buf, sizeof(buf), "%.6f,%.6f,%.2f,%d,%.2f,%d,%d",
             status.lat, status.lon, status.speed, status.sats, status.hdop,
             status.sats_in_view, status.gps_fix ? 1 : 0);

    pGpsChar->setValue((uint8_t*)buf, strlen(buf));
    pGpsChar->notify();
}

void BLEHandler::updateDiagnostics(DiagSection section, const String& json) {
    const size_t idx = (size_t)section;
    if (idx >= (size_t)DiagSection::Count || !pDiagChars[idx]) return;

    // 512 is the ceiling on an attribute value in the ATT spec, so this is the
    // hard limit and not a tuning choice. Refused loudly rather than trimmed:
    // buildTelemetry() splits the report into sections precisely so this cannot
    // happen, and if it does the fix is to move a field, which needs somebody to
    // know about it.
    if (json.length() > 512) {
        Serial.printf("[BLE] Diagnostics section %u is %u bytes, over the 512 limit - not sent.\n",
                      (unsigned)idx, (unsigned)json.length());
        pDiagChars[idx]->setValue("{\"error\":\"section too large\"}");
    } else {
        pDiagChars[idx]->setValue((uint8_t*)json.c_str(), json.length());
    }
    pDiagChars[idx]->notify();
}

void BLEHandler::setSettingsCallback(std::function<void(const TrackerSettings&)> callback) {
    _settingsCallback = callback;
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
