#include "ble_handler.h"
#include <Arduino.h>

const char* BLEHandler::SERVICE_UUID  = "ae000100-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::APN_CHAR_UUID      = "ae000101-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::BROKER_CHAR_UUID   = "ae000102-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::USER_CHAR_UUID     = "ae000103-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::PASS_CHAR_UUID     = "ae000104-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::INTERVAL_CHAR_UUID = "ae000105-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEHandler::STATUS_CHAR_UUID   = "ae000106-1fb5-459e-8fcc-c5c9c331914b";

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

        Serial.printf("[BLE] Write to %s\n", uuid.c_str());
    }
};

BLEHandler::BLEHandler() : pServer(nullptr), pService(nullptr), _settings(nullptr) {}

void BLEHandler::begin(const String& deviceName, TrackerSettings& settings) {
    _settings = &settings;
    BLEDevice::init(deviceName.c_str());
    pServer = BLEDevice::createServer();
    pService = pServer->createService(SERVICE_UUID);

    TrackerBLECallbacks* cb = new TrackerBLECallbacks(this, _settings);

    pApnChar = pService->createCharacteristic(APN_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pApnChar->setCallbacks(cb);
    pApnChar->setValue(_settings->apn.c_str());

    pBrokerChar = pService->createCharacteristic(BROKER_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pBrokerChar->setCallbacks(cb);
    pBrokerChar->setValue(_settings->mqtt_broker.c_str());

    pUserChar = pService->createCharacteristic(USER_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pUserChar->setCallbacks(cb);
    pUserChar->setValue(_settings->mqtt_user.c_str());

    pPassChar = pService->createCharacteristic(PASS_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pPassChar->setCallbacks(cb);

    pIntervalChar = pService->createCharacteristic(INTERVAL_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pIntervalChar->setCallbacks(cb);
    pIntervalChar->setValue((uint8_t*)&_settings->report_interval_mins, 4);

    pStatusChar = pService->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    pService->start();
    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->start();
}

bool BLEHandler::isConnected() {
    return pServer->getConnectedCount() > 0;
}

void BLEHandler::updateStatus(const TrackerStatus& status) {
    if (pStatusChar) {
        char buf[64];
        snprintf(buf, sizeof(buf), "V:%.2f Fix:%d Sats:%d", status.battery_voltage, status.gps_fix, status.sats);
        pStatusChar->setValue(buf);
        pStatusChar->notify();
    }
}

void BLEHandler::loop() {
    // NimBLE handles most things, but we can check connection timers if needed
}
