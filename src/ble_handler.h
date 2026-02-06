#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <NimBLEDevice.h>
#include <functional>
#include <vector>

struct TrackerSettings {
    String apn = "hologram";
    String mqtt_broker = "mqtt.aceselectronics.com.au";
    String mqtt_user = "";
    String mqtt_pass = "";
    uint32_t report_interval_mins = 60;
};

struct TrackerStatus {
    float battery_voltage;
    bool gps_fix;
    int sats;
    String last_report;
};

class BLEHandler {
public:
    BLEHandler();
    void begin(const String& deviceName, TrackerSettings& settings);
    void updateStatus(const TrackerStatus& status);
    bool isConnected();
    void loop();

    void setSettingsCallback(std::function<void(const TrackerSettings&)> callback);

    static const char* SERVICE_UUID;
    static const char* APN_CHAR_UUID;
    static const char* BROKER_CHAR_UUID;
    static const char* USER_CHAR_UUID;
    static const char* PASS_CHAR_UUID;
    static const char* INTERVAL_CHAR_UUID;
    static const char* STATUS_CHAR_UUID;

private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pApnChar;
    BLECharacteristic* pBrokerChar;
    BLECharacteristic* pUserChar;
    BLECharacteristic* pPassChar;
    BLECharacteristic* pIntervalChar;
    BLECharacteristic* pStatusChar;

    TrackerSettings* _settings;
    std::function<void(const TrackerSettings&)> _settingsCallback;
};

#endif
