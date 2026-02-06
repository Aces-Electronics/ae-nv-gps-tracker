#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <NimBLEDevice.h>
#include <functional>
#include <vector>

struct TrackerSettings {
    String name = ""; // Added
    String apn = "hologram";
    String mqtt_broker = "mqtt.aceselectronics.com.au";
    String mqtt_user = "";
    String mqtt_pass = "";
    uint32_t report_interval_mins = 60;
};

struct TrackerStatus {
    float battery_voltage;
    int battery_soc;
    bool gps_fix;
    int sats;
    float hdop; // Added
    float lat;
    float lon;
    float speed;
    int rssi;
    String gsm_status;
    String last_report;
};

class BLEHandler {
public:
    BLEHandler();
    void begin(const String& deviceName, TrackerSettings& settings, float batteryVoltage, int batterySoc);
    void updateStatus(const TrackerStatus& status);
    void updateGps(const TrackerStatus& status); // New method
    bool isConnected();
    void loop();

    void setSettingsCallback(std::function<void(const TrackerSettings&)> callback);
    std::function<void(const TrackerSettings&)> getSettingsCallback() { return _settingsCallback; }

    static const char* SERVICE_UUID;
    
    static const char* NAME_CHAR_UUID;         // Added: beb5483e-36e1-4688-b7f5-ea07361b2040
    
    // UUIDs matching ae-ble-app/lib/models/tracker.dart
    static const char* GPS_DATA_CHAR_UUID;     // beb5483e-36e1-4688-b7f5-ea07361b2030
    static const char* STATUS_CHAR_UUID;       // beb5483e-36e1-4688-b7f5-ea07361b2031
    
    static const char* WIFI_SSID_CHAR_UUID;    // beb5483e-36e1-4688-b7f5-ea07361b2640
    static const char* BROKER_CHAR_UUID;       // beb5483e-36e1-4688-b7f5-ea07361b2645
    static const char* USER_CHAR_UUID;         // beb5483e-36e1-4688-b7f5-ea07361b2646
    static const char* PASS_CHAR_UUID;         // beb5483e-36e1-4688-b7f5-ea07361b2647
    
    // Custom/Legacy UUIDs (Not in App yet)
    static const char* APN_CHAR_UUID;          // ae000101...
    static const char* INTERVAL_CHAR_UUID;     // ae000105...

private:
    BLEServer* pServer;
    BLEService* pService;
    
    BLECharacteristic* pGpsChar;
    BLECharacteristic* pStatusChar;
    BLECharacteristic* pNameChar;
    BLECharacteristic* pWifiSsidChar;
    
    BLECharacteristic* pApnChar;
    BLECharacteristic* pBrokerChar;
    BLECharacteristic* pUserChar;
    BLECharacteristic* pPassChar;
    BLECharacteristic* pIntervalChar;
    
    TrackerSettings* _settings;
    std::function<void(const TrackerSettings&)> _settingsCallback;
    
    // Connection parameter update tracking
    uint16_t _pendingConnHandle;
    unsigned long _connTime;

public:
    void scheduleConnParamsUpdate(uint16_t connHandle);
};

#endif
