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

    // Wake-on-motion sensitivity: 0 = Low, 1 = Medium, 2 = High. It selects the
    // FLOOR of the adaptive threshold ladder, not a fixed threshold -- the ski
    // still finds its own level from whichever floor is chosen. See the
    // MOTION_SENS_* constants in main.cpp for the values and their margins
    // against the measured noise floor.
    uint8_t motion_sensitivity = 1;

    // Verbose telemetry. Off by default: the normal frame is the bare essentials
    // for tracking a ski, and everything else -- the CAN decodes, the IMU vector,
    // the charger and motion internals -- rides along only when this is set.
    //
    // Worth knowing before turning it off and forgetting: a lean frame cannot be
    // investigated after the fact. Whatever happened while this was false is not
    // recoverable, because it was never sent. Leave it on while decode work is
    // still in progress.
    bool debug_payload = false;
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
    static const char* INTERVAL_CHAR_UUID;     // beb5483e-36e1-4688-b7f5-ea07361b2050

    // Wake-on-motion sensitivity, one byte: 0 = Low, 1 = Medium, 2 = High.
    // A single byte rather than the threshold in mg, because the levels are the
    // contract: the milligram values behind them are a tuning decision that has
    // already changed twice, and an app that wrote raw thresholds would pin them.
    static const char* MOTION_SENS_CHAR_UUID;
    static const char* DEBUG_PAYLOAD_CHAR_UUID; // beb5483e-36e1-4688-b7f5-ea07361b2052

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
    BLECharacteristic* pMotionSensChar;
    BLECharacteristic* pDebugPayloadChar;
    
    TrackerSettings* _settings;
    std::function<void(const TrackerSettings&)> _settingsCallback;
    
    // Connection parameter update tracking
    uint16_t _pendingConnHandle;
    unsigned long _connTime;

public:
    void scheduleConnParamsUpdate(uint16_t connHandle);
};

#endif
