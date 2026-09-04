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
    uint8_t motion_sensitivity = 1;   // 1 = Medium, the shipped default

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
    float battery_voltage = 0.0f;
    int battery_soc = 0;
    bool gps_fix = false;
    int sats = 0;            // satellites USED in the fix; 0 until there is one

    // Satellites in view, which is a different question from the one above and
    // the only one with an answer during acquisition. It is what somebody
    // standing next to the ski is actually watching: "used" sits at 0 for the
    // whole search and then jumps, so a screen showing only that looks broken
    // for two minutes and then works.
    int sats_in_view = 0;

    float hdop = 0.0f;
    float lat = 0.0f;
    float lon = 0.0f;
    float speed = 0.0f;
    int rssi = 0;
    String gsm_status;
    String last_report;
};

// The diagnostics characteristics, one per telemetry section.
//
// Sectioned because a characteristic value stops at 512 bytes and a full
// verbose report is about twice that -- see buildTelemetry() in main.cpp, which
// decides what belongs in each. They are also the groups the app draws.
enum class DiagSection : uint8_t {
    Device = 0,   // identity, firmware, uplink cadence
    Position,
    Power,
    Motion,       // wake reason, IMU, threshold ladder
    Engine,       // CAN decodes off the ski's bus
    Count,
};

class BLEHandler {
public:
    BLEHandler();
    void begin(const String& deviceName, TrackerSettings& settings, float batteryVoltage, int batterySoc);
    void updateStatus(const TrackerStatus& status);
    void updateGps(const TrackerStatus& status); // New method

    // Publishes one section of the diagnostics report. `json` is a serialised
    // object; anything that would not fit the characteristic is refused rather
    // than truncated, because half a JSON document is not a smaller JSON
    // document -- it is a parse error on the phone with no explanation.
    void updateDiagnostics(DiagSection section, const String& json);
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

    // Diagnostics, 2060-2064, indexed by DiagSection. Read | Notify: read so a
    // screen opens with data instead of waiting for the next tick, notify so it
    // stays live while somebody watches.
    static const char* DIAG_CHAR_UUIDS[(size_t)DiagSection::Count];

    // Read-only, READ_ENC, and never read for its contents: the app reads it on
    // connect purely to force the link to encrypt before it subscribes to
    // anything. Same UUID as the shunt's, because the app looks for that one
    // UUID across every service rather than per product.
    static const char* PAIRING_CHAR_UUID;

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
    BLECharacteristic* pDiagChars[(size_t)DiagSection::Count];
    BLECharacteristic* pPairingChar;
    
    TrackerSettings* _settings;
    std::function<void(const TrackerSettings&)> _settingsCallback;
    
    // Connection parameter update tracking
    uint16_t _pendingConnHandle;
    unsigned long _connTime;

public:
    void scheduleConnParamsUpdate(uint16_t connHandle);
};

// The six-digit pairing PIN, derived from the last three bytes of the Wi-Fi
// base MAC. The same derivation the app uses to display it, and the same one
// every other AE product uses -- see MacUtils.generatePinFromMac.
uint32_t generatePinFromMac();

#endif
