#include "can_handler.h"
#include "utilities.h"
#include "driver/twai.h"

static CanFrameSummary s_seen[CAN_MAX_IDS];
static size_t s_seenCount = 0;
static bool   s_installed = false;
static bool   s_tableFullLogged = false;

// power_handler parks Rs high (standby) on every wake. The transceiver is only
// brought out of it for as long as the controller is actually running.
static void transceiverActive(bool active) {
    pinMode(DB_CAN_RS, OUTPUT);
    digitalWrite(DB_CAN_RS, active ? LOW : HIGH);
}

static bool timingFor(CanBitrate rate, twai_timing_config_t& out) {
    switch (rate) {
        case CanBitrate::Rate125k: out = TWAI_TIMING_CONFIG_125KBITS(); return true;
        case CanBitrate::Rate250k: out = TWAI_TIMING_CONFIG_250KBITS(); return true;
        case CanBitrate::Rate500k: out = TWAI_TIMING_CONFIG_500KBITS(); return true;
        default: return false;
    }
}

bool canBeginListenOnly(CanBitrate rate) {
    if (s_installed) canEnd();

    twai_timing_config_t t_config;
    if (!timingFor(rate, t_config)) {
        Serial.println("[CAN] Refusing to start at an unknown bit rate.");
        return false;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)DB_CAN_TX, (gpio_num_t)DB_CAN_RX, TWAI_MODE_LISTEN_ONLY);

    // The default of 5 is easily overrun by a busy bus between receive calls.
    // Frames dropped here are only ever counted frames, never missed IDs, but
    // there is no reason to drop more than necessary.
    g_config.rx_queue_len = 32;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] driver_install failed: %s\n", esp_err_to_name(err));
        return false;
    }

    transceiverActive(true);

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN] start failed: %s\n", esp_err_to_name(err));
        twai_driver_uninstall();
        transceiverActive(false);
        return false;
    }

    s_installed = true;
    Serial.printf("[CAN] Listening at %s (listen-only, TX pin idle)\n", canBitrateName(rate));
    return true;
}

void canEnd() {
    if (!s_installed) return;
    twai_stop();
    twai_driver_uninstall();
    transceiverActive(false);
    s_installed = false;
}

CanBitrate canDetectBitrate(uint32_t msPerRate) {
    // 250k first: powersports and marine buses are usually J1939-derived. 125k
    // is last because hearing nothing at all is the common case there and it
    // costs the longest to rule out.
    const CanBitrate candidates[] = {
        CanBitrate::Rate250k, CanBitrate::Rate500k, CanBitrate::Rate125k
    };

    for (CanBitrate rate : candidates) {
        if (!canBeginListenOnly(rate)) continue;

        uint32_t frames = 0;
        uint32_t start = millis();
        twai_message_t msg;
        while (millis() - start < msPerRate) {
            if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) frames++;
        }
        canEnd();

        Serial.printf("[CAN] %s -> %u frames in %ums\n",
                      canBitrateName(rate), frames, msPerRate);
        if (frames > 0) return rate;
    }

    Serial.println("[CAN] Nothing heard at any rate. Check CAN-H/CAN-L, ground, and that the bus is awake.");
    return CanBitrate::Unknown;
}

static void recordFrame(const twai_message_t& msg) {
    const bool extd = msg.extd != 0;

    for (size_t i = 0; i < s_seenCount; i++) {
        if (s_seen[i].id == msg.identifier && s_seen[i].extended == extd) {
            s_seen[i].count++;
            s_seen[i].dlc = msg.data_length_code;
            memcpy(s_seen[i].lastData, msg.data, sizeof(s_seen[i].lastData));
            return;
        }
    }

    if (s_seenCount >= CAN_MAX_IDS) {
        if (!s_tableFullLogged) {
            // Said once, not per frame: on a bus with more IDs than the table
            // holds this would otherwise be the only thing on the console.
            Serial.printf("[CAN] ID table full at %u -- further new IDs are not being recorded.\n",
                          (unsigned)CAN_MAX_IDS);
            s_tableFullLogged = true;
        }
        return;
    }

    CanFrameSummary& e = s_seen[s_seenCount++];
    e.id       = msg.identifier;
    e.extended = extd;
    e.dlc      = msg.data_length_code;
    e.count    = 1;
    memcpy(e.lastData, msg.data, sizeof(e.lastData));
}

size_t canSniff(uint32_t durationMs) {
    if (!s_installed) {
        Serial.println("[CAN] canSniff() with no controller running.");
        return s_seenCount;
    }

    uint32_t total = 0;
    uint32_t start = millis();
    twai_message_t msg;

    while (millis() - start < durationMs) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) continue;
        total++;
        recordFrame(msg);
    }

    Serial.printf("[CAN] %u frames in %ums, %u distinct IDs\n",
                  total, durationMs, (unsigned)s_seenCount);
    return s_seenCount;
}

void canResetSeen() {
    s_seenCount = 0;
    s_tableFullLogged = false;
}

size_t canSeenCount() {
    return s_seenCount;
}

const CanFrameSummary* canSeen(size_t index) {
    if (index >= s_seenCount) return nullptr;
    return &s_seen[index];
}

void canLogSeen() {
    Serial.printf("\n[CAN] === %u distinct IDs ===\n", (unsigned)s_seenCount);
    for (size_t i = 0; i < s_seenCount; i++) {
        const CanFrameSummary& e = s_seen[i];
        // Extended IDs are 29-bit, standard are 11-bit; widths are fixed per
        // kind so the two do not line up misleadingly in the same column.
        if (e.extended) {
            Serial.printf("  0x%08X x%u n=%-6u ", e.id, e.dlc, e.count);
        } else {
            Serial.printf("      0x%03X x%u n=%-6u ", e.id, e.dlc, e.count);
        }
        for (uint8_t b = 0; b < e.dlc && b < 8; b++) {
            Serial.printf("%02X ", e.lastData[b]);
        }
        Serial.println();
    }
    Serial.println();
}

void canSnifferLoop() {
    Serial.println("\n=== CAN SNIFFER (listen-only) ===");
    Serial.println("This build never reaches the tracker cycle: no modem, no MQTT, no sleep.");
    Serial.println("Listen-only means nothing is ever driven onto the bus.\n");

    CanBitrate rate = CanBitrate::Unknown;
    while (rate == CanBitrate::Unknown) {
        rate = canDetectBitrate();
        if (rate == CanBitrate::Unknown) {
            Serial.println("[CAN] Retrying detection in 3s...\n");
            delay(3000);
        }
    }

    Serial.printf("[CAN] Detected %s. Sniffing.\n", canBitrateName(rate));
    if (!canBeginListenOnly(rate)) {
        Serial.println("[CAN] Could not restart at the detected rate. Halting.");
        while (true) delay(1000);
    }

    canResetSeen();
    while (true) {
        canSniff(10000);
        canLogSeen();
    }
}

const char* canBitrateName(CanBitrate r) {
    switch (r) {
        case CanBitrate::Rate125k: return "125 kbit/s";
        case CanBitrate::Rate250k: return "250 kbit/s";
        case CanBitrate::Rate500k: return "500 kbit/s";
        default:                   return "unknown";
    }
}
