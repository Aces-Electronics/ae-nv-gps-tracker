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
            for (uint8_t b = 0; b < 8; b++) {
                if (msg.data[b] < s_seen[i].minData[b]) s_seen[i].minData[b] = msg.data[b];
                if (msg.data[b] > s_seen[i].maxData[b]) s_seen[i].maxData[b] = msg.data[b];
            }
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
    for (uint8_t b = 0; b < 8; b++) { e.minData[b] = msg.data[b]; e.maxData[b] = msg.data[b]; }
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
        // Only the bytes that have actually moved, with the span they moved
        // over. Everything else is constant and not worth reading.
        Serial.print(" var:");
        bool any = false;
        for (uint8_t b = 0; b < e.dlc && b < 8; b++) {
            if (e.maxData[b] != e.minData[b]) {
                Serial.printf(" b%u[%02X-%02X]", b, e.minData[b], e.maxData[b]);
                any = true;
            }
        }
        if (!any) Serial.print(" none (all bytes constant)");
        Serial.println();
    }
    Serial.println();
}

bool canSelfTest(CanBitrate rate) {
    Serial.printf("\n[CAN] Self-test at %s (no other node needed)\n", canBitrateName(rate));

    twai_timing_config_t t_config;
    if (!timingFor(rate, t_config)) return false;

    if (s_installed) canEnd();

    // NO_ACK rather than LISTEN_ONLY: this one has to transmit. It is still
    // safe on a live bus in the sense that it never expects an ACK, but it does
    // drive the lines -- so this is a bench test, not something to run plugged
    // into a ski.
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)DB_CAN_TX, (gpio_num_t)DB_CAN_RX, TWAI_MODE_NO_ACK);
    g_config.rx_queue_len = 16;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] driver_install failed: %s\n", esp_err_to_name(err));
        return false;
    }
    transceiverActive(true);   // Rs low = normal mode; standby cannot transmit
    delay(5);

    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] start failed");
        twai_driver_uninstall();
        transceiverActive(false);
        return false;
    }

    twai_message_t tx = {};
    tx.identifier = 0x123;
    tx.data_length_code = 8;
    for (int i = 0; i < 8; i++) tx.data[i] = 0xA0 + i;

    bool ok = false;
    if (twai_transmit(&tx, pdMS_TO_TICKS(200)) != ESP_OK) {
        Serial.println("[CAN] transmit failed - controller could not send.");
    } else {
        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(300)) != ESP_OK) {
            Serial.println("[CAN] transmitted, but nothing came back.");
            Serial.println("[CAN]   TX reached the controller; the loop through the transceiver did not.");
            Serial.println("[CAN]   Suspect Rs stuck high (standby), no transceiver power, or CANH/CANL.");
        } else if (rx.identifier != tx.identifier || rx.data_length_code != tx.data_length_code) {
            Serial.printf("[CAN] got a frame but it is not ours: id=0x%03X dlc=%u\n",
                          rx.identifier, rx.data_length_code);
        } else {
            bool same = true;
            for (int i = 0; i < 8; i++) if (rx.data[i] != tx.data[i]) same = false;
            Serial.printf("[CAN] LOOPBACK OK: id=0x%03X dlc=%u data%s intact\n",
                          rx.identifier, rx.data_length_code, same ? "" : " NOT");
            ok = same;
        }
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        Serial.printf("[CAN] tx_err=%u rx_err=%u bus_err=%u tx_failed=%u state=%d\n",
                      st.tx_error_counter, st.rx_error_counter,
                      (unsigned)st.bus_error_count, (unsigned)st.tx_failed_count, (int)st.state);
    }

    twai_stop();
    twai_driver_uninstall();
    transceiverActive(false);
    s_installed = false;
    Serial.printf("[CAN] Self-test %s\n\n", ok ? "PASSED" : "did not pass");
    return ok;
}

EngineData canReadEngine(uint32_t listenMs) {
    EngineData e;

    if (!canBeginListenOnly(CanBitrate::Rate500k)) {
        Serial.println("[CAN] Could not open the bus this wake.");
        return e;
    }

    const uint32_t t0 = millis();
    twai_message_t msg;
    uint32_t frames = 0;
    while (millis() - t0 < listenMs) {
        if (twai_receive(&msg, pdMS_TO_TICKS(20)) != ESP_OK) continue;
        frames++;
        e.busAlive = true;

        if (msg.identifier == 0x102 && msg.data_length_code >= 2) {
            e.rpm = (((uint32_t)msg.data[0] << 8) | msg.data[1]) / 4;
            e.rpmValid = true;
        } else if (msg.identifier == 0x342 && msg.data_length_code >= 5) {
            e.tempRaw = msg.data[4];
            e.tempValid = true;
        }
    }

    canEnd();   // also puts the transceiver back in standby

    if (!e.busAlive) {
        Serial.println("[CAN] Bus quiet - ignition off.");
    } else {
        Serial.printf("[CAN] %u frames; rpm=%ld temp_raw=%u\n",
                      frames, e.rpmValid ? (long)e.rpm : -1L, e.tempRaw);
    }
    return e;
}

void canRawCapture(uint32_t seconds) {
    struct Entry { uint32_t ms; uint16_t id; uint8_t dlc; uint8_t data[8]; };
    const size_t cap = (size_t)seconds * 1200;   // headroom over the ~980/s seen

    // PSRAM: this board has it, and a capture worth having is far larger than
    // the internal heap should be asked for.
    Entry* buf = (Entry*)ps_malloc(cap * sizeof(Entry));
    if (!buf) buf = (Entry*)malloc(cap * sizeof(Entry));
    if (!buf) {
        Serial.printf("[CAN] Could not allocate %u frames for capture.\n", (unsigned)cap);
        return;
    }

    Serial.printf("[CAN] Raw capture: %us, room for %u frames. Buffering...\n",
                  (unsigned)seconds, (unsigned)cap);

    size_t n = 0;
    const uint32_t t0 = millis();
    twai_message_t msg;
    while (millis() - t0 < seconds * 1000 && n < cap) {
        if (twai_receive(&msg, pdMS_TO_TICKS(5)) != ESP_OK) continue;
        buf[n].ms  = millis() - t0;
        buf[n].id  = (uint16_t)msg.identifier;
        buf[n].dlc = msg.data_length_code;
        memcpy(buf[n].data, msg.data, 8);
        n++;
    }

    Serial.printf("[CAN] Captured %u frames. Dumping CSV.\n", (unsigned)n);
    Serial.println("---8<--- BEGIN CAN CSV ---8<---");
    Serial.println("ms,id,dlc,d0,d1,d2,d3,d4,d5,d6,d7");
    for (size_t i = 0; i < n; i++) {
        Serial.printf("%lu,%03X,%u", (unsigned long)buf[i].ms, buf[i].id, buf[i].dlc);
        for (int b = 0; b < 8; b++) Serial.printf(",%02X", buf[i].data[b]);
        Serial.println();
    }
    Serial.println("---8<--- END CAN CSV ---8<---");
    free(buf);
}

int32_t canDecodeRpm(const CanFrameSummary& f) {
    if (f.id != 0x102 || f.extended || f.dlc < 2) return -1;
    return (((uint32_t)f.lastData[0] << 8) | f.lastData[1]) / 4;
}

static const CanFrameSummary* findSeen(uint32_t id) {
    for (size_t i = 0; i < s_seenCount; i++) {
        if (s_seen[i].id == id && !s_seen[i].extended) return &s_seen[i];
    }
    return nullptr;
}

void canWatchLoop() {
    Serial.println("\n[CAN] === WATCH: engine-data IDs over time ===");
    Serial.println("[CAN] RPM: 0x102 b0/b1 big-endian /4, confirmed against the tacho.");
    Serial.println("[CAN] Hold a steady idle, then rev slowly up and back.\n");

    const uint32_t t0 = millis();
    uint32_t lastPrint = 0;
    twai_message_t msg;

    while (true) {
        // Drain hard between prints. At ~980 frames/s the queue overruns in
        // well under the print interval, and a stale payload would look like a
        // signal that had stopped moving.
        while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) recordFrame(msg);

        if (millis() - lastPrint < 200) continue;
        lastPrint = millis();

        const CanFrameSummary* f102 = findSeen(0x102);
        const CanFrameSummary* f122 = findSeen(0x122);
        const CanFrameSummary* f320 = findSeen(0x320);
        const CanFrameSummary* f342 = findSeen(0x342);
        const CanFrameSummary* f110 = findSeen(0x110);

        Serial.printf("[%7.1fs] RPM=%5ld", (millis() - t0) / 1000.0f,
                      f102 ? (long)canDecodeRpm(*f102) : -1L);
        if (f102) Serial.printf(" (raw %02X%02X)", f102->lastData[0], f102->lastData[1]);
        if (f122) Serial.printf(" | 0x122 %02X %02X %02X %02X %02X %02X",
                                f122->lastData[0], f122->lastData[1], f122->lastData[2],
                                f122->lastData[3], f122->lastData[4], f122->lastData[5]);
        if (f110) Serial.printf(" | 110.b3=%02X", f110->lastData[3]);
        if (f320) Serial.printf(" | 320.b4=%02X", f320->lastData[4]);
        if (f342) Serial.printf(" | 342.b4=%02X", f342->lastData[4]);
        Serial.println();
    }
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
    // One discovery pass for the ID list and per-byte ranges, then straight into
    // the time series -- the table says which bytes are alive, the series is
    // what can be held against a gauge.
    canSniff(10000);
    canLogSeen();
    canRawCapture(10);
    canWatchLoop();
}

const char* canBitrateName(CanBitrate r) {
    switch (r) {
        case CanBitrate::Rate125k: return "125 kbit/s";
        case CanBitrate::Rate250k: return "250 kbit/s";
        case CanBitrate::Rate500k: return "500 kbit/s";
        default:                   return "unknown";
    }
}
