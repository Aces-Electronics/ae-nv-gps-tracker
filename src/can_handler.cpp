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

void canDrain(uint32_t ms) {
    const uint32_t t0 = millis();
    twai_message_t msg;
    while (millis() - t0 < ms) {
        if (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) recordFrame(msg);
    }
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
    uint32_t badChecksum = 0;
    while (millis() - t0 < listenMs) {
        if (twai_receive(&msg, pdMS_TO_TICKS(20)) != ESP_OK) continue;
        frames++;
        e.busAlive = true;

        // A corrupted 0x102 would otherwise be published as a real rpm. The
        // checksum costs seven XORs and makes that impossible. 0x342 is not
        // checked: it does not run the scheme, so checking it would reject
        // every frame.
        if (msg.identifier == 0x102 && canChecksumValid(msg.data, msg.data_length_code)) {
            e.rpm = (((uint32_t)msg.data[0] << 8) | msg.data[1]) / 4;
            e.rpmValid = true;
            e.coolantRaw = msg.data[3];
            e.coolantValid = true;
        } else if (msg.identifier == 0x102) {
            badChecksum++;
        } else if (msg.identifier == 0x012 && canChecksumValid(msg.data, msg.data_length_code)) {
            // 1..9; anything else means the frame is not what we think it is.
            if (msg.data[2] >= 1 && msg.data[2] <= 9) {
                e.trimPos = msg.data[2];
                e.trimValid = true;
            }
        } else if (msg.identifier == 0x342 && msg.data_length_code >= 8) {
            e.tempRaw = msg.data[4];
            e.tempValid = true;
            // b6/b7 are payload on this ID, not counter+checksum.
            e.engineMinutes = ((uint16_t)msg.data[6] << 8) | msg.data[7];
            e.hoursValid = true;
        }
    }

    canEnd();   // also puts the transceiver back in standby

    if (!e.busAlive) {
        Serial.println("[CAN] Bus quiet - ignition off.");
    } else {
        Serial.printf("[CAN] %u frames; rpm=%ld coolant_raw=%u temp_raw=%u", frames,
                      e.rpmValid ? (long)e.rpm : -1L, e.coolantRaw, e.tempRaw);
        if (e.trimValid)  Serial.printf(" trim=%u/9", e.trimPos);
        if (e.hoursValid) Serial.printf(" hours=%uh%02um",
                                        e.engineMinutes / 60, e.engineMinutes % 60);
        if (badChecksum) Serial.printf(" (%u 0x102 frames failed checksum)", badChecksum);
        Serial.println();
    }
    return e;
}

void canRawCapture(uint32_t seconds, const uint32_t* ids, size_t nIds) {
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

    // The window is short and it cannot be repeated without a reboot, so say
    // out loud when it opens. The last session lost this data by capturing at a
    // moment nobody was driving the engine.
    Serial.printf("\n[CAN] Raw capture: %us, room for %u frames.\n",
                  (unsigned)seconds, (unsigned)cap);
    Serial.println("[CAN] Every byte of every ID goes to CSV.");
    Serial.println("[CAN] Do the thing you came to do, slowly, holding each position.");
    Serial.println("[CAN] For a driver input, hold each position for ~3s: the plateaus are");
    Serial.println("[CAN] what map a control's positions onto a byte's values. For anything");
    Serial.println("[CAN] that has to be told apart from engine speed, add a fast transient");
    Serial.println("[CAN] at the end -- a lead or a lag is what separates cause from effect.\n");
    for (int i = 5; i > 0; i--) { Serial.printf("[CAN] starting in %d...\n", i); delay(1000); }
    Serial.println("[CAN] *** GO ***");

    size_t n = 0;
    const uint32_t t0 = millis();
    twai_message_t msg;
    while (millis() - t0 < seconds * 1000 && n < cap) {
        if (twai_receive(&msg, pdMS_TO_TICKS(5)) != ESP_OK) continue;
        if (nIds) {
            bool want = false;
            for (size_t k = 0; k < nIds; k++) if (ids[k] == msg.identifier) { want = true; break; }
            if (!want) continue;
        }
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

bool canChecksumValid(const uint8_t* data, uint8_t dlc) {
    if (!data || dlc < 8) return false;
    uint8_t x = 0;
    for (int i = 0; i < 7; i++) x ^= data[i];
    return x == data[7];
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
        if (f102) Serial.printf(" (raw %02X%02X) | 0x102 b2-b5 %02X %02X %02X %02X %s",
                                f102->lastData[0], f102->lastData[1],
                                f102->lastData[2], f102->lastData[3],
                                f102->lastData[4], f102->lastData[5],
                                canChecksumValid(f102->lastData, f102->dlc) ? "ok" : "BAD");
        if (f122) Serial.printf(" | 0x122 %02X %02X %02X %02X %02X %02X",
                                f122->lastData[0], f122->lastData[1], f122->lastData[2],
                                f122->lastData[3], f122->lastData[4], f122->lastData[5]);
        if (f110) Serial.printf(" | 110.b3=%02X", f110->lastData[3]);
        if (f320) Serial.printf(" | 320.b4=%02X", f320->lastData[4]);
        if (f342) Serial.printf(" | 342.b4=%02X", f342->lastData[4]);
        Serial.println();
    }
}

void canWatchChanges(uint32_t learnMs) {
    Serial.printf("\n[CAN] === WATCH: what just changed? ===\n");
    Serial.printf("[CAN] Learning which bytes move on their own for %.0fs.\n", learnMs / 1000.0f);
    Serial.println("[CAN] LEAVE THE SKI ALONE until it says GO -- anything moving now");
    Serial.println("[CAN] is treated as noise and stays hidden for the rest of the run.\n");

    canResetSeen();
    canDrain(learnMs);

    // A byte that never moved while nothing was touched is a byte worth
    // watching. Everything else is engine data, a counter or a checksum.
    struct Base { uint32_t id; uint8_t val[8]; bool watch[8]; };
    static Base base[CAN_MAX_IDS];
    size_t nBase = 0, nWatched = 0;
    for (size_t i = 0; i < canSeenCount() && nBase < CAN_MAX_IDS; i++) {
        const CanFrameSummary* f = canSeen(i);
        base[nBase].id = f->id;
        for (int b = 0; b < 8; b++) {
            base[nBase].val[b]   = f->lastData[b];
            base[nBase].watch[b] = (f->minData[b] == f->maxData[b]);
            if (base[nBase].watch[b]) nWatched++;
        }
        nBase++;
    }
    Serial.printf("[CAN] Watching %u stable bytes across %u IDs. *** GO ***\n",
                  (unsigned)nWatched, (unsigned)nBase);
    Serial.println("[CAN] Work ONE control at a time, slowly, holding each position.\n");

    const uint32_t t0 = millis();
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK) continue;
        if (msg.data_length_code < 8) continue;
        for (size_t i = 0; i < nBase; i++) {
            if (base[i].id != msg.identifier) continue;
            for (int b = 0; b < 8; b++) {
                if (!base[i].watch[b] || msg.data[b] == base[i].val[b]) continue;
                Serial.printf("[%7.1fs] 0x%03X b%d: %02X -> %02X\n",
                              (millis() - t0) / 1000.0f, (unsigned)base[i].id, b,
                              base[i].val[b], msg.data[b]);
                base[i].val[b] = msg.data[b];
            }
            break;
        }
    }
}

void canProbeControl(uint32_t seconds, const uint32_t* ids, size_t nIds, uint32_t learnMs) {
    Serial.println("\n[CAN] === PROBE: record and narrate one interaction ===");
    Serial.printf("[CAN] Learning which bytes move on their own for %.0fs.\n", learnMs / 1000.0f);
    Serial.println("[CAN] LEAVE THE SKI ALONE until it says GO.\n");

    canResetSeen();
    canDrain(learnMs);

    struct Base {
        uint32_t id;
        uint8_t  val[8];
        bool     watch[8];
        // Narration-only dither suppression, tracked per byte.
        uint16_t events[8];
        uint8_t  lo[8], hi[8];
        bool     muted[8];
    };
    static Base base[CAN_MAX_IDS];
    size_t nBase = 0, nWatched = 0;
    for (size_t i = 0; i < canSeenCount() && nBase < CAN_MAX_IDS; i++) {
        const CanFrameSummary* f = canSeen(i);
        base[nBase].id = f->id;
        for (int b = 0; b < 8; b++) {
            base[nBase].val[b]   = f->lastData[b];
            base[nBase].watch[b] = (f->minData[b] == f->maxData[b]);
            base[nBase].events[b] = 0;
            base[nBase].lo[b] = base[nBase].hi[b] = f->lastData[b];
            base[nBase].muted[b] = false;
            if (base[nBase].watch[b]) nWatched++;
        }
        nBase++;
    }

    // Name the blind spot rather than leaving it implicit. Counters and
    // checksums are expected here; anything else listed is a payload byte that
    // will not be narrated, and is worth reading out of the CSV by hand.
    Serial.printf("[CAN] Narrating %u stable bytes. NOT narrated (moved while learning):\n",
                  (unsigned)nWatched);
    for (size_t i = 0; i < nBase; i++) {
        bool any = false;
        for (int b = 0; b < 6; b++) if (!base[i].watch[b]) {
            if (!any) { Serial.printf("[CAN]   0x%03X:", (unsigned)base[i].id); any = true; }
            Serial.printf(" b%d", b);
        }
        if (any) Serial.println("   <- payload, hidden from narration, present in the CSV");
    }

    struct Entry { uint32_t ms; uint16_t id; uint8_t dlc; uint8_t data[8]; };
    const size_t cap = (size_t)seconds * (nIds ? 400 : 1200);
    Entry* buf = (Entry*)ps_malloc(cap * sizeof(Entry));
    if (!buf) buf = (Entry*)malloc(cap * sizeof(Entry));
    if (!buf) { Serial.println("[CAN] Could not allocate the record. Aborting."); return; }

    Serial.printf("\n[CAN] Recording %us. *** GO *** -- work ONE control, holding each state.\n\n",
                  (unsigned)seconds);

    size_t n = 0;
    const uint32_t t0 = millis();
    twai_message_t msg;
    while (millis() - t0 < seconds * 1000) {
        if (twai_receive(&msg, pdMS_TO_TICKS(5)) != ESP_OK) continue;
        if (msg.data_length_code < 8) continue;

        bool want = (nIds == 0);
        for (size_t k = 0; k < nIds && !want; k++) if (ids[k] == msg.identifier) want = true;
        if (want && n < cap) {
            buf[n].ms = millis() - t0;
            buf[n].id = (uint16_t)msg.identifier;
            buf[n].dlc = msg.data_length_code;
            memcpy(buf[n].data, msg.data, 8);
            n++;
        }

        for (size_t i = 0; i < nBase; i++) {
            if (base[i].id != msg.identifier) continue;
            for (int b = 0; b < 8; b++) {
                if (!base[i].watch[b] || msg.data[b] == base[i].val[b]) continue;
                const uint8_t prev = base[i].val[b];
                base[i].val[b] = msg.data[b];
                if (base[i].muted[b]) continue;

                if (msg.data[b] < base[i].lo[b]) base[i].lo[b] = msg.data[b];
                if (msg.data[b] > base[i].hi[b]) base[i].hi[b] = msg.data[b];
                base[i].events[b]++;

                // A byte that has flickered many times but never left a span of
                // one count is a sensor dithering on a rounding boundary. Mute
                // the narration for it; the CSV still has every frame. The span
                // test is what keeps this safe -- an analog sweep walks through
                // many values and can never be mistaken for dither.
                if (base[i].events[b] > 15 && (base[i].hi[b] - base[i].lo[b]) <= 1) {
                    base[i].muted[b] = true;
                    Serial.printf("[%6.1fs] 0x%03X b%d: dithering %02X/%02X, muting "
                                  "narration (still recorded)\n",
                                  (millis() - t0) / 1000.0f, (unsigned)base[i].id, b,
                                  base[i].lo[b], base[i].hi[b]);
                    continue;
                }
                Serial.printf("[%6.1fs] 0x%03X b%d: %02X -> %02X\n",
                              (millis() - t0) / 1000.0f, (unsigned)base[i].id, b,
                              prev, msg.data[b]);
            }
            break;
        }
    }

    Serial.printf("\n[CAN] Recorded %u frames. Dumping CSV.\n", (unsigned)n);
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
    // NO ID filter. The flap has never been seen, so it could be on any ID, and
    // filtering to a guess is how a session gets thrown away. The whole bus for
    // 40 s costs a ~2.5 minute dump, which is the right trade for a signal we
    // have never once observed.
    canProbeControl(40);

    // Stop here rather than falling into canWatchChanges(). Running the watcher
    // after the probe reprints a learn phase and a second "*** GO ***", which
    // reads as the tool having restarted -- and worse, invites acting on a
    // prompt that is no longer recording. One session, one prompt, then halt.
    Serial.println("\n[CAN] Probe complete. Reset the board to run another.");
    canEnd();
    while (true) delay(1000);
}

const char* canBitrateName(CanBitrate r) {
    switch (r) {
        case CanBitrate::Rate125k: return "125 kbit/s";
        case CanBitrate::Rate250k: return "250 kbit/s";
        case CanBitrate::Rate500k: return "500 kbit/s";
        default:                   return "unknown";
    }
}
