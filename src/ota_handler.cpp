#include "ota_handler.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_ota_ops.h>

// Bounded because this is a battery device. A tracker that cannot complete a
// download must give up and go back to sleep, not retry every five minutes for
// the two days the retained command lives on the topic.
static const uint8_t  OTA_MAX_ATTEMPTS      = 3;
static const uint32_t OTA_TOTAL_TIMEOUT_MS  = 10UL * 60UL * 1000UL;
static const uint32_t OTA_STALL_TIMEOUT_MS  = 45UL * 1000UL;
static const uint32_t OTA_CONNECT_TIMEOUT_S = 30;
static const int      OTA_MIN_CSQ           = 10;   // ~ -93 dBm

// The mux the OTA socket uses. Deliberately not 0: that is the one PubSubClient
// holds, and although the session is closed before we get here, reusing the mux
// would leave the modem's socket table pointing at a client object we are about
// to abandon.
static const uint8_t OTA_MUX = 1;

// One image is ~600 KB and TinyGSM hands out at most TINY_GSM_RX_BUFFER bytes
// per +CARECV round trip, so the chunk size is what decides how many AT
// exchanges the transfer costs.
static const size_t OTA_CHUNK = 1024;

// The modem UART is the bottleneck, and it is the reason downloads die.
//
// TinyGSM drains a socket byte at a time over this link. At 115200 baud that is
// about 11.5 KB/s, while LTE Cat-M1 delivers up to ~37 KB/s -- so the network
// fills the modem's socket buffer roughly three times faster than it can be
// emptied. That is exactly what "+CAURC: buffer full" reports, and a 680 KB
// image needs 59 seconds of pure UART time at 115200 before any AT overhead.
//
// 921600 gives ~92 KB/s against the radio's ~37, so the UART stops being the
// constraint rather than merely keeping pace -- 460800's ~46 KB/s would leave
// only a 25% margin, and the whole failure mode here is a margin that ran out.
// Both rates were verified on this hardware: +IPR accepted, link alive at the
// new rate, and the restore clean.
//
// Raised only for the download and put back afterwards: AT+IPR is not persisted
// without AT&W, so a modem left fast would be unreachable on the next boot, when
// Serial1 comes up at 115200 again.
static const uint32_t OTA_FAST_BAUD = 921600;
static const uint32_t OTA_BASE_BAUD = 115200;

// Switches the modem and the host UART together, then proves the link still
// works. Returns false having already restored the base rate, so a modem that
// does not take the new rate costs one failed AT exchange rather than a device
// that can no longer be talked to -- which, on a tracker whose only remote fix
// path is this very OTA, would be unrecoverable in the field.
static bool otaSetBaud(TinyGsm& modem, uint32_t baud) {
    modem.sendAT(GF("+IPR="), baud);
    // The reply comes back at the OLD rate; the modem switches after sending it.
    if (modem.waitResponse(2000L) != 1) {
        Serial.printf("[OTA] Modem refused +IPR=%lu; staying at %lu\n",
                      (unsigned long)baud, (unsigned long)OTA_BASE_BAUD);
        return false;
    }
    delay(100);
    Serial1.updateBaudRate(baud);
    delay(100);

    if (!modem.testAT(2000)) {
        Serial.printf("[OTA] No response after switching to %lu; reverting.\n",
                      (unsigned long)baud);
        Serial1.updateBaudRate(OTA_BASE_BAUD);
        delay(100);
        // Best effort: if the modem did switch and we did not, this reaches it
        // at the new rate; if it never switched, it is already listening here.
        modem.sendAT(GF("+IPR="), OTA_BASE_BAUD);
        modem.waitResponse(2000L);
        Serial1.updateBaudRate(OTA_BASE_BAUD);
        delay(100);
        modem.testAT(2000);
        return false;
    }
    Serial.printf("[OTA] Modem UART now %lu baud.\n", (unsigned long)baud);
    return true;
}

struct OtaUrl {
    bool     https = false;
    String   host;
    uint16_t port = 80;
    String   path;
};

static bool parseUrl(const String& url, OtaUrl& out) {
    int schemeEnd = url.indexOf("://");
    if (schemeEnd < 0) return false;

    String scheme = url.substring(0, schemeEnd);
    scheme.toLowerCase();
    if (scheme == "https") {
        out.https = true;
        out.port  = 443;
    } else if (scheme == "http") {
        out.https = false;
        out.port  = 80;
    } else {
        return false;
    }

    int hostStart = schemeEnd + 3;
    int pathStart = url.indexOf('/', hostStart);
    String hostPort = (pathStart < 0) ? url.substring(hostStart)
                                      : url.substring(hostStart, pathStart);
    out.path = (pathStart < 0) ? "/" : url.substring(pathStart);

    int colon = hostPort.indexOf(':');
    if (colon >= 0) {
        out.host = hostPort.substring(0, colon);
        out.port = (uint16_t)hostPort.substring(colon + 1).toInt();
    } else {
        out.host = hostPort;
    }

    return out.host.length() > 0 && out.port > 0;
}

bool otaParseCommand(const uint8_t* payload, unsigned int length, OtaCommand& out) {
    // clearRetainedOta() publishes a zero-length retained message to withdraw a
    // command. Treating that as "no command" is the whole point of the check.
    if (payload == nullptr || length == 0) return false;

    // Elastic in ArduinoJson 7, where StaticJsonDocument is deprecated. Nothing
    // unbounded can reach here: `length` is whatever arrived inside the MQTT
    // receive buffer, so the old 384-byte ceiling was never the real limit.
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        Serial.printf("[OTA] Downlink is not JSON (%s)\n", err.c_str());
        return false;
    }

    out.url     = doc["url"] | "";
    out.version = doc["version"] | "";
    out.md5     = doc["md5"] | "";
    out.force   = doc["force"] | false;

    if (out.url.length() == 0) {
        Serial.println("[OTA] Downlink has no url; ignoring.");
        return false;
    }

    // No md5, no update. Update::end() only verifies when a digest was set, so
    // accepting a command without one would flash whatever the socket returned
    // -- including a truncated body or a captive-portal page -- and mark it
    // bootable. This device has no rollback, so that is unrecoverable.
    out.md5.trim();
    out.md5.toLowerCase();
    if (out.md5.length() != 32) {
        Serial.printf("[OTA] Downlink md5 is not 32 hex chars ('%s'); refusing.\n", out.md5.c_str());
        return false;
    }

    return true;
}

bool otaShouldApply(const OtaCommand& cmd, const char* runningVersion) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* slot    = esp_ota_get_next_update_partition(NULL);

    // A unit still on huge_app.csv. That table is NOT app0-with-no-otadata: it
    // has otadata at 0xe000 and its single app0 is subtype ota_0, so the "no
    // slot" case does not present as NULL. esp_ota_get_next_update_partition()
    // walks ota_0..ota_15, finds only ota_0, and falls through to
    // `return default_ota` -- handing back the partition we are executing from
    // (esp-idf components/app_update/esp_ota_ops.c:575).
    //
    // Update.begin() does not compare that against the running partition either
    // (Updater.cpp takes esp_ota_get_next_update_partition(NULL) and only checks
    // it is non-NULL), so without this test the first OTA command a
    // single-slot unit accepts erases the code it is running from. On an S3
    // executing XIP that is a crash part-way through the write, with no factory
    // partition to fall back to and no second slot to roll back to: the unit is
    // bricked and cannot be recovered over the air.
    //
    // Reachable whenever this build ends up on the old table -- an app-only
    // esptool flash at 0x10000 is enough -- so it is checked here rather than
    // assumed away.
    if (slot == NULL || running == NULL || slot->address == running->address) {
        Serial.println("[OTA] No second app slot in this partition table. This unit needs a USB reflash.");
        return false;
    }
    Serial.printf("[OTA] Target slot '%s' at 0x%06x, %u bytes\n",
                  slot->label, (unsigned)slot->address, (unsigned)slot->size);

    if (!cmd.force && runningVersion != NULL && runningVersion[0] != '\0' &&
        cmd.version == runningVersion) {
        Serial.printf("[OTA] Already running %s; ignoring retained command.\n", runningVersion);
        return false;
    }

    Preferences p;
    p.begin("tracker", true);
    String  lastMd5 = p.getString("ota_md5", "");
    uint8_t tries   = (uint8_t)p.getUInt("ota_tries", 0);
    p.end();

    // The version check above is the primary guard, but it only works once the
    // build actually carries a version AND the backend records it. Until both
    // are true a retained command would be re-offered on every five-minute wake
    // for the full 48h expiry -- 576 downloads of the same image. This counter
    // is what bounds that, so it is not belt-and-braces, it is the guard that is
    // load-bearing today.
    if (!cmd.force && lastMd5 == cmd.md5 && tries >= OTA_MAX_ATTEMPTS) {
        Serial.printf("[OTA] Image %s already attempted %u times; giving up on it.\n",
                      cmd.md5.c_str(), tries);
        return false;
    }

    return true;
}

bool otaLinkIsCatM(TinyGsm& modem, String& actOut) {
    actOut = "";

    modem.sendAT(GF("+CPSI?"));
    if (modem.waitResponse(5000L, GF("+CPSI: ")) != 1) {
        Serial.println("[OTA] +CPSI did not answer; cannot judge the link.");
        return false;
    }
    actOut = modem.stream.readStringUntil('\n');
    actOut.trim();
    modem.waitResponse();

    String act = actOut;
    act.toUpperCase();

    // The SIM7080G reports the access technology first: "LTE CAT-M1,Online,..."
    // or "NB-IOT,Online,..." or "NO SERVICE". Only Cat-M1 is worth 600 KB; a
    // half-finished NB-IoT transfer costs the same battery as a finished one and
    // delivers nothing.
    if (act.indexOf("CAT-M") < 0) {
        Serial.printf("[OTA] Link is '%s', not LTE-M. Not starting the download.\n", actOut.c_str());
        return false;
    }

    int csq = modem.getSignalQuality();
    if (csq == 99 || csq < OTA_MIN_CSQ) {
        Serial.printf("[OTA] Link is LTE-M but CSQ is %d (need >= %d). Not starting the download.\n",
                      csq, OTA_MIN_CSQ);
        return false;
    }

    Serial.printf("[OTA] Link OK: %s (CSQ %d)\n", actOut.c_str(), csq);
    return true;
}

// Counted before the transfer starts, not after it fails: a download that
// browns the device out or trips the crash handler must still burn an attempt,
// or the "give up after three" rule never fires for the failure mode that
// matters most.
static void recordAttempt(const OtaCommand& cmd) {
    Preferences p;
    p.begin("tracker", false);
    if (p.getString("ota_md5", "") == cmd.md5) {
        p.putUInt("ota_tries", p.getUInt("ota_tries", 0) + 1);
    } else {
        p.putString("ota_md5", cmd.md5);
        p.putUInt("ota_tries", 1);
    }
    p.end();
}

static void markApplied(const OtaCommand& cmd) {
    Preferences p;
    p.begin("tracker", false);
    p.putString("ota_md5", cmd.md5);
    p.putUInt("ota_tries", OTA_MAX_ATTEMPTS);
    p.end();
}

// Kept for the life of the boot rather than freed. The device either reboots
// into the new image or deep-sleeps, and deep sleep resets RAM, so there is
// nothing to leak into; deleting it would instead leave the modem's socket
// table holding a dangling pointer for OTA_MUX.
static TinyGsmClient* otaClient(TinyGsm& modem, bool https) {
    static TinyGsmClient*       plain  = nullptr;
    static TinyGsmClientSecure* secure = nullptr;
    if (https) {
        if (secure == nullptr) secure = new TinyGsmClientSecure(modem, OTA_MUX);
        return secure;
    }
    if (plain == nullptr) plain = new TinyGsmClient(modem, OTA_MUX);
    return plain;
}

bool otaDownloadAndApply(TinyGsm& modem, const OtaCommand& cmd) {
    OtaUrl u;
    if (!parseUrl(cmd.url, u)) {
        Serial.printf("[OTA] Cannot parse url '%s'\n", cmd.url.c_str());
        return false;
    }

    recordAttempt(cmd);

    Serial.printf("[OTA] GET %s://%s:%u%s\n", u.https ? "https" : "http",
                  u.host.c_str(), u.port, u.path.c_str());

    // TLS is terminated in the modem (AT+CASSLCFG), not on the ESP32 -- there is
    // no WiFiClientSecure here and no room for a cert bundle. TinyGSM does not
    // set an authmode, so the SIM7080 does not validate the server certificate.
    // The md5 from the MQTT command is what makes the image trustworthy, which
    // is why otaParseCommand() refuses a command without one.
    TinyGsmClient* net = otaClient(modem, u.https);
    // Raise the UART BEFORE the socket is opened.
    //
    // Doing it after the connection was streaming hung the device outright: the
    // modem already had data queued (+CAURC: buffer full arrives while the HTTP
    // headers are still being parsed), and changing the rate mid-stream corrupts
    // the framing. TinyGSM's modemRead() then waits up to setTimeout() PER BYTE
    // -- ten seconds each, a thousand bytes a chunk -- so the transfer never
    // returns and the stall check below never gets to run. Observed as seven
    // minutes of complete silence after the switch.
    //
    // Nothing is in flight here, so the switch is safe.
    const bool fastBaud = otaSetBaud(modem, OTA_FAST_BAUD);

    // Scope guard rather than a restore at the end.
    //
    // There are five early returns between here and the end of this function --
    // connect failure, header failure, a bad content length, an Update.begin()
    // failure -- and every one of them would otherwise leave the modem fast
    // while the next boot opens Serial1 at 115200. modemPowerOn() can recover
    // from that, but only after a reboot, and relying on remembering five exit
    // paths is how the sixth one gets missed.
    struct BaudGuard {
        TinyGsm& modem;
        bool     active;
        ~BaudGuard() { if (active) otaSetBaud(modem, OTA_BASE_BAUD); }
    } baudGuard{modem, fastBaud};

    // 2s, not 10. TinyGSM's modemRead() waits this long PER BYTE, so the socket
    // timeout multiplies by the chunk size when a stream goes bad: at 10s and a
    // 1024-byte chunk that is nearly three hours inside a single read() call,
    // which is why the mid-stream baud switch presented as a total hang rather
    // than as the stall the loop below is meant to catch. The stall and overall
    // timeouts are only checked between chunks, so they cannot rescue a read
    // that never returns.
    //
    // 2s is still ~180x the 11ms a 1024-byte chunk takes at 921600, so it is not
    // tight for a slow link -- it just stops one bad chunk costing hours. The
    // real fix is not corrupting the stream in the first place, above.
    net->setTimeout(2000);

    if (!net->connect(u.host.c_str(), u.port, OTA_CONNECT_TIMEOUT_S)) {
        Serial.println("[OTA] Connect failed.");
        return false;
    }

    String req = "GET " + u.path + " HTTP/1.1\r\n";
    req += "Host: " + u.host + "\r\n";
    req += "User-Agent: ae-gps-tracker/";
    req += FIRMWARE_VERSION;
    req += "\r\n";
    req += "Accept: application/octet-stream\r\n";
    req += "Connection: close\r\n\r\n";
    net->print(req);

    String statusLine = net->readStringUntil('\n');
    statusLine.trim();
    Serial.printf("[OTA] %s\n", statusLine.c_str());
    if (statusLine.indexOf(" 200") < 0) {
        // Redirects are not followed on purpose. /api/ota/<file> answers 200
        // directly; a 3xx here means the URL in the command is not the one the
        // backend meant to hand out, and guessing at the target is how a device
        // ends up flashing an error page.
        Serial.println("[OTA] Server did not answer 200; aborting.");
        net->stop();
        return false;
    }

    long contentLength = -1;
    while (net->connected() || net->available()) {
        String line = net->readStringUntil('\n');
        line.trim();
        if (line.length() == 0) break;  // end of headers
        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("content-length:")) {
            contentLength = line.substring(line.indexOf(':') + 1).toInt();
        }
    }

    if (contentLength <= 0) {
        Serial.println("[OTA] No Content-Length; cannot size the write.");
        net->stop();
        return false;
    }
    Serial.printf("[OTA] Image is %ld bytes\n", contentLength);

    // Repeated from otaShouldApply() rather than trusted: Update.begin() picks
    // its own target with esp_ota_get_next_update_partition(NULL), which on a
    // single-slot table hands back the running partition, and begin() does not
    // check for that. Getting here with the wrong caller ordering would erase
    // the running image, so the last thing before begin() confirms the target.
    const esp_partition_t* target  = esp_ota_get_next_update_partition(NULL);
    const esp_partition_t* current = esp_ota_get_running_partition();
    if (target == NULL || current == NULL || target->address == current->address) {
        Serial.println("[OTA] Refusing to write: the only app slot is the one we are running from.");
        net->stop();
        return false;
    }

    if (!Update.begin((size_t)contentLength, U_FLASH)) {
        Serial.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
        net->stop();
        return false;
    }

    // AFTER begin(), never before: begin() resets the expected digest
    // (Updater.cpp sets _target_md5 = emptyString), so a setMD5() call made
    // first is silently discarded and the image is written unverified.
    Update.setMD5(cmd.md5.c_str());

    uint8_t  buf[OTA_CHUNK];
    size_t   written  = 0;
    uint32_t started  = millis();
    uint32_t lastData = millis();
    uint32_t lastLog  = millis();

    while (written < (size_t)contentLength) {
        if (millis() - started > OTA_TOTAL_TIMEOUT_MS) {
            Serial.println("[OTA] Overall timeout.");
            break;
        }

        int avail = net->available();
        if (avail > 0) {
            size_t want = (size_t)avail < OTA_CHUNK ? (size_t)avail : OTA_CHUNK;
            size_t room = (size_t)contentLength - written;
            if (want > room) want = room;

            int got = net->read(buf, want);
            if (got > 0) {
                if (Update.write(buf, got) != (size_t)got) {
                    Serial.printf("[OTA] Flash write failed: %s\n", Update.errorString());
                    break;
                }
                written += got;
                lastData = millis();

                // One line per 64 KB. The read loop runs thousands of times and
                // this is on the same UART the modem debug uses.
                if ((written % 65536) < (size_t)got) {
                    // Rate matters more than progress here: a download that dies
                    // at 58% did so because it could not keep up, and the only
                    // way to know that from a log is to have measured it.
                    const uint32_t ms = millis() - started;
                    Serial.printf("[OTA] %u / %ld bytes  (%.1f KB/s avg)\n",
                                  (unsigned)written, contentLength,
                                  ms ? (written / 1024.0f) / (ms / 1000.0f) : 0.0f);
                    lastLog = millis();
                }
            }
            continue;
        }

        if (!net->connected() && net->available() == 0) {
            Serial.println("[OTA] Socket closed early.");
            break;
        }
        if (millis() - lastData > OTA_STALL_TIMEOUT_MS) {
            Serial.println("[OTA] Stalled.");
            break;
        }
        // 2ms rather than 20. The modem's socket buffer is filling from the
        // network the whole time this loop is idle, so sleeping a fixed 20ms per
        // empty poll spends exactly the headroom the transfer does not have.
        delay(2);
    }

    net->stop();

    // The base rate is restored by baudGuard's destructor on every path out of
    // this function, including the early returns above.

    {
        const uint32_t ms = millis() - started;
        Serial.printf("[OTA] Transfer: %u bytes in %.1fs (%.1f KB/s)\n",
                      (unsigned)written, ms / 1000.0f,
                      ms ? (written / 1024.0f) / (ms / 1000.0f) : 0.0f);
    }
    (void)lastLog;

    if (written != (size_t)contentLength) {
        Serial.printf("[OTA] Short read: %u of %ld bytes.\n", (unsigned)written, contentLength);
        Update.abort();
        return false;
    }

    // end(true) is what compares the digest and flips otadata. It returns false
    // on a mismatch and leaves the running image untouched.
    if (!Update.end(true)) {
        Serial.printf("[OTA] Verify/finalise failed: %s\n", Update.errorString());
        return false;
    }

    markApplied(cmd);
    Serial.printf("[OTA] Wrote and verified %s. Rebooting.\n", cmd.version.c_str());
    return true;
}
