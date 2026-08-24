#pragma once

#include <Arduino.h>
#include <TinyGsmClient.h>

// Set from CI (OTA_VERSION) via platformio.ini. Empty on a local build, which
// otaShouldApply() treats as "version unknown" rather than as a version that
// could match.
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION ""
#endif

// What the backend publishes to ae/downlink/<imei>/OTA.
//
// There is no `size` field. The worker deletes size, firmware_id, triggered_by
// and triggered_by_name before publishing, to keep the frame inside
// PubSubClient's buffer (ae-nv-web-app/backend-worker/src/index.ts:965), so the
// image length has to come from the HTTP Content-Length instead.
struct OtaCommand {
    String url;
    String version;
    String md5;
    bool   force = false;
};

// True if `payload` is a usable OTA command. A zero-length payload is the
// backend clearing the retained topic (clearRetainedOta publishes an empty
// string), so it must parse as "nothing to do" and not as a command.
bool otaParseCommand(const uint8_t* payload, unsigned int length, OtaCommand& out);

// True if this image is worth downloading: not the version we are already
// running, and not one we have already failed on OTA_MAX_ATTEMPTS times.
bool otaShouldApply(const OtaCommand& cmd, const char* runningVersion);

// True if the modem is camped on LTE-M. NB-IoT tops out around 30 kbit/s
// uplink-limited and is not worth starting a 600 KB transfer on.
bool otaLinkIsCatM(TinyGsm& modem, String& actOut);

// Fetch and flash. Returns true only if the image is written, MD5-verified and
// marked bootable; the caller is expected to reboot. Does its own NVS
// bookkeeping so an attempt is counted even if the download crashes the device.
bool otaDownloadAndApply(TinyGsm& modem, const OtaCommand& cmd);
