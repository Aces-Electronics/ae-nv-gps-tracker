#include "crash_handler.h"
#include <esp_debug_helpers.h>
#include <esp_attr.h>
#include <cstdio>
#include <Preferences.h>
#include <rom/rtc.h>
#include <esp_private/panic_internal.h>

// RTC Memory - detailed crash info survives reset
#define CRASH_BUFFER_SIZE 2048
#define CRASH_MAGIC 0xDEADBEEF

typedef struct {
    uint32_t magic;
    uint32_t timestamp;
    char buffer[CRASH_BUFFER_SIZE];
} rtc_crash_data_t;

// Reserve space in RTC memory that isn't wiped on software reset
RTC_NOINIT_ATTR rtc_crash_data_t rtc_crash_info;

static Preferences crashPrefs;

void crash_handler_init() {
    // Hooks are done via linker --wrap
}

bool crash_handler_process_on_boot() {
    if (rtc_crash_info.magic == CRASH_MAGIC) {
        Serial.println("[CRASH] Found crash log in RTC memory!");
        
        crashPrefs.begin("crash", false);
        String newLog = String(rtc_crash_info.buffer);
        crashPrefs.putString("log", newLog);
        crashPrefs.end();
        
        Serial.println("[CRASH] Log saved to NVS. Payload size: " + String(newLog.length()));
        
        // Clear magic so we don't process it again
        rtc_crash_info.magic = 0;
        return true; 
    }
    return false;
}

String crash_handler_get_log() {
    crashPrefs.begin("crash", true);
    String log = crashPrefs.getString("log", "No Crash Log Available");
    crashPrefs.end();
    return log;
}

bool crash_handler_has_log() {
    crashPrefs.begin("crash", true);
    const bool has = crashPrefs.isKey("log");
    crashPrefs.end();
    return has;
}

void crash_handler_clear_log() {
    crashPrefs.begin("crash", false);
    crashPrefs.remove("log");
    crashPrefs.end();
}


// Internal helper for architecture-specific backtrace files
extern "C" void append_to_rtc_buffer(const char* format, ...) {
    if (rtc_crash_info.magic != CRASH_MAGIC) return;

    // Formats straight into the RTC buffer, against the space actually left in
    // it. It used to stage through a char buf[128] first, which silently cut
    // every caller off at 127 characters -- and the register dump is ONE printf
    // of about 700 bytes. So the log that reached the cloud held MEPC, RA, SP,
    // GP, TP and half of T0, then stopped mid-value with "Backtrace:" run
    // straight onto the end of it. Thirty of the thirty-six registers the dump
    // exists to capture were being thrown away, in a 2 KB buffer that had ample
    // room for all of them.
    //
    // The old length check also dropped an over-long append entirely rather than
    // taking what fitted, so the tail of a crash log was all-or-nothing.
    size_t current_len = strnlen(rtc_crash_info.buffer, CRASH_BUFFER_SIZE);
    if (current_len >= CRASH_BUFFER_SIZE - 1) return;

    va_list args;
    va_start(args, format);
    vsnprintf(rtc_crash_info.buffer + current_len,
              CRASH_BUFFER_SIZE - current_len, format, args);
    va_end(args);
}

// Architecture-specific implementations are included via #ifdef
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    #include "arch/riscv_backtrace.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
    #include "arch/xtensa_backtrace.h"
#endif
