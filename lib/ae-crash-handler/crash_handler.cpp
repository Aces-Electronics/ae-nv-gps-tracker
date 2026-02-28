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

// Internal helper for architecture-specific backtrace files
extern "C" void append_to_rtc_buffer(const char* format, ...) {
    if (rtc_crash_info.magic != CRASH_MAGIC) return;
    
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    size_t current_len = strlen(rtc_crash_info.buffer);
    if (current_len + strlen(buf) < CRASH_BUFFER_SIZE - 1) {
        strcat(rtc_crash_info.buffer, buf);
    }
}

// Architecture-specific implementations are included via #ifdef
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    #include "arch/riscv_backtrace.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
    #include "arch/xtensa_backtrace.h"
#endif
