#include <xtensa/xtruntime-frames.h>
#include <esp_private/panic_internal.h>

/**
 * Xtensa Backtrace Implementation (ESP32-S3)
 * Uses ESP-IDF's built-in stack walking for reliability
 */

extern "C" void __real_esp_panic_handler(void *info);
extern "C" void append_to_rtc_buffer(const char* format, ...);

extern "C" void __wrap_esp_panic_handler(void *info) {
    if (rtc_crash_info.magic != CRASH_MAGIC) {
        rtc_crash_info.magic = CRASH_MAGIC;
        rtc_crash_info.timestamp = millis();
        rtc_crash_info.buffer[0] = '\0';

        panic_info_t *panic_info = (panic_info_t *)info;
        const XtExcFrame *frame = (const XtExcFrame *)panic_info->frame;

        append_to_rtc_buffer("Panic:%lums\nPC:0x%08lX\nCause:0x%lX\nAddr:0x%lX\n\nBacktrace:", 
            millis(), frame->pc, frame->exccause, frame->excvaddr
        );

        // Xtensa Stack Walk via ESP-IDF helper
        esp_backtrace_frame_t bt_frame = {
            .pc = frame->pc,
            .sp = frame->a1,
            .next_pc = frame->a0
        };
        
        // Print PC of the frame where the exception occurred
        append_to_rtc_buffer("0x%08lX ", esp_cpu_process_stack_pc(bt_frame.pc));

        for (int i = 0; i < 32; i++) {
            if (!esp_backtrace_get_next_frame(&bt_frame)) break;
            append_to_rtc_buffer("0x%08lX ", esp_cpu_process_stack_pc(bt_frame.pc));
        }
        append_to_rtc_buffer("\n");
    }
    __real_esp_panic_handler(info);
}
