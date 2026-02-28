#include <riscv/rvruntime-frames.h>

/**
 * RISC-V Backtrace Implementation (ESP32-C3)
 * Extracted from AE Smart Shunt firmware
 */

extern "C" void __real_esp_panic_handler(void *info);
extern "C" void append_to_rtc_buffer(const char* format, ...);

extern "C" void __wrap_esp_panic_handler(void *info) {
    if (rtc_crash_info.magic != CRASH_MAGIC) {
        rtc_crash_info.magic = CRASH_MAGIC;
        rtc_crash_info.timestamp = millis();
        rtc_crash_info.buffer[0] = '\0';

        panic_info_t *panic_info = (panic_info_t *)info;
        const RvExcFrame *frame = (const RvExcFrame *)panic_info->frame;

        append_to_rtc_buffer("Panic:%lums\nPC:0x%08lX\nCause:0x%lX\nVal:0x%lX\nRA:0x%lX\nSP:0x%08lX\n\nBacktrace:", 
            millis(), frame->mepc, frame->mcause, frame->mtval, frame->ra, frame->sp
        );

        // RISC-V Stack Walk
        uint32_t fp = frame->s0;
        uint32_t *current_fp = reinterpret_cast<uint32_t*>(static_cast<uintptr_t>(fp));
        append_to_rtc_buffer("0x%08lX ", frame->mepc);
        
        for (int i = 0; i < 32; i++) {
            if ((uint32_t)current_fp < 0x3FC00000 || (uint32_t)current_fp & 3) break;
            
            uint32_t next_fp = current_fp[0]; 
            uint32_t ret_addr = current_fp[1]; 
            
            if (ret_addr == 0) break;
            append_to_rtc_buffer("0x%08lX ", ret_addr);
            
            if (next_fp == 0 || next_fp <= (uint32_t)current_fp) break;
            current_fp = reinterpret_cast<uint32_t*>(static_cast<uintptr_t>(next_fp));
        }
        append_to_rtc_buffer("\n");
    }
    __real_esp_panic_handler(info);
}
