#pragma once

#include <Arduino.h>

/**
 * @brief Initialize the crash handler. 
 * Should be called early in setup().
 */
void crash_handler_init();

/**
 * @brief Checks if a crash occurred in the previous session.
 * If found, copies the log from RTC memory to NVS and clears RTC.
 * 
 * @return true if a crash log was found and processed.
 */
bool crash_handler_process_on_boot();

/**
 * @brief Retrieves the last saved crash log from NVS.
 * 
 * @return String containing the crash log and backtrace.
 */
String crash_handler_get_log();

/**
 * @brief Whether NVS still holds a crash log.
 *
 * Distinct from crash_handler_process_on_boot(), which answers "did the previous
 * boot panic" and can only be true once -- it consumes the RTC magic. This asks
 * the durable copy instead, so a log captured on an earlier boot and never
 * delivered can still be found, dumped and uploaded.
 *
 * @return true if a stored crash log is present.
 */
bool crash_handler_has_log();

/**
 * @brief Clears the crash log from NVS.
 */
void crash_handler_clear_log();

