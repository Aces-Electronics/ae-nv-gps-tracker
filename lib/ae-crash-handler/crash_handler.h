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
