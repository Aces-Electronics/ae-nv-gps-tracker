#ifndef BLE_PIN_H
#define BLE_PIN_H

// The six-digit BLE pairing PIN, derived the way the phone app derives the one it displays:
// from the BLE (Bluetooth) address, last octet minus two (wrapping inside the octet, never
// borrowing from the one before), then the last three octets as a big-endian number modulo
// 1,000,000. Callers zero-pad to six digits when printing.
//
// This takes the Bluetooth address, not the Wi-Fi base MAC. The app only ever has the address
// a phone reports and subtracts two from it, so deriving from that same address is what makes
// the PIN on the phone's screen match the passkey here on any SDK build. Deriving from the
// base MAC only agrees when Bluetooth = base + 2, which is an SDK setting: it holds on the
// ESP32 and ESP32-C3, but the ESP32-S3 on the Arduino 2.0.17 core reserves two universal MACs
// and puts Bluetooth at base + 1 (confirmed on hardware: a PDU board's BLE address was
// ...03:ED for Wi-Fi MAC ...03:EC), so the base-MAC PIN was one off from the displayed one.
//
// Pure arithmetic with no Arduino headers so it runs under `pio test -e native`.
#include <stdint.h>

inline uint32_t blePinFromBtMac(const uint8_t bt[6]) {
    const uint8_t last = (uint8_t)(bt[5] - 2);
    const uint32_t val = ((uint32_t)bt[3] << 16) | ((uint32_t)bt[4] << 8) | last;
    return val % 1000000;
}

#endif // BLE_PIN_H
