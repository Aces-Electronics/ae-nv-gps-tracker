// Host tests for the tracker's BLE pairing passkey.
//
// The phone app derives the PIN it shows from the BLE address it sees: last octet minus
// two, then the last three octets modulo 1,000,000 (MacUtils.getWifiMacFromBleMac and
// generatePinFromMac). The tracker used the Wi-Fi base MAC instead, on the assumption that
// the BLE address is base + 2. On the ESP32-S3 with the Arduino 2.0.17 core the SDK reserves
// two universal MACs, so it is base + 1 and the two derivations differ by one: the PIN on the
// phone's screen was rejected. The tracker now derives it from its own BLE address, which is
// what the app does, so the two agree whatever the SDK's offset is.
//
// Expected values come from real devices, not from the code under test: two PDU boards
// (same ESP32-S3, same SDK) printed 964343 and 960171 on their consoles while their BLE
// addresses, read in the AE app, were ...14:39 and ...03:ED. scripts/ble_pin.py in the
// superproject, an independent implementation, gives the same numbers.
#include <unity.h>

#include "../../src/ble_pin.h"

void setUp(void) {}
void tearDown(void) {}

static const uint8_t kBoardA[6] = {0xAC, 0x27, 0x6E, 0xD5, 0x03, 0xED};  // BLE address, prints 960171
static const uint8_t kBoardB[6] = {0xAC, 0x27, 0x6E, 0xD5, 0x14, 0x39};  // BLE address, prints 964343

void test_matches_what_a_real_board_printed() {
    TEST_ASSERT_EQUAL_UINT32(960171, blePinFromBtMac(kBoardA));
    TEST_ASSERT_EQUAL_UINT32(964343, blePinFromBtMac(kBoardB));
}

void test_is_not_the_pin_of_the_bluetooth_address_itself() {
    // Deriving from the address without the minus-two is the PIN the app never shows.
    TEST_ASSERT_NOT_EQUAL_UINT32(960173, blePinFromBtMac(kBoardA));
}

void test_the_subtraction_wraps_inside_the_last_octet_without_borrowing() {
    const uint8_t one[6] = {0xAC, 0x27, 0x6E, 0xD5, 0x03, 0x01};   // 0x01 - 2 = 0xFF, not a borrow from 0x03
    const uint8_t zero[6] = {0xAC, 0x27, 0x6E, 0xD5, 0x03, 0x00};  // 0x00 - 2 = 0xFE
    TEST_ASSERT_EQUAL_UINT32(960191, blePinFromBtMac(one));
    TEST_ASSERT_EQUAL_UINT32(960190, blePinFromBtMac(zero));
}

void test_small_values_are_left_for_the_caller_to_zero_pad() {
    const uint8_t small[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
    TEST_ASSERT_EQUAL_UINT32(1, blePinFromBtMac(small));
}

void test_only_the_last_three_octets_matter() {
    const uint8_t other[6] = {0x11, 0x22, 0x33, 0xD5, 0x03, 0xED};
    TEST_ASSERT_EQUAL_UINT32(960171, blePinFromBtMac(other));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_matches_what_a_real_board_printed);
    RUN_TEST(test_is_not_the_pin_of_the_bluetooth_address_itself);
    RUN_TEST(test_the_subtraction_wraps_inside_the_last_octet_without_borrowing);
    RUN_TEST(test_small_values_are_left_for_the_caller_to_zero_pad);
    RUN_TEST(test_only_the_last_three_octets_matter);
    return UNITY_END();
}
