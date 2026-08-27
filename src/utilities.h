#pragma once

#define LILYGO_T_SIM7080G_S3

// Modem Pins (T-SIM7080G-S3)
#define MODEM_TX             5
#define MODEM_RX             4
#define MODEM_PWRKEY         41
#define MODEM_DTR            42
#define MODEM_RI             3

// I2C Pins (SDA=15, SCL=7)
#define I2C_SDA              15
#define I2C_SCL              7

// PMU / Battery Pins
#define PMU_IRQ              6
#define ONE_WIRE_PIN         42 
#define BAT_ADC_PIN          1  // Battery Voltage ADC

// Telemetry/Status LEDs (NeoPixel)
#define NEOPIXEL_PIN         48 // Built-in RGB on some S3 boards, check specific revision
#define NEOPIXEL_COUNT       1

// GPS/GNSS is internal to the SIM7080G, accessed via AT commands

// ---------------------------------------------------------------------------
// AE Tracker Daughter Board (hardware/ae-tracker-db) -- 16-pin header J4
//
// J4 plugs onto the LilyGo's expansion header. The mapping below is locked at
// both ends: J4.1/J4.2 are 3V3/GND and J4.15/J4.16 are GND/DC5, which is
// exactly how the LilyGo pinout brackets those twelve GPIOs.
//
// J4.7 (GPIO3) and J4.8 (GPIO46) are deliberately NC. Both are ESP32-S3
// strapping pins, and GPIO3 is already MODEM_RI on this board.
//
// Every pin below is GPIO0-21, so all of them are RTC GPIOs: they can hold a
// level across deep sleep, and the IMU interrupts can act as ext1 wake sources.
// ---------------------------------------------------------------------------

// CAN -- SN65HVD231 (U1). The S3's TWAI controller routes to any GPIO.
#define DB_CAN_TX            17 // J4.4  -> U1.1 (D), transceiver driver input
#define DB_CAN_RX            18 // J4.5  -> U1.4 (R), transceiver receiver output

// SN65HVD231 Rs: LOW = high-speed mode, HIGH = standby (~370uA). Nothing on the
// board biases it, so it floats until firmware drives it -- drive LOW before
// opening the bus, HIGH before deep sleep.
#define DB_CAN_RS            14 // J4.14 -> U1.8 (Rs)

// IMU -- LIS3DH (U2). This is a SECOND I2C bus; the AXP2101 PMU keeps the
// I2C_SDA/I2C_SCL pair above, and these pins are not shared with it.
#define DB_IMU_SDA           10 // J4.10 -> U2.6 (SDI)
#define DB_IMU_SCL           11 // J4.11 -> U2.4 (SPC)
#define DB_IMU_INT1          12 // J4.12 -> U2.11 (INT1)
#define DB_IMU_INT2          13 // J4.13 -> U2.9 (INT2)

// U2 CS is strapped high through R12, selecting I2C. SDO/SA0 is left floating
// on the board, so the address is not pinned to one value -- probe both.
#define DB_IMU_ADDR_LOW      0x18
#define DB_IMU_ADDR_HIGH     0x19

// Charger -- BQ25176J (U3), behind the TPS1H000-Q1 (U4) supply switch.
//
// SUPPLY_EN carries a 1.8M pull-up to 3.3V (R2), so the jetski supply is ON by
// default: while the ESP32 is in reset, unflashed, or asleep with the pin
// released. Driving it LOW is the only thing that stops the board drawing from
// the jetski battery.
#define DB_SUPPLY_EN         16 // J4.3  -> U4.1 (IN)

// Both open-drain with 10k pull-ups (R10/R11), so both idle HIGH.
// STAT: LOW = charging, HIGH = charged or disabled, blinking = fault.
// PG is active low: LOW = input supply present and within the charger's window.
#define DB_CHG_STAT          8  // J4.6  -> U3.5 (STAT)
#define DB_CHG_PG            9  // J4.9  -> U3.6 (~PG)
