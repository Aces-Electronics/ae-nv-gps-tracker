#pragma once

// The LILYGO_T_SIM7080G_S3 board marker is set in platformio.ini's build_flags,
// which is where it reaches libraries as well as this header. It was defined
// here too until every file including this one warned about redefining it.
// Nothing in the tree tests it -- the LilyGo original picked a pin block with
// it and those conditionals are long gone -- so the second copy bought nothing.

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

// U2 CS is strapped high through R12, selecting I2C.
//
// SDO/SA0 is left floating on the board. The LIS3DH carries an internal
// pull-up on that pin (~20.4k typ, datasheet Table 3), NOT a pull-down, so
// floating sits high and the expected address is 0x19. Both are probed anyway,
// and imuBegin() logs which one answered.
//
// Worth knowing before anyone "fixes" the float by strapping SA0 to GND: that
// would select 0x18, but it also puts the rail across the internal pull-up for
// ~162uA continuously at 3.3V -- on its own more than the <100uA deep-sleep
// budget. Tie it to VDD_IO if determinism is wanted; that keeps 0x19 and costs
// nothing. Grounding it is only sensible alongside CTRL_REG0 (0x1E) = 0x90,
// which disables the pull-up.
#define DB_IMU_ADDR_LOW      0x18
#define DB_IMU_ADDR_HIGH     0x19

// No I2C pull-ups are fitted on the daughter board: R8/R9 were pull-downs and
// came off rather than going back on the right way up, so the bus rests on the
// S3's internal pull-ups. Arduino's I2C init enables those (esp32-hal-i2c.c
// sets sda/scl_pullup_en), but they are weak -- around 45k -- and 45k against
// the bus capacitance needs far longer to reach VIH than 100kHz standard mode
// allows. Dropping the clock buys the line the time it needs to get there.
//
// This is a workaround for missing parts, not a design choice. Fit real 4.7k
// pull-ups and this goes back to 100000 or higher.
#define DB_I2C_HZ            50000

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
