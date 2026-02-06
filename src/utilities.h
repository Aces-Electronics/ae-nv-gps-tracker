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
