// pins.h — RP2040 GPIO pin assignments
//
// 4x Trinamic stepper drivers (TMC2209): STEP, DIR, EN, + UART for config
// SPI to FPGA
// Sensor inputs (limit switches, part-present photogate)
// Pneumatic outputs (air jets)

#ifndef PINS_H
#define PINS_H

// ---- SPI to FPGA ----
#define PIN_SPI_SCK   2
#define PIN_SPI_MOSI  3   // RP2040 → FPGA
#define PIN_SPI_MISO  4   // FPGA → RP2040
#define PIN_SPI_CS    5   // Active-low chip select
#define FPGA_SPI_INST spi0

// ---- Stepper Axis 0: Feeder ----
#define PIN_STEP_0    6
#define PIN_DIR_0     7
#define PIN_EN_0      8

// ---- Stepper Axis 1: Conveyor / Presentation ----
#define PIN_STEP_1    9
#define PIN_DIR_1     10
#define PIN_EN_1      11

// ---- Stepper Axis 2: Singulation ----
#define PIN_STEP_2    12
#define PIN_DIR_2     13
#define PIN_EN_2      14

// ---- Stepper Axis 3: Sort Diverter ----
#define PIN_STEP_3    15
#define PIN_DIR_3     16
#define PIN_EN_3      17

// ---- Trinamic UART (shared single-wire bus, directly addressed) ----
#define PIN_TMC_UART  18

// ---- Limit / Home Switches (active-low with internal pull-up) ----
#define PIN_HOME_0    19
#define PIN_HOME_1    20
#define PIN_HOME_2    21
#define PIN_HOME_3    22

// ---- Sensor Inputs ----
#define PIN_PART_DETECT  26  // Photogate: part present in inspection zone (ADC capable)
#define PIN_BIN_FULL     27  // Reject bin full sensor

// ---- Pneumatic Outputs ----
#define PIN_AIR_JET_0    28  // Air jet for part separation / singulation

// ---- Status LED ----
#define PIN_LED          25  // Onboard LED (Pico)

// ---- Stepper array helpers ----
#define NUM_AXES 4

static const uint8_t STEP_PINS[NUM_AXES] = {PIN_STEP_0, PIN_STEP_1, PIN_STEP_2, PIN_STEP_3};
static const uint8_t DIR_PINS[NUM_AXES]  = {PIN_DIR_0,  PIN_DIR_1,  PIN_DIR_2,  PIN_DIR_3};
static const uint8_t EN_PINS[NUM_AXES]   = {PIN_EN_0,   PIN_EN_1,   PIN_EN_2,   PIN_EN_3};
static const uint8_t HOME_PINS[NUM_AXES] = {PIN_HOME_0, PIN_HOME_1, PIN_HOME_2, PIN_HOME_3};

#endif // PINS_H
