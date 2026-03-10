// tmc2209.h — TMC2209 stepper driver UART configuration
//
// Single-wire UART interface (bit-banged) at 115200 baud.
// All 4 TMC2209 drivers share one bus on PIN_TMC_UART (GP18),
// each addressed by its MS1/MS2 pin-configured slave address (0-3).

#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "stepper.h"

// ---- TMC2209 Register Addresses ----

// General Configuration
#define TMC_REG_GCONF          0x00
#define TMC_REG_GSTAT          0x01
#define TMC_REG_IFCNT          0x02  // Interface transmission counter
#define TMC_REG_SLAVECONF      0x03
#define TMC_REG_OTP_PROG       0x04
#define TMC_REG_OTP_READ       0x05
#define TMC_REG_IOIN           0x06  // Read-only input pins status
#define TMC_REG_FACTORY_CONF   0x07

// Velocity-based mode control
#define TMC_REG_IHOLD_IRUN    0x10  // Current control
#define TMC_REG_TPOWERDOWN    0x11  // Delay before power-down
#define TMC_REG_TSTEP         0x12  // Actual measured time between microsteps (read-only)
#define TMC_REG_TPWMTHRS      0x13  // Upper velocity threshold for StealthChop
#define TMC_REG_TCOOLTHRS     0x14  // Lower velocity threshold for CoolStep / StallGuard
#define TMC_REG_VACTUAL       0x22  // Actual velocity (UART-controlled motion)

// StallGuard / CoolStep
#define TMC_REG_SGTHRS        0x40  // StallGuard threshold
#define TMC_REG_SG_RESULT     0x41  // StallGuard result (read-only)
#define TMC_REG_COOLCONF      0x42  // CoolStep configuration

// Chopper configuration
#define TMC_REG_MSCNT         0x6A  // Microstep counter (read-only)
#define TMC_REG_MSCURACT      0x6B  // Actual microstep current (read-only)
#define TMC_REG_CHOPCONF      0x6C  // Chopper configuration
#define TMC_REG_DRV_STATUS    0x6F  // Driver status (read-only)
#define TMC_REG_PWMCONF       0x70  // StealthChop PWM configuration
#define TMC_REG_PWM_SCALE     0x71  // PWM amplitude scaling (read-only)
#define TMC_REG_PWM_AUTO      0x72  // PWM auto-tuning results (read-only)

// ---- GCONF bit definitions ----
#define GCONF_I_SCALE_ANALOG      (1 << 0)  // 0 = internal Vref, 1 = AIN
#define GCONF_INTERNAL_RSENSE     (1 << 1)  // 0 = external Rsense
#define GCONF_EN_SPREADCYCLE      (1 << 2)  // 0 = StealthChop, 1 = SpreadCycle
#define GCONF_SHAFT               (1 << 3)  // 1 = inverse motor direction
#define GCONF_INDEX_OTPW          (1 << 4)  // INDEX pin = overtemp warning
#define GCONF_INDEX_STEP          (1 << 5)  // INDEX pin shows step pulses
#define GCONF_PDN_DISABLE         (1 << 6)  // 1 = disable PDN_UART standby
#define GCONF_MSTEP_REG_SELECT    (1 << 7)  // 1 = microstep via MRES in CHOPCONF
#define GCONF_MULTISTEP_FILT      (1 << 8)  // 1 = step pulse filter enabled
#define GCONF_TEST_MODE           (1 << 9)  // Reserved, must be 0

// ---- CHOPCONF bit positions ----
#define CHOPCONF_TOFF_SHIFT       0   // bits [3:0]  Off-time
#define CHOPCONF_HSTRT_SHIFT      4   // bits [6:4]  Hysteresis start
#define CHOPCONF_HEND_SHIFT       7   // bits [10:7] Hysteresis end
#define CHOPCONF_TBL_SHIFT        15  // bits [16:15] Comparator blank time
#define CHOPCONF_VSENSE_SHIFT     17  // bit 17: 0=high Rsense, 1=low Rsense
#define CHOPCONF_MRES_SHIFT       24  // bits [27:24] Microstep resolution
#define CHOPCONF_INTPOL_SHIFT     28  // bit 28: interpolation to 256 usteps
#define CHOPCONF_DEDGE_SHIFT      29  // bit 29: double edge step pulses
#define CHOPCONF_DISS2G_SHIFT     30  // bit 30: disable short-to-GND prot
#define CHOPCONF_DISS2VS_SHIFT    31  // bit 31: disable S2VS prot

// ---- IHOLD_IRUN bit positions ----
#define IHOLD_SHIFT               0   // bits [4:0]  Standstill current (0-31)
#define IRUN_SHIFT                8   // bits [12:8] Run current (0-31)
#define IHOLDDELAY_SHIFT          16  // bits [19:16] Hold delay (0-15)

// ---- DRV_STATUS bit positions (read-only) ----
#define DRV_STATUS_OTPW           (1 << 0)   // Overtemperature pre-warning
#define DRV_STATUS_OT             (1 << 1)   // Overtemperature
#define DRV_STATUS_S2GA           (1 << 2)   // Short to GND phase A
#define DRV_STATUS_S2GB           (1 << 3)   // Short to GND phase B
#define DRV_STATUS_S2VSA          (1 << 4)   // Short to supply phase A
#define DRV_STATUS_S2VSB          (1 << 5)   // Short to supply phase B
#define DRV_STATUS_OLA            (1 << 6)   // Open load phase A
#define DRV_STATUS_OLB            (1 << 7)   // Open load phase B
#define DRV_STATUS_T120           (1 << 8)   // Temp > 120°C
#define DRV_STATUS_T143           (1 << 9)   // Temp > 143°C
#define DRV_STATUS_T150           (1 << 10)  // Temp > 150°C
#define DRV_STATUS_T157           (1 << 11)  // Temp > 157°C
#define DRV_STATUS_CS_ACTUAL_MASK (0x1F << 16)
#define DRV_STATUS_STEALTHCHOP    (1 << 30)
#define DRV_STATUS_STST           (1 << 31)  // Standstill indicator

// ---- Sense Resistor (ohms) ----
#define TMC2209_RSENSE            0.11f

// ---- TMC2209 UART sync byte ----
#define TMC_SYNC                  0x05

// ---- Functions ----

// Initialize the TMC2209 UART bus and configure all 4 drivers with defaults.
// Call after stepper GPIO init (stepper_init).
void tmc2209_init(void);

// Set motor RMS current in milliamps for a specific axis.
// Configures IHOLD_IRUN register with appropriate CS value.
// Hold current is set to 50% of run current by default.
void tmc2209_set_current(axis_id_t axis, uint16_t current_ma);

// Set microstepping resolution for a specific axis.
// Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256.
// Also enables interpolation to 256 for values < 256.
void tmc2209_set_microsteps(axis_id_t axis, uint16_t usteps);

// Write a 32-bit value to a TMC2209 register.
// axis selects the slave address (0-3).
void tmc2209_write_reg(axis_id_t axis, uint8_t reg, uint32_t value);

// Read a 32-bit value from a TMC2209 register.
// Returns true on success, false on CRC error or timeout.
bool tmc2209_read_reg(axis_id_t axis, uint8_t reg, uint32_t *value);

// Configure StealthChop mode (quiet operation).
// Enables StealthChop with automatic PWM tuning.
void tmc2209_enable_stealthchop(axis_id_t axis);

// Configure SpreadCycle mode (higher torque, more noise).
void tmc2209_enable_spreadcycle(axis_id_t axis);

// Set StallGuard threshold (0-255). Higher = less sensitive.
// Only active in SpreadCycle mode.
void tmc2209_set_stallguard_threshold(axis_id_t axis, uint8_t threshold);

// Read driver status register (temperature flags, shorts, stall, etc.)
// Returns raw DRV_STATUS register value.
bool tmc2209_read_status(axis_id_t axis, uint32_t *status);

#endif // TMC2209_H
