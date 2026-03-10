// tmc2209.c — TMC2209 single-wire UART driver (bit-banged)
//
// Implements the TMC2209 single-wire UART protocol at 115200 baud.
// TX and RX share a single GPIO pin (PIN_TMC_UART / GP18).
// All 4 drivers on the bus are addressed by slave address 0-3.
//
// Datagram format:
//   Write: [sync=0x05] [slave_addr] [reg|0x80] [data3] [data2] [data1] [data0] [crc]
//   Read request: [sync=0x05] [slave_addr] [reg] [crc]   (4 bytes)
//   Read response: [sync=0x05] [0xFF] [reg] [data3] [data2] [data1] [data0] [crc]  (8 bytes)

#include "tmc2209.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <string.h>

// ---- UART Timing ----
// 115200 baud = 8.6805 us per bit
// At 125 MHz system clock, that's ~1085 cycles per bit.
// We use busy_wait_us_32 for sub-microsecond timing with a correction loop.
#define TMC_BAUD_RATE        115200
#define TMC_BIT_TIME_US      8   // ~8.68 us per bit (truncated, fine-tuned in code)

// Precise bit time in microseconds (we use a combination of us delay + nop padding)
// 1e6 / 115200 = 8.6805... us
// We delay 8 us then burn ~85 cpu cycles at 125 MHz for the remaining 0.68 us
// 0.68 us * 125 = 85 cycles. Each nop is 1 cycle.
#define TMC_BIT_FUDGE_CYCLES 85

// Inter-byte and response timing
#define TMC_READ_DELAY_US    500   // Delay after read request before sampling response
#define TMC_BYTE_GAP_US      0    // No mandatory gap between bytes in same datagram
#define TMC_WRITE_SETTLE_US  500  // Settle time after write for driver to process
#define TMC_MAX_RETRIES      3    // Max retries on CRC failure

// ---- Static helpers ----

// Spin-wait for precise bit timing
static inline void tmc_bit_delay(void) {
    busy_wait_us_32(TMC_BIT_TIME_US);
    // Fine-tune: burn remaining fractional microsecond
    // ~85 nop iterations at 125 MHz = ~0.68 us
    for (volatile int i = 0; i < TMC_BIT_FUDGE_CYCLES; i++) {
        __asm volatile("nop");
    }
}

// CRC8 calculation per TMC2209 datasheet.
// Polynomial: x^8 + x^2 + x + 1 (0x07), init = 0, no reflect, no final XOR.
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc >> 7) ^ (byte >> 7)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte <<= 1;
        }
    }
    return crc;
}

// Configure pin as output (for TX)
static inline void tmc_pin_output(void) {
    gpio_set_dir(PIN_TMC_UART, GPIO_OUT);
    gpio_put(PIN_TMC_UART, 1);  // Idle high
}

// Configure pin as input (for RX)
static inline void tmc_pin_input(void) {
    gpio_set_dir(PIN_TMC_UART, GPIO_IN);
    gpio_pull_up(PIN_TMC_UART);  // Idle high with pull-up
}

// Send a single byte via bit-banged UART (LSB first, 8N1)
static void tmc_send_byte(uint8_t byte) {
    // Start bit (low)
    gpio_put(PIN_TMC_UART, 0);
    tmc_bit_delay();

    // 8 data bits, LSB first
    for (int bit = 0; bit < 8; bit++) {
        gpio_put(PIN_TMC_UART, (byte >> bit) & 1);
        tmc_bit_delay();
    }

    // Stop bit (high)
    gpio_put(PIN_TMC_UART, 1);
    tmc_bit_delay();
}

// Receive a single byte via bit-banged UART (LSB first, 8N1)
// Returns true if a byte was received, false on timeout.
static bool tmc_recv_byte(uint8_t *byte, uint32_t timeout_us) {
    // Wait for start bit (falling edge) with timeout
    uint64_t deadline = time_us_64() + timeout_us;

    while (gpio_get(PIN_TMC_UART)) {
        if (time_us_64() >= deadline) {
            return false;
        }
    }

    // Align to center of start bit
    busy_wait_us_32(TMC_BIT_TIME_US / 2);

    // Verify start bit is still low
    if (gpio_get(PIN_TMC_UART)) {
        return false;  // False start / noise
    }

    // Wait to center of first data bit
    tmc_bit_delay();

    // Read 8 data bits, LSB first
    uint8_t val = 0;
    for (int bit = 0; bit < 8; bit++) {
        if (gpio_get(PIN_TMC_UART)) {
            val |= (1 << bit);
        }
        tmc_bit_delay();
    }

    // Stop bit (just wait through it)
    // We're already centered on the stop bit after the last data bit delay
    *byte = val;
    return true;
}

// Send a complete datagram (array of bytes)
static void tmc_send_datagram(const uint8_t *data, uint8_t len) {
    tmc_pin_output();
    busy_wait_us_32(10);  // Brief setup time

    for (uint8_t i = 0; i < len; i++) {
        tmc_send_byte(data[i]);
    }

    // Keep line high after last stop bit
    gpio_put(PIN_TMC_UART, 1);
}

// ---- Public API ----

void tmc2209_write_reg(axis_id_t axis, uint8_t reg, uint32_t value) {
    if (axis >= NUM_AXES) return;

    uint8_t datagram[8];
    datagram[0] = TMC_SYNC;              // Sync byte
    datagram[1] = (uint8_t)axis;         // Slave address (0-3)
    datagram[2] = reg | 0x80;            // Register + write flag
    datagram[3] = (value >> 24) & 0xFF;  // Data MSB first
    datagram[4] = (value >> 16) & 0xFF;
    datagram[5] = (value >> 8)  & 0xFF;
    datagram[6] = value & 0xFF;
    datagram[7] = tmc_crc8(datagram, 7);

    // Disable interrupts during UART transaction for timing accuracy
    uint32_t irq_state = save_and_disable_interrupts();
    tmc_send_datagram(datagram, 8);
    restore_interrupts(irq_state);

    // Wait for the driver to process the write
    busy_wait_us_32(TMC_WRITE_SETTLE_US);
}

bool tmc2209_read_reg(axis_id_t axis, uint8_t reg, uint32_t *value) {
    if (axis >= NUM_AXES || value == NULL) return false;

    // Build read request datagram (4 bytes)
    uint8_t request[4];
    request[0] = TMC_SYNC;
    request[1] = (uint8_t)axis;
    request[2] = reg & 0x7F;  // Register without write flag
    request[3] = tmc_crc8(request, 3);

    for (int retry = 0; retry < TMC_MAX_RETRIES; retry++) {
        // Send read request with interrupts disabled
        uint32_t irq_state = save_and_disable_interrupts();
        tmc_send_datagram(request, 4);

        // Switch pin to input and wait for response
        tmc_pin_input();
        restore_interrupts(irq_state);

        // Wait for driver to prepare response
        busy_wait_us_32(TMC_READ_DELAY_US);

        // Read 8-byte response with interrupts disabled
        uint8_t response[8];
        bool ok = true;

        irq_state = save_and_disable_interrupts();
        for (int i = 0; i < 8; i++) {
            // Generous timeout for first byte (driver may be slow),
            // short timeout for subsequent bytes
            uint32_t timeout = (i == 0) ? 5000 : 2000;
            if (!tmc_recv_byte(&response[i], timeout)) {
                ok = false;
                break;
            }
        }
        restore_interrupts(irq_state);

        // Switch pin back to output (idle high)
        tmc_pin_output();

        if (!ok) {
            busy_wait_us_32(TMC_WRITE_SETTLE_US);
            continue;
        }

        // Validate response
        // Check sync byte
        if (response[0] != TMC_SYNC) {
            continue;
        }

        // Check master address (0xFF = reply to master)
        if (response[1] != 0xFF) {
            continue;
        }

        // Check register address matches
        if (response[2] != reg) {
            continue;
        }

        // Verify CRC
        uint8_t expected_crc = tmc_crc8(response, 7);
        if (response[7] != expected_crc) {
            continue;
        }

        // Extract 32-bit value (MSB first)
        *value = ((uint32_t)response[3] << 24) |
                 ((uint32_t)response[4] << 16) |
                 ((uint32_t)response[5] << 8)  |
                 ((uint32_t)response[6]);

        return true;
    }

    return false;  // All retries failed
}

// Convert RMS current in mA to TMC2209 current scale (CS) value (0-31).
// Formula from TMC2209 datasheet:
//   I_rms = (CS + 1) / 32 * Vfs / (sqrt(2) * Rsense)
//   Solving for CS:  CS = I_rms * 32 * sqrt(2) * Rsense / Vfs - 1
// Vfs = 0.325V (when vsense = 0) or 0.180V (when vsense = 1)
//
// Returns the CS value and sets *vsense to the appropriate mode.
static uint8_t current_to_cs(uint16_t current_ma, bool *vsense) {
    float i_rms = current_ma / 1000.0f;

    // Try with vsense = 0 (Vfs = 0.325V, higher range) first
    float cs_f = (i_rms * 32.0f * 1.41421f * TMC2209_RSENSE) / 0.325f - 1.0f;

    if (cs_f < 16.0f) {
        // Current is low enough for high-sensitivity mode (vsense = 1, Vfs = 0.180V)
        // This gives better current regulation at low currents
        *vsense = true;
        cs_f = (i_rms * 32.0f * 1.41421f * TMC2209_RSENSE) / 0.180f - 1.0f;
    } else {
        *vsense = false;
    }

    // Clamp to valid range
    if (cs_f < 0.0f) cs_f = 0.0f;
    if (cs_f > 31.0f) cs_f = 31.0f;

    return (uint8_t)(cs_f + 0.5f);  // Round to nearest
}

// Convert microstep value (1,2,4,...,256) to MRES field value (8,7,6,...,0)
static uint8_t usteps_to_mres(uint16_t usteps) {
    switch (usteps) {
        case 256: return 0;
        case 128: return 1;
        case  64: return 2;
        case  32: return 3;
        case  16: return 4;
        case   8: return 5;
        case   4: return 6;
        case   2: return 7;
        case   1: return 8;
        default:  return 4;  // Default to 16 microsteps
    }
}

void tmc2209_set_current(axis_id_t axis, uint16_t current_ma) {
    if (axis >= NUM_AXES) return;

    bool vsense;
    uint8_t cs = current_to_cs(current_ma, &vsense);

    // Hold current = 50% of run current (minimum 1)
    uint8_t ihold = cs / 2;
    if (ihold < 1 && cs > 0) ihold = 1;

    // Build IHOLD_IRUN value
    uint32_t ihold_irun = 0;
    ihold_irun |= ((uint32_t)ihold & 0x1F) << IHOLD_SHIFT;
    ihold_irun |= ((uint32_t)cs    & 0x1F) << IRUN_SHIFT;
    ihold_irun |= ((uint32_t)6     & 0x0F) << IHOLDDELAY_SHIFT;  // Moderate hold delay

    tmc2209_write_reg(axis, TMC_REG_IHOLD_IRUN, ihold_irun);

    // Update CHOPCONF vsense bit
    // Read current CHOPCONF first, then modify vsense
    uint32_t chopconf;
    if (tmc2209_read_reg(axis, TMC_REG_CHOPCONF, &chopconf)) {
        if (vsense) {
            chopconf |= (1u << CHOPCONF_VSENSE_SHIFT);
        } else {
            chopconf &= ~(1u << CHOPCONF_VSENSE_SHIFT);
        }
        tmc2209_write_reg(axis, TMC_REG_CHOPCONF, chopconf);
    }
}

void tmc2209_set_microsteps(axis_id_t axis, uint16_t usteps) {
    if (axis >= NUM_AXES) return;

    uint8_t mres = usteps_to_mres(usteps);

    // Read current CHOPCONF to preserve other settings
    uint32_t chopconf;
    if (!tmc2209_read_reg(axis, TMC_REG_CHOPCONF, &chopconf)) {
        // If read fails, use a sensible default CHOPCONF
        // TOFF=3, HSTRT=4, HEND=1, TBL=2 (typical good defaults)
        chopconf = (3u << CHOPCONF_TOFF_SHIFT) |
                   (4u << CHOPCONF_HSTRT_SHIFT) |
                   (1u << CHOPCONF_HEND_SHIFT) |
                   (2u << CHOPCONF_TBL_SHIFT);
    }

    // Clear MRES field and set new value
    chopconf &= ~(0x0Fu << CHOPCONF_MRES_SHIFT);
    chopconf |= ((uint32_t)mres & 0x0F) << CHOPCONF_MRES_SHIFT;

    // Enable interpolation to 256 microsteps if native resolution is lower
    if (usteps < 256) {
        chopconf |= (1u << CHOPCONF_INTPOL_SHIFT);
    } else {
        chopconf &= ~(1u << CHOPCONF_INTPOL_SHIFT);
    }

    tmc2209_write_reg(axis, TMC_REG_CHOPCONF, chopconf);
}

void tmc2209_enable_stealthchop(axis_id_t axis) {
    if (axis >= NUM_AXES) return;

    // GCONF: disable SpreadCycle (enables StealthChop), enable UART control
    uint32_t gconf = 0;
    gconf |= GCONF_PDN_DISABLE;        // Disable PDN standby (use UART)
    gconf |= GCONF_MSTEP_REG_SELECT;   // Microstep resolution via CHOPCONF MRES
    gconf |= GCONF_MULTISTEP_FILT;     // Step pulse filter
    // Bit 2 (EN_SPREADCYCLE) = 0 → StealthChop enabled
    tmc2209_write_reg(axis, TMC_REG_GCONF, gconf);

    // PWMCONF: Enable automatic PWM tuning for StealthChop
    // Default PWM values: PWM_OFS=36, PWM_GRAD=0, pwm_freq=01 (2/1024 fCLK),
    // pwm_autoscale=1, pwm_autograd=1, freewheel=0, PWM_REG=4, PWM_LIM=12
    uint32_t pwmconf = 0;
    pwmconf |= (36u << 0);        // PWM_OFS [7:0] = 36
    pwmconf |= (0u  << 8);        // PWM_GRAD [15:8] = 0 (auto-tuned)
    pwmconf |= (1u  << 16);       // pwm_freq [17:16] = 01 (2/1024 fCLK)
    pwmconf |= (1u  << 18);       // pwm_autoscale = 1
    pwmconf |= (1u  << 19);       // pwm_autograd = 1
    pwmconf |= (0u  << 20);       // freewheel [21:20] = 00 (normal operation)
    pwmconf |= (4u  << 24);       // PWM_REG [27:24] = 4
    pwmconf |= (12u << 28);       // PWM_LIM [31:28] = 12
    tmc2209_write_reg(axis, TMC_REG_PWMCONF, pwmconf);

    // TPWMTHRS: StealthChop active when TSTEP >= this value.
    // Set high to keep StealthChop active at typical sorting speeds.
    // 0 = disable threshold (StealthChop always), we use a generous threshold.
    tmc2209_write_reg(axis, TMC_REG_TPWMTHRS, 0);
}

void tmc2209_enable_spreadcycle(axis_id_t axis) {
    if (axis >= NUM_AXES) return;

    // GCONF: enable SpreadCycle
    uint32_t gconf = 0;
    gconf |= GCONF_EN_SPREADCYCLE;      // SpreadCycle mode
    gconf |= GCONF_PDN_DISABLE;         // Disable PDN standby (use UART)
    gconf |= GCONF_MSTEP_REG_SELECT;    // Microstep resolution via CHOPCONF
    gconf |= GCONF_MULTISTEP_FILT;      // Step pulse filter
    tmc2209_write_reg(axis, TMC_REG_GCONF, gconf);
}

void tmc2209_set_stallguard_threshold(axis_id_t axis, uint8_t threshold) {
    if (axis >= NUM_AXES) return;
    tmc2209_write_reg(axis, TMC_REG_SGTHRS, (uint32_t)threshold);
}

bool tmc2209_read_status(axis_id_t axis, uint32_t *status) {
    return tmc2209_read_reg(axis, TMC_REG_DRV_STATUS, status);
}

void tmc2209_init(void) {
    // Initialize the UART pin
    gpio_init(PIN_TMC_UART);
    gpio_set_dir(PIN_TMC_UART, GPIO_OUT);
    gpio_put(PIN_TMC_UART, 1);  // Idle high
    gpio_pull_up(PIN_TMC_UART);

    // Allow bus to settle
    busy_wait_us_32(10000);  // 10 ms

    // Configure each TMC2209 driver
    for (int axis = 0; axis < NUM_AXES; axis++) {
        // Clear status flags (write 1 to clear)
        tmc2209_write_reg(axis, TMC_REG_GSTAT, 0x07);

        // Set GCONF for StealthChop + UART control
        tmc2209_enable_stealthchop(axis);

        // CHOPCONF: Good defaults for quiet operation
        // TOFF=3, HSTRT=4, HEND=1, TBL=2, MRES=4 (16 usteps), INTPOL=1
        uint32_t chopconf = 0;
        chopconf |= (3u  << CHOPCONF_TOFF_SHIFT);     // Off-time = 3
        chopconf |= (4u  << CHOPCONF_HSTRT_SHIFT);    // Hysteresis start = 4
        chopconf |= (1u  << CHOPCONF_HEND_SHIFT);     // Hysteresis end = 1
        chopconf |= (2u  << CHOPCONF_TBL_SHIFT);      // Blank time = 2
        chopconf |= (4u  << CHOPCONF_MRES_SHIFT);     // 16 microsteps
        chopconf |= (1u  << CHOPCONF_INTPOL_SHIFT);   // Interpolation to 256
        tmc2209_write_reg(axis, TMC_REG_CHOPCONF, chopconf);

        // Set default current (800 mA)
        tmc2209_set_current(axis, 800);

        // TPOWERDOWN: Delay before standstill current reduction (~2 seconds)
        tmc2209_write_reg(axis, TMC_REG_TPOWERDOWN, 20);

        // TCOOLTHRS: CoolStep / StallGuard lower velocity threshold (disabled by default)
        tmc2209_write_reg(axis, TMC_REG_TCOOLTHRS, 0);

        // SGTHRS: StallGuard threshold (moderate sensitivity)
        tmc2209_write_reg(axis, TMC_REG_SGTHRS, 100);
    }
}
