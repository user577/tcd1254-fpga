// fpga_spi.c — SPI master interface to FPGA vision system

#include "fpga_spi.h"
#include "pins.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/mutex.h"
#include <string.h>

// ---- SPI bus mutex (serializes access from both cores) ----

static mutex_t spi_mtx;

void fpga_spi_lock(void)   { mutex_enter_blocking(&spi_mtx); }
void fpga_spi_unlock(void) { mutex_exit(&spi_mtx); }

// ---- SPI helpers ----

static inline void cs_select(void) {
    mutex_enter_blocking(&spi_mtx);
    gpio_put(PIN_SPI_CS, 0);
}

static inline void cs_deselect(void) {
    gpio_put(PIN_SPI_CS, 1);
    mutex_exit(&spi_mtx);
}

static void spi_write_byte(uint8_t b) {
    spi_write_blocking(FPGA_SPI_INST, &b, 1);
}

static uint8_t spi_read_byte(void) {
    uint8_t b;
    spi_read_blocking(FPGA_SPI_INST, 0x00, &b, 1);
    return b;
}

// ---- Init ----

void fpga_spi_init(void) {
    // Init SPI bus mutex (must be called before multicore_launch_core1)
    mutex_init(&spi_mtx);

    // Init SPI0 at 10 MHz, Mode 0 (CPOL=0, CPHA=0)
    spi_init(FPGA_SPI_INST, 10 * 1000 * 1000);
    spi_set_format(FPGA_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);

    // CS is manually controlled (not SPI peripheral)
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1);  // Deselected
}

// ---- Status ----

uint8_t fpga_read_status(void) {
    cs_select();
    spi_write_byte(FPGA_CMD_STATUS);
    uint8_t status = spi_read_byte();
    cs_deselect();
    return status;
}

bool fpga_is_running(void) {
    return (fpga_read_status() & FPGA_STATUS_CCD_RUNNING) != 0;
}

bool fpga_frame_ready(void) {
    return (fpga_read_status() & FPGA_STATUS_FRAME_READY) != 0;
}

// ---- Shadow / position ----

uint16_t fpga_read_shadow(void) {
    cs_select();
    spi_write_byte(FPGA_CMD_READ_SHADOW);
    uint8_t hi = spi_read_byte();
    uint8_t lo = spi_read_byte();
    cs_deselect();
    return ((uint16_t)hi << 8) | lo;
}

float fpga_read_shadow_px(void) {
    uint16_t raw = fpga_read_shadow();
    if (raw == NO_SHADOW)
        return -1.0f;
    return (float)raw;
}

// ---- Exposure config ----

void fpga_set_exposure(uint32_t sh_period, uint32_t icg_period,
                       uint8_t mode, uint8_t avg_count) {
    uint8_t buf[10];
    // Big-endian SH period (4 bytes)
    buf[0] = (sh_period >> 24) & 0xFF;
    buf[1] = (sh_period >> 16) & 0xFF;
    buf[2] = (sh_period >>  8) & 0xFF;
    buf[3] = (sh_period >>  0) & 0xFF;
    // Big-endian ICG period (4 bytes)
    buf[4] = (icg_period >> 24) & 0xFF;
    buf[5] = (icg_period >> 16) & 0xFF;
    buf[6] = (icg_period >>  8) & 0xFF;
    buf[7] = (icg_period >>  0) & 0xFF;
    // Mode and avg
    buf[8] = mode;
    buf[9] = avg_count;

    cs_select();
    spi_write_byte(FPGA_CMD_WRITE_EXPO);
    spi_write_blocking(FPGA_SPI_INST, buf, 10);
    cs_deselect();
}

// ---- Flash config ----

void fpga_set_flash(uint8_t lamp_mask, uint16_t delay_us,
                    uint16_t duration_us, uint8_t flags) {
    uint8_t buf[6];
    buf[0] = lamp_mask;
    buf[1] = (delay_us >> 8) & 0xFF;
    buf[2] = (delay_us >> 0) & 0xFF;
    buf[3] = (duration_us >> 8) & 0xFF;
    buf[4] = (duration_us >> 0) & 0xFF;
    buf[5] = flags;

    cs_select();
    spi_write_byte(FPGA_CMD_WRITE_FLASH);
    spi_write_blocking(FPGA_SPI_INST, buf, 6);
    cs_deselect();
}

// ---- Trigger ----

void fpga_trigger_capture(void) {
    cs_select();
    spi_write_byte(FPGA_CMD_TRIGGER);
    cs_deselect();
}

// ---- Raw frame read ----

void fpga_read_frame(uint16_t *pixels, uint16_t count) {
    cs_select();
    spi_write_byte(FPGA_CMD_READ_FRAME);

    for (uint16_t i = 0; i < count; i++) {
        uint8_t hi = spi_read_byte();  // Upper 4 bits (12-bit pixel)
        uint8_t lo = spi_read_byte();  // Lower 8 bits
        pixels[i] = ((uint16_t)(hi & 0x0F) << 8) | lo;
    }

    cs_deselect();
}
