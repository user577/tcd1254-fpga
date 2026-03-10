// fpga_spi.h — SPI interface to FPGA vision system

#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <stdint.h>
#include <stdbool.h>

// SPI commands (match spi_peripheral.v)
#define FPGA_CMD_WRITE_REG     0x01
#define FPGA_CMD_READ_REG      0x02
#define FPGA_CMD_READ_SHADOW   0x03
#define FPGA_CMD_READ_FRAME    0x04
#define FPGA_CMD_WRITE_EXPO    0x10
#define FPGA_CMD_WRITE_FLASH   0x11
#define FPGA_CMD_TRIGGER       0x20
#define FPGA_CMD_STATUS        0xFE

// Status register bits
#define FPGA_STATUS_CCD_RUNNING   (1 << 1)
#define FPGA_STATUS_FRAME_READY   (1 << 2)
#define FPGA_STATUS_SHADOW_VALID  (1 << 3)

// No-shadow sentinel
#define NO_SHADOW 0xFFFF

void     fpga_spi_init(void);

// Status
uint8_t  fpga_read_status(void);
bool     fpga_is_running(void);
bool     fpga_frame_ready(void);

// Shadow / position
uint16_t fpga_read_shadow(void);
float    fpga_read_shadow_px(void);  // Returns pixel position, or -1.0 if no shadow

// Exposure config
void     fpga_set_exposure(uint32_t sh_period, uint32_t icg_period,
                           uint8_t mode, uint8_t avg_count);

// Flash config
void     fpga_set_flash(uint8_t lamp_mask, uint16_t delay_us,
                        uint16_t duration_us, uint8_t flags);

// Trigger single capture
void     fpga_trigger_capture(void);

// Read raw frame (2200 x 12-bit pixels into caller's buffer)
void     fpga_read_frame(uint16_t *pixels, uint16_t count);

#endif // FPGA_SPI_H
