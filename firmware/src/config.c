// config.c — Persistent configuration stored in RP2040 flash
//
// Uses the last sector of flash (4 KB) for config storage.
// CRC32 validates integrity on load.

#include "config.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// Flash layout: RP2040 has 2 MB flash, XIP base at 0x10000000.
// Use the last 4 KB sector for config storage.
#define FLASH_CONFIG_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_CONFIG_ADDR   (XIP_BASE + FLASH_CONFIG_OFFSET)

// ---- CRC32 (standard polynomial) ----

static uint32_t crc32(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

// ---- Defaults ----

void config_defaults(system_config_t *cfg) {
    memset(cfg, 0, sizeof(system_config_t));

    cfg->magic   = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;

    // Default axis configs (same for all axes initially)
    for (int i = 0; i < NUM_AXES; i++) {
        cfg->axes[i].steps_per_mm   = 80;
        cfg->axes[i].max_speed      = 5000;
        cfg->axes[i].accel          = 10000;
        cfg->axes[i].home_speed     = 1000;
        cfg->axes[i].travel_mm      = 100.0f;
        cfg->axes[i].invert_dir     = false;
        cfg->axes[i].invert_enable  = true;   // TMC2209 active-low
        cfg->axes[i].tmc_current_ma = 800;
        cfg->axes[i].tmc_microsteps = 16;
    }

    // FPGA exposure defaults
    cfg->default_sh   = 20;
    cfg->default_icg  = 500000;
    cfg->default_mode = 1;       // Position mode (shadow detect)
    cfg->default_avg  = 1;

    // Flash defaults
    cfg->flash_lamp_mask   = 0x01;  // Lamp 0 only
    cfg->flash_delay_us    = 0;
    cfg->flash_duration_us = 500;
    cfg->flash_flags       = 0;

    // Sort defaults
    cfg->sort.min_shadow_width_px  = 5.0f;
    cfg->sort.max_shadow_width_px  = 50.0f;
    cfg->sort.grouped_threshold_px = 80.0f;
    cfg->sort.rescan_attempts      = 3;
    cfg->sort.multi_angle_enabled  = false;

    // Sequence defaults
    cfg->sequence.feed_steps       = 500;
    cfg->sequence.feed_speed       = 2000;
    cfg->sequence.present_steps    = 300;
    cfg->sequence.present_speed    = 1500;
    cfg->sequence.sort_accept_steps = 100;
    cfg->sequence.sort_reject_steps = 200;
    cfg->sequence.sort_speed       = 3000;
    cfg->sequence.singulate_steps  = 150;
    cfg->sequence.singulate_speed  = 1000;
    cfg->sequence.settle_ms        = 50;
    cfg->sequence.capture_icg      = 500000;

    cfg->auto_start = false;
    cfg->auto_home  = false;

    // Compute checksum over all fields except the checksum itself
    cfg->checksum = crc32(cfg, offsetof(system_config_t, checksum));
}

// ---- Load from flash ----

void config_load(system_config_t *cfg) {
    const system_config_t *flash_cfg = (const system_config_t *)FLASH_CONFIG_ADDR;

    // Check magic
    if (flash_cfg->magic != CONFIG_MAGIC) {
        config_defaults(cfg);
        return;
    }

    // Check version
    if (flash_cfg->version != CONFIG_VERSION) {
        config_defaults(cfg);
        return;
    }

    // Verify CRC32
    uint32_t expected_crc = crc32(flash_cfg, offsetof(system_config_t, checksum));
    if (flash_cfg->checksum != expected_crc) {
        config_defaults(cfg);
        return;
    }

    // Valid — copy to RAM
    memcpy(cfg, flash_cfg, sizeof(system_config_t));
}

// ---- Save to flash ----

void config_save(const system_config_t *cfg) {
    // Prepare a copy with updated CRC
    system_config_t save_buf;
    memcpy(&save_buf, cfg, sizeof(system_config_t));
    save_buf.magic   = CONFIG_MAGIC;
    save_buf.version = CONFIG_VERSION;
    save_buf.checksum = crc32(&save_buf, offsetof(system_config_t, checksum));

    // Flash writes must be done with interrupts disabled
    // (flash erase/program uses the XIP bus, which stalls the CPU)
    uint32_t ints = save_and_disable_interrupts();

    // Erase the config sector (4 KB)
    flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE);

    // Write config (must be multiple of FLASH_PAGE_SIZE = 256 bytes)
    // Pad to page boundary
    uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, FLASH_PAGE_SIZE);

    size_t cfg_size = sizeof(system_config_t);
    size_t pages = (cfg_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    for (size_t p = 0; p < pages; p++) {
        memset(page_buf, 0xFF, FLASH_PAGE_SIZE);
        size_t offset = p * FLASH_PAGE_SIZE;
        size_t remain = cfg_size - offset;
        if (remain > FLASH_PAGE_SIZE) remain = FLASH_PAGE_SIZE;
        memcpy(page_buf, (const uint8_t *)&save_buf + offset, remain);
        flash_range_program(FLASH_CONFIG_OFFSET + offset, page_buf, FLASH_PAGE_SIZE);
    }

    restore_interrupts(ints);
}

// ---- Apply config to all subsystems ----

void config_apply(const system_config_t *cfg) {
    // Apply stepper configs
    for (int i = 0; i < NUM_AXES; i++) {
        stepper_set_config(i, &cfg->axes[i]);
    }

    // Apply FPGA exposure settings
    fpga_set_exposure(cfg->default_sh, cfg->default_icg,
                      cfg->default_mode, cfg->default_avg);

    // Apply flash settings
    fpga_set_flash(cfg->flash_lamp_mask, cfg->flash_delay_us,
                   cfg->flash_duration_us, cfg->flash_flags);

    // Apply sort/sequence config
    sort_seq_set_config(&cfg->sort, &cfg->sequence);
}
