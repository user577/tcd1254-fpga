// config.h — Persistent configuration (stored in RP2040 flash)
//
// Python sends config over USB serial as JSON or binary packets.
// RP2040 stores in flash, loads on boot, runs headless.

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "stepper.h"
#include "sort_sequence.h"

#define CONFIG_MAGIC   0x54434431  // 'TCD1'
#define CONFIG_VERSION 1

// Full system configuration — persisted to flash
typedef struct {
    uint32_t          magic;
    uint32_t          version;

    // Per-axis stepper config
    axis_config_t     axes[NUM_AXES];

    // FPGA exposure defaults
    uint32_t          default_sh;
    uint32_t          default_icg;
    uint8_t           default_mode;
    uint8_t           default_avg;

    // Flash lamp defaults
    uint8_t           flash_lamp_mask;
    uint16_t          flash_delay_us;
    uint16_t          flash_duration_us;
    uint8_t           flash_flags;

    // Sort/classify parameters
    sort_config_t     sort;
    sequence_config_t sequence;

    // System flags
    bool              auto_start;        // Start sorting on power-up
    bool              auto_home;         // Home all axes on power-up

    uint32_t          checksum;          // CRC32 of all preceding fields
} system_config_t;

// Load config from flash (returns defaults if flash empty/corrupt)
void config_load(system_config_t *cfg);

// Save config to flash
void config_save(const system_config_t *cfg);

// Apply loaded config to all subsystems
void config_apply(const system_config_t *cfg);

// Reset to factory defaults
void config_defaults(system_config_t *cfg);

#endif // CONFIG_H
