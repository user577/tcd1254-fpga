// sort_sequence.h — Autonomous sort/singulation state machine
//
// Runs the feed → capture → classify → sort loop headless.
// Python configures the parameters, then the RP2040 runs independently.
//
// Sequence:
//   1. Feeder advances part to inspection zone (photogate trigger)
//   2. Conveyor positions part under CCD line sensor
//   3. FPGA captures frame, runs shadow detection
//   4. RP2040 reads shadow result, classifies (single vs grouped)
//   5. Sort diverter routes part to accept/reject/re-singulate bin
//   6. If grouped: singulation axis separates, re-inspect
//   7. Repeat

#ifndef SORT_SEQUENCE_H
#define SORT_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

// Sort result — what to do with a detected part
typedef enum {
    SORT_ACCEPT    = 0,  // Single part — route to accept bin
    SORT_REJECT    = 1,  // Grouped/unknown — route to reject bin
    SORT_RESCAN    = 2,  // Try singulation, then re-inspect
} sort_action_t;

// Classification thresholds (set from Python config)
typedef struct {
    float    min_shadow_width_px;     // Minimum valid part shadow
    float    max_shadow_width_px;     // Maximum for single part
    float    grouped_threshold_px;    // Width above this = grouped
    uint8_t  rescan_attempts;         // Max singulation retries before reject
    bool     multi_angle_enabled;     // Use flash array for multi-angle inspection
    uint8_t  flash_lamp_mask;         // Bitmask of lamps for multi-angle capture
    uint32_t air_jet_ms;              // Air jet pulse duration in ms (0 = disabled)
} sort_config_t;

// Sequence timing (set from Python config)
typedef struct {
    uint32_t feed_steps;              // Steps to advance feeder per cycle
    uint32_t feed_speed;              // Feeder speed (steps/sec)
    uint32_t present_steps;           // Steps to move part to CCD
    uint32_t present_speed;           // Presentation speed
    uint32_t sort_accept_steps;       // Sort gate position for accept
    uint32_t sort_reject_steps;       // Sort gate position for reject
    uint32_t sort_speed;              // Sort gate speed
    uint32_t singulate_steps;         // Singulation travel
    uint32_t singulate_speed;         // Singulation speed
    uint32_t settle_ms;              // Wait time after positioning (vibration settle)
    uint32_t capture_icg;            // CCD integration time (fM cycles)
} sequence_config_t;

// Sort statistics
typedef struct {
    uint32_t parts_inspected;
    uint32_t parts_accepted;
    uint32_t parts_rejected;
    uint32_t parts_rescanned;
    uint32_t no_part_cycles;          // Empty cycles (no part detected)
    uint32_t total_cycles;
    uint32_t bin_full_pauses;         // Times paused due to reject bin full
} sort_stats_t;

// Sequence state (readable for status reporting)
typedef enum {
    SEQ_IDLE         = 0,
    SEQ_FEEDING      = 1,
    SEQ_PRESENTING   = 2,
    SEQ_CAPTURING    = 3,
    SEQ_CLASSIFYING  = 4,
    SEQ_SORTING      = 5,
    SEQ_SINGULATING  = 6,
    SEQ_HOMING       = 7,
    SEQ_ERROR        = 8,
} seq_state_t;

// Initialize the sort sequence engine
void sort_seq_init(void);

// Configuration (called from Python config handler)
void sort_seq_set_config(const sort_config_t *sort_cfg,
                         const sequence_config_t *seq_cfg);

// Control
void sort_seq_start(void);            // Begin autonomous sorting
void sort_seq_stop(void);             // Stop after current cycle
void sort_seq_estop(void);            // Emergency stop immediately
void sort_seq_home_all(void);         // Home all axes

// Status
seq_state_t    sort_seq_get_state(void);
sort_stats_t   sort_seq_get_stats(void);
void           sort_seq_reset_stats(void);

// Main loop — call from core1 (runs continuously)
void sort_seq_run(void);

#endif // SORT_SEQUENCE_H
