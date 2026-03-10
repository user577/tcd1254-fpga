// sort_sequence.c — Autonomous sort/singulation state machine
//
// Runs on core1. Continuously cycles through:
//   FEED → PRESENT → CAPTURE → CLASSIFY → SORT → (SINGULATE) → repeat
//
// All timing and threshold parameters come from Python config via USB serial.

#include "sort_sequence.h"
#include "stepper.h"
#include "fpga_spi.h"
#include "pins.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <string.h>

// ---- Internal state ----
static sort_config_t     sort_cfg;
static sequence_config_t seq_cfg;
static sort_stats_t      stats;
static seq_state_t       state;
static bool              run_flag;     // Set by start(), cleared by stop()
static bool              estop_flag;   // Emergency stop

// Multi-angle capture results (populated in SEQ_CAPTURING, consumed in SEQ_CLASSIFYING)
#define MAX_MULTI_ANGLES 8
static float   multi_shadow_px[MAX_MULTI_ANGLES];
static uint8_t multi_shadow_count;

// ---- Init ----

void sort_seq_init(void) {
    memset(&sort_cfg, 0, sizeof(sort_cfg));
    memset(&seq_cfg, 0, sizeof(seq_cfg));
    memset(&stats, 0, sizeof(stats));
    state = SEQ_IDLE;
    run_flag = false;
    estop_flag = false;

    // Defaults
    sort_cfg.min_shadow_width_px  = 5.0f;
    sort_cfg.max_shadow_width_px  = 50.0f;
    sort_cfg.grouped_threshold_px = 80.0f;
    sort_cfg.rescan_attempts      = 3;
    sort_cfg.multi_angle_enabled  = false;
    sort_cfg.flash_lamp_mask      = 0x01;  // Lamp 0 only (single angle)
    sort_cfg.air_jet_ms           = 20;    // 20 ms default pulse

    seq_cfg.feed_steps     = 500;
    seq_cfg.feed_speed     = 2000;
    seq_cfg.present_steps  = 300;
    seq_cfg.present_speed  = 1500;
    seq_cfg.sort_accept_steps = 100;
    seq_cfg.sort_reject_steps = 200;
    seq_cfg.sort_speed     = 3000;
    seq_cfg.singulate_steps = 150;
    seq_cfg.singulate_speed = 1000;
    seq_cfg.settle_ms      = 50;
    seq_cfg.capture_icg    = 500000;

    // Part detect pin (photogate, ADC-capable but we use digital threshold)
    gpio_init(PIN_PART_DETECT);
    gpio_set_dir(PIN_PART_DETECT, GPIO_IN);

    // Bin full sensor (active-low, external pull-up)
    gpio_init(PIN_BIN_FULL);
    gpio_set_dir(PIN_BIN_FULL, GPIO_IN);
    gpio_pull_up(PIN_BIN_FULL);

    // Air jet output
    gpio_init(PIN_AIR_JET_0);
    gpio_set_dir(PIN_AIR_JET_0, GPIO_OUT);
    gpio_put(PIN_AIR_JET_0, 0);
}

// ---- Configuration ----

void sort_seq_set_config(const sort_config_t *s_cfg,
                         const sequence_config_t *sq_cfg) {
    if (s_cfg)  sort_cfg = *s_cfg;
    if (sq_cfg) seq_cfg  = *sq_cfg;
}

// ---- Control ----

void sort_seq_start(void) {
    estop_flag = false;
    run_flag = true;
    stats.total_cycles = 0;
    state = SEQ_FEEDING;
}

void sort_seq_stop(void) {
    run_flag = false;
    // Will finish current cycle, then go to IDLE
}

void sort_seq_estop(void) {
    estop_flag = true;
    run_flag = false;
    stepper_estop_all();
    gpio_put(PIN_AIR_JET_0, 0);
    state = SEQ_IDLE;
}

void sort_seq_home_all(void) {
    state = SEQ_HOMING;
    for (int i = 0; i < NUM_AXES; i++) {
        stepper_enable(i, true);
    }
    // Home each axis sequentially
    stepper_home(AXIS_FEEDER);
    stepper_home(AXIS_CONVEYOR);
    stepper_home(AXIS_SINGULATE);
    stepper_home(AXIS_SORT);
    state = SEQ_IDLE;
}

// ---- Status ----

seq_state_t sort_seq_get_state(void) {
    return state;
}

sort_stats_t sort_seq_get_stats(void) {
    return stats;
}

void sort_seq_reset_stats(void) {
    memset(&stats, 0, sizeof(stats));
}

// ---- Classification ----

static sort_action_t classify_shadow(float shadow_width_px) {
    if (shadow_width_px < sort_cfg.min_shadow_width_px) {
        // Too narrow — noise or no part
        return SORT_REJECT;
    }
    if (shadow_width_px <= sort_cfg.max_shadow_width_px) {
        // Within single-part range
        return SORT_ACCEPT;
    }
    if (shadow_width_px > sort_cfg.grouped_threshold_px) {
        // Definitely grouped
        return SORT_RESCAN;
    }
    // Between max_single and grouped_threshold — ambiguous, reject
    return SORT_REJECT;
}

// Wait for all motion to complete (poll stepper busy flags)
static void wait_motion_complete(void) {
    while (stepper_any_busy()) {
        if (estop_flag) return;
        sleep_us(100);
    }
}

// Wait for vibration to settle
static void wait_settle(void) {
    sleep_ms(seq_cfg.settle_ms);
}

// Check part-present photogate
static bool part_detected(void) {
    // Photogate is active-low (beam broken = part present)
    return !gpio_get(PIN_PART_DETECT);
}

// Check reject bin full sensor (active-low: low = bin full)
static bool bin_full(void) {
    return !gpio_get(PIN_BIN_FULL);
}

// Fire air jet for configured pulse duration
static void fire_air_jet(void) {
    if (sort_cfg.air_jet_ms == 0) return;
    gpio_put(PIN_AIR_JET_0, 1);
    sleep_ms(sort_cfg.air_jet_ms);
    gpio_put(PIN_AIR_JET_0, 0);
}

// Count set bits in a byte (number of lamps enabled)
static uint8_t popcount8(uint8_t x) {
    uint8_t count = 0;
    while (x) { count += x & 1; x >>= 1; }
    return count;
}

// ---- Main loop (runs on core1) ----

void sort_seq_run(void) {
    uint8_t rescan_count = 0;

    while (true) {
        if (estop_flag) {
            state = SEQ_IDLE;
            sleep_ms(10);
            continue;
        }

        switch (state) {
            case SEQ_IDLE:
                sleep_ms(10);
                break;

            case SEQ_FEEDING:
                // Check bin-full sensor before starting cycle
                if (bin_full()) {
                    stats.bin_full_pauses++;
                    state = SEQ_ERROR;
                    break;
                }

                // Advance feeder to bring next part to inspection zone
                stepper_enable(AXIS_FEEDER, true);
                stepper_move_steps(AXIS_FEEDER,
                                   (int32_t)seq_cfg.feed_steps,
                                   seq_cfg.feed_speed);
                wait_motion_complete();
                if (estop_flag) break;

                // Check if part arrived
                if (!part_detected()) {
                    stats.no_part_cycles++;
                    stats.total_cycles++;
                    if (run_flag) {
                        state = SEQ_FEEDING;  // Try again
                    } else {
                        state = SEQ_IDLE;
                    }
                    break;
                }

                state = SEQ_PRESENTING;
                break;

            case SEQ_PRESENTING:
                // Move part to CCD inspection position
                stepper_enable(AXIS_CONVEYOR, true);
                stepper_move_steps(AXIS_CONVEYOR,
                                   (int32_t)seq_cfg.present_steps,
                                   seq_cfg.present_speed);
                wait_motion_complete();
                if (estop_flag) break;

                wait_settle();
                state = SEQ_CAPTURING;
                break;

            case SEQ_CAPTURING:
                if (sort_cfg.multi_angle_enabled) {
                    // Multi-angle capture: one frame per enabled lamp
                    uint8_t mask = sort_cfg.flash_lamp_mask;
                    uint8_t n_lamps = popcount8(mask);
                    if (n_lamps == 0) n_lamps = 1;

                    multi_shadow_count = 0;

                    for (uint8_t lamp = 0; lamp < 8 && multi_shadow_count < n_lamps; lamp++) {
                        if (!(mask & (1 << lamp))) continue;

                        // Set flash to fire only this lamp
                        fpga_set_flash((uint8_t)(1 << lamp), 0, 0,
                                       0);  // Use system defaults for delay/dur
                        fpga_trigger_capture();

                        // Poll for frame ready (with timeout)
                        uint32_t timeout = 500000;
                        uint32_t start = time_us_32();
                        while (!fpga_frame_ready()) {
                            if (estop_flag) break;
                            if ((time_us_32() - start) > timeout) {
                                state = SEQ_ERROR;
                                break;
                            }
                            sleep_us(100);
                        }
                        if (estop_flag || state == SEQ_ERROR) break;

                        float px = fpga_read_shadow_px();
                        if (multi_shadow_count < MAX_MULTI_ANGLES) {
                            multi_shadow_px[multi_shadow_count++] = px;
                        }
                    }

                    // Restore original lamp mask
                    fpga_set_flash(mask, 0, 0, 0);

                    if (estop_flag || state == SEQ_ERROR) break;
                } else {
                    // Single capture
                    fpga_trigger_capture();

                    // Poll for frame ready (with timeout)
                    uint32_t timeout = 500000;
                    uint32_t start = time_us_32();
                    while (!fpga_frame_ready()) {
                        if (estop_flag) break;
                        if ((time_us_32() - start) > timeout) {
                            state = SEQ_ERROR;
                            break;
                        }
                        sleep_us(100);
                    }
                    if (estop_flag || state == SEQ_ERROR) break;

                    multi_shadow_count = 0;  // Signal single-capture path
                }

                state = SEQ_CLASSIFYING;
                break;

            case SEQ_CLASSIFYING: {
                float shadow_px;

                if (sort_cfg.multi_angle_enabled && multi_shadow_count > 0) {
                    // Multi-angle: take minimum shadow width across angles
                    // (smallest width is the most conservative single-part estimate)
                    float min_px = multi_shadow_px[0];
                    for (uint8_t i = 1; i < multi_shadow_count; i++) {
                        if (multi_shadow_px[i] >= 0 && (min_px < 0 || multi_shadow_px[i] < min_px)) {
                            min_px = multi_shadow_px[i];
                        }
                    }
                    shadow_px = min_px;
                } else {
                    // Single capture: read shadow result from FPGA
                    shadow_px = fpga_read_shadow_px();
                }
                stats.parts_inspected++;

                sort_action_t action;
                if (shadow_px < 0) {
                    // No shadow detected
                    action = SORT_REJECT;
                } else {
                    action = classify_shadow(shadow_px);
                }

                state = SEQ_SORTING;

                // Route to appropriate bin
                stepper_enable(AXIS_SORT, true);

                switch (action) {
                    case SORT_ACCEPT:
                        stepper_move_steps(AXIS_SORT,
                                           (int32_t)seq_cfg.sort_accept_steps,
                                           seq_cfg.sort_speed);
                        stats.parts_accepted++;
                        rescan_count = 0;
                        break;

                    case SORT_REJECT:
                        stepper_move_steps(AXIS_SORT,
                                           (int32_t)seq_cfg.sort_reject_steps,
                                           seq_cfg.sort_speed);
                        fire_air_jet();  // Assist part diversion
                        stats.parts_rejected++;
                        rescan_count = 0;
                        break;

                    case SORT_RESCAN:
                        if (rescan_count < sort_cfg.rescan_attempts) {
                            // Singulate and re-inspect
                            state = SEQ_SINGULATING;
                            rescan_count++;
                            stats.parts_rescanned++;
                        } else {
                            // Too many retries — reject
                            stepper_move_steps(AXIS_SORT,
                                               (int32_t)seq_cfg.sort_reject_steps,
                                               seq_cfg.sort_speed);
                            stats.parts_rejected++;
                            rescan_count = 0;
                        }
                        break;
                }
                break;
            }

            case SEQ_SORTING:
                wait_motion_complete();
                if (estop_flag) break;

                // Return sort gate to home
                stepper_move_steps(AXIS_SORT,
                                   -(int32_t)seq_cfg.sort_accept_steps,
                                   seq_cfg.sort_speed);
                wait_motion_complete();

                stats.total_cycles++;
                if (run_flag) {
                    state = SEQ_FEEDING;
                } else {
                    state = SEQ_IDLE;
                }
                break;

            case SEQ_SINGULATING:
                // Separate grouped parts using singulation axis
                stepper_enable(AXIS_SINGULATE, true);
                stepper_move_steps(AXIS_SINGULATE,
                                   (int32_t)seq_cfg.singulate_steps,
                                   seq_cfg.singulate_speed);
                fire_air_jet();  // Assist part separation
                wait_motion_complete();
                if (estop_flag) break;

                // Return singulation axis
                stepper_move_steps(AXIS_SINGULATE,
                                   -(int32_t)seq_cfg.singulate_steps,
                                   seq_cfg.singulate_speed);
                wait_motion_complete();
                if (estop_flag) break;

                wait_settle();

                // Re-inspect
                state = SEQ_CAPTURING;
                break;

            case SEQ_HOMING:
                // Handled by sort_seq_home_all() synchronously
                sleep_ms(10);
                break;

            case SEQ_ERROR:
                // Stay in error until reset
                run_flag = false;
                sleep_ms(100);
                break;
        }
    }
}
