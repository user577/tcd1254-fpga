// main.c — RP2040 system controller for TCD1254 sort/inspect system
//
// Core 0: USB serial config interface (Python sends JSON/binary config)
// Core 1: Autonomous sort sequence state machine
//
// Boot sequence:
//   1. Init all peripherals (SPI, PIO steppers, GPIO)
//   2. Load config from flash (or defaults if empty/corrupt)
//   3. Apply config to FPGA + steppers
//   4. If auto_home: home all axes
//   5. If auto_start: begin sorting
//   6. Core 1 runs sort_seq_run() forever
//   7. Core 0 listens for USB serial commands

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include "pins.h"
#include "fpga_spi.h"
#include "stepper.h"
#include "sort_sequence.h"
#include "config.h"

// ---- System config (shared, core0 writes / core1 reads) ----
static system_config_t sys_config;

// ---- Frame download / streaming ----
static uint16_t frame_buf[2200];
static bool streaming = false;

// ---- USB serial command protocol ----
//
// Simple line-based text protocol for configuration:
//   CMD [args...]\n
//
// Commands:
//   PING              → responds "PONG\n"
//   STATUS            → responds with JSON status
//   CONFIG_GET        → responds with JSON config dump
//   CONFIG_SET key=val → set a config parameter
//   CONFIG_SAVE       → persist current config to flash
//   CONFIG_RESET      → reset to factory defaults
//   START             → begin autonomous sorting
//   STOP              → stop after current cycle
//   ESTOP             → emergency stop
//   HOME              → home all axes
//   HOME n            → home axis n
//   MOVE n steps spd  → move axis n by steps at speed
//   JOG n dir spd     → jog axis n in direction at speed
//   HALT n            → stop axis n
//   ENABLE n 0|1      → enable/disable axis n driver
//   EXPO sh icg m a   → set FPGA exposure
//   FLASH m d dur f   → set FPGA flash config
//   TRIGGER           → trigger single capture
//   SHADOW            → read shadow result
//   FRAME             → capture + binary frame download (4400 bytes)
//   STREAM_START      → start continuous frame streaming
//   STREAM_STOP       → stop continuous frame streaming
//   STATS             → get sort statistics
//   STATS_RESET       → reset sort statistics

#define CMD_BUF_SIZE 256
static char cmd_buf[CMD_BUF_SIZE];
static int  cmd_pos = 0;

// Parse an integer from a string, advancing the pointer
static int32_t parse_int(char **p) {
    while (**p == ' ') (*p)++;
    int32_t val = strtol(*p, p, 10);
    return val;
}

static uint32_t parse_uint(char **p) {
    while (**p == ' ') (*p)++;
    uint32_t val = strtoul(*p, p, 10);
    return val;
}

static float parse_float(char **p) {
    while (**p == ' ') (*p)++;
    float val = strtof(*p, p);
    return val;
}

// Send JSON status response
static void cmd_status(void) {
    seq_state_t st = sort_seq_get_state();
    const char *state_names[] = {
        "idle", "feeding", "presenting", "capturing",
        "classifying", "sorting", "singulating", "homing", "error"
    };
    const char *sn = (st <= SEQ_ERROR) ? state_names[st] : "unknown";

    printf("{\"state\":\"%s\"", sn);
    printf(",\"fpga_running\":%s", fpga_is_running() ? "true" : "false");

    // Axis status
    printf(",\"axes\":[");
    for (int i = 0; i < NUM_AXES; i++) {
        axis_status_t as = stepper_get_status(i);
        if (i > 0) printf(",");
        printf("{\"pos\":%ld,\"busy\":%s,\"homed\":%s,\"home_sw\":%s,\"en\":%s}",
               (long)as.position,
               as.busy ? "true" : "false",
               as.homed ? "true" : "false",
               as.home_switch ? "true" : "false",
               as.enabled ? "true" : "false");
    }
    printf("]}\n");
}

// Send sort stats
static void cmd_stats(void) {
    sort_stats_t s = sort_seq_get_stats();
    printf("{\"inspected\":%lu,\"accepted\":%lu,\"rejected\":%lu,"
           "\"rescanned\":%lu,\"no_part\":%lu,\"cycles\":%lu}\n",
           (unsigned long)s.parts_inspected,
           (unsigned long)s.parts_accepted,
           (unsigned long)s.parts_rejected,
           (unsigned long)s.parts_rescanned,
           (unsigned long)s.no_part_cycles,
           (unsigned long)s.total_cycles);
}

// Process a complete command line
static void process_command(char *line) {
    // Trim trailing whitespace
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) return;

    char *p = line;

    if (strncmp(p, "PING", 4) == 0) {
        printf("PONG\n");

    } else if (strncmp(p, "STATUS", 6) == 0) {
        cmd_status();

    } else if (strncmp(p, "CONFIG_RESET", 12) == 0) {
        config_defaults(&sys_config);
        config_apply(&sys_config);
        printf("OK defaults\n");

    } else if (strncmp(p, "CONFIG_SAVE", 11) == 0) {
        config_save(&sys_config);
        printf("OK saved\n");

    } else if (strncmp(p, "CONFIG_GET", 10) == 0) {
        // Dump key config values as JSON
        printf("{\"magic\":\"0x%08lX\",\"version\":%lu",
               (unsigned long)sys_config.magic,
               (unsigned long)sys_config.version);
        printf(",\"default_sh\":%lu,\"default_icg\":%lu",
               (unsigned long)sys_config.default_sh,
               (unsigned long)sys_config.default_icg);
        printf(",\"default_mode\":%u,\"default_avg\":%u",
               sys_config.default_mode, sys_config.default_avg);
        printf(",\"auto_start\":%s,\"auto_home\":%s",
               sys_config.auto_start ? "true" : "false",
               sys_config.auto_home ? "true" : "false");

        // Flash lamp
        printf(",\"flash_lamp_mask\":%u,\"flash_delay_us\":%u",
               sys_config.flash_lamp_mask,
               sys_config.flash_delay_us);
        printf(",\"flash_duration_us\":%u,\"flash_flags\":%u",
               sys_config.flash_duration_us,
               sys_config.flash_flags);

        // Sort config
        printf(",\"sort\":{\"min_w\":%.1f,\"max_w\":%.1f,\"grouped\":%.1f,\"rescan\":%u,\"multi_angle\":%s}",
               (double)sys_config.sort.min_shadow_width_px,
               (double)sys_config.sort.max_shadow_width_px,
               (double)sys_config.sort.grouped_threshold_px,
               sys_config.sort.rescan_attempts,
               sys_config.sort.multi_angle_enabled ? "true" : "false");

        // Sequence config
        printf(",\"seq\":{\"feed_steps\":%lu,\"feed_speed\":%lu",
               (unsigned long)sys_config.sequence.feed_steps,
               (unsigned long)sys_config.sequence.feed_speed);
        printf(",\"present_steps\":%lu,\"present_speed\":%lu",
               (unsigned long)sys_config.sequence.present_steps,
               (unsigned long)sys_config.sequence.present_speed);
        printf(",\"sort_accept_steps\":%lu,\"sort_reject_steps\":%lu,\"sort_speed\":%lu",
               (unsigned long)sys_config.sequence.sort_accept_steps,
               (unsigned long)sys_config.sequence.sort_reject_steps,
               (unsigned long)sys_config.sequence.sort_speed);
        printf(",\"singulate_steps\":%lu,\"singulate_speed\":%lu",
               (unsigned long)sys_config.sequence.singulate_steps,
               (unsigned long)sys_config.sequence.singulate_speed);
        printf(",\"settle_ms\":%lu,\"capture_icg\":%lu}",
               (unsigned long)sys_config.sequence.settle_ms,
               (unsigned long)sys_config.sequence.capture_icg);

        // Per-axis config summary
        printf(",\"axes\":[");
        for (int i = 0; i < NUM_AXES; i++) {
            const axis_config_t *ac = &sys_config.axes[i];
            if (i > 0) printf(",");
            printf("{\"steps_per_mm\":%lu,\"max_speed\":%lu,\"accel\":%lu",
                   (unsigned long)ac->steps_per_mm,
                   (unsigned long)ac->max_speed,
                   (unsigned long)ac->accel);
            printf(",\"home_speed\":%lu,\"travel_mm\":%.2f",
                   (unsigned long)ac->home_speed,
                   (double)ac->travel_mm);
            printf(",\"invert_dir\":%s,\"invert_enable\":%s",
                   ac->invert_dir ? "true" : "false",
                   ac->invert_enable ? "true" : "false");
            printf(",\"tmc_current_ma\":%u,\"tmc_microsteps\":%u}",
                   ac->tmc_current_ma,
                   ac->tmc_microsteps);
        }
        printf("]}\n");

    } else if (strncmp(p, "CONFIG_SET ", 11) == 0) {
        p += 11;
        // Parse key=value pairs
        char *eq = strchr(p, '=');
        if (!eq) { printf("ERR syntax\n"); return; }
        *eq = '\0';
        char *key = p;
        char *val = eq + 1;

        // ---- Per-axis config: axis.N.field ----
        if (strncmp(key, "axis.", 5) == 0 && key[6] == '.') {
            int ax = key[5] - '0';
            if (ax < 0 || ax >= NUM_AXES) {
                printf("ERR axis index: %d\n", ax);
                return;
            }
            const char *field = key + 7;
            axis_config_t *ac = &sys_config.axes[ax];

            if (strcmp(field, "steps_per_mm") == 0) {
                ac->steps_per_mm = strtoul(val, NULL, 10);
            } else if (strcmp(field, "max_speed") == 0) {
                ac->max_speed = strtoul(val, NULL, 10);
            } else if (strcmp(field, "accel") == 0) {
                ac->accel = strtoul(val, NULL, 10);
            } else if (strcmp(field, "home_speed") == 0) {
                ac->home_speed = strtoul(val, NULL, 10);
            } else if (strcmp(field, "travel_mm") == 0) {
                ac->travel_mm = strtof(val, NULL);
            } else if (strcmp(field, "invert_dir") == 0) {
                ac->invert_dir = (atoi(val) != 0);
            } else if (strcmp(field, "invert_enable") == 0) {
                ac->invert_enable = (atoi(val) != 0);
            } else if (strcmp(field, "tmc_current_ma") == 0) {
                ac->tmc_current_ma = (uint16_t)strtoul(val, NULL, 10);
            } else if (strcmp(field, "tmc_microsteps") == 0) {
                ac->tmc_microsteps = (uint8_t)strtoul(val, NULL, 10);
            } else {
                printf("ERR unknown axis field: %s\n", field);
                return;
            }

        // ---- System flags ----
        } else if (strcmp(key, "auto_start") == 0) {
            sys_config.auto_start = (atoi(val) != 0);
        } else if (strcmp(key, "auto_home") == 0) {
            sys_config.auto_home = (atoi(val) != 0);

        // ---- FPGA exposure defaults ----
        } else if (strcmp(key, "default_sh") == 0) {
            sys_config.default_sh = strtoul(val, NULL, 10);
        } else if (strcmp(key, "default_icg") == 0) {
            sys_config.default_icg = strtoul(val, NULL, 10);
        } else if (strcmp(key, "default_mode") == 0) {
            sys_config.default_mode = (uint8_t)strtoul(val, NULL, 10);
        } else if (strcmp(key, "default_avg") == 0) {
            sys_config.default_avg = (uint8_t)strtoul(val, NULL, 10);

        // ---- Flash lamp ----
        } else if (strcmp(key, "flash_lamp_mask") == 0) {
            sys_config.flash_lamp_mask = (uint8_t)strtoul(val, NULL, 10);
        } else if (strcmp(key, "flash_delay_us") == 0) {
            sys_config.flash_delay_us = (uint16_t)strtoul(val, NULL, 10);
        } else if (strcmp(key, "flash_duration_us") == 0) {
            sys_config.flash_duration_us = (uint16_t)strtoul(val, NULL, 10);
        } else if (strcmp(key, "flash_flags") == 0) {
            sys_config.flash_flags = (uint8_t)strtoul(val, NULL, 10);

        // ---- Sort config ----
        } else if (strcmp(key, "sort.min_w") == 0) {
            sys_config.sort.min_shadow_width_px = strtof(val, NULL);
        } else if (strcmp(key, "sort.max_w") == 0) {
            sys_config.sort.max_shadow_width_px = strtof(val, NULL);
        } else if (strcmp(key, "sort.grouped") == 0) {
            sys_config.sort.grouped_threshold_px = strtof(val, NULL);
        } else if (strcmp(key, "sort.rescan") == 0) {
            sys_config.sort.rescan_attempts = atoi(val);
        } else if (strcmp(key, "sort.multi_angle") == 0) {
            sys_config.sort.multi_angle_enabled = (atoi(val) != 0);

        // ---- Sequence timing ----
        } else if (strcmp(key, "seq.feed_steps") == 0) {
            sys_config.sequence.feed_steps = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.feed_speed") == 0) {
            sys_config.sequence.feed_speed = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.present_steps") == 0) {
            sys_config.sequence.present_steps = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.present_speed") == 0) {
            sys_config.sequence.present_speed = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.sort_accept_steps") == 0) {
            sys_config.sequence.sort_accept_steps = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.sort_reject_steps") == 0) {
            sys_config.sequence.sort_reject_steps = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.sort_speed") == 0) {
            sys_config.sequence.sort_speed = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.singulate_steps") == 0) {
            sys_config.sequence.singulate_steps = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.singulate_speed") == 0) {
            sys_config.sequence.singulate_speed = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.settle_ms") == 0) {
            sys_config.sequence.settle_ms = strtoul(val, NULL, 10);
        } else if (strcmp(key, "seq.capture_icg") == 0) {
            sys_config.sequence.capture_icg = strtoul(val, NULL, 10);

        } else {
            printf("ERR unknown key: %s\n", key);
            return;
        }
        printf("OK %s=%s\n", key, val);

    } else if (strncmp(p, "START", 5) == 0) {
        config_apply(&sys_config);
        sort_seq_start();
        printf("OK running\n");

    } else if (strncmp(p, "STOP", 4) == 0) {
        sort_seq_stop();
        printf("OK stopping\n");

    } else if (strncmp(p, "ESTOP", 5) == 0) {
        sort_seq_estop();
        printf("OK estop\n");

    } else if (strncmp(p, "HOME", 4) == 0) {
        p += 4;
        if (*p == ' ') {
            int axis = parse_int(&p);
            if (axis >= 0 && axis < NUM_AXES) {
                stepper_enable(axis, true);
                stepper_home(axis);
                printf("OK homed %d\n", axis);
            } else {
                printf("ERR axis\n");
            }
        } else {
            sort_seq_home_all();
            printf("OK homed all\n");
        }

    } else if (strncmp(p, "MOVE ", 5) == 0) {
        p += 5;
        int axis = parse_int(&p);
        int32_t steps = parse_int(&p);
        uint32_t speed = parse_uint(&p);
        if (axis >= 0 && axis < NUM_AXES && speed > 0) {
            stepper_enable(axis, true);
            stepper_move_steps(axis, steps, speed);
            printf("OK move %d %ld %lu\n", axis, (long)steps, (unsigned long)speed);
        } else {
            printf("ERR args\n");
        }

    } else if (strncmp(p, "JOG ", 4) == 0) {
        p += 4;
        int axis = parse_int(&p);
        int dir = parse_int(&p);
        uint32_t speed = parse_uint(&p);
        if (axis >= 0 && axis < NUM_AXES && speed > 0) {
            stepper_enable(axis, true);
            stepper_jog(axis, dir != 0, speed);
            printf("OK jog\n");
        } else {
            printf("ERR args\n");
        }

    } else if (strncmp(p, "HALT ", 5) == 0) {
        p += 5;
        int axis = parse_int(&p);
        if (axis >= 0 && axis < NUM_AXES) {
            stepper_stop(axis);
            printf("OK halt %d\n", axis);
        } else {
            printf("ERR axis\n");
        }

    } else if (strncmp(p, "ENABLE ", 7) == 0) {
        p += 7;
        int axis = parse_int(&p);
        int en = parse_int(&p);
        if (axis >= 0 && axis < NUM_AXES) {
            stepper_enable(axis, en != 0);
            printf("OK enable %d %d\n", axis, en);
        } else {
            printf("ERR axis\n");
        }

    } else if (strncmp(p, "EXPO ", 5) == 0) {
        p += 5;
        uint32_t sh  = parse_uint(&p);
        uint32_t icg = parse_uint(&p);
        uint32_t m   = parse_uint(&p);
        uint32_t a   = parse_uint(&p);
        sys_config.default_sh   = sh;
        sys_config.default_icg  = icg;
        sys_config.default_mode = (uint8_t)m;
        sys_config.default_avg  = (uint8_t)a;
        fpga_set_exposure(sh, icg, (uint8_t)m, (uint8_t)a);
        printf("OK expo\n");

    } else if (strncmp(p, "FLASH ", 6) == 0) {
        p += 6;
        uint32_t mask = parse_uint(&p);
        uint32_t delay = parse_uint(&p);
        uint32_t dur = parse_uint(&p);
        uint32_t flags = parse_uint(&p);
        sys_config.flash_lamp_mask   = (uint8_t)mask;
        sys_config.flash_delay_us    = (uint16_t)delay;
        sys_config.flash_duration_us = (uint16_t)dur;
        sys_config.flash_flags       = (uint8_t)flags;
        fpga_set_flash((uint8_t)mask, (uint16_t)delay, (uint16_t)dur, (uint8_t)flags);
        printf("OK flash\n");

    } else if (strncmp(p, "TRIGGER", 7) == 0) {
        fpga_trigger_capture();
        printf("OK trigger\n");

    } else if (strncmp(p, "SHADOW", 6) == 0) {
        float px = fpga_read_shadow_px();
        printf("{\"shadow_px\":%.1f}\n", (double)px);

    } else if (strncmp(p, "FRAME", 5) == 0) {
        // Single frame capture + binary download
        fpga_trigger_capture();

        // Poll for frame ready with 500ms timeout
        absolute_time_t deadline = make_timeout_time_ms(500);
        bool ready = false;
        while (!time_reached(deadline)) {
            if (fpga_frame_ready()) {
                ready = true;
                break;
            }
            sleep_us(100);
        }

        if (!ready) {
            printf("ERR frame timeout\n");
        } else {
            fpga_read_frame(frame_buf, 2200);

            // Flush any pending text output before binary transfer
            fflush(stdout);

            // Send 4-byte header: sync (0xAA 0x55) + length 4400 big-endian (0x11 0x30)
            uint8_t header[4] = {0xAA, 0x55, 0x11, 0x30};
            fwrite(header, 1, 4, stdout);
            fwrite(frame_buf, 1, 4400, stdout);
            fflush(stdout);
        }

    } else if (strncmp(p, "STREAM_START", 12) == 0) {
        streaming = true;
        printf("OK streaming\n");

    } else if (strncmp(p, "STREAM_STOP", 11) == 0) {
        streaming = false;
        printf("OK stopped\n");

    } else if (strncmp(p, "STATS_RESET", 11) == 0) {
        sort_seq_reset_stats();
        printf("OK stats reset\n");

    } else if (strncmp(p, "STATS", 5) == 0) {
        cmd_stats();

    } else {
        printf("ERR unknown: %s\n", line);
    }
}

// ---- Core 1 entry point ----

static void core1_entry(void) {
    sort_seq_run();  // Never returns
}

// ---- LED status ----

static void led_init(void) {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
}

static void led_update(void) {
    static uint32_t last_toggle = 0;
    uint32_t now = time_us_32();

    seq_state_t st = sort_seq_get_state();

    uint32_t period_us;
    switch (st) {
        case SEQ_IDLE:    period_us = 1000000; break;  // 1 Hz slow blink
        case SEQ_ERROR:   period_us = 100000;  break;  // 10 Hz fast blink
        case SEQ_HOMING:  period_us = 250000;  break;  // 4 Hz
        default:          period_us = 500000;  break;  // 2 Hz when running
    }

    if ((now - last_toggle) >= period_us) {
        gpio_xor_mask(1u << PIN_LED);
        last_toggle = now;
    }
}

// ---- Main ----

int main(void) {
    // Init stdio (USB serial)
    stdio_init_all();

    // LED
    led_init();

    // Init subsystems
    fpga_spi_init();
    stepper_init();
    sort_seq_init();

    // Load config from flash
    config_load(&sys_config);

    // Apply config to hardware
    config_apply(&sys_config);

    // Auto-home if configured
    if (sys_config.auto_home) {
        sort_seq_home_all();
    }

    // Launch core 1 for sort sequence
    multicore_launch_core1(core1_entry);

    // Auto-start if configured
    if (sys_config.auto_start) {
        sort_seq_start();
    }

    printf("TCD1254 controller ready\n");

    // Core 0: USB serial command loop
    while (true) {
        int c = getchar_timeout_us(1000);  // 1ms timeout
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                if (cmd_pos > 0) {
                    cmd_buf[cmd_pos] = '\0';
                    process_command(cmd_buf);
                    cmd_pos = 0;
                }
            } else if (cmd_pos < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_pos++] = (char)c;
            }
        }

        led_update();

        // Continuous streaming mode
        if (streaming) {
            bool ready = fpga_frame_ready();
            if (ready) {
                fpga_read_frame(frame_buf, 2200);

                fflush(stdout);
                uint8_t hdr[4] = {0xAA, 0x55, 0x11, 0x30};
                fwrite(hdr, 1, 4, stdout);
                fwrite(frame_buf, 1, 4400, stdout);
                fflush(stdout);
            }
        }
    }

    return 0;
}
