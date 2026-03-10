// stepper.c — PIO-based stepper motor control for 4 axes
//
// Uses RP2040 PIO state machines for jitter-free step pulse generation.
// Trapezoidal velocity profile computed in software, executed by PIO.
// Each axis gets one PIO SM.

#include "stepper.h"
#include "tmc2209.h"
#include "pins.h"
#include "stepper.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <string.h>
#include <math.h>

// ---- Internal state per axis ----
typedef struct {
    axis_config_t config;
    axis_status_t status;
    PIO           pio;
    uint          sm;
    bool          homing;
    int32_t       target_steps;
} axis_state_t;

static axis_state_t axes[NUM_AXES];
static uint pio_offset;

// PIO clock runs at 1 MHz (1 us per cycle)
// Step period in PIO cycles = 1,000,000 / speed_steps_per_sec
#define PIO_CLK_HZ 1000000

// Minimum step pulse high time (us) — most drivers need >= 1 us
#define MIN_PULSE_US 2

// ---- Init ----

void stepper_init(void) {
    memset(axes, 0, sizeof(axes));

    // Load PIO program (use pio0 for axes 0-3, it has 4 SMs)
    PIO pio = pio0;
    pio_offset = pio_add_program(pio, &stepper_program);

    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].pio = pio;
        axes[i].sm  = i;

        // Init STEP pin via PIO
        stepper_pio_program_init(pio, i, pio_offset, STEP_PINS[i]);

        // Init DIR pin
        gpio_init(DIR_PINS[i]);
        gpio_set_dir(DIR_PINS[i], GPIO_OUT);
        gpio_put(DIR_PINS[i], 0);

        // Init EN pin (default disabled)
        gpio_init(EN_PINS[i]);
        gpio_set_dir(EN_PINS[i], GPIO_OUT);
        gpio_put(EN_PINS[i], 1);  // TMC2209: EN active-low, so 1 = disabled

        // Init HOME switch (input with pull-up)
        gpio_init(HOME_PINS[i]);
        gpio_set_dir(HOME_PINS[i], GPIO_IN);
        gpio_pull_up(HOME_PINS[i]);

        // Default config
        axes[i].config.steps_per_mm  = 80;     // Typical for GT2 belt + 16 microsteps
        axes[i].config.max_speed     = 5000;    // Steps/sec
        axes[i].config.accel         = 10000;   // Steps/sec²
        axes[i].config.home_speed    = 1000;    // Steps/sec
        axes[i].config.travel_mm     = 100.0f;
        axes[i].config.invert_dir    = false;
        axes[i].config.invert_enable = true;    // TMC2209 = active-low
        axes[i].config.tmc_current_ma = 800;
        axes[i].config.tmc_microsteps = 16;
    }

    // Initialize TMC2209 drivers via UART (must be after GPIO init above)
    tmc2209_init();

    // Apply per-axis TMC config from defaults
    for (int i = 0; i < NUM_AXES; i++) {
        tmc2209_set_current(i, axes[i].config.tmc_current_ma);
        tmc2209_set_microsteps(i, axes[i].config.tmc_microsteps);
    }
}

// ---- Configuration ----

void stepper_set_config(axis_id_t axis, const axis_config_t *config) {
    if (axis >= NUM_AXES) return;

    // Detect TMC parameter changes to avoid unnecessary UART traffic
    bool current_changed = (axes[axis].config.tmc_current_ma != config->tmc_current_ma);
    bool usteps_changed  = (axes[axis].config.tmc_microsteps != config->tmc_microsteps);

    axes[axis].config = *config;

    // Apply TMC2209 settings if they changed
    if (current_changed) {
        tmc2209_set_current(axis, config->tmc_current_ma);
    }
    if (usteps_changed) {
        tmc2209_set_microsteps(axis, config->tmc_microsteps);
    }
}

const axis_config_t* stepper_get_config(axis_id_t axis) {
    if (axis >= NUM_AXES) return NULL;
    return &axes[axis].config;
}

// ---- Enable/disable ----

void stepper_enable(axis_id_t axis, bool enable) {
    if (axis >= NUM_AXES) return;
    bool pin_val = enable;
    if (axes[axis].config.invert_enable)
        pin_val = !pin_val;
    gpio_put(EN_PINS[axis], pin_val);
    axes[axis].status.enabled = enable;
}

// ---- PIO motion helpers ----

// Send a burst of steps at a given speed to the PIO SM
static void pio_send_steps(axis_id_t axis, uint32_t steps, uint32_t speed) {
    if (steps == 0 || speed == 0) return;

    // Period in PIO cycles (1 us resolution)
    // Full step period = 1e6 / speed. Half-period for high + low phases.
    uint32_t half_period = PIO_CLK_HZ / (speed * 2);
    if (half_period < MIN_PULSE_US)
        half_period = MIN_PULSE_US;

    axis_state_t *a = &axes[axis];
    pio_sm_put_blocking(a->pio, a->sm, half_period);
    pio_sm_put_blocking(a->pio, a->sm, steps);
}

// ---- Motion commands ----

void stepper_move_steps(axis_id_t axis, int32_t steps, uint32_t speed) {
    if (axis >= NUM_AXES || steps == 0) return;
    axis_state_t *a = &axes[axis];

    // Set direction
    bool dir = (steps > 0);
    if (a->config.invert_dir) dir = !dir;
    gpio_put(DIR_PINS[axis], dir);

    uint32_t abs_steps = (steps > 0) ? steps : -steps;

    // Trapezoidal profile: accel ramp → cruise → decel ramp
    uint32_t accel = a->config.accel;
    if (accel == 0) accel = 1;  // Avoid divide-by-zero

    // Steps needed to accelerate from 0 to cruise speed:
    //   v² = 2 * a * s  →  s = v² / (2a)
    uint32_t accel_steps = (uint64_t)speed * speed / (2 * accel);
    uint32_t decel_steps = accel_steps;

    // If not enough room for full accel+decel, truncate
    if (accel_steps + decel_steps > abs_steps) {
        accel_steps = abs_steps / 2;
        decel_steps = abs_steps - accel_steps;
    }

    uint32_t cruise_steps = abs_steps - accel_steps - decel_steps;

    a->status.busy = true;
    a->target_steps = steps;

    // Acceleration ramp: send small bursts at increasing speed
    uint32_t ramp_segments = 20;  // Number of discrete speed steps
    if (accel_steps > 0 && ramp_segments > accel_steps)
        ramp_segments = accel_steps;

    uint32_t start_speed = speed / ramp_segments;
    if (start_speed < 100) start_speed = 100;

    if (accel_steps > 0) {
        uint32_t steps_per_seg = accel_steps / ramp_segments;
        if (steps_per_seg < 1) steps_per_seg = 1;

        for (uint32_t i = 0; i < ramp_segments; i++) {
            uint32_t seg_speed = start_speed + (speed - start_speed) * (i + 1) / ramp_segments;
            uint32_t seg_steps = (i == ramp_segments - 1)
                ? (accel_steps - steps_per_seg * (ramp_segments - 1))
                : steps_per_seg;
            pio_send_steps(axis, seg_steps, seg_speed);
        }
    }

    // Cruise phase
    if (cruise_steps > 0) {
        pio_send_steps(axis, cruise_steps, speed);
    }

    // Deceleration ramp (reverse of accel)
    if (decel_steps > 0) {
        uint32_t steps_per_seg = decel_steps / ramp_segments;
        if (steps_per_seg < 1) steps_per_seg = 1;

        for (uint32_t i = ramp_segments; i > 0; i--) {
            uint32_t seg_speed = start_speed + (speed - start_speed) * i / ramp_segments;
            uint32_t seg_steps = (i == 1)
                ? (decel_steps - steps_per_seg * (ramp_segments - 1))
                : steps_per_seg;
            pio_send_steps(axis, seg_steps, seg_speed);
        }
    }

    // Update position
    a->status.position += steps;
    a->status.busy = false;
}

void stepper_move_mm(axis_id_t axis, float mm, uint32_t speed) {
    if (axis >= NUM_AXES) return;
    int32_t steps = (int32_t)(mm * axes[axis].config.steps_per_mm);
    stepper_move_steps(axis, steps, speed);
}

void stepper_jog(axis_id_t axis, bool direction, uint32_t speed) {
    if (axis >= NUM_AXES) return;
    axis_state_t *a = &axes[axis];

    bool dir = direction;
    if (a->config.invert_dir) dir = !dir;
    gpio_put(DIR_PINS[axis], dir);

    // Send a large number of steps — stop() will drain the FIFO
    pio_send_steps(axis, 1000000, speed);
    a->status.busy = true;
}

void stepper_stop(axis_id_t axis) {
    if (axis >= NUM_AXES) return;
    // Drain the PIO FIFO and restart SM to stop immediately
    pio_sm_set_enabled(axes[axis].pio, axes[axis].sm, false);
    pio_sm_clear_fifos(axes[axis].pio, axes[axis].sm);
    pio_sm_restart(axes[axis].pio, axes[axis].sm);
    pio_sm_set_enabled(axes[axis].pio, axes[axis].sm, true);
    axes[axis].status.busy = false;
}

void stepper_estop(axis_id_t axis) {
    if (axis >= NUM_AXES) return;
    stepper_stop(axis);
    stepper_enable(axis, false);
}

void stepper_estop_all(void) {
    for (int i = 0; i < NUM_AXES; i++) {
        stepper_estop(i);
    }
}

// ---- Homing ----

void stepper_home(axis_id_t axis) {
    if (axis >= NUM_AXES) return;
    axis_state_t *a = &axes[axis];

    stepper_enable(axis, true);

    // Move toward home switch at home_speed
    bool dir = false;  // Negative direction toward home
    if (a->config.invert_dir) dir = !dir;
    gpio_put(DIR_PINS[axis], dir);

    uint32_t speed = a->config.home_speed;
    uint32_t half_period = PIO_CLK_HZ / (speed * 2);
    if (half_period < MIN_PULSE_US) half_period = MIN_PULSE_US;

    a->status.busy = true;
    a->homing = true;

    // Step one at a time, checking home switch
    uint32_t max_steps = (uint32_t)(a->config.travel_mm * a->config.steps_per_mm * 1.2f);

    for (uint32_t i = 0; i < max_steps; i++) {
        // Check home switch (active-low with pull-up)
        if (!gpio_get(HOME_PINS[axis])) {
            break;  // Switch triggered
        }
        pio_send_steps(axis, 1, speed);
        sleep_us(PIO_CLK_HZ / speed + 10);  // Wait for step to complete
    }

    // Back off slowly
    dir = true;
    if (a->config.invert_dir) dir = !dir;
    gpio_put(DIR_PINS[axis], dir);

    for (uint32_t i = 0; i < 200; i++) {
        if (gpio_get(HOME_PINS[axis])) {
            break;  // Switch released
        }
        pio_send_steps(axis, 1, speed / 4);
        sleep_us(PIO_CLK_HZ / (speed / 4) + 10);
    }

    a->status.position = 0;
    a->status.homed = true;
    a->status.busy = false;
    a->homing = false;
}

void stepper_set_position(axis_id_t axis, int32_t steps) {
    if (axis >= NUM_AXES) return;
    axes[axis].status.position = steps;
}

// ---- Status ----

axis_status_t stepper_get_status(axis_id_t axis) {
    if (axis >= NUM_AXES) {
        axis_status_t empty = {0};
        return empty;
    }
    // Update home switch state
    axes[axis].status.home_switch = !gpio_get(HOME_PINS[axis]);  // Active-low
    return axes[axis].status;
}

bool stepper_is_busy(axis_id_t axis) {
    if (axis >= NUM_AXES) return false;
    // Check if PIO TX FIFO is empty and SM is stalled (waiting for pull)
    return !pio_sm_is_tx_fifo_empty(axes[axis].pio, axes[axis].sm);
}

bool stepper_any_busy(void) {
    for (int i = 0; i < NUM_AXES; i++) {
        if (stepper_is_busy(i)) return true;
    }
    return false;
}

// ---- Tick (unused — PIO handles timing) ----

void stepper_tick(void) {
    // No-op: PIO state machines handle step timing autonomously.
    // This function exists for interface compatibility.
}
