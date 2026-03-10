// stepper.h — PIO-based stepper motor control for 4 axes

#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#define NUM_AXES 4

// Axis identification
typedef enum {
    AXIS_FEEDER    = 0,
    AXIS_CONVEYOR  = 1,
    AXIS_SINGULATE = 2,
    AXIS_SORT      = 3,
} axis_id_t;

// Axis configuration (set from Python, persisted to flash)
typedef struct {
    uint32_t steps_per_mm;       // Microsteps per mm of travel
    uint32_t max_speed;          // Steps/sec (cruise speed)
    uint32_t accel;              // Steps/sec² acceleration
    uint32_t home_speed;         // Steps/sec during homing
    float    travel_mm;          // Max travel distance
    bool     invert_dir;         // Swap direction sense
    bool     invert_enable;      // EN pin polarity (TMC2209 = active-low)
    uint16_t tmc_current_ma;     // Motor RMS current (for TMC UART config)
    uint8_t  tmc_microsteps;     // Microstepping (1,2,4,8,16,32,64,128,256)
} axis_config_t;

// Motion status
typedef struct {
    int32_t  position;           // Current position in steps (signed)
    bool     busy;               // Currently moving
    bool     homed;              // Home position established
    bool     home_switch;        // Current state of home switch
    bool     enabled;            // Driver enabled
} axis_status_t;

// Initialize PIO stepper engine for all axes
void stepper_init(void);

// Configure an axis (call before moves)
void stepper_set_config(axis_id_t axis, const axis_config_t *config);
const axis_config_t* stepper_get_config(axis_id_t axis);

// Motion commands
void stepper_move_steps(axis_id_t axis, int32_t steps, uint32_t speed);
void stepper_move_mm(axis_id_t axis, float mm, uint32_t speed);
void stepper_jog(axis_id_t axis, bool direction, uint32_t speed);
void stepper_stop(axis_id_t axis);      // Controlled deceleration
void stepper_estop(axis_id_t axis);     // Immediate stop
void stepper_estop_all(void);

// Homing
void stepper_home(axis_id_t axis);
void stepper_set_position(axis_id_t axis, int32_t steps);

// Enable/disable driver
void stepper_enable(axis_id_t axis, bool enable);

// Status
axis_status_t stepper_get_status(axis_id_t axis);
bool stepper_is_busy(axis_id_t axis);
bool stepper_any_busy(void);

// Called from timer ISR — drives the step generation
void stepper_tick(void);

#endif // STEPPER_H
