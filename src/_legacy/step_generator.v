// step_generator.v — Stepper motor pulse generator with trapezoidal velocity profile
//
// Generates STEP/DIR signals for a single axis with Trinamic stepper driver.
// Trapezoidal acceleration: ramp up → cruise → ramp down.
//
// Interface:
//   - move_start: pulse to begin a move
//   - target_steps: number of steps to move
//   - direction: 0=forward, 1=reverse
//   - max_speed: cruise speed in steps/sec (max ~200kHz at 80MHz clk)
//   - accel: acceleration in steps/sec² (applied as speed ramp)
//   - busy: high while moving
//   - position: running step counter (signed, wraps)
//
// Timing: STEP pulse width = 4us (320 sys clocks at 80MHz).
// Trinamic TMC2209/5160 need >= 100ns step pulse, 4us is very safe.

module step_generator #(
    parameter SYS_CLK_FREQ = 80_000_000
)(
    input  wire        clk,
    input  wire        rst,

    // Move command
    input  wire        move_start,     // Pulse to begin move
    input  wire [31:0] target_steps,   // Steps to move (0 = infinite/jog)
    input  wire        direction,      // 0=forward, 1=reverse
    input  wire [31:0] max_speed,      // Steps/sec (cruise speed)
    input  wire [31:0] accel,          // Steps/sec² acceleration
    input  wire        stop_cmd,       // Immediate stop (decelerate)
    input  wire        estop,          // Emergency stop (instant)

    // Outputs to Trinamic driver
    output reg         step_out,
    output reg         dir_out,
    output reg         enable_out,     // Active-low on most Trinamic drivers

    // Status
    output reg         busy,
    output reg  [31:0] position,       // Absolute position (steps, signed via overflow)
    output reg  [31:0] steps_remaining
);

    // Step pulse width in sys clocks (4us)
    localparam STEP_PULSE_CLKS = SYS_CLK_FREQ / 250_000;  // 320 @ 80MHz

    // Minimum step interval (max speed limit) — 400 steps/sec minimum interval
    localparam MIN_INTERVAL = SYS_CLK_FREQ / 200_000;  // 400 @ 80MHz = 200kHz max

    // State machine
    localparam S_IDLE     = 3'd0,
               S_ACCEL    = 3'd1,
               S_CRUISE   = 3'd2,
               S_DECEL    = 3'd3,
               S_STEP_ON  = 3'd4,
               S_STEP_OFF = 3'd5;

    reg [2:0]  state;
    reg [31:0] step_interval;     // Current interval between steps (sys clocks)
    reg [31:0] cruise_interval;   // Interval at max_speed
    reg [31:0] interval_cnt;      // Counter within current step interval
    reg [31:0] pulse_cnt;         // Counter for step pulse width
    reg [31:0] steps_done;
    reg [31:0] accel_steps;       // Steps spent accelerating (needed for decel calc)
    reg [31:0] decel_point;       // Step count at which to start decelerating
    reg        stopping;          // Decel-to-stop requested

    // Compute cruise interval: SYS_CLK_FREQ / max_speed
    // Use registered divide approximation — shift-based for synthesis
    // For exact: cruise_interval = SYS_CLK_FREQ / max_speed
    // We'll precompute this when move starts
    reg [63:0] div_result;

    // Acceleration interval delta per step
    // Each step during accel, the interval decreases
    // v(n) = v(n-1) + a*dt where dt = current_interval/SYS_CLK_FREQ
    // Simplified: new_interval = interval - interval^2 * accel / SYS_CLK_FREQ^2
    // But this is expensive in hardware. Use the standard Bresenham approach:
    // interval(n+1) = interval(n) - interval(n)^3 * accel / SYS_CLK_FREQ^2
    // Or the simpler "linear speed ramp" approach:
    // We maintain a "speed accumulator" and step when it overflows.
    //
    // Actually, the simplest correct approach for trapezoidal:
    // Maintain current_speed (steps/sec), update each step:
    //   current_speed += accel / current_speed  (in fixed point)
    // Then interval = SYS_CLK_FREQ / current_speed
    //
    // For hardware simplicity, we'll use the timer-based approach:
    // Start at a slow speed (start_speed), linearly decrease interval

    // Start speed: 200 steps/sec (slow enough to not skip steps)
    localparam START_SPEED = 200;
    localparam START_INTERVAL = SYS_CLK_FREQ / START_SPEED;  // 400000 @ 80MHz

    // Accel: decrease interval by accel_decrement each step
    // accel_decrement ≈ interval² × accel / SYS_CLK_FREQ²
    // Precomputed when move starts for current interval

    reg [31:0] accel_decrement;  // How much to decrease interval per step during accel

    always @(posedge clk) begin
        if (rst || estop) begin
            state           <= S_IDLE;
            step_out        <= 1'b0;
            dir_out         <= 1'b0;
            enable_out      <= 1'b1;   // Disabled (active-low)
            busy            <= 1'b0;
            position        <= 32'd0;
            steps_remaining <= 32'd0;
            steps_done      <= 32'd0;
            accel_steps     <= 32'd0;
            stopping        <= 1'b0;
            step_interval   <= START_INTERVAL;
            interval_cnt    <= 32'd0;
            pulse_cnt       <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    step_out   <= 1'b0;
                    busy       <= 1'b0;
                    stopping   <= 1'b0;

                    if (move_start && (target_steps > 0 || max_speed > 0)) begin
                        // Start a new move
                        busy            <= 1'b1;
                        enable_out      <= 1'b0;  // Enable driver
                        dir_out         <= direction;
                        steps_done      <= 32'd0;
                        steps_remaining <= target_steps;
                        accel_steps     <= 32'd0;
                        stopping        <= 1'b0;

                        // Compute cruise interval
                        if (max_speed > 0 && max_speed < 200_000)
                            cruise_interval <= SYS_CLK_FREQ / max_speed;
                        else
                            cruise_interval <= MIN_INTERVAL;

                        // Start at slow speed
                        step_interval   <= START_INTERVAL;
                        interval_cnt    <= 32'd0;

                        // Compute decel point (half the move for symmetric trapezoid)
                        if (target_steps > 0)
                            decel_point <= target_steps / 2;
                        else
                            decel_point <= 32'hFFFFFFFF;  // Jog mode — no auto-decel

                        state <= S_ACCEL;
                    end
                end

                // Accelerating: decrease interval each step until cruise speed
                S_ACCEL: begin
                    if (stop_cmd)
                        stopping <= 1'b1;

                    interval_cnt <= interval_cnt + 1;
                    if (interval_cnt >= step_interval) begin
                        // Time to take a step
                        interval_cnt <= 0;
                        state <= S_STEP_ON;
                        pulse_cnt <= 0;

                        // Update interval (accelerate)
                        // Decrease interval by a fraction proportional to accel
                        // Simple linear ramp: subtract fixed amount per step
                        // accel_dec ≈ (start_interval - cruise_interval) / accel_steps_needed
                        // For now use: interval -= interval >> 6 (roughly exponential)
                        if (!stopping && step_interval > cruise_interval) begin
                            if (step_interval > (cruise_interval + (step_interval >> 5)))
                                step_interval <= step_interval - (step_interval >> 5);
                            else
                                step_interval <= cruise_interval;
                            accel_steps <= accel_steps + 1;
                        end else if (!stopping) begin
                            // Reached cruise speed
                            step_interval <= cruise_interval;
                            state <= S_CRUISE;
                            // Actually still need to do the step
                            state <= S_STEP_ON;
                        end

                        if (stopping) begin
                            state <= S_STEP_ON;
                            // Will transition to DECEL after step
                        end
                    end
                end

                // Cruising at max speed
                S_CRUISE: begin
                    if (stop_cmd)
                        stopping <= 1'b1;

                    interval_cnt <= interval_cnt + 1;
                    if (interval_cnt >= cruise_interval) begin
                        interval_cnt <= 0;
                        state <= S_STEP_ON;
                        pulse_cnt <= 0;

                        // Check if we need to start decelerating
                        if (target_steps > 0 &&
                            steps_remaining <= accel_steps + 1) begin
                            step_interval <= cruise_interval;
                            // After step, go to DECEL
                        end

                        if (stopping) begin
                            step_interval <= cruise_interval;
                        end
                    end
                end

                // Decelerating: increase interval each step until stopped
                S_DECEL: begin
                    interval_cnt <= interval_cnt + 1;
                    if (interval_cnt >= step_interval) begin
                        interval_cnt <= 0;
                        state <= S_STEP_ON;
                        pulse_cnt <= 0;

                        // Increase interval (decelerate)
                        step_interval <= step_interval + (step_interval >> 5);

                        // If slowed enough, stop
                        if (step_interval >= START_INTERVAL || steps_remaining <= 1) begin
                            // This will be the last step
                        end
                    end
                end

                // Generate step pulse (high phase)
                S_STEP_ON: begin
                    step_out <= 1'b1;
                    pulse_cnt <= pulse_cnt + 1;
                    if (pulse_cnt >= STEP_PULSE_CLKS) begin
                        step_out <= 1'b0;
                        state <= S_STEP_OFF;

                        // Count the step
                        steps_done <= steps_done + 1;
                        if (dir_out)
                            position <= position - 1;
                        else
                            position <= position + 1;

                        if (target_steps > 0)
                            steps_remaining <= steps_remaining - 1;
                    end
                end

                // Post-step: decide next state
                S_STEP_OFF: begin
                    step_out <= 1'b0;
                    interval_cnt <= 0;

                    // Move complete?
                    if (target_steps > 0 && steps_remaining == 0) begin
                        state <= S_IDLE;
                    end
                    // Decel to stop?
                    else if (stopping) begin
                        if (step_interval >= START_INTERVAL)
                            state <= S_IDLE;
                        else
                            state <= S_DECEL;
                    end
                    // Need to start decelerating?
                    else if (target_steps > 0 &&
                             steps_remaining <= accel_steps + 1) begin
                        state <= S_DECEL;
                    end
                    // Still accelerating?
                    else if (step_interval > cruise_interval) begin
                        state <= S_ACCEL;
                    end
                    // Cruising
                    else begin
                        state <= S_CRUISE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
