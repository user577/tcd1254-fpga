// ccd_driver.v — TCD1254 CCD timing engine
//
// Generates fM (master clock), SH (sample & hold), ICG (integration clear gate).
// All outputs active-HIGH here; external 74HCT04 inverts them for the CCD.
//
// Pixel rate = fM / 4.  One pixel every 4 fM cycles.
// At 80 MHz sys_clk and fM = 4 MHz: fM toggles every 10 sys_clks (period = 20).
// Pixel clock = 1 MHz -> 1 pixel every 80 sys_clks.
//
// State machine: IDLE -> ICG_LOW -> SH_PULSE -> SH_TO_ICG -> READOUT -> INTEGRATE
//
// ICG/SH timing (from TCD1254 datasheet, matching reference timer_conf.c):
//   1. ICG goes LOW (active)
//   2. SH goes HIGH ~1 us after ICG falls
//   3. SH stays HIGH for >= 1000 ns (2 us used)
//   4. SH goes LOW
//   5. ICG goes HIGH >= 1000 ns after SH falls (2 us used)
//   6. Readout begins: pixels clocked out at fM/4

module ccd_driver #(
    parameter SYS_CLK_FREQ   = 80_000_000,
    parameter FM_FREQ         = 4_000_000,
    parameter TOTAL_PIXELS    = 2200,        // Total pixels including dummy
    parameter SH_DELAY_US     = 1,           // Delay after ICG falls before SH rises
    parameter SH_PULSE_US     = 2,           // SH high pulse width
    parameter SH_TO_ICG_US    = 2            // Delay after SH falls before ICG rises
)(
    input  wire        clk,          // System clock
    input  wire        rst,
    input  wire        start,        // Begin acquisition
    input  wire        stop,         // Stop acquisition (returns to IDLE)
    input  wire [31:0] sh_period,    // SH period in fM cycles (integration time)
    input  wire [31:0] icg_period,   // ICG period in fM cycles (frame time)

    output reg         fm_out,       // Master clock to CCD (via 74HCT04)
    output reg         sh_out,       // Sample & hold (via 74HCT04)
    output reg         icg_out,      // Integration clear gate (via 74HCT04)
    output reg         pixel_valid,  // Strobes HIGH for 1 sys_clk when pixel data ready
    output reg  [11:0] pixel_index,  // Current pixel number during readout
    output reg         frame_done,   // Pulses HIGH for 1 sys_clk at end of frame
    output wire        running       // 1 when acquisition active
);

    // ----- Timing constants in system clocks -----
    localparam FM_HALF_PERIOD  = SYS_CLK_FREQ / (2 * FM_FREQ);       // 10 @ 80/4
    localparam FM_PERIOD       = FM_HALF_PERIOD * 2;                   // 20
    localparam PIXEL_PERIOD    = SYS_CLK_FREQ / (FM_FREQ / 4);       // 80 @ 80/1M
    localparam SH_DELAY_CLKS   = (SYS_CLK_FREQ / 1_000_000) * SH_DELAY_US;  // 80
    localparam SH_PULSE_CLKS   = (SYS_CLK_FREQ / 1_000_000) * SH_PULSE_US;  // 160
    localparam SH_ICG_CLKS     = (SYS_CLK_FREQ / 1_000_000) * SH_TO_ICG_US; // 160
    localparam READOUT_CLKS    = TOTAL_PIXELS * PIXEL_PERIOD;          // 176000
    localparam ADC_PIPELINE    = 3;  // AD9226 pipeline delay in pixel clocks

    // ----- State machine -----
    localparam S_IDLE       = 3'd0,
               S_ICG_LOW    = 3'd1,   // ICG active (low on CCD, high here)
               S_SH_PULSE   = 3'd2,   // SH active
               S_SH_TO_ICG  = 3'd3,   // Wait after SH before ICG releases
               S_READOUT    = 3'd4,   // Clocking out pixels
               S_INTEGRATE  = 3'd5;   // Waiting for next frame

    reg [2:0]  state;
    reg [31:0] cnt;           // General-purpose counter
    reg [31:0] fm_cnt;        // fM toggle counter
    reg [31:0] pixel_cnt;     // Counts sys_clks within a pixel period
    reg [31:0] integ_cnt;     // Integration timer (sys_clk counts)
    reg        active;

    assign running = active;

    // ----- fM generation (free-running when active) -----
    always @(posedge clk) begin
        if (rst || !active) begin
            fm_out <= 1'b0;
            fm_cnt <= 0;
        end else begin
            if (fm_cnt == FM_HALF_PERIOD - 1) begin
                fm_cnt <= 0;
                fm_out <= ~fm_out;
            end else begin
                fm_cnt <= fm_cnt + 1;
            end
        end
    end

    // Compute integration wait in sys_clks (icg_period is in fM cycles)
    // Registered pipeline to meet timing: multiply and subtract in separate cycles.
    // Only recomputed when icg_period changes (start or command update).
    reg [36:0] frame_clks_r;
    reg [31:0] integ_wait;

    always @(posedge clk) begin
        if (rst) begin
            frame_clks_r <= 0;
            integ_wait   <= 0;
        end else begin
            frame_clks_r <= {5'd0, icg_period} * FM_PERIOD;
            integ_wait   <= (frame_clks_r > READOUT_CLKS) ?
                            (frame_clks_r[31:0] - READOUT_CLKS - 1) : 32'd0;
        end
    end

    // ----- Main state machine -----
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            cnt         <= 0;
            pixel_cnt   <= 0;
            pixel_index <= 0;
            pixel_valid <= 0;
            frame_done  <= 0;
            sh_out      <= 1'b0;
            icg_out     <= 1'b0;
            active      <= 0;
            integ_cnt   <= 0;
        end else if (stop) begin
            state       <= S_IDLE;
            sh_out      <= 1'b0;
            icg_out     <= 1'b0;
            active      <= 1'b0;
            pixel_valid <= 1'b0;
            frame_done  <= 1'b0;
        end else begin
            pixel_valid <= 1'b0;
            frame_done  <= 1'b0;

            case (state)
                S_IDLE: begin
                    sh_out  <= 1'b0;
                    icg_out <= 1'b0;
                    if (start) begin
                        active  <= 1'b1;
                        state   <= S_ICG_LOW;
                        cnt     <= 0;
                        icg_out <= 1'b1;  // ICG active (74HCT04 inverts to LOW)
                    end
                end

                // ICG active — wait SH_DELAY then raise SH
                S_ICG_LOW: begin
                    icg_out <= 1'b1;
                    cnt <= cnt + 1;
                    if (cnt >= SH_DELAY_CLKS - 1) begin
                        state  <= S_SH_PULSE;
                        cnt    <= 0;
                        sh_out <= 1'b1;
                    end
                end

                // SH pulse active
                S_SH_PULSE: begin
                    sh_out <= 1'b1;
                    cnt <= cnt + 1;
                    if (cnt >= SH_PULSE_CLKS - 1) begin
                        state  <= S_SH_TO_ICG;
                        cnt    <= 0;
                        sh_out <= 1'b0;
                    end
                end

                // Wait after SH falls before ICG releases
                S_SH_TO_ICG: begin
                    cnt <= cnt + 1;
                    if (cnt >= SH_ICG_CLKS - 1) begin
                        state       <= S_READOUT;
                        cnt         <= 0;
                        icg_out     <= 1'b0;  // ICG releases
                        pixel_cnt   <= 0;
                        pixel_index <= 0;
                    end
                end

                // Readout: clock out all pixels
                S_READOUT: begin
                    pixel_cnt <= pixel_cnt + 1;

                    if (pixel_cnt == PIXEL_PERIOD - 1) begin
                        pixel_cnt   <= 0;
                        pixel_index <= pixel_index + 1;

                        // Pixel data valid after ADC pipeline delay
                        if (pixel_index >= ADC_PIPELINE) begin
                            pixel_valid <= 1'b1;
                        end

                        // All pixels read (including pipeline flush)
                        if (pixel_index == TOTAL_PIXELS + ADC_PIPELINE - 1) begin
                            frame_done <= 1'b1;
                            state      <= S_INTEGRATE;
                            integ_cnt  <= 0;
                        end
                    end
                end

                // Integration period — wait for remaining frame time
                S_INTEGRATE: begin
                    integ_cnt <= integ_cnt + 1;
                    if (integ_cnt >= integ_wait) begin
                        state   <= S_ICG_LOW;
                        cnt     <= 0;
                        icg_out <= 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
