// flash_trigger.v — FPGA-synchronized flash lamp controller
//
// Fires flash lamp GPIO outputs during the CCD integration window,
// synchronized to the ICG/SH timing from ccd_driver.
//
// Features:
//   - Up to 8 flash outputs (active-high, drive MOSFETs)
//   - Programmable delay from integration start (ICG→SH→readout boundary)
//   - Programmable pulse duration (microseconds)
//   - Auto-sequence mode: cycles through set bits in lamp_mask,
//     one lamp per frame, for multi-angle capture
//
// Timing within a CCD frame:
//   ICG falls → SH pulse → ICG rises → readout → INTEGRATE
//   Flash fires during INTEGRATE phase:
//     integrate_start + flash_delay → pulse ON for flash_duration
//
// The frame_done pulse from ccd_driver marks the end of readout and
// the start of the integration window — this is our reference point.

module flash_trigger #(
    parameter SYS_CLK_FREQ = 80_000_000,
    parameter MAX_LAMPS    = 8
)(
    input  wire        clk,
    input  wire        rst,

    // Configuration (from cmd_parser)
    input  wire [7:0]  lamp_mask,         // Bitmask: which lamps can fire
    input  wire [15:0] flash_delay_us,    // Delay from frame_done to flash (us)
    input  wire [15:0] flash_duration_us, // Flash pulse width (us)
    input  wire        auto_sequence,     // Cycle through lamps one per frame
    input  wire        flash_enabled,     // Master enable

    // CCD timing reference
    input  wire        frame_done,        // Pulse from ccd_driver at end of readout

    // Outputs
    output reg [MAX_LAMPS-1:0] flash_out, // GPIO to MOSFET drivers
    output reg [2:0]           current_lamp_id  // Which lamp is currently active
);

    // Convert microseconds to system clock counts
    // Use registered multiply to meet timing at 80 MHz
    localparam CLKS_PER_US = SYS_CLK_FREQ / 1_000_000;  // 80

    reg [31:0] delay_clks;
    reg [31:0] duration_clks;

    always @(posedge clk) begin
        delay_clks    <= {16'd0, flash_delay_us} * CLKS_PER_US;
        duration_clks <= {16'd0, flash_duration_us} * CLKS_PER_US;
    end

    // State machine
    localparam S_IDLE    = 2'd0,
               S_DELAY   = 2'd1,
               S_PULSE   = 2'd2,
               S_DONE    = 2'd3;

    reg [1:0]  state;
    reg [31:0] cnt;

    // Auto-sequence: track which lamp fires next
    reg [2:0] seq_idx;  // Index into the set bits of lamp_mask

    // Find the Nth set bit in lamp_mask
    // Simple scan — MAX_LAMPS is small (8)
    function [2:0] nth_set_bit;
        input [7:0] mask;
        input [2:0] n;
        reg [2:0] count;
        reg [2:0] result;
        integer i;
        begin
            count  = 0;
            result = 0;
            for (i = 0; i < MAX_LAMPS; i = i + 1) begin
                if (mask[i]) begin
                    if (count == n)
                        result = i[2:0];
                    count = count + 1;
                end
            end
            nth_set_bit = result;
        end
    endfunction

    // Count set bits in mask
    function [3:0] popcount;
        input [7:0] mask;
        integer i;
        reg [3:0] c;
        begin
            c = 0;
            for (i = 0; i < MAX_LAMPS; i = i + 1)
                c = c + {3'd0, mask[i]};
            popcount = c;
        end
    endfunction

    wire [3:0] lamp_count = popcount(lamp_mask);
    wire [2:0] active_lamp = auto_sequence ? nth_set_bit(lamp_mask, seq_idx)
                                           : 3'd0;  // Not used when all fire

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            cnt            <= 0;
            flash_out      <= {MAX_LAMPS{1'b0}};
            seq_idx        <= 0;
            current_lamp_id <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    flash_out <= {MAX_LAMPS{1'b0}};

                    if (frame_done && flash_enabled && lamp_mask != 0) begin
                        if (delay_clks == 0) begin
                            // No delay — fire immediately
                            state <= S_PULSE;
                            cnt   <= 0;
                            if (auto_sequence) begin
                                flash_out[active_lamp] <= 1'b1;
                                current_lamp_id <= active_lamp;
                            end else begin
                                flash_out <= lamp_mask;
                                current_lamp_id <= nth_set_bit(lamp_mask, 0);
                            end
                        end else begin
                            state <= S_DELAY;
                            cnt   <= 0;
                        end
                    end
                end

                S_DELAY: begin
                    cnt <= cnt + 1;
                    if (cnt >= delay_clks - 1) begin
                        state <= S_PULSE;
                        cnt   <= 0;
                        // Turn on flash
                        if (auto_sequence) begin
                            flash_out[active_lamp] <= 1'b1;
                            current_lamp_id <= active_lamp;
                        end else begin
                            flash_out <= lamp_mask;
                            current_lamp_id <= nth_set_bit(lamp_mask, 0);
                        end
                    end
                end

                S_PULSE: begin
                    cnt <= cnt + 1;
                    if (cnt >= duration_clks - 1) begin
                        flash_out <= {MAX_LAMPS{1'b0}};
                        state <= S_DONE;

                        // Advance auto-sequence index for next frame
                        if (auto_sequence) begin
                            if (seq_idx + 1 >= lamp_count[2:0])
                                seq_idx <= 0;
                            else
                                seq_idx <= seq_idx + 1;
                        end
                    end
                end

                S_DONE: begin
                    // Wait for next frame_done
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
