// shadow_detect.v — Hardware gradient-based shadow detector (streaming)
//
// Reads pixels directly from frame buffer via rd_addr/pixel_data interface.
// No large local arrays — uses shift registers for sliding windows.
//
// Algorithm (matches STM32 reference, no smoothing for V1):
//   Pass 1: Stream pixels SEARCH_START..SEARCH_END, find max rising gradient
//           gradient = pixel[pos] - pixel[pos - GRAD_WINDOW]
//   Pass 2: Stream pixels from (rise_pos+20)..SEARCH_END, find max falling gradient
//           gradient = pixel[pos - GRAD_WINDOW] - pixel[pos]
//   Result: center = (rise_pos + fall_pos) / 2, x10 for 0.1px resolution
//
// BRAM read pipeline: 4 cycles per pixel (set addr, wait, latch, process)
// Total time: ~2 x (SEARCH_END - SEARCH_START) x 4 clocks ~ 15600 clocks ~ 195 us @ 80 MHz

module shadow_detect #(
    parameter TOTAL_PIXELS     = 2200,
    parameter SEARCH_START     = 150,
    parameter SEARCH_END       = 2000,
    parameter GRAD_WINDOW      = 20,
    parameter SHADOW_THRESHOLD = 10,
    parameter SHADOW_MIN_WIDTH = 20,
    parameter SHADOW_MAX_WIDTH = 1000,
    parameter NO_SHADOW        = 16'hFFFF
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // Pulse to begin detection
    input  wire [11:0] pixel_data,     // Data from frame_buffer read port (1-cycle latency)

    output reg  [11:0] rd_addr,        // Address to frame_buffer
    output reg  [15:0] result,         // Shadow center x 10, or 0xFFFF
    output reg         done            // Pulses when detection complete
);

    // ---- State machine ----
    localparam S_IDLE       = 3'd0,
               S_PREFILL    = 3'd1,   // Fill gradient delay line
               S_SCAN_RISE  = 3'd2,   // Find steepest rising gradient
               S_PREFILL2   = 3'd3,   // Refill delay line for pass 2
               S_SCAN_FALL  = 3'd4,   // Find steepest falling gradient
               S_RESULT     = 3'd5;

    reg [2:0]  state;
    reg [11:0] pos;           // Current pixel position being read
    reg [1:0]  phase;         // 0=set addr, 1=wait BRAM, 2=latch data, 3=process
    reg [11:0] fill_cnt;      // Prefill counter

    // Gradient delay line — shift register of GRAD_WINDOW entries
    reg [11:0] dl [0:GRAD_WINDOW-1];
    wire [11:0] dl_oldest = dl[GRAD_WINDOW-1];

    // Pipeline register for pixel data (breaks BRAM→gradient timing path)
    reg [11:0] px_latched;

    // Results
    reg signed [15:0] max_rise_grad;
    reg [11:0] rise_pos;
    reg signed [15:0] max_fall_grad;
    reg [11:0] fall_pos;

    // Gradient computation (uses latched data, not raw BRAM output)
    wire signed [15:0] rise_grad = $signed({4'b0, px_latched}) - $signed({4'b0, dl_oldest});
    wire signed [15:0] fall_grad = $signed({4'b0, dl_oldest}) - $signed({4'b0, px_latched});

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            done          <= 1'b0;
            result        <= NO_SHADOW;
            rd_addr       <= 0;
            pos           <= 0;
            phase         <= 0;
            fill_cnt      <= 0;
            max_rise_grad <= 0;
            max_fall_grad <= 0;
            rise_pos      <= 0;
            fall_pos      <= 0;
            px_latched    <= 0;
            for (i = 0; i < GRAD_WINDOW; i = i + 1)
                dl[i] <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        state    <= S_PREFILL;
                        pos      <= SEARCH_START;
                        fill_cnt <= 0;
                        phase    <= 2'd0;
                        max_rise_grad <= 0;
                        rise_pos      <= 0;
                        for (i = 0; i < GRAD_WINDOW; i = i + 1)
                            dl[i] <= 0;
                    end
                end

                // ---- Pass 1: Prefill delay line ----
                S_PREFILL: begin
                    case (phase)
                        2'd0: begin
                            rd_addr <= pos;
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            phase <= 2'd2;
                        end
                        2'd2: begin
                            // Latch BRAM output (pipeline register)
                            px_latched <= pixel_data;
                            phase      <= 2'd3;
                        end
                        2'd3: begin
                            for (i = GRAD_WINDOW-1; i > 0; i = i - 1)
                                dl[i] <= dl[i-1];
                            dl[0] <= px_latched;

                            fill_cnt <= fill_cnt + 1;
                            pos      <= pos + 1;
                            phase    <= 2'd0;

                            if (fill_cnt == GRAD_WINDOW - 1) begin
                                state <= S_SCAN_RISE;
                            end
                        end
                    endcase
                end

                // ---- Pass 1: Scan for rising edge ----
                S_SCAN_RISE: begin
                    case (phase)
                        2'd0: begin
                            rd_addr <= pos;
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            phase <= 2'd2;
                        end
                        2'd2: begin
                            px_latched <= pixel_data;
                            phase      <= 2'd3;
                        end
                        2'd3: begin
                            // Shift delay line
                            for (i = GRAD_WINDOW-1; i > 0; i = i - 1)
                                dl[i] <= dl[i-1];
                            dl[0] <= px_latched;

                            // Compare: rising gradient = new - old
                            if (rise_grad > max_rise_grad) begin
                                max_rise_grad <= rise_grad;
                                rise_pos      <= pos - GRAD_WINDOW / 2;
                            end

                            pos   <= pos + 1;
                            phase <= 2'd0;

                            if (pos >= SEARCH_END) begin
                                if (max_rise_grad < SHADOW_THRESHOLD || rise_pos == 0) begin
                                    result <= NO_SHADOW;
                                    done   <= 1'b1;
                                    state  <= S_IDLE;
                                end else begin
                                    state    <= S_PREFILL2;
                                    pos      <= rise_pos + SHADOW_MIN_WIDTH;
                                    fill_cnt <= 0;
                                    phase    <= 2'd0;
                                    max_fall_grad <= 0;
                                    fall_pos      <= 0;
                                    for (i = 0; i < GRAD_WINDOW; i = i + 1)
                                        dl[i] <= 0;
                                end
                            end
                        end
                    endcase
                end

                // ---- Pass 2: Prefill delay line ----
                S_PREFILL2: begin
                    case (phase)
                        2'd0: begin
                            rd_addr <= pos;
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            phase <= 2'd2;
                        end
                        2'd2: begin
                            px_latched <= pixel_data;
                            phase      <= 2'd3;
                        end
                        2'd3: begin
                            for (i = GRAD_WINDOW-1; i > 0; i = i - 1)
                                dl[i] <= dl[i-1];
                            dl[0] <= px_latched;

                            fill_cnt <= fill_cnt + 1;
                            pos      <= pos + 1;
                            phase    <= 2'd0;

                            if (fill_cnt == GRAD_WINDOW - 1) begin
                                state <= S_SCAN_FALL;
                            end
                        end
                    endcase
                end

                // ---- Pass 2: Scan for falling edge ----
                S_SCAN_FALL: begin
                    case (phase)
                        2'd0: begin
                            rd_addr <= pos;
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            phase <= 2'd2;
                        end
                        2'd2: begin
                            px_latched <= pixel_data;
                            phase      <= 2'd3;
                        end
                        2'd3: begin
                            for (i = GRAD_WINDOW-1; i > 0; i = i - 1)
                                dl[i] <= dl[i-1];
                            dl[0] <= px_latched;

                            if (fall_grad > max_fall_grad) begin
                                max_fall_grad <= fall_grad;
                                fall_pos      <= pos - GRAD_WINDOW / 2;
                            end

                            pos   <= pos + 1;
                            phase <= 2'd0;

                            if (pos >= SEARCH_END) begin
                                state <= S_RESULT;
                            end
                        end
                    endcase
                end

                // ---- Compute result ----
                S_RESULT: begin
                    if (max_fall_grad < SHADOW_THRESHOLD || fall_pos == 0) begin
                        result <= NO_SHADOW;
                    end else if ((fall_pos - rise_pos) < SHADOW_MIN_WIDTH ||
                                 (fall_pos - rise_pos) > SHADOW_MAX_WIDTH) begin
                        result <= NO_SHADOW;
                    end else begin
                        result <= ((rise_pos + fall_pos) >> 1) * 16'd10;
                    end
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
