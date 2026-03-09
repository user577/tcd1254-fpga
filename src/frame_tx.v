// frame_tx.v — Frame serialization controller
//
// Two modes:
//   Position mode (mode=1): After each frame, sends 2-byte shadow position (LE)
//   Raw mode (mode=2):      Sends 0xAA 0x55 sync + all pixels (2 bytes each, LE)
//
// Flow control: waits for UART TX ready between bytes.
// Frame decimation in raw mode: if UART can't keep up, skips to latest frame.

module frame_tx #(
    parameter TOTAL_PIXELS = 2200
)(
    input  wire        clk,
    input  wire        rst,

    // Control
    input  wire [7:0]  mode,           // 1 = position, 2 = raw
    input  wire        frame_ready,    // Pulse: new frame in read buffer
    input  wire [15:0] shadow_pos,     // From shadow_detect
    input  wire        shadow_done,    // shadow_detect finished

    // Frame buffer read interface
    output reg  [11:0] fb_rd_addr,
    input  wire [11:0] fb_rd_data,

    // UART TX interface
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,

    // Status
    output reg         transmitting
);

    localparam S_IDLE       = 4'd0,
               S_POS_LO     = 4'd1,
               S_POS_HI     = 4'd2,
               S_RAW_SYNC0  = 4'd3,
               S_RAW_SYNC1  = 4'd4,
               S_RAW_ADDR   = 4'd5,   // Set BRAM read address
               S_RAW_WAIT1  = 4'd6,   // BRAM read latency cycle 1
               S_RAW_WAIT2  = 4'd7,   // BRAM read latency cycle 2 — data valid
               S_RAW_LO     = 4'd8,
               S_RAW_HI     = 4'd9,
               S_RAW_NEXT   = 4'd10,
               S_WAIT_SHADOW = 4'd11;

    reg [3:0]  state;
    reg [11:0] px_idx;
    reg [11:0] px_data_latched;
    reg        frame_pending;

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            tx_start      <= 1'b0;
            tx_data       <= 0;
            fb_rd_addr    <= 0;
            px_idx        <= 0;
            transmitting  <= 1'b0;
            frame_pending <= 1'b0;
            px_data_latched <= 0;
        end else begin
            tx_start <= 1'b0;

            // Latch pending frame
            if (frame_ready)
                frame_pending <= 1'b1;

            case (state)
                S_IDLE: begin
                    transmitting <= 1'b0;
                    if (frame_pending) begin
                        frame_pending <= 1'b0;
                        transmitting  <= 1'b1;
                        if (mode == 8'd2) begin
                            // Raw mode: send sync bytes
                            state <= S_RAW_SYNC0;
                        end else begin
                            // Position mode: wait for shadow detection
                            state <= S_WAIT_SHADOW;
                        end
                    end
                end

                // ---- Position mode ----
                S_WAIT_SHADOW: begin
                    if (shadow_done) begin
                        state <= S_POS_LO;
                    end
                end

                S_POS_LO: begin
                    if (!tx_busy) begin
                        tx_data  <= shadow_pos[7:0];   // Low byte first (LE)
                        tx_start <= 1'b1;
                        state    <= S_POS_HI;
                    end
                end

                S_POS_HI: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= shadow_pos[15:8];  // High byte
                        tx_start <= 1'b1;
                        state    <= S_IDLE;
                    end
                end

                // ---- Raw mode ----
                S_RAW_SYNC0: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'hAA;
                        tx_start <= 1'b1;
                        state    <= S_RAW_SYNC1;
                    end
                end

                S_RAW_SYNC1: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= 8'h55;
                        tx_start <= 1'b1;
                        px_idx   <= 0;
                        state    <= S_RAW_ADDR;
                    end
                end

                S_RAW_ADDR: begin
                    // Set BRAM read address
                    if (!tx_busy && !tx_start) begin
                        fb_rd_addr <= px_idx;
                        state      <= S_RAW_WAIT1;
                    end
                end

                S_RAW_WAIT1: begin
                    // BRAM registers the read this cycle
                    state <= S_RAW_WAIT2;
                end

                S_RAW_WAIT2: begin
                    // Data now valid on fb_rd_data
                    px_data_latched <= fb_rd_data;
                    state           <= S_RAW_LO;
                end

                S_RAW_LO: begin
                    if (!tx_busy) begin
                        tx_data  <= px_data_latched[7:0];
                        tx_start <= 1'b1;
                        state    <= S_RAW_HI;
                    end
                end

                S_RAW_HI: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= {4'b0000, px_data_latched[11:8]};
                        tx_start <= 1'b1;
                        state    <= S_RAW_NEXT;
                    end
                end

                S_RAW_NEXT: begin
                    if (!tx_busy && !tx_start) begin
                        if (px_idx == TOTAL_PIXELS - 1) begin
                            state <= S_IDLE;
                        end else begin
                            px_idx <= px_idx + 1;
                            state  <= S_RAW_ADDR;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
