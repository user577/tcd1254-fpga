// frame_buffer.v — Double-buffered (ping-pong) BRAM for pixel data
//
// ECP5 LFE5U-45F: 108 × DP16KD (18 Kbit each) = ~1.9 Mbit BRAM.
// Double-buffered 2200 × 12 bits = 52,800 bits → ~6 EBR blocks total.
//
// Implementation: single unified memory (2 × TOTAL_PIXELS × 12 bits).
// buf_sel selects which half is written (the other is read).
// This pattern infers cleanly to ECP5 DP16KD Block RAM.

module frame_buffer #(
    parameter DATA_WIDTH   = 12,
    parameter TOTAL_PIXELS = 2200
)(
    input  wire                  clk,
    input  wire                  rst,

    // Write port (from adc_capture)
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire [11:0]           wr_addr,
    input  wire                  wr_en,

    // Read port (from frame_tx / shadow_detect)
    input  wire [11:0]           rd_addr,
    output reg  [DATA_WIDTH-1:0] rd_data,

    // Buffer swap control
    input  wire                  swap         // Pulse to swap buffers (on frame_done)
);

    // Buffer select: 0 = write to lower half / read from upper half
    //                1 = write to upper half / read from lower half
    reg buf_sel;

    always @(posedge clk) begin
        if (rst)
            buf_sel <= 1'b0;
        else if (swap)
            buf_sel <= ~buf_sel;
    end

    // Unified memory: lower half [0..TOTAL_PIXELS-1], upper half [TOTAL_PIXELS..2*TOTAL_PIXELS-1]
    reg [DATA_WIDTH-1:0] mem [0:2*TOTAL_PIXELS-1];

    // Write port — writes to buf_sel half
    wire [12:0] wr_full_addr = buf_sel ? (TOTAL_PIXELS + wr_addr) : {1'b0, wr_addr};

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_full_addr] <= wr_data;
    end

    // Read port — reads from opposite half (1-cycle latency)
    wire [12:0] rd_full_addr = buf_sel ? {1'b0, rd_addr} : (TOTAL_PIXELS + rd_addr);

    always @(posedge clk) begin
        rd_data <= mem[rd_full_addr];
    end

endmodule
