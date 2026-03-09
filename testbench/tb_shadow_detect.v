// tb_shadow_detect.v — Testbench for streaming shadow detection algorithm
// Creates a synthetic frame in a memory array and verifies detection

`timescale 1ns / 1ps

module tb_shadow_detect;

    reg         clk;
    reg         rst;
    reg         start;
    wire [11:0] rd_addr;
    reg  [11:0] pixel_data;
    wire [15:0] result;
    wire        done;

    localparam TOTAL_PIXELS = 2200;
    localparam CLK_PERIOD   = 12.5;  // 80 MHz

    shadow_detect #(
        .TOTAL_PIXELS(TOTAL_PIXELS),
        .SEARCH_START(150),
        .SEARCH_END(2000)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .pixel_data(pixel_data),
        .rd_addr(rd_addr),
        .result(result),
        .done(done)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Synthetic frame: flat baseline with a shadow (dip) at pixels 800-1200
    // The CCD outputs HIGHER values in shadow (inverted signal), so
    // rising gradient = shadow start, falling = shadow end
    localparam SHADOW_START = 800;
    localparam SHADOW_END   = 1200;
    localparam BASELINE     = 500;
    localparam SHADOW_PEAK  = 3000;
    localparam EDGE_WIDTH   = 30;

    // Synthetic pixel data memory
    reg [11:0] frame_mem [0:TOTAL_PIXELS-1];

    integer ii;
    initial begin
        for (ii = 0; ii < TOTAL_PIXELS; ii = ii + 1) begin
            if (ii < SHADOW_START - EDGE_WIDTH)
                frame_mem[ii] = BASELINE;
            else if (ii < SHADOW_START)
                // Rising edge (signal goes UP = shadow region)
                frame_mem[ii] = BASELINE +
                    ((SHADOW_PEAK - BASELINE) * (ii - (SHADOW_START - EDGE_WIDTH))) / EDGE_WIDTH;
            else if (ii < SHADOW_END)
                frame_mem[ii] = SHADOW_PEAK;
            else if (ii < SHADOW_END + EDGE_WIDTH)
                // Falling edge (signal recovers)
                frame_mem[ii] = SHADOW_PEAK -
                    ((SHADOW_PEAK - BASELINE) * (ii - SHADOW_END)) / EDGE_WIDTH;
            else
                frame_mem[ii] = BASELINE;
        end
    end

    // Respond to read address with 1-cycle latency (simulating BRAM)
    always @(posedge clk) begin
        if (rd_addr < TOTAL_PIXELS)
            pixel_data <= frame_mem[rd_addr];
        else
            pixel_data <= 12'd0;
    end

    // Expected center: (800 + 1200) / 2 = 1000, in 0.1px = 10000
    localparam EXPECTED_CENTER = 1000;
    localparam TOLERANCE       = 30;

    initial begin
        $dumpfile("tb_shadow_detect.vcd");
        $dumpvars(0, tb_shadow_detect);

        rst   = 1;
        start = 0;
        #500;
        rst = 0;
        #100;

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait(done);
        @(posedge clk);

        $display("Shadow result = %0d (0x%04X)", result, result);

        if (result == 16'hFFFF) begin
            $display("ERROR: No shadow detected (expected center ~%0d)", EXPECTED_CENTER);
        end else begin
            $display("Shadow center = %0d.%0d pixels", result / 10, result % 10);
            if (result / 10 > EXPECTED_CENTER - TOLERANCE &&
                result / 10 < EXPECTED_CENTER + TOLERANCE) begin
                $display("=== Shadow Detect Testbench PASSED (within +/-%0d px) ===", TOLERANCE);
            end else begin
                $display("WARNING: Center outside expected range (%0d +/- %0d)",
                         EXPECTED_CENTER, TOLERANCE);
            end
        end

        #100;
        $finish;
    end

    initial begin
        #10_000_000;
        $display("ERROR: Timeout");
        $finish;
    end

endmodule
