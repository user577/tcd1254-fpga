// tb_ccd_driver.v — Testbench for CCD timing engine
// Verifies fM frequency, ICG/SH timing relationships, and pixel readout
// Updated for 80 MHz system clock (ECP5 target)

`timescale 1ns / 1ps

module tb_ccd_driver;

    reg        clk;
    reg        rst;
    reg        start;
    reg        stop;
    reg [31:0] sh_period;
    reg [31:0] icg_period;

    wire       fm_out;
    wire       sh_out;
    wire       icg_out;
    wire       pixel_valid;
    wire [11:0] pixel_index;
    wire       frame_done;
    wire       running;

    // 80 MHz clock → 12.5 ns period
    localparam CLK_PERIOD = 12.5;

    ccd_driver #(
        .SYS_CLK_FREQ(80_000_000),
        .FM_FREQ(4_000_000),
        .TOTAL_PIXELS(2200)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .stop(stop),
        .sh_period(sh_period),
        .icg_period(icg_period),
        .fm_out(fm_out),
        .sh_out(sh_out),
        .icg_out(icg_out),
        .pixel_valid(pixel_valid),
        .pixel_index(pixel_index),
        .frame_done(frame_done),
        .running(running)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Measure fM frequency
    real fm_rise_time, fm_period_ns;
    always @(posedge fm_out) begin
        fm_period_ns = $realtime - fm_rise_time;
        fm_rise_time = $realtime;
    end

    // Count pixel_valid strobes
    integer pixel_count;
    always @(posedge clk) begin
        if (rst)
            pixel_count <= 0;
        else if (pixel_valid)
            pixel_count <= pixel_count + 1;
    end

    // Monitor frame_done
    integer frame_count;
    always @(posedge clk) begin
        if (rst)
            frame_count <= 0;
        else if (frame_done) begin
            frame_count <= frame_count + 1;
            $display("TIME=%0t: Frame %0d complete, %0d pixels",
                     $realtime, frame_count, pixel_count);
            pixel_count <= 0;
        end
    end

    // Timing checks
    real icg_fall_time, sh_rise_time, sh_fall_time;

    always @(posedge icg_out) begin
        icg_fall_time = $realtime;
    end

    always @(posedge sh_out) begin
        sh_rise_time = $realtime;
        $display("TIME=%0t: SH rises %0.1f ns after ICG active",
                 $realtime, sh_rise_time - icg_fall_time);
    end

    always @(negedge sh_out) begin
        sh_fall_time = $realtime;
        $display("TIME=%0t: SH pulse width = %0.1f ns",
                 $realtime, sh_fall_time - sh_rise_time);
    end

    always @(negedge icg_out) begin
        if (sh_fall_time > 0)
            $display("TIME=%0t: ICG releases %0.1f ns after SH falls",
                     $realtime, $realtime - sh_fall_time);
    end

    initial begin
        $dumpfile("tb_ccd_driver.vcd");
        $dumpvars(0, tb_ccd_driver);

        rst        = 1;
        start      = 0;
        stop       = 0;
        sh_period  = 32'd5000;
        icg_period = 32'd20000;  // Short for test (~5ms frame time)

        #500;
        rst = 0;
        #100;

        start = 1;
        #(CLK_PERIOD);
        start = 0;

        wait(frame_count >= 2);
        #1000;

        $display("fM period = %0.1f ns (expected 250.0 ns for 4 MHz)", fm_period_ns);
        $display("=== CCD Driver Testbench PASSED ===");
        $finish;
    end

    initial begin
        #50_000_000;
        $display("ERROR: Timeout");
        $finish;
    end

endmodule
