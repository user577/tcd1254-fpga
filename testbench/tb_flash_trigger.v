// tb_flash_trigger.v — Testbench for flash lamp trigger module
`timescale 1ns / 1ps

module tb_flash_trigger;

    reg        clk = 0;
    reg        rst = 1;
    reg [7:0]  lamp_mask;
    reg [15:0] flash_delay_us;
    reg [15:0] flash_duration_us;
    reg        auto_sequence;
    reg        flash_enabled;
    reg        frame_done;

    wire [7:0] flash_out;
    wire [2:0] current_lamp_id;

    // 80 MHz clock (12.5 ns period)
    always #6.25 clk = ~clk;

    flash_trigger #(
        .SYS_CLK_FREQ(80_000_000)
    ) dut (
        .clk(clk),
        .rst(rst),
        .lamp_mask(lamp_mask),
        .flash_delay_us(flash_delay_us),
        .flash_duration_us(flash_duration_us),
        .auto_sequence(auto_sequence),
        .flash_enabled(flash_enabled),
        .frame_done(frame_done),
        .flash_out(flash_out),
        .current_lamp_id(current_lamp_id)
    );

    // Task: pulse frame_done for 1 clock
    task trigger_frame;
        begin
            @(posedge clk);
            frame_done = 1;
            @(posedge clk);
            frame_done = 0;
        end
    endtask

    // Task: wait N microseconds
    task wait_us(input integer n);
        begin
            #(n * 1000);
        end
    endtask

    integer i;
    integer test_pass;

    initial begin
        $dumpfile("tb_flash_trigger.vcd");
        $dumpvars(0, tb_flash_trigger);

        // Initialize
        lamp_mask        = 8'h00;
        flash_delay_us   = 16'd0;
        flash_duration_us = 16'd500;
        auto_sequence    = 0;
        flash_enabled    = 0;
        frame_done       = 0;
        test_pass        = 1;

        // Release reset
        #200;
        rst = 0;
        #100;

        // ====== Test 1: Single lamp, no delay ======
        $display("\n=== Test 1: Single lamp, no delay, 500us pulse ===");
        lamp_mask        = 8'h01;  // Lamp 0 only
        flash_delay_us   = 16'd0;
        flash_duration_us = 16'd500;
        flash_enabled    = 1;
        auto_sequence    = 0;

        // Let the registered multiply settle
        #200;
        trigger_frame;

        // Flash should fire within a few clocks
        #500;
        if (flash_out[0] !== 1'b1) begin
            $display("FAIL: Lamp 0 did not fire");
            test_pass = 0;
        end else begin
            $display("PASS: Lamp 0 fired");
        end

        if (flash_out[7:1] !== 7'd0) begin
            $display("FAIL: Other lamps fired unexpectedly: %b", flash_out);
            test_pass = 0;
        end

        // Wait for pulse to end (500us = 500000ns)
        #500_000;
        #1000;
        if (flash_out[0] !== 1'b0) begin
            $display("FAIL: Lamp 0 still active after duration");
            test_pass = 0;
        end else begin
            $display("PASS: Lamp 0 turned off after ~500us");
        end

        // ====== Test 2: All lamps fire simultaneously ======
        $display("\n=== Test 2: All 4 lamps, simultaneous ===");
        lamp_mask = 8'h0F;  // Lamps 0-3
        #200;
        trigger_frame;
        #500;

        if (flash_out[3:0] !== 4'hF) begin
            $display("FAIL: Not all lamps fired: %b", flash_out[3:0]);
            test_pass = 0;
        end else begin
            $display("PASS: All 4 lamps fired simultaneously");
        end

        #500_000;
        #1000;

        // ====== Test 3: Auto-sequence (one lamp per frame) ======
        $display("\n=== Test 3: Auto-sequence through 4 lamps ===");
        lamp_mask     = 8'h0F;
        auto_sequence = 1;
        #200;

        for (i = 0; i < 4; i = i + 1) begin
            trigger_frame;
            #500;
            $display("  Frame %0d: flash_out=%b, lamp_id=%0d", i, flash_out, current_lamp_id);
            if (current_lamp_id !== i[2:0]) begin
                $display("  FAIL: Expected lamp %0d, got %0d", i, current_lamp_id);
                test_pass = 0;
            end
            // Wait for pulse to finish
            #500_000;
            #2000;
        end

        // Verify it wraps around
        trigger_frame;
        #500;
        if (current_lamp_id !== 3'd0) begin
            $display("FAIL: Sequence did not wrap to lamp 0 (got %0d)", current_lamp_id);
            test_pass = 0;
        end else begin
            $display("PASS: Sequence wrapped to lamp 0");
        end

        #500_000;
        #2000;

        // ====== Test 4: Delayed flash ======
        $display("\n=== Test 4: Flash with 100us delay ===");
        lamp_mask      = 8'h01;
        auto_sequence  = 0;
        flash_delay_us = 16'd100;
        #200;

        trigger_frame;
        #500;

        // Should NOT be firing yet (delay hasn't elapsed)
        if (flash_out[0] !== 1'b0) begin
            $display("FAIL: Lamp fired before delay elapsed");
            test_pass = 0;
        end else begin
            $display("PASS: Lamp not fired during delay period");
        end

        // Wait for delay (100us = 100000ns)
        #100_000;
        #500;

        if (flash_out[0] !== 1'b1) begin
            $display("FAIL: Lamp did not fire after delay");
            test_pass = 0;
        end else begin
            $display("PASS: Lamp fired after ~100us delay");
        end

        #500_000;
        #2000;

        // ====== Summary ======
        $display("\n========================================");
        if (test_pass)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $display("========================================\n");

        $finish;
    end

endmodule
