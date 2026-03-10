// tb_spi_peripheral.v — Testbench for SPI slave interface
//
// Simulates RP2040 SPI master sending commands to FPGA SPI peripheral.
// Tests: status read, exposure config write, flash config write,
//        shadow result read, trigger capture, raw frame streaming.

`timescale 1ns / 1ps

module tb_spi_peripheral;

    // System clock: 80 MHz → 12.5 ns period
    localparam CLK_PERIOD = 12.5;
    // SPI clock: 10 MHz → 100 ns period (50 ns half)
    localparam SPI_HALF = 50;

    reg        clk = 0;
    reg        rst = 1;

    // SPI signals
    reg        spi_sck  = 0;
    reg        spi_mosi = 0;
    wire       spi_miso;
    reg        spi_cs_n = 1;

    // Register interface
    wire [7:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_wr;
    reg  [7:0] reg_rdata = 8'hA5;

    // Command outputs
    wire       cmd_write_expo;
    wire       cmd_write_flash;
    wire       cmd_trigger;
    wire [79:0] expo_config;
    wire [47:0] flash_config;

    // Frame buffer interface
    wire [11:0] fb_stream_addr;
    reg  [11:0] fb_stream_data = 12'h000;
    wire        fb_streaming;

    // Status inputs
    reg        ccd_running   = 1;
    reg        frame_ready   = 0;
    reg [15:0] shadow_result = 16'h0234;
    reg        shadow_valid  = 1;

    // DUT
    spi_peripheral dut (
        .clk(clk),
        .rst(rst),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .reg_addr(reg_addr),
        .reg_wdata(reg_wdata),
        .reg_wr(reg_wr),
        .reg_rdata(reg_rdata),
        .cmd_write_expo(cmd_write_expo),
        .cmd_write_flash(cmd_write_flash),
        .cmd_trigger(cmd_trigger),
        .expo_config(expo_config),
        .flash_config(flash_config),
        .fb_stream_addr(fb_stream_addr),
        .fb_stream_data(fb_stream_data),
        .fb_streaming(fb_streaming),
        .ccd_running(ccd_running),
        .frame_ready(frame_ready),
        .shadow_result(shadow_result),
        .shadow_valid(shadow_valid)
    );

    // System clock
    always #(CLK_PERIOD/2) clk = ~clk;

    // Provide synthetic frame buffer data (pixel address as data)
    always @(*) begin
        fb_stream_data = fb_stream_addr;
    end

    // ---- SPI Master tasks ----

    // Send one byte over SPI, receive one byte simultaneously
    task spi_transfer;
        input  [7:0] tx_byte;
        output [7:0] rx_byte;
        integer i;
        begin
            rx_byte = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = tx_byte[i];
                #SPI_HALF;
                spi_sck = 1;              // Rising edge — slave samples MOSI
                rx_byte[i] = spi_miso;    // Master samples MISO
                #SPI_HALF;
                spi_sck = 0;              // Falling edge — slave shifts out
            end
        end
    endtask

    // Send a byte and discard response
    task spi_send;
        input [7:0] tx_byte;
        reg [7:0] dummy;
        begin
            spi_transfer(tx_byte, dummy);
        end
    endtask

    // Clock out a byte (send 0x00) and capture response
    task spi_recv;
        output [7:0] rx_byte;
        begin
            spi_transfer(8'h00, rx_byte);
        end
    endtask

    // ---- Test ----
    reg [7:0] rx;
    reg [7:0] rx2;

    initial begin
        $dumpfile("tb_spi_peripheral.vcd");
        $dumpvars(0, tb_spi_peripheral);

        $display("=== SPI Peripheral Testbench ===");

        // Reset
        #200;
        rst = 0;
        #100;

        // ---- Test 1: Status read (cmd 0xFE) ----
        $display("\n--- Test 1: Status read ---");
        ccd_running  = 1;
        frame_ready  = 1;
        shadow_valid = 1;
        #100;

        spi_cs_n = 0;
        #100;
        spi_send(8'hFE);       // Command: status read
        spi_recv(rx);          // Read status byte
        spi_cs_n = 1;
        #200;

        // Expected: bit0=fb_streaming(0), bit1=ccd_running(1), bit2=frame_ready(1), bit3=shadow_valid(1)
        // = 0b00001110 = 0x0E
        $display("  Status byte: 0x%02X (expected 0x0E)", rx);
        if (rx !== 8'h0E) $display("  ** MISMATCH **");

        // ---- Test 2: Shadow read (cmd 0x03) ----
        $display("\n--- Test 2: Shadow read ---");
        shadow_result = 16'h1234;
        #100;

        spi_cs_n = 0;
        #100;
        spi_send(8'h03);       // Command: read shadow
        spi_recv(rx);          // High byte
        spi_recv(rx2);         // Low byte
        spi_cs_n = 1;
        #200;

        $display("  Shadow result: 0x%02X%02X (expected 0x1234)", rx, rx2);
        if ({rx, rx2} !== 16'h1234) $display("  ** MISMATCH **");

        // ---- Test 3: Write exposure config (cmd 0x10, 10 bytes) ----
        $display("\n--- Test 3: Write exposure config ---");
        spi_cs_n = 0;
        #100;
        spi_send(8'h10);       // Command: write expo
        // SH = 0x00000014 (20)
        spi_send(8'h00); spi_send(8'h00); spi_send(8'h00); spi_send(8'h14);
        // ICG = 0x0007A120 (500000)
        spi_send(8'h00); spi_send(8'h07); spi_send(8'hA1); spi_send(8'h20);
        // mode = 0x01, avg = 0x04
        spi_send(8'h01); spi_send(8'h04);
        spi_cs_n = 1;
        #200;

        $display("  cmd_write_expo fired: %b", cmd_write_expo);
        $display("  expo_config: 0x%020X", expo_config);
        // Expected: 0x00000014_0007A120_01_04
        // = 80'h000000140007A12010104 (but we need to check the latched value)
        #100;

        // ---- Test 4: Write flash config (cmd 0x11, 6 bytes) ----
        $display("\n--- Test 4: Write flash config ---");
        spi_cs_n = 0;
        #100;
        spi_send(8'h11);       // Command: write flash
        spi_send(8'h0F);       // lamp_mask = 0x0F (lamps 0-3)
        spi_send(8'h00);       // delay_us high
        spi_send(8'h64);       // delay_us low = 100
        spi_send(8'h01);       // duration_us high
        spi_send(8'hF4);       // duration_us low = 500
        spi_send(8'h03);       // flags = auto_seq + raw
        spi_cs_n = 1;
        #200;

        $display("  cmd_write_flash fired");
        $display("  flash_config: 0x%012X", flash_config);

        // ---- Test 5: Trigger capture (cmd 0x20) ----
        $display("\n--- Test 5: Trigger capture ---");
        spi_cs_n = 0;
        #100;
        spi_send(8'h20);       // Command: trigger
        spi_cs_n = 1;
        #200;

        // cmd_trigger should have pulsed
        $display("  Trigger sent");

        // ---- Test 6: Read raw frame start (cmd 0x04, first 4 pixels) ----
        $display("\n--- Test 6: Frame stream (first 4 pixels) ---");
        spi_cs_n = 0;
        #100;
        spi_send(8'h04);       // Command: read frame

        // Read 4 pixels (8 bytes — 2 per pixel)
        begin : frame_read_block
            integer p;
            reg [7:0] hi, lo;
            for (p = 0; p < 4; p = p + 1) begin
                spi_recv(hi);
                spi_recv(lo);
                $display("  Pixel[%0d]: hi=0x%02X lo=0x%02X → %0d",
                         p, hi, lo, {hi[3:0], lo});
            end
        end
        spi_cs_n = 1;
        #200;

        $display("  fb_streaming after CS deassert: %b (expected 0)", fb_streaming);

        // ---- Test 7: Register write (cmd 0x01) ----
        $display("\n--- Test 7: Register write ---");
        spi_cs_n = 0;
        #100;
        spi_send(8'h01);       // Command: write register
        spi_send(8'h42);       // Address = 0x42
        spi_send(8'hBE);       // Data = 0xBE
        spi_send(8'hEF);       // Data = 0xEF (auto-increment to addr 0x43)
        spi_cs_n = 1;
        #200;

        $display("  Register write test complete");

        // ---- Done ----
        #1000;
        $display("\n=== SPI Peripheral Testbench Complete ===");
        $finish;
    end

    // Monitor command pulses
    always @(posedge clk) begin
        if (cmd_write_expo)
            $display("TIME=%0t: cmd_write_expo pulse", $realtime);
        if (cmd_write_flash)
            $display("TIME=%0t: cmd_write_flash pulse", $realtime);
        if (cmd_trigger)
            $display("TIME=%0t: cmd_trigger pulse", $realtime);
        if (reg_wr)
            $display("TIME=%0t: reg_wr addr=0x%02X data=0x%02X",
                     $realtime, reg_addr, reg_wdata);
    end

    // Timeout
    initial begin
        #500_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
