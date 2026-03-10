// tb_top.v — Full system testbench for Colorlight i9 ECP5 target
// Tests: PLL → CCD driver → ADC capture → frame buffer → shadow detect → UART TX
//        + flash trigger command parsing and GPIO output

`timescale 1ns / 1ps

module tb_top;

    reg        CLK_25M;
    wire       ADC_CLK_PIN, FM_PIN, SH_PIN, ICG_PIN;
    wire       UART_TX;
    reg        UART_RX;
    wire       LED;
    wire       FLASH_0, FLASH_1, FLASH_2, FLASH_3;
    // SPI to RP2040
    reg        SPI_SCK  = 0;
    reg        SPI_MOSI = 0;
    wire       SPI_MISO;
    reg        SPI_CS_N = 1;

    // ADC data pins
    reg        ADC_D0, ADC_D1, ADC_D2, ADC_D3;
    reg        ADC_D4, ADC_D5, ADC_D6, ADC_D7;
    reg        ADC_D8, ADC_D9, ADC_D10, ADC_D11;

    top dut (
        .CLK_25M(CLK_25M),
        .ADC_D0(ADC_D0), .ADC_D1(ADC_D1), .ADC_D2(ADC_D2), .ADC_D3(ADC_D3),
        .ADC_D4(ADC_D4), .ADC_D5(ADC_D5), .ADC_D6(ADC_D6), .ADC_D7(ADC_D7),
        .ADC_D8(ADC_D8), .ADC_D9(ADC_D9), .ADC_D10(ADC_D10), .ADC_D11(ADC_D11),
        .ADC_CLK_PIN(ADC_CLK_PIN),
        .FM_PIN(FM_PIN),
        .SH_PIN(SH_PIN),
        .ICG_PIN(ICG_PIN),
        .UART_TX(UART_TX),
        .UART_RX(UART_RX),
        .FLASH_0(FLASH_0),
        .FLASH_1(FLASH_1),
        .FLASH_2(FLASH_2),
        .FLASH_3(FLASH_3),
        .SPI_SCK(SPI_SCK),
        .SPI_MOSI(SPI_MOSI),
        .SPI_MISO(SPI_MISO),
        .SPI_CS_N(SPI_CS_N),
        .LED(LED)
    );

    // 25 MHz clock → 40 ns period
    localparam CLK_25M_PERIOD = 40.0;
    initial CLK_25M = 0;
    always #(CLK_25M_PERIOD/2) CLK_25M = ~CLK_25M;

    // Synthetic ADC data: drive ramp pattern on D[11:0]
    reg [11:0] adc_synth;
    always @(*) begin
        {ADC_D11, ADC_D10, ADC_D9, ADC_D8,
         ADC_D7,  ADC_D6,  ADC_D5, ADC_D4,
         ADC_D3,  ADC_D2,  ADC_D1, ADC_D0} = adc_synth;
    end

    // Generate ramp on ADC data (changes with ADC_CLK)
    reg [11:0] ramp_counter;
    always @(posedge ADC_CLK_PIN or posedge dut.rst) begin
        if (dut.rst) begin
            ramp_counter <= 0;
            adc_synth    <= 0;
        end else begin
            adc_synth    <= ramp_counter;
            ramp_counter <= ramp_counter + 1;
            if (ramp_counter >= 4095)
                ramp_counter <= 0;
        end
    end

    // UART RX monitor — capture bytes from FPGA TX
    localparam UART_BIT_PERIOD = 1_000_000_000 / 921_600;  // ~1085 ns

    task uart_receive;
        output [7:0] rx_byte;
        integer i;
        begin
            @(negedge UART_TX);
            #(UART_BIT_PERIOD / 2);
            #UART_BIT_PERIOD;
            for (i = 0; i < 8; i = i + 1) begin
                rx_byte[i] = UART_TX;
                #UART_BIT_PERIOD;
            end
            if (UART_TX !== 1'b1)
                $display("WARNING: Missing stop bit");
        end
    endtask

    task uart_send_byte;
        input [7:0] byte_val;
        integer i;
        begin
            UART_RX = 1'b0;
            #UART_BIT_PERIOD;
            for (i = 0; i < 8; i = i + 1) begin
                UART_RX = byte_val[i];
                #UART_BIT_PERIOD;
            end
            UART_RX = 1'b1;
            #UART_BIT_PERIOD;
        end
    endtask

    // Send 'ER' exposure command (12 bytes)
    task send_command;
        input [31:0] sh;
        input [31:0] icg;
        input [7:0]  mode;
        input [7:0]  avg;
        begin
            uart_send_byte(8'h45);
            uart_send_byte(8'h52);
            uart_send_byte(sh[31:24]);
            uart_send_byte(sh[23:16]);
            uart_send_byte(sh[15:8]);
            uart_send_byte(sh[7:0]);
            uart_send_byte(icg[31:24]);
            uart_send_byte(icg[23:16]);
            uart_send_byte(icg[15:8]);
            uart_send_byte(icg[7:0]);
            uart_send_byte(mode);
            uart_send_byte(avg);
        end
    endtask

    // Send 'FL' flash command (8 bytes)
    task send_flash_command;
        input [7:0]  lamp_mask;
        input [15:0] delay_us;
        input [15:0] duration_us;
        input [7:0]  flags;
        begin
            uart_send_byte(8'h46);      // 'F'
            uart_send_byte(8'h4C);      // 'L'
            uart_send_byte(lamp_mask);
            uart_send_byte(delay_us[15:8]);
            uart_send_byte(delay_us[7:0]);
            uart_send_byte(duration_us[15:8]);
            uart_send_byte(duration_us[7:0]);
            uart_send_byte(flags);
        end
    endtask

    // Monitor UART output
    reg [7:0] rx_byte;
    integer rx_count;

    initial begin
        rx_count = 0;
        fork
            begin
                forever begin
                    uart_receive(rx_byte);
                    rx_count = rx_count + 1;
                    if (rx_count <= 10 || rx_count % 100 == 0)
                        $display("TIME=%0t: UART RX byte #%0d = 0x%02X",
                                 $realtime, rx_count, rx_byte);
                end
            end
        join_none
    end

    // ---- SPI master tasks (simulating RP2040) ----
    localparam SPI_HALF = 50;  // 10 MHz SPI clock

    task spi_xfer;
        input  [7:0] tx;
        output [7:0] rx;
        integer i;
        begin
            rx = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                SPI_MOSI = tx[i];
                #SPI_HALF;
                SPI_SCK = 1;
                rx[i] = SPI_MISO;
                #SPI_HALF;
                SPI_SCK = 0;
            end
        end
    endtask

    task spi_send_byte;
        input [7:0] tx;
        reg [7:0] dummy;
        begin
            spi_xfer(tx, dummy);
        end
    endtask

    task spi_read_byte;
        output [7:0] rx;
        begin
            spi_xfer(8'h00, rx);
        end
    endtask

    // Monitor flash GPIO
    always @(posedge FLASH_0)
        $display("TIME=%0t: FLASH_0 ON", $realtime);
    always @(negedge FLASH_0)
        $display("TIME=%0t: FLASH_0 OFF", $realtime);
    always @(posedge FLASH_1)
        $display("TIME=%0t: FLASH_1 ON", $realtime);
    always @(negedge FLASH_1)
        $display("TIME=%0t: FLASH_1 OFF", $realtime);

    // Main test
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        UART_RX   = 1'b1;
        adc_synth = 0;

        $display("=== TCD1254 FPGA Top-Level Testbench (ECP5 + Flash) ===");
        $display("Waiting for PLL lock and auto-start...");

        #5000;

        // Wait for CCD to start
        wait(dut.ccd_running == 1'b1);
        $display("TIME=%0t: CCD running", $realtime);

        // Let it run for a frame in position mode
        #10_000_000;
        $display("TIME=%0t: UART bytes received: %0d", $realtime, rx_count);

        // ---- Test flash command ----
        $display("\n--- Sending flash command: lamp0, 0 delay, 200us ---");
        send_flash_command(8'h01, 16'd0, 16'd200, 8'h02);  // lamp0, raw capture

        // Wait for flash to fire on next frame_done
        #5_000_000;
        $display("TIME=%0t: Flash test complete", $realtime);

        // ---- Test auto-sequence flash ----
        $display("\n--- Sending auto-sequence flash: lamps 0-3 ---");
        send_flash_command(8'h0F, 16'd0, 16'd100, 8'h03);  // auto_seq + raw

        // Wait for several frames to see lamp cycling
        #20_000_000;
        $display("TIME=%0t: Auto-sequence test complete", $realtime);

        // Send raw mode command to verify interleaved ER/FL commands work
        $display("\n--- Sending raw mode command ---");
        send_command(32'd5000, 32'd500000, 8'd2, 8'd1);

        #20_000_000;
        $display("TIME=%0t: Total UART bytes: %0d", $realtime, rx_count);

        // ---- Test SPI interface (simulating RP2040) ----
        $display("\n--- SPI: Reading status ---");
        SPI_CS_N = 0;
        #100;
        spi_send_byte(8'hFE);        // Status command
        begin : spi_status_block
            reg [7:0] spi_rx;
            spi_read_byte(spi_rx);
            $display("  SPI status: 0x%02X", spi_rx);
        end
        SPI_CS_N = 1;
        #500;

        $display("\n--- SPI: Reading shadow ---");
        SPI_CS_N = 0;
        #100;
        spi_send_byte(8'h03);        // Read shadow command
        begin : spi_shadow_block
            reg [7:0] hi, lo;
            spi_read_byte(hi);
            spi_read_byte(lo);
            $display("  SPI shadow: 0x%02X%02X", hi, lo);
        end
        SPI_CS_N = 1;
        #500;

        $display("\n--- SPI: Setting exposure via SPI ---");
        SPI_CS_N = 0;
        #100;
        spi_send_byte(8'h10);        // Write expo command
        // SH = 10 (0x0000000A)
        spi_send_byte(8'h00); spi_send_byte(8'h00);
        spi_send_byte(8'h00); spi_send_byte(8'h0A);
        // ICG = 250000 (0x0003D090)
        spi_send_byte(8'h00); spi_send_byte(8'h03);
        spi_send_byte(8'hD0); spi_send_byte(8'h90);
        // mode=1, avg=2
        spi_send_byte(8'h01); spi_send_byte(8'h02);
        SPI_CS_N = 1;
        #1000;

        $display("  SPI expo config applied (SH=10, ICG=250000)");

        #5_000_000;
        $display("=== Testbench Complete ===");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
