// tb_top.v — Full system testbench for Colorlight i9 ECP5 target
// Tests: PLL → CCD driver → ADC capture → frame buffer → shadow detect → UART TX

`timescale 1ns / 1ps

module tb_top;

    reg        CLK_25M;
    wire       ADC_CLK_PIN, FM_PIN, SH_PIN, ICG_PIN;
    wire       UART_TX;
    reg        UART_RX;
    wire       LED;

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
                    $display("TIME=%0t: UART RX byte #%0d = 0x%02X",
                             $realtime, rx_count, rx_byte);
                end
            end
        join_none
    end

    // Main test
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        UART_RX   = 1'b1;
        adc_synth = 0;

        $display("=== TCD1254 FPGA Top-Level Testbench (ECP5) ===");
        $display("Waiting for PLL lock and auto-start...");

        #5000;

        // Wait for CCD to start
        wait(dut.ccd_running == 1'b1);
        $display("TIME=%0t: CCD running", $realtime);

        // Let it run for a few frames
        #10_000_000;
        $display("TIME=%0t: UART bytes received: %0d", $realtime, rx_count);

        // Send raw mode command
        $display("Sending raw mode command...");
        send_command(32'd5000, 32'd500000, 8'd2, 8'd1);

        #20_000_000;
        $display("TIME=%0t: Total UART bytes: %0d", $realtime, rx_count);

        $display("=== Testbench Complete ===");
        $finish;
    end

    initial begin
        #100_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
