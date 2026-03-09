// top.v — Top-level integration for TCD1254 FPGA CCD readout
// Target: Colorlight i9 v7.2 (LFE5U-45F-6BG381C)
//
// 25 MHz oscillator → PLL → 80 MHz system clock
// GPIO via J1/J2 HUB75 connectors for ADC + CCD control
// UART via J3 GPIO pins to external USB-UART adapter

module top (
    input  wire       CLK_25M,      // 25 MHz oscillator (P6)

    // J1 connector: ADC data bits D[7:0]
    input  wire       ADC_D0,       // J1_R0
    input  wire       ADC_D1,       // J1_G0
    input  wire       ADC_D2,       // J1_B0
    input  wire       ADC_D3,       // J1_R1
    input  wire       ADC_D4,       // J1_G1
    input  wire       ADC_D5,       // J1_B1
    input  wire       ADC_D6,       // J1_A
    input  wire       ADC_D7,       // J1_B_CTRL

    // J2 connector: ADC data bits D[11:8] + control outputs
    input  wire       ADC_D8,       // J2_R0
    input  wire       ADC_D9,       // J2_G0
    input  wire       ADC_D10,      // J2_B0
    input  wire       ADC_D11,      // J2_R1
    output wire       ADC_CLK_PIN,  // J2_G1 — clock to AD9226
    output wire       FM_PIN,       // J2_B1 — fM to 74HCT04 → CCD
    output wire       SH_PIN,       // J2_A  — SH to 74HCT04 → CCD
    output wire       ICG_PIN,      // J2_B_CTRL — ICG to 74HCT04 → CCD

    // J3 connector: UART
    output wire       UART_TX,      // J3_R0 — FPGA → host
    input  wire       UART_RX,      // J3_G0 — host → FPGA

    // Onboard LED (active-low, shared with ADC_D9 pin — directly drive unused)
    output wire       LED
);

    // ========== Parameters ==========
    localparam SYS_CLK_FREQ = 80_000_000;
    localparam FM_FREQ      = 4_000_000;
    localparam BAUD_RATE    = 921_600;
    localparam TOTAL_PIXELS = 2200;

    // ========== PLL ==========
    wire clk;        // 80 MHz system clock
    wire pll_locked;

    pll pll_inst (
        .clk_25m(CLK_25M),
        .clk_sys(clk),
        .locked(pll_locked)
    );

    // ========== Power-on Reset ==========
    reg [7:0] rst_shift = 8'hFF;
    wire rst = rst_shift[7];

    always @(posedge clk) begin
        if (!pll_locked)
            rst_shift <= 8'hFF;
        else
            rst_shift <= {rst_shift[6:0], 1'b0};
    end

    // ========== ADC Data Bus ==========
    wire [11:0] adc_data_pins = {ADC_D11, ADC_D10, ADC_D9, ADC_D8,
                                  ADC_D7,  ADC_D6,  ADC_D5, ADC_D4,
                                  ADC_D3,  ADC_D2,  ADC_D1, ADC_D0};

    // Synchronize ADC data to system clock (2-stage)
    reg [11:0] adc_data_sync1, adc_data;
    always @(posedge clk) begin
        adc_data_sync1 <= adc_data_pins;
        adc_data       <= adc_data_sync1;
    end

    // ========== Forward Declarations ==========
    wire        cmd_valid_pulse;
    reg [7:0]   current_mode;
    reg [3:0]   auto_start_cnt;
    reg         auto_start;

    // ========== CCD Driver ==========
    wire        fm_out, sh_out, icg_out;
    wire        pixel_valid;
    wire [11:0] pixel_index;
    wire        frame_done;
    wire        ccd_running;
    wire [31:0] sh_period_w, icg_period_w;

    // Restart CCD on new command: stop current, then re-start next cycle
    reg cmd_restart;
    always @(posedge clk) begin
        if (rst)
            cmd_restart <= 1'b0;
        else
            cmd_restart <= cmd_valid_pulse;  // Delayed 1 cycle after stop
    end

    ccd_driver #(
        .SYS_CLK_FREQ(SYS_CLK_FREQ),
        .FM_FREQ(FM_FREQ),
        .TOTAL_PIXELS(TOTAL_PIXELS)
    ) ccd_inst (
        .clk(clk),
        .rst(rst),
        .start(cmd_restart | auto_start),
        .stop(cmd_valid_pulse),
        .sh_period(sh_period_w),
        .icg_period(icg_period_w),
        .fm_out(fm_out),
        .sh_out(sh_out),
        .icg_out(icg_out),
        .pixel_valid(pixel_valid),
        .pixel_index(pixel_index),
        .frame_done(frame_done),
        .running(ccd_running)
    );

    assign FM_PIN  = fm_out;
    assign SH_PIN  = sh_out;
    assign ICG_PIN = icg_out;

    // ========== ADC Clock Generator ==========
    wire adc_clk_out;

    adc_clk_gen #(
        .SYS_CLK_FREQ(SYS_CLK_FREQ),
        .FM_FREQ(FM_FREQ)
    ) adc_clk_inst (
        .clk(clk),
        .rst(rst),
        .enable(ccd_running),
        .adc_clk(adc_clk_out)
    );

    assign ADC_CLK_PIN = adc_clk_out;

    // ========== ADC Capture ==========
    wire [11:0] cap_pixel_data;
    wire [11:0] cap_pixel_addr;
    wire        cap_wr_en;

    adc_capture #(
        .TOTAL_PIXELS(TOTAL_PIXELS)
    ) adc_cap_inst (
        .clk(clk),
        .rst(rst),
        .adc_data(adc_data),
        .pixel_valid(pixel_valid),
        .pixel_index(pixel_index),
        .pixel_data(cap_pixel_data),
        .pixel_addr(cap_pixel_addr),
        .pixel_wr_en(cap_wr_en)
    );

    // ========== Frame Buffer ==========
    // ECP5 has 108 × 18Kbit EBR blocks = ~1.9 Mbit BRAM
    // Double-buffer 2200×12 = 52,800 bits → uses ~3 EBR blocks per buffer = 6 total
    wire [11:0] fb_rd_addr;
    wire [11:0] fb_rd_data;

    // Mux read address between shadow_detect and frame_tx
    wire [11:0] shadow_rd_addr;
    wire [11:0] tx_rd_addr;
    wire        shadow_active;

    assign fb_rd_addr = shadow_active ? shadow_rd_addr : tx_rd_addr;

    // Delay swap by 1 cycle so the final pixel write (through adc_capture's
    // pipeline register) completes before the buffer flips.
    reg frame_done_d;
    always @(posedge clk) begin
        if (rst)
            frame_done_d <= 1'b0;
        else
            frame_done_d <= frame_done;
    end

    frame_buffer #(
        .TOTAL_PIXELS(TOTAL_PIXELS)
    ) fb_inst (
        .clk(clk),
        .rst(rst),
        .wr_data(cap_pixel_data),
        .wr_addr(cap_pixel_addr),
        .wr_en(cap_wr_en),
        .rd_addr(fb_rd_addr),
        .rd_data(fb_rd_data),
        .swap(frame_done_d)
    );

    // ========== Shadow Detect ==========
    wire [15:0] shadow_result;
    wire        shadow_done;

    // Use frame_done_d (after swap) to trigger processing on the correct buffer
    shadow_detect #(
        .TOTAL_PIXELS(TOTAL_PIXELS)
    ) shadow_inst (
        .clk(clk),
        .rst(rst),
        .start(frame_done_d & (current_mode == 8'd1)),
        .pixel_data(fb_rd_data),
        .rd_addr(shadow_rd_addr),
        .result(shadow_result),
        .done(shadow_done)
    );

    // Shadow is active from swap until shadow_done (in position mode)
    reg shadow_active_reg;
    always @(posedge clk) begin
        if (rst)
            shadow_active_reg <= 1'b0;
        else if (frame_done_d && current_mode == 8'd1)
            shadow_active_reg <= 1'b1;
        else if (shadow_done)
            shadow_active_reg <= 1'b0;
    end
    assign shadow_active = shadow_active_reg;

    // ========== UART TX ==========
    wire       uart_tx_line;
    wire       uart_tx_busy;
    wire [7:0] uart_tx_data;
    wire       uart_tx_start;

    uart_tx #(
        .CLK_FREQ(SYS_CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_tx_inst (
        .clk(clk),
        .rst(rst),
        .data(uart_tx_data),
        .start(uart_tx_start),
        .tx(uart_tx_line),
        .busy(uart_tx_busy)
    );

    assign UART_TX = uart_tx_line;

    // ========== UART RX ==========
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;

    uart_rx #(
        .CLK_FREQ(SYS_CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_rx_inst (
        .clk(clk),
        .rst(rst),
        .rx(UART_RX),
        .data(uart_rx_data),
        .valid(uart_rx_valid)
    );

    // ========== Command Parser ==========
    wire [31:0] cmd_sh_period;
    wire [31:0] cmd_icg_period;
    wire [7:0]  cmd_mode;
    wire [7:0]  cmd_avg;

    cmd_parser cmd_inst (
        .clk(clk),
        .rst(rst),
        .rx_data(uart_rx_data),
        .rx_valid(uart_rx_valid),
        .sh_period(cmd_sh_period),
        .icg_period(cmd_icg_period),
        .mode(cmd_mode),
        .avg_count(cmd_avg),
        .cmd_valid(cmd_valid_pulse)
    );

    // Latch current mode/settings
    reg [31:0] current_sh, current_icg;

    always @(posedge clk) begin
        if (rst) begin
            current_mode <= 8'd1;
            current_sh   <= 32'd20;
            current_icg  <= 32'd500000;
        end else if (cmd_valid_pulse) begin
            current_mode <= cmd_mode;
            current_sh   <= cmd_sh_period;
            current_icg  <= cmd_icg_period;
        end
    end

    assign sh_period_w  = current_sh;
    assign icg_period_w = current_icg;

    // ========== Frame TX ==========
    wire ftx_transmitting;

    frame_tx #(
        .TOTAL_PIXELS(TOTAL_PIXELS)
    ) frame_tx_inst (
        .clk(clk),
        .rst(rst),
        .mode(current_mode),
        .frame_ready(frame_done_d),
        .shadow_pos(shadow_result),
        .shadow_done(shadow_done),
        .fb_rd_addr(tx_rd_addr),
        .fb_rd_data(fb_rd_data),
        .tx_data(uart_tx_data),
        .tx_start(uart_tx_start),
        .tx_busy(uart_tx_busy),
        .transmitting(ftx_transmitting)
    );

    // ========== Auto-start on power-up ==========
    always @(posedge clk) begin
        if (rst) begin
            auto_start_cnt <= 0;
            auto_start     <= 1'b0;
        end else begin
            auto_start <= 1'b0;
            if (!ccd_running && auto_start_cnt < 4'd10) begin
                auto_start_cnt <= auto_start_cnt + 1;
                if (auto_start_cnt == 4'd9)
                    auto_start <= 1'b1;
            end
        end
    end

    // ========== LED Status ==========
    // Active-low LED on Colorlight i9: show heartbeat when running
    reg [25:0] heartbeat;
    always @(posedge clk) begin
        if (rst)
            heartbeat <= 0;
        else
            heartbeat <= heartbeat + 1;
    end

    // LED off when CCD running (active-low), blink at ~1.2 Hz when idle
    assign LED = ccd_running ? 1'b0 : heartbeat[25];

endmodule
