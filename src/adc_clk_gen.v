// adc_clk_gen.v — ADC clock generation at fM/4 (pixel rate)
//
// Generates 1 MHz clock (at 4 MHz fM) for AD9226.
// Phase-shifted to sample after analog output has settled.
// At 80 MHz sys_clk: pixel period = 80 clocks.

module adc_clk_gen #(
    parameter SYS_CLK_FREQ = 80_000_000,
    parameter FM_FREQ      = 4_000_000,
    parameter PHASE_OFFSET = 40          // sys_clks delay from pixel boundary
)(
    input  wire clk,
    input  wire rst,
    input  wire enable,        // Only generate clock during readout
    output reg  adc_clk
);

    localparam PIXEL_PERIOD = SYS_CLK_FREQ / (FM_FREQ / 4);  // 80 @ 80MHz/1MHz
    localparam HALF_PIXEL   = PIXEL_PERIOD / 2;                // 40

    reg [7:0] cnt;
    reg       enabled_sync;

    always @(posedge clk) begin
        if (rst) begin
            cnt          <= 0;
            adc_clk      <= 1'b0;
            enabled_sync <= 1'b0;
        end else begin
            enabled_sync <= enable;

            if (!enabled_sync) begin
                cnt     <= 0;
                adc_clk <= 1'b0;
            end else begin
                if (cnt == PIXEL_PERIOD - 1) begin
                    cnt <= 0;
                end else begin
                    cnt <= cnt + 1;
                end

                // ADC_CLK rising edge at PHASE_OFFSET, falling at PHASE_OFFSET + HALF_PIXEL
                // Use modular comparison to handle wrap-around
                if (cnt == PHASE_OFFSET - 1)
                    adc_clk <= 1'b1;
                else if (cnt == ((PHASE_OFFSET + HALF_PIXEL - 1) % PIXEL_PERIOD))
                    adc_clk <= 1'b0;
            end
        end
    end

endmodule
