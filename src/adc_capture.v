// adc_capture.v — Captures AD9226 12-bit parallel data on pixel_valid strobe
//
// The pixel_valid strobe from ccd_driver already accounts for ADC pipeline delay.
// We simply latch D[11:0] and present it with a write-enable and address.

module adc_capture #(
    parameter DATA_WIDTH  = 12,
    parameter TOTAL_PIXELS = 2200
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire [DATA_WIDTH-1:0] adc_data,      // AD9226 D[11:0]
    input  wire                  pixel_valid,    // From ccd_driver
    input  wire [11:0]           pixel_index,    // From ccd_driver (includes pipeline offset)

    output reg  [DATA_WIDTH-1:0] pixel_data,     // Captured pixel value
    output reg  [11:0]           pixel_addr,     // Write address (0 to TOTAL_PIXELS-1)
    output reg                   pixel_wr_en     // Write enable pulse
);

    // ADC pipeline offset already handled in ccd_driver's pixel_index
    localparam ADC_PIPELINE = 3;

    always @(posedge clk) begin
        if (rst) begin
            pixel_data  <= 0;
            pixel_addr  <= 0;
            pixel_wr_en <= 1'b0;
        end else begin
            pixel_wr_en <= 1'b0;

            if (pixel_valid) begin
                pixel_data  <= adc_data;
                pixel_addr  <= pixel_index - ADC_PIPELINE;
                pixel_wr_en <= 1'b1;
            end
        end
    end

endmodule
