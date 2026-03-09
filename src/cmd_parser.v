// cmd_parser.v — Command parser & mode controller
//
// Accumulates 12-byte commands matching the STM32 protocol:
//   Byte 0:    'E' (0x45)
//   Byte 1:    'R' (0x52)
//   Bytes 2-5: SH period (big-endian uint32)
//   Bytes 6-9: ICG period (big-endian uint32)
//   Byte 10:   mode (1=continuous position, 2=raw data)
//   Byte 11:   averaging count (1-15)
//
// On valid command, outputs new settings and pulses cmd_valid.

module cmd_parser (
    input  wire       clk,
    input  wire       rst,

    // From uart_rx
    input  wire [7:0] rx_data,
    input  wire       rx_valid,

    // Outputs
    output reg [31:0] sh_period,
    output reg [31:0] icg_period,
    output reg [7:0]  mode,          // 1 = position, 2 = raw
    output reg [7:0]  avg_count,     // 1-15
    output reg        cmd_valid      // Pulses when new command received
);

    reg [7:0] cbuf [0:11];
    reg [3:0] byte_idx;

    always @(posedge clk) begin
        if (rst) begin
            byte_idx   <= 0;
            sh_period  <= 32'd20;      // Default SH period
            icg_period <= 32'd500000;  // Default ICG period
            mode       <= 8'd1;        // Default: position mode
            avg_count  <= 8'd4;        // Default averaging
            cmd_valid  <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;

            if (rx_valid) begin
                // Check for sync bytes — reset index if we see 'E' at wrong position
                if (byte_idx == 0) begin
                    if (rx_data == 8'h45) begin  // 'E'
                        cbuf[0]   <= rx_data;
                        byte_idx <= 1;
                    end
                    // else stay at 0
                end else if (byte_idx == 1) begin
                    if (rx_data == 8'h52) begin  // 'R'
                        cbuf[1]   <= rx_data;
                        byte_idx <= 2;
                    end else if (rx_data == 8'h45) begin
                        // Could be new 'E' — restart
                        cbuf[0]   <= rx_data;
                        byte_idx <= 1;
                    end else begin
                        byte_idx <= 0;
                    end
                end else begin
                    cbuf[byte_idx] <= rx_data;
                    if (byte_idx == 11) begin
                        // Full command received — parse it
                        byte_idx <= 0;

                        // Parse SH period (big-endian, bytes 2-5)
                        sh_period <= {cbuf[2], cbuf[3], cbuf[4], cbuf[5]};

                        // Parse ICG period (big-endian, bytes 6-9)
                        icg_period <= {cbuf[6], cbuf[7], cbuf[8], cbuf[9]};

                        // Mode (byte 10)
                        mode <= cbuf[10];

                        // Averaging count (byte 11 = rx_data)
                        if (rx_data > 0 && rx_data < 16)
                            avg_count <= rx_data;
                        else
                            avg_count <= 8'd1;

                        cmd_valid <= 1'b1;
                    end else begin
                        byte_idx <= byte_idx + 1;
                    end
                end
            end
        end
    end

endmodule
