// cmd_parser.v — Command parser & mode controller
//
// Supports two command types:
//
// Exposure command (12 bytes):
//   Byte 0:    'E' (0x45)
//   Byte 1:    'R' (0x52)
//   Bytes 2-5: SH period (big-endian uint32)
//   Bytes 6-9: ICG period (big-endian uint32)
//   Byte 10:   mode (1=position, 2=raw)
//   Byte 11:   averaging count (1-15)
//
// Flash command (8 bytes):
//   Byte 0:    'F' (0x46)
//   Byte 1:    'L' (0x4C)
//   Byte 2:    lamp_mask (bitmask of lamps to fire)
//   Bytes 3-4: flash_delay_us (big-endian uint16)
//   Bytes 5-6: flash_duration_us (big-endian uint16)
//   Byte 7:    flags (bit0=auto_sequence, bit1=raw_capture)
//
// On valid command, outputs new settings and pulses the appropriate valid signal.

module cmd_parser (
    input  wire       clk,
    input  wire       rst,

    // From uart_rx
    input  wire [7:0] rx_data,
    input  wire       rx_valid,

    // Exposure command outputs
    output reg [31:0] sh_period,
    output reg [31:0] icg_period,
    output reg [7:0]  mode,          // 1 = position, 2 = raw
    output reg [7:0]  avg_count,     // 1-15
    output reg        cmd_valid,     // Pulses when new exposure command received

    // Flash command outputs
    output reg [7:0]  flash_lamp_mask,
    output reg [15:0] flash_delay_us,
    output reg [15:0] flash_duration_us,
    output reg        flash_auto_seq,
    output reg        flash_raw_capture,
    output reg        flash_enabled,
    output reg        flash_cmd_valid  // Pulses when new flash command received
);

    // Unified receive buffer — large enough for the longer command (12 bytes)
    reg [7:0] cbuf [0:11];
    reg [3:0] byte_idx;

    // Track which command type we're receiving
    localparam CMD_NONE  = 2'd0,
               CMD_EXPO  = 2'd1,  // 'ER' — 12 bytes
               CMD_FLASH = 2'd2;  // 'FL' — 8 bytes

    reg [1:0] cmd_type;

    always @(posedge clk) begin
        if (rst) begin
            byte_idx          <= 0;
            cmd_type          <= CMD_NONE;
            sh_period         <= 32'd20;
            icg_period        <= 32'd500000;
            mode              <= 8'd1;
            avg_count         <= 8'd4;
            cmd_valid         <= 1'b0;
            flash_lamp_mask   <= 8'd0;
            flash_delay_us    <= 16'd0;
            flash_duration_us <= 16'd500;
            flash_auto_seq    <= 1'b0;
            flash_raw_capture <= 1'b0;
            flash_enabled     <= 1'b0;
            flash_cmd_valid   <= 1'b0;
        end else begin
            cmd_valid       <= 1'b0;
            flash_cmd_valid <= 1'b0;

            if (rx_valid) begin
                if (byte_idx == 0) begin
                    // First byte — detect command type
                    if (rx_data == 8'h45) begin        // 'E' — exposure
                        cbuf[0]  <= rx_data;
                        byte_idx <= 1;
                        cmd_type <= CMD_EXPO;
                    end else if (rx_data == 8'h46) begin // 'F' — flash
                        cbuf[0]  <= rx_data;
                        byte_idx <= 1;
                        cmd_type <= CMD_FLASH;
                    end
                    // else stay at 0
                end else if (byte_idx == 1) begin
                    // Second byte — verify command header
                    if (cmd_type == CMD_EXPO && rx_data == 8'h52) begin  // 'R'
                        cbuf[1]  <= rx_data;
                        byte_idx <= 2;
                    end else if (cmd_type == CMD_FLASH && rx_data == 8'h4C) begin // 'L'
                        cbuf[1]  <= rx_data;
                        byte_idx <= 2;
                    end else if (rx_data == 8'h45) begin
                        // Could be new 'E' — restart as exposure
                        cbuf[0]  <= rx_data;
                        byte_idx <= 1;
                        cmd_type <= CMD_EXPO;
                    end else if (rx_data == 8'h46) begin
                        // Could be new 'F' — restart as flash
                        cbuf[0]  <= rx_data;
                        byte_idx <= 1;
                        cmd_type <= CMD_FLASH;
                    end else begin
                        byte_idx <= 0;
                        cmd_type <= CMD_NONE;
                    end
                end else begin
                    cbuf[byte_idx] <= rx_data;

                    if (cmd_type == CMD_EXPO && byte_idx == 11) begin
                        // Full exposure command — parse
                        byte_idx <= 0;
                        cmd_type <= CMD_NONE;

                        sh_period  <= {cbuf[2], cbuf[3], cbuf[4], cbuf[5]};
                        icg_period <= {cbuf[6], cbuf[7], cbuf[8], cbuf[9]};
                        mode       <= cbuf[10];

                        if (rx_data > 0 && rx_data < 16)
                            avg_count <= rx_data;
                        else
                            avg_count <= 8'd1;

                        cmd_valid <= 1'b1;

                    end else if (cmd_type == CMD_FLASH && byte_idx == 7) begin
                        // Full flash command — parse
                        byte_idx <= 0;
                        cmd_type <= CMD_NONE;

                        flash_lamp_mask   <= cbuf[2];
                        flash_delay_us    <= {cbuf[3], cbuf[4]};
                        flash_duration_us <= {cbuf[5], cbuf[6]};

                        // flags = rx_data (byte 7)
                        flash_auto_seq    <= rx_data[0];
                        flash_raw_capture <= rx_data[1];

                        // Enable flash if any lamps selected
                        flash_enabled     <= (cbuf[2] != 8'd0);
                        flash_cmd_valid   <= 1'b1;

                    end else begin
                        byte_idx <= byte_idx + 1;
                    end
                end
            end
        end
    end

endmodule
