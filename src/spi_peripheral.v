// spi_peripheral.v — SPI slave interface for RP2040 ↔ FPGA communication
//
// SPI Mode 0 (CPOL=0, CPHA=0): sample on rising SCK, shift on falling SCK.
// RP2040 is SPI master, FPGA is peripheral.
//
// Register-mapped interface:
//   Master sends: [cmd_byte] [addr_byte] [data...]
//   Commands:
//     0x01 = Write register (addr, then N data bytes)
//     0x02 = Read register (addr, then master clocks out data)
//     0x03 = Read shadow result (2 bytes)
//     0x04 = Read raw frame (4400 bytes, streams from frame buffer)
//     0x10 = Write exposure config (SH[4] + ICG[4] + mode[1] + avg[1])
//     0x11 = Write flash config (mask[1] + delay[2] + dur[2] + flags[1])
//     0x20 = Trigger single capture (returns when frame ready)
//     0xFE = Status register read (1 byte: bit0=ccd_running, bit1=frame_ready, etc.)
//
// All multi-byte values are big-endian.

module spi_peripheral (
    input  wire        clk,       // System clock (80 MHz)
    input  wire        rst,

    // SPI pins (directly from RP2040)
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_cs_n,  // Active-low chip select

    // Register interface to system
    output reg  [7:0]  reg_addr,
    output reg  [7:0]  reg_wdata,
    output reg         reg_wr,          // Pulse on write
    input  wire [7:0]  reg_rdata,

    // Direct access to key system signals
    output reg         cmd_write_expo,   // Pulse: exposure config written
    output reg         cmd_write_flash,  // Pulse: flash config written
    output reg         cmd_trigger,      // Pulse: single capture trigger
    output reg  [79:0] expo_config,      // SH[32] + ICG[32] + mode[8] + avg[8]
    output reg  [47:0] flash_config,     // mask[8] + delay[16] + dur[16] + flags[8]

    // Frame buffer read port (for raw frame streaming)
    output reg  [11:0] fb_stream_addr,
    input  wire [11:0] fb_stream_data,
    output reg         fb_streaming,

    // Status inputs
    input  wire        ccd_running,
    input  wire        frame_ready,
    input  wire [15:0] shadow_result,
    input  wire        shadow_valid
);

    // ---- Synchronize SPI signals to system clock (2-stage) ----
    reg [2:0] sck_sync;
    reg [1:0] mosi_sync;
    reg [1:0] cs_sync;

    always @(posedge clk) begin
        sck_sync  <= {sck_sync[1:0], spi_sck};
        mosi_sync <= {mosi_sync[0], spi_mosi};
        cs_sync   <= {cs_sync[0], spi_cs_n};
    end

    wire sck_rise = (sck_sync[2:1] == 2'b01);
    wire sck_fall = (sck_sync[2:1] == 2'b10);
    wire mosi_in  = mosi_sync[1];
    wire cs_active = !cs_sync[1];  // Active-low

    // ---- SPI shift register ----
    reg [7:0] shift_in;
    reg [7:0] shift_out;
    reg [2:0] bit_cnt;
    reg       byte_ready;  // Pulses when 8 bits received

    // MISO output (directly from shift register MSB)
    assign spi_miso = cs_active ? shift_out[7] : 1'bz;

    always @(posedge clk) begin
        if (rst || !cs_active) begin
            bit_cnt    <= 0;
            byte_ready <= 0;
        end else begin
            byte_ready <= 0;

            if (sck_rise) begin
                // Sample MOSI
                shift_in <= {shift_in[6:0], mosi_in};
                bit_cnt  <= bit_cnt + 1;
                if (bit_cnt == 3'd7)
                    byte_ready <= 1;
            end

            if (sck_fall) begin
                // Shift out next MISO bit
                shift_out <= {shift_out[6:0], 1'b0};
            end
        end
    end

    // ---- Command state machine ----
    localparam ST_CMD      = 3'd0,  // Waiting for command byte
               ST_ADDR     = 3'd1,  // Waiting for address byte
               ST_WRITE    = 3'd2,  // Receiving write data
               ST_READ     = 3'd3,  // Sending read data
               ST_EXPO_RX  = 3'd4,  // Receiving 10-byte expo config
               ST_FLASH_RX = 3'd5,  // Receiving 6-byte flash config
               ST_STREAM   = 3'd6;  // Streaming frame buffer

    reg [2:0]  cmd_state;
    reg [7:0]  current_cmd;
    reg [3:0]  data_idx;
    reg [7:0]  config_buf [0:9];  // Buffer for multi-byte configs

    always @(posedge clk) begin
        if (rst) begin
            cmd_state      <= ST_CMD;
            reg_wr         <= 0;
            cmd_write_expo <= 0;
            cmd_write_flash <= 0;
            cmd_trigger    <= 0;
            fb_streaming   <= 0;
            fb_stream_addr <= 0;
        end else begin
            reg_wr         <= 0;
            cmd_write_expo <= 0;
            cmd_write_flash <= 0;
            cmd_trigger    <= 0;

            if (!cs_active) begin
                // CS deasserted — reset state
                cmd_state    <= ST_CMD;
                fb_streaming <= 0;
            end else if (byte_ready) begin
                case (cmd_state)
                    ST_CMD: begin
                        current_cmd <= shift_in;
                        data_idx    <= 0;

                        case (shift_in)
                            8'h01: cmd_state <= ST_ADDR;   // Write reg
                            8'h02: begin                    // Read reg
                                cmd_state <= ST_ADDR;
                            end
                            8'h03: begin                    // Read shadow
                                shift_out <= shadow_result[15:8];
                                cmd_state <= ST_READ;
                            end
                            8'h04: begin                    // Read raw frame
                                fb_stream_addr <= 0;
                                fb_streaming   <= 1;
                                cmd_state <= ST_STREAM;
                            end
                            8'h10: begin                    // Write expo config
                                cmd_state <= ST_EXPO_RX;
                            end
                            8'h11: begin                    // Write flash config
                                cmd_state <= ST_FLASH_RX;
                            end
                            8'h20: begin                    // Trigger capture
                                cmd_trigger <= 1;
                            end
                            8'hFE: begin                    // Status read
                                shift_out <= {4'd0, shadow_valid, frame_ready,
                                             ccd_running, fb_streaming};
                                cmd_state <= ST_READ;
                            end
                            default: ;  // Unknown — stay in CMD
                        endcase
                    end

                    ST_ADDR: begin
                        reg_addr <= shift_in;
                        if (current_cmd == 8'h01)
                            cmd_state <= ST_WRITE;
                        else begin
                            // Read: load response
                            cmd_state <= ST_READ;
                            // reg_rdata will be available next cycle
                        end
                    end

                    ST_WRITE: begin
                        reg_wdata <= shift_in;
                        reg_wr    <= 1;
                        reg_addr  <= reg_addr + 1;
                    end

                    ST_READ: begin
                        if (current_cmd == 8'h03) begin
                            // Shadow result byte 2
                            shift_out <= shadow_result[7:0];
                            cmd_state <= ST_CMD;
                        end else begin
                            shift_out <= reg_rdata;
                            reg_addr  <= reg_addr + 1;
                        end
                    end

                    ST_EXPO_RX: begin
                        config_buf[data_idx] <= shift_in;
                        data_idx <= data_idx + 1;
                        if (data_idx == 4'd9) begin
                            // All 10 bytes received
                            expo_config <= {config_buf[0], config_buf[1],
                                           config_buf[2], config_buf[3],
                                           config_buf[4], config_buf[5],
                                           config_buf[6], config_buf[7],
                                           config_buf[8], shift_in};
                            cmd_write_expo <= 1;
                            cmd_state <= ST_CMD;
                        end
                    end

                    ST_FLASH_RX: begin
                        config_buf[data_idx] <= shift_in;
                        data_idx <= data_idx + 1;
                        if (data_idx == 4'd5) begin
                            // All 6 bytes received
                            flash_config <= {config_buf[0], config_buf[1],
                                            config_buf[2], config_buf[3],
                                            config_buf[4], shift_in};
                            cmd_write_flash <= 1;
                            cmd_state <= ST_CMD;
                        end
                    end

                    ST_STREAM: begin
                        // Stream frame buffer — 2 bytes per pixel, 2200 pixels
                        // Send high byte, then low byte
                        if (data_idx[0] == 0) begin
                            shift_out <= {4'd0, fb_stream_data[11:8]};
                            data_idx[0] <= 1;
                        end else begin
                            shift_out <= fb_stream_data[7:0];
                            data_idx[0] <= 0;
                            fb_stream_addr <= fb_stream_addr + 1;
                            if (fb_stream_addr >= 12'd2199) begin
                                fb_streaming <= 0;
                                cmd_state    <= ST_CMD;
                            end
                        end
                    end

                    default: cmd_state <= ST_CMD;
                endcase
            end
        end
    end

endmodule
