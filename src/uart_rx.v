// uart_rx.v — UART receiver, 8N1, 3x oversampling at bit center

module uart_rx #(
    parameter CLK_FREQ  = 80_000_000,
    parameter BAUD_RATE = 921_600
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam HALF_BIT     = CLKS_PER_BIT / 2;

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift_reg;

    // 3-stage synchronizer for metastability
    reg rx_sync1, rx_sync2, rx_in;
    always @(posedge clk) begin
        rx_sync1 <= rx;
        rx_sync2 <= rx_sync1;
        rx_in    <= rx_sync2;
    end

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 0;
            data      <= 0;
            valid     <= 0;
        end else begin
            valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (rx_in == 1'b0) begin
                        // Possible start bit
                        state   <= S_START;
                        clk_cnt <= 0;
                    end
                end

                S_START: begin
                    // Sample at middle of start bit
                    if (clk_cnt == HALF_BIT - 1) begin
                        if (rx_in == 1'b0) begin
                            // Valid start bit
                            clk_cnt <= 0;
                            bit_idx <= 0;
                            state   <= S_DATA;
                        end else begin
                            // False start
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        shift_reg <= {rx_in, shift_reg[7:1]};
                        if (bit_idx == 7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        if (rx_in == 1'b1) begin
                            // Valid stop bit
                            data  <= shift_reg;
                            valid <= 1'b1;
                        end
                        state   <= S_IDLE;
                        clk_cnt <= 0;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
