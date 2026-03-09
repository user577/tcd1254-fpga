// pll_sim.v — Simulation model for PLL (replaces EHXPLLL for iverilog)
//
// Generates 80 MHz from 25 MHz by simple clock division/multiplication.
// NOT synthesizable — simulation only.

`timescale 1ns / 1ps

module pll (
    input  wire clk_25m,
    output reg  clk_sys,
    output reg  locked
);

    // 80 MHz = 12.5 ns period
    initial begin
        clk_sys = 0;
        locked  = 0;
        // Simulate PLL lock time (~10 us)
        #10_000;
        locked = 1;
    end

    // Generate 80 MHz clock
    always #6.25 clk_sys = ~clk_sys;

endmodule
