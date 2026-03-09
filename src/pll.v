// pll.v — Clock synthesis for ECP5 (Colorlight i9 v7.2)
// 25 MHz input → 80 MHz system clock via EHXPLLL
//
// Parameters: CLKI_DIV=5, CLKFB_DIV=16, CLKOP_DIV=5
//   With FEEDBK_PATH="CLKOP": CLKOP = (CLKI/CLKI_DIV) × CLKFB_DIV = (25/5)×16 = 80 MHz
//   VCO = CLKOP × CLKOP_DIV = 80 × 5 = 400 MHz (range: 400-800 MHz for -6 speed grade)
//   PFD = 25/5 = 5 MHz (range: 3.125-400 MHz)
//
// 80 MHz gives clean division for fM = 4 MHz:
//   FM_HALF_PERIOD = 80/8 = 10 clocks
//   PIXEL_PERIOD   = 80/1 = 80 clocks

module pll (
    input  wire clk_25m,    // 25 MHz oscillator input
    output wire clk_sys,    // 80 MHz system clock output
    output wire locked      // PLL lock indicator
);

    wire clkfb;  // Internal feedback

    (* ICP_CURRENT="12" *)
    (* LPF_RESISTOR="8" *)
    (* MFG_ENABLE_FILTEROPAMP="1" *)
    (* MFG_GMCREF_SEL="2" *)
    EHXPLLL #(
        .PLLRST_ENA("DISABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"),
        .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(5),
        .CLKFB_DIV(16),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(5),
        .CLKOP_CPHASE(4),
        .CLKOP_FPHASE(0),
        .FEEDBK_PATH("CLKOP"),
        .CLKOS_ENABLE("DISABLED"),
        .CLKOS2_ENABLE("DISABLED"),
        .CLKOS3_ENABLE("DISABLED")
    ) pll_inst (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(clk_25m),
        .CLKOP(clk_sys),
        .CLKFB(clk_sys),       // Direct feedback from CLKOP
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b0),
        .PHASESTEP(1'b0),
        .PHASELOADREG(1'b0),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
        .LOCK(locked)
    );

endmodule
