# Makefile — TCD1254 FPGA CCD Readout build flow
# Target: Colorlight i9 v7.2 (ECP5 LFE5U-45F-6BG381C)
# Toolchain: yosys → nextpnr-ecp5 → ecppack → openFPGALoader

PROJ     = top
PACKAGE  = CABGA381
SPEED    = 6
LPF      = constraints/colorlight_i9.lpf
SRC      = $(wildcard src/*.v)

.PHONY: all clean prog sim

all: $(PROJ).bit

# Synthesis
$(PROJ).json: $(SRC)
	yosys -p "synth_ecp5 -top top -json $@" $(SRC)

# Place & Route
$(PROJ).config: $(PROJ).json $(LPF)
	nextpnr-ecp5 --45k --package $(PACKAGE) --speed $(SPEED) \
		--json $< --lpf $(LPF) --textcfg $@ --lpf-allow-unconstrained

# Bitstream generation
$(PROJ).bit: $(PROJ).config
	ecppack --compress --input $< --bit $@

# Flash to Colorlight i9 via JTAG
prog: $(PROJ).bit
	openFPGALoader -b colorlight-i9 $<

# Program to SRAM (volatile, faster for development)
prog-sram: $(PROJ).bit
	openFPGALoader -b colorlight-i9 --write-sram $<

# Simulation (iverilog + vvp)
# Uses testbench/pll_sim.v instead of src/pll.v (EHXPLLL not available in iverilog)
SIM_SRC = $(filter-out src/pll.v,$(SRC))

sim_top:
	iverilog -g2012 -o testbench/tb_top.vvp -I src \
		testbench/pll_sim.v $(SIM_SRC) testbench/tb_top.v
	cd testbench && vvp tb_top.vvp

sim_ccd:
	iverilog -o testbench/tb_ccd_driver.vvp -I src \
		src/ccd_driver.v testbench/tb_ccd_driver.v
	cd testbench && vvp tb_ccd_driver.vvp

sim_shadow:
	iverilog -o testbench/tb_shadow_detect.vvp -I src \
		src/shadow_detect.v testbench/tb_shadow_detect.v
	cd testbench && vvp tb_shadow_detect.vvp

sim_flash:
	iverilog -o testbench/tb_flash_trigger.vvp -I src \
		src/flash_trigger.v testbench/tb_flash_trigger.v
	cd testbench && vvp tb_flash_trigger.vvp

sim_spi:
	iverilog -o testbench/tb_spi_peripheral.vvp -I src \
		src/spi_peripheral.v testbench/tb_spi_peripheral.v
	cd testbench && vvp tb_spi_peripheral.vvp

clean:
	rm -f $(PROJ).json $(PROJ).config $(PROJ).bit
	rm -f testbench/*.vvp testbench/*.vcd

# Utilization report
util: $(PROJ).json
	@echo "=== Resource Utilization ==="
	@yosys -p "synth_ecp5 -top top -noflatten" $(SRC) 2>&1 | \
		grep -E "LUT|DFF|BRAM|DP16KD|CCU2|MULT" || true
