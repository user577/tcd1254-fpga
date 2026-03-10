# TCD1254 FPGA CCD Readout

FPGA-based readout system for the Toshiba TCD1254 linear CCD image sensor, targeting the **Colorlight i9 v7.2** board (Lattice ECP5 LFE5U-45F).

## Overview

Reads 2200 pixels per frame from the TCD1254 via an AD9226 12-bit ADC, performs on-chip shadow detection, and streams data to a host PC over UART at 921600 baud.

### Signal chain

```
TCD1254 CCD ←── 74HCT04 ←── FPGA (fM/SH/ICG)
     ↓
  AD9226 ADC ──→ FPGA (12-bit parallel) ──→ UART ──→ Host PC
```

### Modes

| Mode | Description |
|------|-------------|
| **Position** (1) | On-FPGA shadow detection — streams 2-byte shadow center position per frame |
| **Raw** (2) | Full 2200-pixel frame dump — `0xAA 0x55` sync + 4400 data bytes |

## Architecture

| Module | Function |
|--------|----------|
| `pll.v` | 25 MHz → 80 MHz PLL (EHXPLLL) |
| `ccd_driver.v` | Generates fM (4 MHz), SH, ICG timing signals |
| `adc_clk_gen.v` | ADC sample clock derived from fM |
| `adc_capture.v` | Latches 12-bit ADC data on pixel_valid |
| `frame_buffer.v` | Double-buffered BRAM (2×2200×12 bits, ~6 EBR blocks) |
| `shadow_detect.v` | Thresholding + center-of-mass shadow position |
| `frame_tx.v` | Serializes frame/position data to UART |
| `uart_tx.v` / `uart_rx.v` | 921600 baud UART |
| `cmd_parser.v` | Parses 12-byte host commands (SH/ICG/mode/avg) |
| `top.v` | Top-level integration + power-on auto-start |

## Hardware

- **FPGA board**: [Colorlight i9 v7.2](https://github.com/wuxx/Colorlight-FPGA-Projects) (ECP5 LFE5U-45F, CABGA381)
- **CCD sensor**: Toshiba TCD1254GFG (2048 effective + 152 dummy pixels)
- **ADC**: AD9226 (12-bit, 65 MSPS parallel)
- **Level shifter**: 74HCT04 hex inverter (3.3V FPGA → 5V CCD drive)
- **Host link**: USB-UART adapter (FTDI or similar)

### Pin mapping

ADC data D[0:7] on the lower SODIMM GPIO bank, D[8:11] + CCD control signals on the upper bank, UART on dedicated pins. See `constraints/colorlight_i9.lpf` for exact pin assignments.

## Toolchain

Requires [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build):

```
yosys          — synthesis
nextpnr-ecp5   — place & route
ecppack        — bitstream generation
openFPGALoader — JTAG programming
iverilog + vvp — simulation
```

## Build

```bash
source /path/to/oss-cad-suite/environment

make              # Synthesize → P&R → bitstream (top.bit)
make prog         # Flash to board via JTAG
make prog-sram    # Volatile SRAM load (faster iteration)
```

## Simulation

```bash
make sim_ccd       # CCD driver timing verification
make sim_shadow    # Shadow detection with synthetic data
make sim_top       # Full system (PLL sim → CCD → UART)
```

## Host Software

Python script for capturing data from the FPGA:

```bash
pip install pyserial matplotlib

python host/capture.py                # Position mode (continuous)
python host/capture.py --raw          # Capture + plot one raw frame
python host/capture.py --raw --save   # Save frame to CSV
python host/capture.py --list         # List serial ports
```

## Project Structure

```
src/                 # Verilog RTL modules (11 files)
testbench/           # Testbenches + PLL simulation model
constraints/         # Colorlight i9 pin constraints (.lpf)
host/                # Python host capture script
Makefile             # Build, simulate, and program targets
```

## License

MIT
