# TCD1254 FPGA CCD Readout + Sort System

FPGA-based readout system for the Toshiba TCD1254 linear CCD image sensor, with an RP2040 microcontroller for autonomous part sorting. Targets the **Colorlight i9 v7.2** board (Lattice ECP5 LFE5U-45F).

## Overview

Reads 2200 pixels per frame from the TCD1254 via an AD9226 12-bit ADC, performs on-chip shadow detection, and communicates with an RP2040 controller over SPI. The RP2040 runs the sort/singulation state machine with 4 stepper axes. Python GUI used for configuration only — the system runs headless after setup.

### System Architecture

```
Python GUI ──USB CDC──→ RP2040 (system controller)
                           │
                           ├── SPI ──→ FPGA (vision: CCD readout + shadow detect)
                           │              ├── TCD1254 CCD ← 74HCT04 ← fM/SH/ICG
                           │              ├── AD9226 ADC (12-bit parallel)
                           │              └── Flash lamp GPIO (4x synchronized)
                           │
                           ├── PIO ──→ 4x TMC2209 stepper drivers
                           │           (feeder, conveyor, singulate, sort)
                           │
                           ├── GPIO ──→ Limit/home switches, photogate, air jets
                           │
                           └── Flash ──→ Persistent config (last 4KB sector)
```

### Modes

| Mode | Description |
|------|-------------|
| **Position** (1) | On-FPGA shadow detection — returns 2-byte shadow center position per frame |
| **Raw** (2) | Full 2200-pixel frame dump — `0xAA 0x55` sync + 4400 data bytes |
| **Multi-angle** (3) | Per-lamp frame capture with auto-sequence flash cycling |

## FPGA Architecture

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
| `cmd_parser.v` | Parses host commands (exposure + flash config) |
| `flash_trigger.v` | Frame-synchronized flash lamp GPIO with auto-sequence |
| `spi_peripheral.v` | SPI slave interface to RP2040 (register-mapped commands) |
| `top.v` | Top-level integration |

## RP2040 Firmware

| Module | Function |
|--------|----------|
| `main.c` | Dual-core: Core 0 = USB serial config, Core 1 = sort sequence |
| `fpga_spi.c` | SPI master driver (exposure, flash, shadow read, frame stream) |
| `stepper.c` | PIO-based 4-axis stepper control with trapezoidal velocity profile |
| `sort_sequence.c` | Autonomous feed → capture → classify → sort state machine |
| `config.c` | Flash persistence with CRC32 validation |
| `stepper.pio` | PIO state machine program for step pulse generation |

### USB Serial Protocol

Text-based commands over USB CDC (Python → RP2040):

```
PING                    → PONG
STATUS                  → {JSON: state, fpga, axes}
CONFIG_SET key=value    → OK key=value
CONFIG_SAVE             → OK saved
START / STOP / ESTOP    → OK ...
HOME [axis]             → OK homed
MOVE axis steps speed   → OK move
EXPO sh icg mode avg    → OK expo
FLASH mask delay dur fl → OK flash
SHADOW                  → {shadow_px: 123.0}
STATS                   → {JSON: inspected, accepted, ...}
```

## Hardware

- **FPGA board**: [Colorlight i9 v7.2](https://github.com/wuxx/Colorlight-FPGA-Projects) (ECP5 LFE5U-45F, CABGA381)
- **CCD sensor**: Toshiba TCD1254GFG (2048 effective + 152 dummy pixels)
- **ADC**: AD9226 (12-bit, 65 MSPS parallel)
- **Level shifter**: 74HCT04 hex inverter (3.3V FPGA → 5V CCD drive)
- **Controller**: Raspberry Pi Pico (RP2040)
- **Stepper drivers**: 4x TMC2209 (STEP/DIR/EN + UART)
- **Flash lamps**: Up to 4 LED/xenon lamps via MOSFET drivers

### Pin Mapping

- **J1**: ADC data D[0:7]
- **J2**: ADC data D[8:11] + CCD control (fM, SH, ICG, ADC_CLK)
- **J3**: UART TX/RX + Flash GPIO (4 lamps)
- **J4**: SPI to RP2040 (SCK, MOSI, MISO, CS)

See `constraints/colorlight_i9.lpf` for exact pin assignments.

## Toolchain

Requires [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) for FPGA and [Pico SDK](https://github.com/raspberrypi/pico-sdk) for RP2040:

```
yosys          — synthesis
nextpnr-ecp5   — place & route
ecppack        — bitstream generation
openFPGALoader — JTAG programming
iverilog + vvp — simulation
cmake          — RP2040 firmware build
```

## Build

### FPGA

```bash
source /path/to/oss-cad-suite/environment

make              # Synthesize → P&R → bitstream (top.bit)
make prog         # Flash to board via JTAG
make prog-sram    # Volatile SRAM load (faster iteration)
```

### RP2040 Firmware

```bash
cd firmware
mkdir build && cd build
cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
make
# Flash tcd1254_controller.uf2 to Pico via USB bootloader
```

### Python Host App

```bash
cd host
uv sync
uv run python -m ccd_inspector.app
```

## Simulation

```bash
make sim_ccd       # CCD driver timing verification
make sim_shadow    # Shadow detection with synthetic data
make sim_flash     # Flash trigger testbench
make sim_spi       # SPI peripheral testbench
make sim_top       # Full system (PLL sim → CCD → UART → SPI)
```

## Project Structure

```
src/                          # Verilog RTL modules
src/_legacy/                  # Superseded modules (step_generator.v)
testbench/                    # Testbenches + PLL simulation model
constraints/                  # Colorlight i9 pin constraints (.lpf)
firmware/                     # RP2040 firmware (C + PIO)
  ├── include/                # Headers (pins, stepper, sort_sequence, config, fpga_spi)
  ├── src/                    # C source files
  ├── pio/                    # PIO assembler programs
  └── CMakeLists.txt          # Pico SDK CMake build
host/                         # Python GUI app (PySide6 + pyqtgraph)
  └── ccd_inspector/          # Package
      ├── comm/               # Serial + RP2040 communication
      ├── core/               # Config, calibration, timing
      ├── processing/         # Edge detection, classification
      ├── flash/              # Flash lamp coordination
      └── gui/                # PySide6 tabs and widgets
Makefile                      # FPGA build, simulate, program
```

## License

MIT
