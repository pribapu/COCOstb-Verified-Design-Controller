# SPI Master Controller with a cocotb Verification Environment

[![SPI Master Regression](https://github.com/pribapu/COCOstb-Verified-Design-Controller/actions/workflows/regression.yml/badge.svg)](https://github.com/pribapu/COCOstb-Verified-Design-Controller/actions/workflows/regression.yml)

A parameterizable **SPI master** written in SystemVerilog, verified with a
self-checking **cocotb** (Python) testbench that uses constrained-random
stimulus, an independent slave bus-functional model, a scoreboard, and a
functional-coverage model. The regression reaches **100% functional coverage**,
and **fault-injection (mutation) testing** confirms the scoreboard actually
catches SPI protocol-timing bugs.

```
spi_master_cocotb/
├── rtl/
│   └── spi_master.sv         # the DUT: parameterizable SPI master, all 4 modes
├── tb/
│   ├── spi_bfm.py            # SPI slave bus-functional model + SW reference
│   ├── spi_coverage.py       # functional-coverage model + JSON aggregation
│   ├── test_spi_master.py    # driver + scoreboard + directed/random tests
│   ├── test_waves.py         # short traced run for GTKWave
│   ├── runner.py             # Python runner (Verilator): builds W=8 and W=16
│   └── Makefile              # classic cocotb flow (Icarus Verilog)
├── docs/
│   ├── coverage_report.txt   # captured 100% coverage report
│   ├── coverage.json         # merged coverage database
│   └── spi_waves_mode0_mode3.vcd   # example waveform (open in GTKWave)
├── run.sh                    # convenience wrapper
└── README.md
```

## The design

`spi_master.sv` is a clocked FSM (`IDLE → SETUP → XFER → DONE`) with a
programmable clock divider. Mode, bit order, and divider are **runtime inputs**
latched at the start of each transfer, so one elaboration covers the entire
configuration space — which is what makes constrained-random verification and
coverage closure practical.

Features:

- All four SPI modes (`CPOL`/`CPHA` = 0..3), Motorola timing.
- Configurable word width (`DATA_WIDTH`, tested at 8 and 16).
- Configurable SCLK divider (`sclk half-period = clk_div + 1` system clocks).
- MSB-first or LSB-first.
- Simple CPU-side handshake: pulse `start`, watch `busy`, get a one-cycle
  `done` with `rx_data` valid; full-duplex (`tx_data` out on MOSI while MISO is
  captured into `rx_data`).

SPI timing implemented:

| CPHA | Leading edge | Trailing edge |
|------|--------------|---------------|
| 0    | **sample** MISO | shift MOSI |
| 1    | shift MOSI      | **sample** MISO |

SCLK idles at `CPOL`; a half-period of CS setup precedes the first edge so
CPHA=0 slaves see a stable first bit.

## The verification environment

- **Driver** — `do_transfer()` drives the CPU handshake for one transfer and
  self-checks the result.
- **Slave BFM / monitor** — `SPISlaveBFM` reacts on the SPI pins as an
  independent standard SPI slave: it samples MOSI and drives MISO on the correct
  edges for the configured mode, and records the byte it received, the number of
  SCLK edges it saw, and whether SCLK idled at CPOL.
- **Scoreboard** — every transfer is checked both directions against a pure
  software reference (`spi_reference`): the master's `rx_data` must equal the
  slave's transmitted byte, and the slave's captured byte must equal the
  master's `tx_data`. Because the BFM is written independently of the RTL, this
  is a real cross-check of protocol timing, plus pin-level checks (edge count ==
  2×width, SCLK idle level).
- **Coverage** — `spi_coverage.py` tracks mode, bit order, word width, divider
  class (fast/slow), and special TX/RX data patterns (all-zeros, all-ones,
  0xAA, 0x55, other), plus the mode×order cross. Results are aggregated across
  both width builds into `coverage.json`.

### Tests

| Test | What it checks |
|------|----------------|
| `test_smoke` | one mode-0 transfer end to end |
| `test_reset` | outputs safe out of reset (`cs_n=1`, `busy=0`, `done=0`) |
| `test_directed_all_modes` | full sweep: mode × order × divider × special data |
| `test_back_to_back` | consecutive transfers with no idle gap |
| `test_constrained_random` | 200 randomized transfers per width; closes coverage |

## Results

All 5 tests pass at `DATA_WIDTH` = 8 and 16, and functional coverage closes at
100%:

```
FUNCTIONAL COVERAGE REPORT
  mode            4/4   100.0%
  order           2/2   100.0%
  width           2/2   100.0%
  divider         2/2   100.0%
  tx_special      5/5   100.0%
  rx_special      5/5   100.0%
  mode_x_order    8/8   100.0%
  OVERALL        28/28  100.0%
```

### Fault-injection (mutation) testing

To prove the scoreboard isn't a rubber stamp, five deliberate RTL bugs were
injected; the clean RTL passes and every mutant is **killed** (caught):

| Injected bug | Caught by |
|--------------|-----------|
| End transfer one SCLK edge early | edge-count / RX mismatch |
| CPHA=0 sample dropped | master RX mismatch |
| MSB/LSB bit order inverted | master RX mismatch |
| Chip-select never asserted | transfer never observed / RX mismatch |
| SCLK idles at wrong polarity | "SCLK not idling at CPOL" check |

## How to run

### Option A — Icarus Verilog (the usual local flow)

Install `iverilog`, `gtkwave`, and cocotb, then:

```bash
pip install cocotb
cd tb
make                 # SIM=icarus, DATA_WIDTH=8
make DATA_WIDTH=16
make waves           # open dump.vcd in GTKWave
```

### Option B — Verilator + Python runner (builds W=8 and W=16, checks coverage)

```bash
pip install cocotb verilator     # 'verilator' PyPI wheel ships the binary
cd tb
SIM=verilator python3 runner.py            # full regression + coverage gate
SIM=verilator python3 runner.py --waves    # writes tb/dump.vcd
gtkwave dump.vcd
```

`docs/spi_waves_mode0_mode3.vcd` is a ready-made capture (one mode-0 and one
mode-3 transfer) if you just want to look at the waveform.

## Possible next steps

- Add an APB/AXI-Lite register interface and a TX/RX FIFO for burst transfers.
- Multi-slave chip-select decode.
- Synthesize for a cheap FPGA with the open toolchain (Yosys + nextpnr) — e.g.
  iCEBreaker (iCE40) or Tang Nano (GW1N) — to demonstrate it runs on hardware.
- SVA assertions bound into the RTL for CS/SCLK relationships.

