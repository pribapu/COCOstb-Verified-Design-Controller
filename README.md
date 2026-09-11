# SPI Master Controller — simulation, formal and mutation verified

[![SPI Master Regression](https://github.com/pribapu/COCOstb-Verified-Design-Controller/actions/workflows/regression.yml/badge.svg)](https://github.com/pribapu/COCOstb-Verified-Design-Controller/actions/workflows/regression.yml)

A parameterizable **SPI master** in SystemVerilog, wrapped as an **APB4
peripheral** (TX/RX FIFOs, auto-transfer engine, interrupts, 4 chip-selects,
CS-hold frames), and verified four independent ways:

| Layer | What it gives you | Result |
|---|---|---|
| **cocotb simulation** | constrained-random + directed tests, independent slave BFM, scoreboard, clock-synchronous protocol checker, SPI NOR flash end-to-end | 24 tests (34 runs) on Icarus & Verilator, **80/80 coverage bins** |
| **Formal (SymbiYosys)** | unbounded proofs of protocol timing *and full data integrity* for every mode / order / divider / data word; assume-guarantee between core and wrapper | **51 properties + 24 invariants proven**, 14/14 covers reached |
| **Mutation testing** | 22 injected RTL bugs, each scored against simulation and formal | **22/22 killed** (7 by formal only) |
| **FPGA implementation** | Yosys + nextpnr on iCE40 and ECP5 | **117–187 MHz**, ≤ 7.5% of an iCE40 HX8K |

Everything above runs in CI on every push.

```
rtl/        spi_master.sv            the core: all 4 SPI modes, MSB/LSB first, runtime divider
            spi_apb_wrapper.sv       APB4 regs, FIFOs, engine, IRQ, NUM_CS chip-selects, CS hold
tb/         test_spi_master.py       core tests (driver, scoreboard, coverage)
            test_spi_apb.py          wrapper tests incl. multi-device bus + flash end-to-end
            spi_bfm.py               SPI slave BFM (multi-word frames, per-CS) + reference model
            spi_protocol_checker.py  cycle-by-cycle pin-level protocol checker (rules SPI-1..6)
            spi_flash_model.py       behavioural W25Q-style SPI NOR flash
            spi_flash_driver.py      "firmware" flash driver that only touches APB registers
            spi_apb_bfm.py           APB4 master + register map
            spi_coverage.py          functional coverage model (JSON-merged across builds)
            runner.py / Makefile     Verilator / Icarus flows
formal/     spi_master_props.sv      core properties, bound with `bind` (RTL has no FV code)
            spi_apb_wrapper_props.sv wrapper properties (FIFO integrity, IRQ, pins, APB)
            *.sby                    SymbiYosys jobs: prove / cover / bmc_ext
mutation/   run_mutation.py          mutation campaign -> docs/mutation_report.md
synth/      run_synth.py             iCE40 + ECP5 implementation -> docs/synth_report.md
docs/       verification_plan.md     requirement -> test / checker rule / property / coverage bin
```

## Bugs the verification found

1. **Spurious SCLK edge at frame start.** The core only drove SCLK to CPOL
   when a transfer *started*, so after any CPOL change SCLK toggled on the same
   clock edge CS_n asserted — a phantom clock edge a mode-1/3 slave can latch.
   The existing BFM missed it because it samples SCLK in the same timestep as
   the CS edge. Caught by the new protocol checker (rule SPI-1, 4 failing
   tests at both widths) and by formal property `p_sclk_quiet_on_cs_edge` with
   a 4-cycle counterexample. Fix: SCLK tracks CPOL continuously while idle.
2. **Errored APB writes had side effects.** Unmapped addresses returned
   PSLVERR, but the write strobes only decoded `paddr[4:2]`, so e.g. a write
   to `0x20` *also* wrote CTRL (and could silently enable the engine). Fixed;
   locked down by `test_unmapped_access_no_side_effects` and the spec-decoded
   properties `p_err_no_reg_change` / `p_err_no_push` / `p_err_no_pop`.
3. **An undocumented timing dependency, made explicit.** The wrapper's
   "don't fire on the core's `done` cycle" was described as merely
   conservative. The formal assume-guarantee proof shows it is *required* by
   the core's input contract (mutants W6/W7 prove it), so it is now documented
   and checked.

## The design

**`spi_master.sv`** — clocked FSM (`IDLE → SETUP → XFER → DONE`). Mode, bit
order and divider are runtime inputs latched at `start`, so one elaboration
covers the whole configuration space.

- All four SPI modes, MSB- or LSB-first, `DATA_WIDTH` parameter (8 and 16 tested).
- SCLK half-period = `clk_div + 1` clocks; one half-period of CS setup.
- SCLK follows CPOL while idle, so it's at the idle level before CS asserts.
- Handshake: pulse `start`, watch `busy`, one-cycle `done` with `rx_data`.
  Input contract: `cpol` settled one cycle before `start`, not on the `done`
  cycle (the wrapper guarantees this; the formal proof checks it does).

| CPHA | Leading edge | Trailing edge |
|------|--------------|---------------|
| 0    | **sample** MISO | shift MOSI |
| 1    | shift MOSI      | **sample** MISO |

**`spi_apb_wrapper.sv`** — drops the unmodified core onto an APB4 bus:

| Offset | Register | |
|---|---|---|
| 0x00 | CTRL | EN, TX_FLUSH, RX_FLUSH (W1P) |
| 0x04 | CONFIG | CPOL, CPHA, LSB_FIRST, 4 × IRQ enable |
| 0x08 | DIVIDER | SCLK divider |
| 0x0C | STATUS | BUSY, CS_ACTIVE, FIFO flags/counts, RX_OVERRUN, TX_DROPPED (W1C) |
| 0x10 / 0x14 | TXDATA / RXDATA | FIFO push / pop (8 deep) |
| 0x18 | IRQ_STATUS | DONE, TX_EMPTY, RX_FULL, OVERRUN (W1C) |
| 0x1C | CS_CTRL | CS_SEL (which of `NUM_CS` lines), CS_HOLD (multi-word frames) |

- Hardware engine drains the TX FIFO into the core; software never polls per byte.
- **CS_HOLD** keeps one chip-select low across any number of words, which is
  what real devices (flash, ADCs, displays) need for a command + address + data
  transaction.
- CS pins are registered (glitch-free); mode and CS_SEL changes are applied
  only between frames, so SCLK can never move inside a frame.
- Accesses to unmapped addresses return PSLVERR and have no side effects.

## Verification environment

### Simulation (cocotb)

- **Slave BFM** — an independent Motorola-SPI slave on the pins; supports
  multi-word frames and a chip-select index, with hooks for device models.
- **Scoreboard** — both directions against `spi_reference()`, every transfer.
- **Protocol checker** — samples every pin on every clock (after the edge
  settles) and checks their relationships, like bus assertion IP. It knows
  nothing about the RTL, and it runs in every test:

  | Rule | Check |
  |---|---|
  | SPI-1 | no SCLK edge on the same clock as a CS edge |
  | SPI-2 | SCLK at CPOL at frame start and end |
  | SPI-3 | MOSI never changes on the same edge as a sampling SCLK edge |
  | SPI-4 | while deselected, SCLK only moves *to* CPOL |
  | SPI-5 | a frame is a whole number of words |
  | SPI-6 | at most one chip-select asserted |

- **SPI NOR flash end-to-end** — a behavioural W25Q-style flash (JEDEC ID,
  status/WEL/BUSY, READ, FAST_READ, PAGE PROGRAM with in-page wrap and NOR
  AND-semantics, SECTOR ERASE, commands ignored while busy) sits on CS1. A
  mode-3 device shares the bus on CS2. A Python "firmware" driver that only
  touches APB registers erases, programs, and reads back, including
  write-protect, page wrap and a 48-byte stream (6× the FIFO depth, so it only
  works with correct flow control). Expected data comes from the test's own
  image, not from the model.

| Core tests (`DATA_WIDTH` 8 and 16) | Wrapper tests |
|---|---|
| `test_smoke`, `test_reset` | `test_apb_reset`, `test_reg_rw`, `test_config_latching` |
| `test_directed_all_modes` (mode × order × divider × special data) | `test_tx_fifo_fill_drain`, `test_rx_fifo_overrun`, `test_flush` |
| `test_back_to_back`, `test_variable_idle_gaps` | `test_irq_masking`, `test_burst_end_to_end` |
| `test_sclk_timing`, `test_busy_cs_timing` | `test_unmapped_access_no_side_effects` |
| `test_start_ignored_while_busy`, `test_reset_mid_transfer` | `test_cs_select_lines`, `test_cs_hold_multiword_frame` |
| `test_constrained_random` (200 per width) | `test_mode_switch_timing_sweep` (CPOL flip at *every* cycle offset) |
| | `test_flash_end_to_end` |

**Functional coverage** (80 bins, merged across all three builds, gated at
100% in CI): mode, order, width, divider class, special data patterns,
mode × order, **all 16 mode→mode transitions** (the scenario behind bug 1),
register access, FIFO states, IRQ sources, chip-select lines, frame lengths,
and 11 flash operations.

### Formal (SymbiYosys)

Property modules are attached with `bind` (via the yosys-slang frontend), so
the RTL carries no verification code. Every proof is unbounded (k-induction),
with helper invariants (`inv_*`) that are themselves proven.

- **Core** (`spi_master.sby`, W=8 and W=16): `p_mosi_data` / `p_rx_correct`
  prove *full-duplex data integrity for every mode, bit order, divider, data
  word and MISO pattern*, with all inputs free to change every cycle. There
  are also cycle-exact latency, SCLK half-period, edge/sample counts, the
  handshake, and the SCLK/CS relationships.
- **Wrapper** (`spi_apb_wrapper.sby`), against any legal APB4 master: FIFO
  data integrity for both FIFOs (an arbitrary symbolic word must come out in
  order and uncorrupted), accept/drop/overrun/flush semantics, exact interrupt
  events, no side effects from errored accesses (decoded from the spec, not
  the RTL), and pin-level CS/SCLK behaviour. Address decode in the properties
  comes from the spec, never from the RTL's own wires.
- **Assume-guarantee**: the core proof *assumes* its input contract; the
  wrapper proof binds the same core property set with the contract flipped to
  an *assertion*, re-proving every core property in context.
- **Non-vacuity**: `cover` tasks reach every `c_*` point (all four modes,
  back-to-back, CPOL switch, FIFO full, RX overrun, two-word held frame, ...).

### Mutation testing

`mutation/run_mutation.py` injects 22 realistic bugs (10 core, 12 wrapper) and
runs each through the full simulation regression and the formal flow. Formal
kills are attributed with the helper invariants compiled out, so each one
names the user-facing property that was violated. Full table:
[`docs/mutation_report.md`](docs/mutation_report.md).

**22/22 killed** — 15 by both methods and 7 by formal alone. The formal-only
kills are the reason both layers exist. Each one passes the entire
simulation suite:
- a divider or CPHA re-read mid-word,
- an RX-FIFO underflow on an empty read,
- a TX_EMPTY interrupt on the wrong edge,
- a mode change applied inside a CS-held frame,
- two contract violations that happen to be benign at the pins.

### FPGA implementation

`synth/run_synth.py` (Yosys + nextpnr, IOs unconstrained, 100 MHz target):

| Device | Top | Logic | FFs | Fmax |
|---|---|---|---|---|
| iCE40 HX8K | `spi_master` | 173 LCs | 76 | 160 MHz |
| iCE40 HX8K | `spi_apb_wrapper` | 573 LCs (7.5%) | 269 | 117 MHz |
| ECP5 25F | `spi_master` | 156 LUT4 | 76 | 187 MHz |
| ECP5 25F | `spi_apb_wrapper` | 350 LUT4 (1.4%) | 141 | 162 MHz |

## How to run

```bash
pip install -r requirements.txt

# simulation
cd tb
make                      # Icarus, core, DATA_WIDTH=8   (make DATA_WIDTH=16)
make apb                  # Icarus, APB wrapper + flash end-to-end
SIM=verilator python3 runner.py          # both widths + APB, 100% coverage gate
SIM=verilator python3 runner.py --waves  # waveform capture

# formal, synthesis, mutation (need the OSS CAD Suite on PATH)
cd formal && sby -f spi_master.sby && sby -f spi_apb_wrapper.sby
python3 synth/run_synth.py --check
python3 mutation/run_mutation.py
```

`docs/spi_waves_mode0_mode3.vcd` is a ready-made capture (one mode-0 and one
mode-3 transfer) if you just want to look at the waveform.

## Possible next steps

- AXI4-Lite variant of the register wrapper; DMA request outputs.
- Dual/quad SPI (for flash fast-read modes).
- Run it on hardware (iCEBreaker / ULX3S) against a real SPI flash.
