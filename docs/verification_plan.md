# Verification plan

Every requirement is traced to where it is checked. **Sim** = cocotb test,
**Chk** = rule in the pin-level protocol checker (`tb/spi_protocol_checker.py`),
**FV** = formal property (`formal/*_props.sv`, proven unbounded unless noted),
**Cov** = functional-coverage bin that proves the scenario was exercised.

## spi_master core

| ID | Requirement | Sim | Chk | FV | Cov |
|---|---|---|---|---|---|
| C-1 | Full-duplex data: master RX == slave TX, slave RX == master TX, every mode / order / divider / data | `test_directed_all_modes`, `test_constrained_random` (scoreboard vs. `spi_reference`) | — | `p_mosi_data`, `p_rx_correct` (all 2^8 / 2^16 words, all MISO patterns) | `mode`, `order`, `width`, `divider`, `tx_special`, `rx_special`, `mode_x_order` |
| C-2 | All four SPI modes: sample / shift on the correct edges | `test_directed_all_modes` | SPI-3 | `p_mosi_stable_on_sample`, `p_mosi_data` | `mode` |
| C-3 | SCLK idles at CPOL; never moves on a CS edge; while deselected only moves *to* CPOL | all tests (checker always on) | SPI-1, SPI-2, SPI-4 | `p_sclk_quiet_on_cs_edge`, `p_sclk_idle_at_cs_edge`, `p_sclk_tracks_cpol` | `mode_transition` (all 16 mode pairs) |
| C-4 | SCLK half-period == `clk_div+1`; first edge `2*(clk_div+1)` after CS | `test_sclk_timing` | — | `p_half_period` | `divider` |
| C-5 | Exactly `2*DATA_WIDTH` SCLK edges and `DATA_WIDTH` samples per word | every transfer (BFM edge count) | SPI-5 | `p_edge_count`, `p_edge_bound`, `p_sample_count` | — |
| C-6 | `busy` == CS asserted; `done` one cycle, with CS release; `rx_data` changes only on `done` | `test_busy_cs_timing` | — | `p_busy_is_cs`, `p_done_pulse`, `p_done_idle`, `p_done_is_end`, `p_rx_stable` | — |
| C-7 | Cycle-exact latency `(clk_div+1)*(2W+1)+1`; a transfer always completes | — | — | `p_latency`, `p_latency_bound` | — |
| C-8 | Config / data sampled at `start`; later input changes can't corrupt the word | `test_start_ignored_while_busy` | — | all p_* hold with inputs free every cycle | — |
| C-9 | Reset mid-transfer aborts cleanly and the core recovers | `test_reset_mid_transfer` | reset-aware | properties hold across arbitrary reset | — |
| C-10 | Input contract: `start` >= 1 cycle out of reset, not on the `done` cycle, `cpol` settled | driver respects it | — | assumed standalone; **asserted** inside the wrapper | `c_back_to_back`, `c_cpol_switch` |

## spi_apb_wrapper

| ID | Requirement | Sim | Chk | FV | Cov |
|---|---|---|---|---|---|
| W-1 | Register read/write semantics, W1P flush bits, W1C sticky bits | `test_reg_rw`, `test_irq_masking` | — | `p_done_irq_w1c` | `reg_access` |
| W-2 | Unmapped access -> PSLVERR **and no side effect** | `test_reg_rw`, `test_unmapped_access_no_side_effects` | — | `p_pslverr`, `p_err_no_reg_change`, `p_err_no_push`, `p_err_no_pop` (spec-decoded) | — |
| W-3 | TX/RX FIFOs never lose, duplicate or reorder a word | `test_tx_fifo_fill_drain`, `test_burst_end_to_end` | — | `p_tx_fifo_order`, `p_rx_fifo_order` (symbolic marked word), `p_tx_accept`, `p_rx_accept`, `p_fifo_bounds` | `fifo_state` |
| W-4 | Push when full is dropped and flagged; completion when RX full overruns and is flagged | `test_tx_fifo_fill_drain`, `test_rx_fifo_overrun` | — | `p_drop_flag`, `p_overrun_flag` | `irq_source` |
| W-5 | Flush empties the FIFO without corrupting the in-flight word | `test_flush` | SPI-5 | `p_tx_flush_empties`, `p_rx_flush_empties` | — |
| W-6 | Interrupt sources fire exactly on their event; `irq` honours the enables | `test_irq_masking`, `test_tx_fifo_fill_drain`, `test_rx_fifo_overrun` | — | `p_irq`, `p_done_irq_set/only`, `p_txe_irq_set/only`, `p_rxf_irq_set/only` | `irq_source` |
| W-7 | Engine only starts words when enabled and data is queued; honours the core contract | all APB tests | — | `p_start_only_enabled`, core `contract_ok` as an assertion | — |
| W-8 | Exactly one chip-select, the selected one; pins glitch-free | `test_cs_select_lines` | SPI-6 | `p_cs_onehot`, `p_cs_is_selected`, `p_cs_active_status` | `cs_line` |
| W-9 | CS_HOLD keeps CS asserted across words; release ends the frame | `test_cs_hold_multiword_frame`, flash tests | SPI-5 | `p_pin_hold_keeps_cs`, `p_pin_release`, `p_pin_cs_covers_word` | `frame_words` |
| W-10 | Mode / CS_SEL changes never take effect inside a frame; SCLK never moves on a CS pin edge | `test_config_latching`, `test_mode_switch_timing_sweep` (every cycle offset) | SPI-1, SPI-4 | `p_pin_frame_mode_frozen`, `p_pin_sclk_quiet_on_cs_edge`, `p_pin_sclk_moves_to_cpol` | `c_cpol_switch` |

## System level

| ID | Requirement | Where |
|---|---|---|
| S-1 | A firmware driver can operate a real SPI NOR flash through the APB interface: JEDEC ID, WREN, READ, FAST_READ, PAGE PROGRAM (in-page wrap, NOR AND-semantics), SECTOR ERASE, BUSY polling, write protection without WREN | `test_flash_end_to_end`, `flash_op` coverage (11 bins) |
| S-2 | Two devices in different SPI modes share the bus | `test_flash_end_to_end` (mode-0 flash on CS1, mode-3 device on CS2), checker `BusMode` |
| S-3 | Streams longer than the FIFOs with software flow control, no drop / overrun | `test_flash_end_to_end` (48-byte read), `test_cs_hold_multiword_frame` |
| S-4 | Implementable at 100 MHz on commodity FPGAs | `synth/run_synth.py --check` (CI) |

## Verification of the verification

* **Non-vacuity** -- every formal proof has matching `c_*` cover points (reached
  in the `cover` tasks), and every scenario-level requirement has a coverage bin.
* **Mutation testing** -- `mutation/run_mutation.py` injects realistic bugs
  and requires each to be caught by simulation, formal, or both
  (`docs/mutation_report.md`).
