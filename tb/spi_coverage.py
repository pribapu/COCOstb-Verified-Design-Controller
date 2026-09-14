"""
spi_coverage.py -- lightweight functional coverage model for the SPI master.

Tracks which points in the configuration/data space the random regression has
actually exercised, and reports coverage as hit-bins / total-bins. Coverage is
persisted to JSON so results can be aggregated across multiple sim elaborations
(e.g. different DATA_WIDTH builds).
"""

import json
import os

# Special data values we want to see driven on both MOSI and MISO.
SPECIAL_DATA = {
    "zeros": lambda v, w: v == 0,
    "ones": lambda v, w: v == (1 << w) - 1,
    "alt_AA": lambda v, w: v == (0xAA & ((1 << w) - 1)),
    "alt_55": lambda v, w: v == (0x55 & ((1 << w) - 1)),
    "other": lambda v, w: 0 < v < (1 << w) - 1 and v not in (
        0xAA & ((1 << w) - 1), 0x55 & ((1 << w) - 1)),
}


class Coverage:
    def __init__(self):
        # Define the coverage goals (the full set of bins we require).
        self.goals = {
            "mode": {0, 1, 2, 3},                 # cpol*2 + cpha
            "order": {"msb", "lsb"},
            "width": {8, 16},
            "divider": {"fast", "slow"},          # div==0 vs div>1
            "tx_special": set(SPECIAL_DATA.keys()),
            "rx_special": set(SPECIAL_DATA.keys()),
            "mode_x_order": {(m, o) for m in range(4)
                             for o in ("msb", "lsb")},
            # Transition coverage: mode of transfer N-1 -> mode of transfer N.
            # CPOL changes between frames are exactly what exposed the SCLK
            # idle-level bug, so every ordered pair must be exercised.
            "mode_transition": {(a, b) for a in range(4) for b in range(4)},
            # ---- APB register/FIFO/IRQ bins (sampled by test_spi_apb.py) ----
            "reg_access": {"ctrl", "config", "divider", "status",
                            "txdata", "rxdata", "irq_status", "cs_ctrl"},
            "fifo_state": {"tx_empty", "tx_partial", "tx_full",
                            "rx_empty", "rx_partial", "rx_full"},
            "irq_source": {"done", "tx_empty", "rx_full", "overrun"},
            # ---- multi-device bus / CS-hold framing ----
            "cs_line": {0, 1, 2, 3},
            "frame_words": {"1", "2-8", "9+"},
            # ---- SPI NOR flash end-to-end (driver -> APB -> pins -> model) ----
            "flash_op": {"jedec_id", "wren", "busy_poll", "read", "fast_read",
                         "page_program", "page_wrap", "nor_and", "sector_erase",
                         "no_wel_ignored", "long_stream"},
        }
        self.hits = {k: set() for k in self.goals}
        self._prev_mode = None

    def sample(self, cpol, cpha, lsb_first, width, clk_div, tx, rx):
        mode = cpol * 2 + cpha
        order = "lsb" if lsb_first else "msb"
        if self._prev_mode is not None:
            self.hits["mode_transition"].add((self._prev_mode, mode))
        self._prev_mode = mode
        self.hits["mode"].add(mode)
        self.hits["order"].add(order)
        self.hits["width"].add(width)
        self.hits["divider"].add("fast" if clk_div == 0 else "slow")
        self.hits["mode_x_order"].add((mode, order))
        for name, pred in SPECIAL_DATA.items():
            if pred(tx, width):
                self.hits["tx_special"].add(name)
            if pred(rx, width):
                self.hits["rx_special"].add(name)

    def sample_apb(self, reg=None, tx_fifo_state=None, rx_fifo_state=None,
                   irq_source=None, cs_line=None, frame_words=None, flash_op=None):
        """Record APB register/FIFO/IRQ/bus coverage points (test_spi_apb.py)."""
        if cs_line is not None:
            self.hits["cs_line"].add(cs_line)
        if frame_words is not None:
            self.hits["frame_words"].add(
                "1" if frame_words == 1 else "2-8" if frame_words <= 8 else "9+")
        if flash_op is not None:
            self.hits["flash_op"].add(flash_op)
        if reg is not None:
            self.hits["reg_access"].add(reg)
        if tx_fifo_state is not None:
            self.hits["fifo_state"].add(f"tx_{tx_fifo_state}")
        if rx_fifo_state is not None:
            self.hits["fifo_state"].add(f"rx_{rx_fifo_state}")
        if irq_source is not None:
            self.hits["irq_source"].add(irq_source)

    # ---- persistence / aggregation ----
    def to_dict(self):
        return {k: sorted(map(_key, v)) for k, v in self.hits.items()}

    def merge_file(self, path):
        if os.path.exists(path):
            with open(path) as f:
                prev = json.load(f)
            for k, vals in prev.items():
                if k in self.hits:
                    self.hits[k] |= {_unkey(k, x) for x in vals}
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    # ---- reporting ----
    def report(self):
        lines = ["", "=" * 60, "FUNCTIONAL COVERAGE REPORT", "=" * 60]
        total_hit = total_goal = 0
        for k, goal in self.goals.items():
            hit = self.hits[k] & goal
            total_hit += len(hit)
            total_goal += len(goal)
            pct = 100.0 * len(hit) / len(goal)
            missing = goal - hit
            miss_s = "" if not missing else f"   MISSING: {sorted(map(str, missing))}"
            lines.append(f"  {k:<14} {len(hit):>2}/{len(goal):<2}  {pct:5.1f}%{miss_s}")
        overall = 100.0 * total_hit / total_goal
        lines.append("-" * 60)
        lines.append(f"  {'OVERALL':<14} {total_hit:>2}/{total_goal:<2}  {overall:5.1f}%")
        lines.append("=" * 60)
        return "\n".join(lines), overall


def _key(v):
    return list(v) if isinstance(v, tuple) else v


def _unkey(field, v):
    return tuple(v) if isinstance(v, list) else v
