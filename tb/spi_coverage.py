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
        }
        self.hits = {k: set() for k in self.goals}

    def sample(self, cpol, cpha, lsb_first, width, clk_div, tx, rx):
        mode = cpol * 2 + cpha
        order = "lsb" if lsb_first else "msb"
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

    # ---- persistence / aggregation ----
    def to_dict(self):
        return {k: sorted(list(map(_key, v))) for k, v in self.hits.items()}

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
    return tuple(v) if field == "mode_x_order" else v
