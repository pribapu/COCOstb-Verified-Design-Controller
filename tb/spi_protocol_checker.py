"""
spi_protocol_checker.py -- passive, clock-synchronous SPI pin-level protocol
checker.

The slave BFM (spi_bfm.py) is *reactive*: it wakes up on SCLK / CS_n edges, so
it only ever sees the bus at the instants it is already looking at. This
checker is the complement: it samples every pin on every system-clock cycle
(after the edge has settled, in the ReadOnly phase) and checks the
*relationships between* pins cycle by cycle, which is how a real bus
protocol checker / assertion IP works. It knows nothing about the RTL's
internals -- only the pins and the mode the test says it configured.

Rules (numbered so failure messages are greppable):

  SPI-1  SCLK must not change on the same clock cycle that chip-select
         asserts or deasserts (no SCLK edge coincident with a CS edge).
  SPI-2  SCLK must be at the CPOL idle level whenever a frame begins or ends.
  SPI-3  MOSI must not change on the same clock edge as an SCLK *sampling*
         edge for the frame's CPHA (a coincident change is a setup/hold race;
         a change one clock either side leaves >= 1 clk of margin).
  SPI-4  While no chip-select is asserted, SCLK may only move *to* the
         configured CPOL level (following a CPOL change) -- never pulse.
  SPI-5  A frame contains a whole number of words: SCLK edge count is a
         non-zero multiple of 2 * width (frames aborted by reset excepted).
  SPI-6  At most one chip-select line is asserted at a time.

`cfg` is any object with `cpol` / `cpha` attributes (the slave BFM is used
for this: tests already program it with the mode they expect on the bus).
"""

from cocotb.triggers import ReadOnly, RisingEdge
from cocotb.utils import get_sim_time


class SPIProtocolViolation(AssertionError):
    pass


class SPIProtocolChecker:
    def __init__(self, dut, width, cfg, cs_n=None, allow_empty_frames=False):
        self.dut = dut
        self.width = width
        self.cfg = cfg
        self.cs_n = cs_n if cs_n is not None else dut.cs_n
        self.cs_bits = len(self.cs_n)
        self.allow_empty_frames = allow_empty_frames
        # statistics, useful for tests / coverage
        self.frames = 0
        self.sclk_edges_total = 0
        self.sample_edges_checked = 0

    # ------------------------------------------------------------------
    def _cs_active(self, cs_val):
        return cs_val != (1 << self.cs_bits) - 1

    def _fail(self, rule, msg):
        t = get_sim_time(unit="ns")
        raise SPIProtocolViolation(f"[{rule}] @ {t} ns: {msg}")

    async def run(self):
        dut = self.dut
        prev = None           # (cs, sclk, mosi) from the previous cycle
        in_frame = False
        frame_edges = 0
        frame_cpol, frame_cpha = 0, 0

        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()

            if int(dut.rst_n.value) == 0:
                # Reset aborts everything: forget history, don't check.
                prev, in_frame, frame_edges = None, False, 0
                continue

            cs = int(self.cs_n.value)
            sclk = int(dut.sclk.value)
            mosi = int(dut.mosi.value)
            active = self._cs_active(cs)
            cpol, cpha = int(self.cfg.cpol), int(self.cfg.cpha)

            asserted = ~cs & ((1 << self.cs_bits) - 1)
            if asserted & (asserted - 1):
                self._fail("SPI-6", f"more than one chip-select asserted: "
                           f"cs_n=0b{cs:0{self.cs_bits}b}")

            if prev is None:
                prev = (cs, sclk, mosi)
                in_frame = active
                continue

            p_cs, p_sclk, p_mosi = prev
            p_active = self._cs_active(p_cs)
            sclk_moved = sclk != p_sclk

            if active and not p_active:
                # The mode is latched at frame start (as the RTL does), so a
                # CONFIG change mid-frame only affects the *next* frame.
                frame_cpol, frame_cpha = cpol, cpha

            if active != p_active:
                if sclk_moved:
                    self._fail("SPI-1", f"SCLK {p_sclk}->{sclk} on the same cycle chip-select "
                               f"{'asserted' if active else 'deasserted'}")
                if sclk != frame_cpol:
                    self._fail("SPI-2", f"SCLK={sclk} at frame {'start' if active else 'end'}, "
                               f"expected CPOL={frame_cpol}")

            if active and not p_active:            # ---- frame start
                in_frame = True
                frame_edges = 0
                self.frames += 1
            elif active and sclk_moved:            # ---- SCLK edge inside a frame
                leading = (sclk != frame_cpol)     # idle -> active level
                sampling = leading if frame_cpha == 0 else not leading
                frame_edges += 1
                self.sclk_edges_total += 1
                if sampling:
                    self.sample_edges_checked += 1
                    if mosi != p_mosi:
                        self._fail("SPI-3", "MOSI changed on the same clock edge as a "
                                   "sampling SCLK edge")
            elif not active and p_active:          # ---- frame end
                if in_frame:
                    ok = frame_edges % (2 * self.width) == 0 and \
                        (frame_edges > 0 or self.allow_empty_frames)
                    if not ok:
                        self._fail("SPI-5", f"frame had {frame_edges} SCLK edges, expected a "
                                   f"non-zero multiple of {2 * self.width}")
                in_frame = False
            elif not active and sclk_moved:        # ---- SCLK moving with no CS
                if sclk != cpol:
                    self._fail("SPI-4", f"SCLK moved {p_sclk}->{sclk} while deselected, away "
                               f"from CPOL={cpol}")

            prev = (cs, sclk, mosi)
