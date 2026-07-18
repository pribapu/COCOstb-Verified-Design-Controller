"""
test_spi_master.py -- cocotb testbench for the SPI master controller.

Structure:
  * Driver      : do_transfer() drives the CPU-side handshake for one transfer.
  * BFM/Monitor : SPISlaveBFM reacts on the SPI pins as an independent slave and
                  records what it observed (edge count, idle level, rx word).
  * Scoreboard  : every transfer is checked against spi_reference() -- both the
                  master's rx and the slave's rx -- plus pin-level protocol checks.
  * Coverage    : each transfer samples the functional-coverage model.

Tests:
  * test_smoke                  : one transfer, mode 0.
  * test_reset                  : outputs sane out of reset.
  * test_directed_all_modes     : full sweep of mode x order x divider x special data.
  * test_back_to_back           : consecutive transfers with no idle gap.
  * test_sclk_timing            : SCLK half-period == (clk_div + 1) clks, several dividers.
  * test_busy_cs_timing         : busy/cs_n assert/deassert on the exact expected cycles.
  * test_start_ignored_while_busy : a spurious start mid-transfer can't restart/corrupt it.
  * test_reset_mid_transfer     : reset asserted mid-transfer aborts safely and recovers.
  * test_variable_idle_gaps     : transfers separated by random idle gaps, mode changes.
  * test_constrained_random     : randomized regression; drives coverage to goal.
"""

import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from spi_bfm import SPISlaveBFM, spi_reference
from spi_coverage import Coverage

CLK_NS = 10
COV = Coverage()
COV_FILE = os.environ.get("SPI_COV_FILE", "spi_cov.json")


def _width(dut):
    return len(dut.tx_data)


async def _start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


async def _reset(dut):
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.cpol.value = 0
    dut.cpha.value = 0
    dut.lsb_first.value = 0
    dut.clk_div.value = 0
    dut.tx_data.value = 0
    dut.miso.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def do_transfer(dut, bfm, cpol, cpha, lsb, div, tx, slave_tx, check=True):
    """Drive one SPI transfer and self-check the result."""
    w = _width(dut)
    mask = (1 << w) - 1
    tx &= mask
    slave_tx &= mask

    # configure DUT
    dut.cpol.value = cpol
    dut.cpha.value = cpha
    dut.lsb_first.value = lsb
    dut.clk_div.value = div
    dut.tx_data.value = tx
    # configure slave BFM for this transfer
    bfm.cpol = cpol
    bfm.cpha = cpha
    bfm.lsb_first = lsb
    bfm.tx_byte = slave_tx
    bfm.received = None

    # single-cycle start pulse
    await RisingEdge(dut.clk)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # wait for completion
    timeout = 5000
    while True:
        await RisingEdge(dut.clk)
        if int(dut.done.value) == 1:
            break
        timeout -= 1
        assert timeout > 0, "transfer timed out (done never asserted)"

    rx = int(dut.rx_data.value)
    # let the BFM finish its final sample
    await RisingEdge(dut.clk)

    if check:
        exp_m, exp_s = spi_reference(tx, slave_tx, w, cpol, cpha, lsb)
        mode = cpol * 2 + cpha
        ctx = (f"mode={mode} lsb={lsb} div={div} w={w} "
               f"tx=0x{tx:0{ (w+3)//4 }X} slave_tx=0x{slave_tx:0{ (w+3)//4 }X}")
        assert rx == exp_m, f"master RX mismatch: got 0x{rx:X} exp 0x{exp_m:X} [{ctx}]"
        assert bfm.received == exp_s, (
            f"slave RX mismatch: got {bfm.received} exp 0x{exp_s:X} [{ctx}]")
        assert bfm.edge_count == 2 * w, (
            f"edge count {bfm.edge_count} exp {2*w} [{ctx}]")
        assert bfm.idle_ok, f"SCLK not idling at CPOL [{ctx}]"
        COV.sample(cpol, cpha, lsb, w, div, tx, slave_tx)

    return rx


async def _measure_half_periods(dut, n_toggles):
    """Cycle count between each pair of consecutive SCLK toggles.

    Deliberately ignores the interval *before* the first toggle: that
    interval also includes the CS-setup half-period and so is twice as long
    as the steady-state edge spacing -- a separate, already-documented
    behaviour, not what this helper measures. Returns ``n_toggles - 1`` gaps.
    """
    periods = []
    prev_level = int(dut.sclk.value)
    cycles = 0
    seen_first = False
    while len(periods) < n_toggles - 1:
        await RisingEdge(dut.clk)
        cycles += 1
        cur = int(dut.sclk.value)
        if cur != prev_level:
            prev_level = cur
            if seen_first:
                periods.append(cycles)
            seen_first = True
            cycles = 0
    return periods


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_smoke(dut):
    """One mode-0 transfer end to end."""
    await _start_clock(dut)
    await _reset(dut)
    bfm = SPISlaveBFM(dut, _width(dut))
    cocotb.start_soon(bfm.run())
    rx = await do_transfer(dut, bfm, cpol=0, cpha=0, lsb=0, div=1,
                           tx=0xA5, slave_tx=0x3C)
    dut._log.info(f"smoke rx=0x{rx:X}")


@cocotb.test()
async def test_reset(dut):
    """Outputs are in a safe state after reset."""
    await _start_clock(dut)
    await _reset(dut)
    assert int(dut.cs_n.value) == 1, "cs_n should be deasserted after reset"
    assert int(dut.busy.value) == 0, "busy should be low after reset"
    assert int(dut.done.value) == 0, "done should be low after reset"


@cocotb.test()
async def test_directed_all_modes(dut):
    """Exhaustive sweep of mode x order x divider x special data patterns."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())
    mask = (1 << w) - 1
    specials = [0x00 & mask, 0xFF & mask, 0xAA & mask, 0x55 & mask]
    for cpol in (0, 1):
        for cpha in (0, 1):
            for lsb in (0, 1):
                for div in (0, 3):
                    for tx in specials + [random.randint(1, mask - 1)]:
                        stx = specials[(tx ^ div) % 4] if tx in specials \
                            else random.randint(1, mask - 1)
                        await do_transfer(dut, bfm, cpol, cpha, lsb, div, tx, stx)


@cocotb.test()
async def test_back_to_back(dut):
    """Consecutive transfers issued with no idle cycles between them."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())
    mask = (1 << w) - 1
    for i in range(8):
        await do_transfer(dut, bfm, cpol=0, cpha=1, lsb=0, div=0,
                          tx=random.randint(0, mask), slave_tx=random.randint(0, mask))


@cocotb.test()
async def test_sclk_timing(dut):
    """SCLK half-period must equal (clk_div + 1) system-clock cycles."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())

    for div in (0, 1, 4, 15):
        mon = cocotb.start_soon(_measure_half_periods(dut, 2 * w))
        await do_transfer(dut, bfm, cpol=0, cpha=0, lsb=0, div=div,
                          tx=0x5A, slave_tx=0xC3)
        periods = await mon
        expected = div + 1
        assert periods and all(p == expected for p in periods), (
            f"div={div}: expected steady {expected}-cycle half-periods, "
            f"got {periods}")


@cocotb.test()
async def test_busy_cs_timing(dut):
    """busy/cs_n assert the cycle after `start` and deassert the cycle `done` pulses,
    with no glitches in between."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())

    assert int(dut.busy.value) == 0
    assert int(dut.cs_n.value) == 1

    dut.cpol.value = 0
    dut.cpha.value = 0
    dut.lsb_first.value = 0
    dut.clk_div.value = 1
    dut.tx_data.value = 0x3C & ((1 << w) - 1)
    bfm.cpol = 0
    bfm.cpha = 0
    bfm.lsb_first = 0
    bfm.tx_byte = 0x81 & ((1 << w) - 1)

    await RisingEdge(dut.clk)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    assert int(dut.busy.value) == 1, "busy did not assert the cycle after start"
    assert int(dut.cs_n.value) == 0, "cs_n did not assert the cycle after start"

    timeout = 5000
    while True:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "transfer timed out (done never asserted)"
        if int(dut.done.value) == 1:
            assert int(dut.busy.value) == 0, "busy must drop the cycle done pulses"
            assert int(dut.cs_n.value) == 1, "cs_n must deassert the cycle done pulses"
            break
        assert int(dut.busy.value) == 1, "busy glitched low mid-transfer"
        assert int(dut.cs_n.value) == 0, "cs_n glitched high mid-transfer"

    await RisingEdge(dut.clk)


@cocotb.test()
async def test_start_ignored_while_busy(dut):
    """A spurious `start` pulse asserted mid-transfer must not restart or corrupt it."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    mask = (1 << w) - 1
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())

    dut.cpol.value = 0
    dut.cpha.value = 0
    dut.lsb_first.value = 0
    dut.clk_div.value = 2
    tx, stx = 0x96 & mask, 0x69 & mask
    dut.tx_data.value = tx
    bfm.cpol = 0
    bfm.cpha = 0
    bfm.lsb_first = 0
    bfm.tx_byte = stx

    await RisingEdge(dut.clk)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    done_count = 0

    async def _count_done():
        nonlocal done_count
        while True:
            await RisingEdge(dut.clk)
            if int(dut.done.value) == 1:
                done_count += 1

    monitor = cocotb.start_soon(_count_done())

    # A few cycles into the transfer, fire a spurious start with different
    # data -- since the DUT only samples `start` while idle, this must be a
    # no-op for the in-flight transfer.
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.start.value = 1
    dut.tx_data.value = tx ^ mask
    await RisingEdge(dut.clk)
    dut.start.value = 0

    timeout = 5000
    while done_count == 0:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "transfer timed out (done never asserted)"

    monitor.kill()
    assert done_count == 1, f"expected exactly one done pulse, got {done_count}"

    rx = int(dut.rx_data.value)
    exp_m, _ = spi_reference(tx, stx, w, 0, 0, 0)
    assert rx == exp_m, (
        f"spurious start corrupted the transfer: got 0x{rx:X} exp 0x{exp_m:X}")
    assert bfm.edge_count == 2 * w, f"edge count {bfm.edge_count} exp {2*w}"
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_reset_mid_transfer(dut):
    """Reset asserted mid-transfer must abort safely, leaving the controller
    ready for a normal transfer immediately after."""
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    mask = (1 << w) - 1
    bfm = SPISlaveBFM(dut, w)
    bfm_task = cocotb.start_soon(bfm.run())

    dut.cpol.value = 0
    dut.cpha.value = 0
    dut.lsb_first.value = 0
    dut.clk_div.value = 2
    dut.tx_data.value = 0xAA & mask
    bfm.cpol = 0
    bfm.cpha = 0
    bfm.lsb_first = 0
    bfm.tx_byte = 0x55 & mask

    await RisingEdge(dut.clk)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    for _ in range(3):
        await RisingEdge(dut.clk)
    assert int(dut.busy.value) == 1, "expected transfer in flight before reset"

    dut.rst_n.value = 0
    dut.start.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    assert int(dut.busy.value) == 0, "busy must clear on reset"
    assert int(dut.done.value) == 0, "done must clear on reset"
    assert int(dut.cs_n.value) == 1, "cs_n must deassert on reset"

    # Retire the stale BFM (still mid-transfer against the aborted exchange)
    # before starting a fresh one, so only one coroutine ever drives miso.
    bfm_task.kill()
    dut.miso.value = 0
    bfm2 = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm2.run())
    await do_transfer(dut, bfm2, cpol=1, cpha=1, lsb=1, div=0,
                      tx=0x3C & mask, slave_tx=0xC3 & mask)


@cocotb.test()
async def test_variable_idle_gaps(dut):
    """Transfers separated by varying idle gaps (including zero), with the
    mode/order/divider changing each time -- CS re-arming must not depend on
    how long the bus has been idle."""
    seed = int(os.environ.get("SPI_SEED", "1")) + 1
    random.seed(seed)
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())
    mask = (1 << w) - 1

    for _ in range(30):
        gap = random.choice([0, 0, 1, 2, 5, 20])
        for _ in range(gap):
            await RisingEdge(dut.clk)
        cpol = random.randint(0, 1)
        cpha = random.randint(0, 1)
        lsb = random.randint(0, 1)
        div = random.choice([0, 1, 3])
        await do_transfer(dut, bfm, cpol, cpha, lsb, div,
                          random.randint(0, mask), random.randint(0, mask))


@cocotb.test()
async def test_constrained_random(dut):
    """Constrained-random regression; also fills any remaining coverage bins."""
    seed = int(os.environ.get("SPI_SEED", "1"))
    random.seed(seed)
    await _start_clock(dut)
    await _reset(dut)
    w = _width(dut)
    bfm = SPISlaveBFM(dut, w)
    cocotb.start_soon(bfm.run())
    mask = (1 << w) - 1

    for _ in range(200):
        cpol = random.randint(0, 1)
        cpha = random.randint(0, 1)
        lsb = random.randint(0, 1)
        div = random.choice([0, 0, 1, 2, 4])   # bias toward fast
        tx = random.randint(0, mask)
        stx = random.randint(0, mask)
        await do_transfer(dut, bfm, cpol, cpha, lsb, div, tx, stx)

    # Persist + merge coverage and print the report.
    COV.merge_file(COV_FILE)
    report, overall = COV.report()
    dut._log.info(report)
    # This build's own axes (everything except cross-width) must be complete.
    _, per_build = COV.report()
    dut._log.info(f"Overall coverage after merge: {overall:.1f}%")
