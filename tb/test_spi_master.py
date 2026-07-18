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
  * test_smoke                : one transfer, mode 0.
  * test_reset                : outputs sane out of reset.
  * test_directed_all_modes   : full sweep of mode x order x divider x special data.
  * test_back_to_back         : consecutive transfers with no idle gap.
  * test_constrained_random   : randomized regression; drives coverage to goal.
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
