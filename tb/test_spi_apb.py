"""
test_spi_apb.py -- cocotb testbench for spi_apb_wrapper (register file, TX/RX
FIFOs, auto-transfer engine, IRQ).

Uses the same SPISlaveBFM / spi_reference model as test_spi_master.py on the
wrapper's pass-through SPI pins -- the underlying protocol timing is already
proven by test_spi_master.py, so these tests focus on the APB-side contract:
register read/write semantics, FIFO fill/drain/overrun, flush, and IRQ
masking, plus one end-to-end burst that exercises the whole datapath.

Note: TX_EMPTY sets the instant the engine *pops* the last queued byte --
i.e. when the final transfer *starts*, not when it finishes. Waiting for a
transfer (or burst) to actually complete therefore polls for
"TX_EMPTY and not BUSY" (see wait_engine_idle) or a growing RX FIFO count,
never TX_EMPTY alone.

Assumes the default elaboration (DATA_WIDTH=8, FIFO_DEPTH=8) -- DATA_WIDTH
sweeps are already covered by test_spi_master.py against the un-wrapped core.
"""

import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from spi_apb_bfm import (
    APBMaster,
    REG_CONFIG,
    REG_CTRL,
    REG_DIVIDER,
    REG_IRQ_STATUS,
    REG_RXDATA,
    REG_STATUS,
    REG_TXDATA,
)
from spi_bfm import SPISlaveBFM, spi_reference
from spi_coverage import Coverage

CLK_NS = 10
WIDTH = 8
FIFO_DEPTH = 8

CTRL_EN = 1 << 0
CTRL_TX_FLUSH = 1 << 1
CTRL_RX_FLUSH = 1 << 2

CFG_CPOL = 1 << 0
CFG_CPHA = 1 << 1
CFG_LSB = 1 << 2
CFG_IRQ_EN_DONE = 1 << 3
CFG_IRQ_EN_TXE = 1 << 4
CFG_IRQ_EN_RXF = 1 << 5
CFG_IRQ_EN_OV = 1 << 6

ST_BUSY = 1 << 0
ST_TX_EMPTY = 1 << 2
ST_TX_FULL = 1 << 3
ST_RX_EMPTY = 1 << 4
ST_RX_FULL = 1 << 5
ST_RX_OVERRUN = 1 << 6
ST_TX_DROPPED = 1 << 7

IRQ_DONE = 1 << 0
IRQ_TXE = 1 << 1
IRQ_RXF = 1 << 2
IRQ_OV = 1 << 3

COV = Coverage()
COV_FILE = os.environ.get("SPI_COV_FILE", "spi_cov_apb.json")


async def _start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


async def _reset(dut):
    dut.rst_n.value = 0
    dut.psel.value = 0
    dut.penable.value = 0
    dut.pwrite.value = 0
    dut.paddr.value = 0
    dut.pwdata.value = 0
    dut.miso.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def _tx_count(status):
    return (status >> 8) & 0xFF


def _rx_count(status):
    return (status >> 16) & 0xFF


def _fifo_bin(count):
    if count == 0:
        return "empty"
    if count == FIFO_DEPTH:
        return "full"
    return "partial"


async def status_of(apb):
    st, err = await apb.read(REG_STATUS)
    assert err == 0
    return st


async def wait_engine_idle(dut, apb, timeout=20000):
    """Wait until every queued transfer has both started and finished."""
    while True:
        st = await status_of(apb)
        if (st & ST_TX_EMPTY) and not (st & ST_BUSY):
            # The edge-detected IRQ_STATUS sticky bits (TX_EMPTY/RX_FULL) are
            # one extra clock behind the FIFO counts they're derived from;
            # give them a few cycles to settle before a caller checks them.
            for _ in range(4):
                await RisingEdge(dut.clk)
            return await status_of(apb)
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "engine never returned to idle"


async def wait_rx_count(dut, apb, n, timeout=20000):
    while _rx_count(await status_of(apb)) < n:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, f"RX FIFO never reached count {n}"


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_apb_reset(dut):
    """After reset: idle status, no IRQ, PSLVERR clear."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)

    st = await status_of(apb)
    assert st == (ST_TX_EMPTY | ST_RX_EMPTY), f"unexpected reset STATUS 0x{st:X}"
    irqst, err = await apb.read(REG_IRQ_STATUS)
    assert irqst == 0 and err == 0
    assert int(dut.irq.value) == 0
    COV.sample_apb(reg="status")
    COV.sample_apb(reg="irq_status")


@cocotb.test()
async def test_reg_rw(dut):
    """Register read/write semantics: RW regs read back, WO/RO regs behave, bad address errors."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)

    # CONFIG: only bits [6:0] are stored, upper bits read back as 0.
    err = await apb.write(REG_CONFIG, 0xFFFFFFFF)
    assert err == 0
    val, err = await apb.read(REG_CONFIG)
    assert val == 0x7F, f"CONFIG readback 0x{val:X}"
    COV.sample_apb(reg="config")

    # DIVIDER: full DIV_WIDTH readback.
    await apb.write(REG_DIVIDER, 0x1234)
    val, err = await apb.read(REG_DIVIDER)
    assert val == 0x1234, f"DIVIDER readback 0x{val:X}"
    COV.sample_apb(reg="divider")

    # CTRL: EN sticks, FLUSH bits are pulses (never stored / always read 0).
    await apb.write(REG_CTRL, CTRL_EN | CTRL_TX_FLUSH | CTRL_RX_FLUSH)
    val, err = await apb.read(REG_CTRL)
    assert val == CTRL_EN, f"CTRL readback 0x{val:X}"
    await apb.write(REG_CTRL, 0)  # disable EN again for the rest of the test
    COV.sample_apb(reg="ctrl")

    # TXDATA is write-only: push one byte, confirm via STATUS, read returns 0.
    err = await apb.write(REG_TXDATA, 0xA5)
    assert err == 0
    st = await status_of(apb)
    assert _tx_count(st) == 1 and not (st & ST_TX_EMPTY)
    rd, err = await apb.read(REG_TXDATA)
    assert rd == 0, "TXDATA read should return 0"
    await apb.write(REG_CTRL, CTRL_TX_FLUSH)  # clean up
    COV.sample_apb(reg="txdata")

    # RXDATA empty read: returns 0, does not underflow / error.
    rd, err = await apb.read(REG_RXDATA)
    assert rd == 0 and err == 0
    COV.sample_apb(reg="rxdata")

    # Unmapped offset -> PSLVERR.
    _, err = await apb.read(0x1C)
    assert err == 1, "expected PSLVERR on unmapped offset"


@cocotb.test()
async def test_config_latching(dut):
    """CONFIG changed mid-transfer only affects the *next* transfer."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 0, 0, 0x3C

    await apb.write(REG_CONFIG, 0)  # mode 0 (cpol=0, cpha=0)
    await apb.write(REG_TXDATA, 0xA5)
    await apb.write(REG_CTRL, CTRL_EN)

    # Wait for the transfer to actually start, then switch CONFIG mid-flight.
    timeout = 5000
    while not (await status_of(apb)) & ST_BUSY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "transfer never started"
    await apb.write(REG_CONFIG, CFG_CPOL | CFG_CPHA | CFG_LSB)  # mode 3, lsb-first

    # Wait for it to finish; it must still reflect the mode latched at start.
    timeout = 5000
    while (await status_of(apb)) & ST_RX_EMPTY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "transfer never completed"
    rx, err = await apb.read(REG_RXDATA)
    exp_m, _ = spi_reference(0xA5, 0x3C, WIDTH, 0, 0, 0)
    assert rx == exp_m, f"in-flight transfer used the new CONFIG: got 0x{rx:X} exp 0x{exp_m:X}"
    assert bfm.idle_ok and bfm.edge_count == 2 * WIDTH

    await apb.write(REG_CTRL, 0)
    COV.sample_apb(irq_source="done")


@cocotb.test()
async def test_tx_fifo_fill_drain(dut):
    """Fill the TX FIFO to full, verify overflow drop, then drain via hardware."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 0, 0, 0x00

    pushed = [0x10 + i for i in range(FIFO_DEPTH)]
    for i, b in enumerate(pushed):
        await apb.write(REG_TXDATA, b)
        st = await status_of(apb)
        assert _tx_count(st) == i + 1
        COV.sample_apb(tx_fifo_state=_fifo_bin(_tx_count(st)))
    st = await status_of(apb)
    assert st & ST_TX_FULL

    # One more push while full: dropped, TX_DROPPED sets, count unchanged.
    await apb.write(REG_TXDATA, 0xEE)
    st = await status_of(apb)
    assert st & ST_TX_DROPPED and _tx_count(st) == FIFO_DEPTH

    # Clear TX_DROPPED (W1C) and confirm it clears.
    await apb.write(REG_STATUS, ST_TX_DROPPED)
    st = await status_of(apb)
    assert not (st & ST_TX_DROPPED)

    # Enable the engine and let hardware drain + complete all FIFO_DEPTH bytes.
    await apb.write(REG_CTRL, CTRL_EN)
    await wait_engine_idle(dut, apb)

    # IRQ_STATUS.TX_EMPTY must have latched on the drain-to-empty edge.
    irqst, _ = await apb.read(REG_IRQ_STATUS)
    assert irqst & IRQ_TXE
    COV.sample_apb(irq_source="tx_empty")

    # Pop the RX FIFO and confirm order matches push order.
    for b in pushed:
        rx, err = await apb.read(REG_RXDATA)
        assert err == 0
        exp_m, _ = spi_reference(b, 0x00, WIDTH, 0, 0, 0)
        assert rx == exp_m, f"FIFO order mismatch: got 0x{rx:X} exp 0x{exp_m:X}"
    st = await status_of(apb)
    assert st & ST_RX_EMPTY
    COV.sample_apb(tx_fifo_state="empty", rx_fifo_state="empty")

    await apb.write(REG_CTRL, 0)


@cocotb.test()
async def test_rx_fifo_overrun(dut):
    """RX FIFO fills to depth, then a further completed transfer overruns it."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 0, 0, 0x55

    async def push_and_finish_one(byte):
        await apb.write(REG_TXDATA, byte)
        await apb.write(REG_CTRL, CTRL_EN)
        await wait_engine_idle(dut, apb)
        await apb.write(REG_CTRL, 0)

    # Fill RX FIFO to exactly full without ever reading RXDATA.
    for i in range(FIFO_DEPTH):
        await push_and_finish_one(0x20 + i)
        st = await status_of(apb)
        assert _rx_count(st) == i + 1
        COV.sample_apb(rx_fifo_state=_fifo_bin(_rx_count(st)))
    st = await status_of(apb)
    assert st & ST_RX_FULL and not (st & ST_RX_OVERRUN)
    irqst, _ = await apb.read(REG_IRQ_STATUS)
    assert irqst & IRQ_RXF, "IRQ_STATUS.RX_FULL must latch when the RX FIFO fills"
    COV.sample_apb(irq_source="rx_full")
    await apb.write(REG_IRQ_STATUS, IRQ_RXF)

    # One more completed transfer while RX FIFO is still full -> overrun.
    await push_and_finish_one(0x99)
    st = await status_of(apb)
    assert st & ST_RX_OVERRUN, "expected RX_OVERRUN after overflowing RX FIFO"
    assert _rx_count(st) == FIFO_DEPTH, "overrun byte must be dropped, not stored"
    irqst, _ = await apb.read(REG_IRQ_STATUS)
    assert irqst & IRQ_OV
    COV.sample_apb(irq_source="overrun")

    await apb.write(REG_IRQ_STATUS, IRQ_OV)
    await apb.write(REG_CTRL, CTRL_RX_FLUSH)


@cocotb.test()
async def test_irq_masking(dut):
    """irq only asserts for sources whose CONFIG.IRQ_EN_* bit is set."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 0, 0, 0x11

    async def one_transfer(irq_en_bits):
        await apb.write(REG_CONFIG, irq_en_bits)
        await apb.write(REG_TXDATA, 0x77)
        await apb.write(REG_CTRL, CTRL_EN)
        await wait_engine_idle(dut, apb)
        await apb.write(REG_CTRL, 0)

    # DONE masked off: sticky sets, but irq stays low.
    await one_transfer(0)
    assert int(dut.irq.value) == 0, "irq must stay low while IRQ_EN_DONE=0"
    irqst, _ = await apb.read(REG_IRQ_STATUS)
    assert irqst & IRQ_DONE
    await apb.write(REG_IRQ_STATUS, IRQ_DONE)  # clear before re-enabling

    # DONE enabled: irq asserts on the next transfer, clears on W1C.
    await one_transfer(CFG_IRQ_EN_DONE)
    assert int(dut.irq.value) == 1, "irq should assert: DONE fired and enabled"
    await apb.write(REG_IRQ_STATUS, IRQ_DONE)
    await RisingEdge(dut.clk)
    assert int(dut.irq.value) == 0, "irq should clear after W1C"

    await apb.write(REG_CONFIG, 0)
    await apb.write(REG_STATUS, ST_TX_DROPPED)


@cocotb.test()
async def test_burst_end_to_end(dut):
    """Push N bytes, enable the engine, and cross-check the full hardware-driven burst."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    slave_tx = 0x66
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 1, 0, slave_tx

    await apb.write(REG_CONFIG, CFG_CPHA)  # mode 1 (cpol=0, cpha=1)
    pushed = [0x01, 0x23, 0x45, 0x67, 0x89]
    for b in pushed:
        await apb.write(REG_TXDATA, b)
    await apb.write(REG_CTRL, CTRL_EN)

    await wait_rx_count(dut, apb, len(pushed))
    await wait_engine_idle(dut, apb)
    await apb.write(REG_CTRL, 0)

    for b in pushed:
        rx, err = await apb.read(REG_RXDATA)
        assert err == 0
        exp_m, _ = spi_reference(b, slave_tx, WIDTH, 0, 1, 0)
        assert rx == exp_m, f"burst mismatch: got 0x{rx:X} exp 0x{exp_m:X} (tx=0x{b:X})"
    st = await status_of(apb)
    assert st & ST_TX_EMPTY and st & ST_RX_EMPTY


@cocotb.test()
async def test_flush(dut):
    """TX_FLUSH/RX_FLUSH clear queued (not in-flight) data without corrupting a transfer."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH)
    cocotb.start_soon(bfm.run())
    bfm.cpol, bfm.cpha, bfm.lsb_first, bfm.tx_byte = 0, 0, 0, 0x2A

    # Flush an idle, non-empty TX FIFO.
    await apb.write(REG_TXDATA, 0x01)
    await apb.write(REG_TXDATA, 0x02)
    await apb.write(REG_CTRL, CTRL_TX_FLUSH)
    st = await status_of(apb)
    assert st & ST_TX_EMPTY and _tx_count(st) == 0

    # Push two bytes, enable the engine, flush mid-flight after byte #1 starts.
    await apb.write(REG_TXDATA, 0xAA)
    await apb.write(REG_TXDATA, 0xBB)
    await apb.write(REG_CTRL, CTRL_EN)
    timeout = 5000
    while not (await status_of(apb)) & ST_BUSY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "engine never started"
    await apb.write(REG_CTRL, CTRL_EN | CTRL_TX_FLUSH)  # keep EN, flush queued byte #2

    timeout = 5000
    while (await status_of(apb)) & ST_RX_EMPTY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "in-flight transfer never completed"
    st = await status_of(apb)
    assert _tx_count(st) == 0, "flushed byte must not have been sent"
    assert _rx_count(st) == 1, "only the in-flight transfer should have completed"
    rx, _ = await apb.read(REG_RXDATA)
    exp_m, _ = spi_reference(0xAA, 0x2A, WIDTH, 0, 0, 0)
    assert rx == exp_m, "in-flight transfer's data must not be corrupted by the flush"
    assert bfm.idle_ok and bfm.edge_count == 2 * WIDTH

    await apb.write(REG_CTRL, CTRL_RX_FLUSH)

    # RX flush.
    await apb.write(REG_TXDATA, 0x03)
    await apb.write(REG_CTRL, CTRL_EN)
    timeout = 5000
    while (await status_of(apb)) & ST_RX_EMPTY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0
    await apb.write(REG_CTRL, CTRL_RX_FLUSH)
    st = await status_of(apb)
    assert st & ST_RX_EMPTY and _rx_count(st) == 0


@cocotb.test()
async def test_apb_coverage_report(dut):
    """Persist + report APB-side coverage (mirrors test_constrained_random's role)."""
    await _start_clock(dut)
    await _reset(dut)
    COV.merge_file(COV_FILE)
    report, overall = COV.report()
    dut._log.info(report)
    dut._log.info(f"APB coverage after merge: {overall:.1f}%")
