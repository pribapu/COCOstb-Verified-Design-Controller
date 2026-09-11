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
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from spi_apb_bfm import (
    CFG_CPHA,
    CFG_CPOL,
    CFG_IRQ_EN_DONE,
    CFG_LSB,
    CS_HOLD,
    CTRL_EN,
    CTRL_RX_FLUSH,
    CTRL_TX_FLUSH,
    IRQ_DONE,
    IRQ_OV,
    IRQ_RXF,
    IRQ_TXE,
    REG_CONFIG,
    REG_CS_CTRL,
    REG_CTRL,
    REG_DIVIDER,
    REG_IRQ_STATUS,
    REG_RXDATA,
    REG_STATUS,
    REG_TXDATA,
    ST_BUSY,
    ST_CS_ACTIVE,
    ST_RX_EMPTY,
    ST_RX_FULL,
    ST_RX_OVERRUN,
    ST_TX_DROPPED,
    ST_TX_EMPTY,
    ST_TX_FULL,
    APBMaster,
    mode_bits,
)
from spi_bfm import SPISlaveBFM, spi_reference
from spi_coverage import Coverage
from spi_flash_driver import SpiFlashDriver
from spi_flash_model import CMD_PP, CMD_SE, SPIFlashModel
from spi_protocol_checker import SPIProtocolChecker

CLK_NS = 10
WIDTH = 8
FIFO_DEPTH = 8
NUM_CS = 4
# Generous sim-time budget per test (the longest, the flash end-to-end, needs
# ~120 us): a DUT bug that stalls the bus fails the test instead of hanging.
TEST_TIMEOUT_US = 2000

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


def start_slave(dut, cs_index=0, checker=True):
    """Slave BFM on chip-select line `cs_index`, plus (by default) the
    independent pin-level protocol checker watching the whole bus."""
    bfm = SPISlaveBFM(dut, WIDTH, cs_index=cs_index)
    bfm.task = cocotb.start_soon(bfm.run())
    if checker:
        cocotb.start_soon(SPIProtocolChecker(dut, WIDTH, cfg=bfm,
                                             allow_empty_frames=True).run())
    return bfm


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
@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
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


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
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
    _, err = await apb.read(0x20)
    assert err == 1, "expected PSLVERR on unmapped offset"


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_config_latching(dut):
    """CONFIG changed mid-transfer only affects the *next* transfer."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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
    # Tell the BFM/checker the *configured* mode is now 3: the in-flight frame
    # keeps its latched mode 0 (the BFM latches per transfer, the checker per
    # frame), but once idle SCLK must follow the new CPOL.
    bfm.cpol, bfm.cpha, bfm.lsb_first = 1, 1, 1

    # Wait for it to finish; it must still reflect the mode latched at start.
    timeout = 5000
    while (await status_of(apb)) & ST_RX_EMPTY:
        await RisingEdge(dut.clk)
        timeout -= 1
        assert timeout > 0, "transfer never completed"
    rx, err = await apb.read(REG_RXDATA)
    assert err == 0
    exp_m, _ = spi_reference(0xA5, 0x3C, WIDTH, 0, 0, 0)
    assert rx == exp_m, f"in-flight transfer used the new CONFIG: got 0x{rx:X} exp 0x{exp_m:X}"
    assert bfm.idle_ok and bfm.edge_count == 2 * WIDTH

    await apb.write(REG_CTRL, 0)
    COV.sample_apb(irq_source="done")


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_tx_fifo_fill_drain(dut):
    """Fill the TX FIFO to full, verify overflow drop, then drain via hardware."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_rx_fifo_overrun(dut):
    """RX FIFO fills to depth, then a further completed transfer overruns it."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_irq_masking(dut):
    """irq only asserts for sources whose CONFIG.IRQ_EN_* bit is set."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_burst_end_to_end(dut):
    """Push N bytes, enable the engine, and cross-check the full hardware-driven burst."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_flush(dut):
    """TX_FLUSH/RX_FLUSH clear queued (not in-flight) data without corrupting a transfer."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
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


class BusMode:
    """Checker `cfg` for a bus with several devices in different SPI modes:
    inside a frame, the expected mode is that of the device whose chip-select
    is asserted; while deselected, SCLK may only move to the CPOL software
    last configured (drivers report it via on_select)."""

    def __init__(self, dut, devices):
        self.dut = dut
        self.devices = devices          # {cs_index: object with cpol/cpha}
        self.idle_cpol = 0

    def _selected(self):
        asserted = ~int(self.dut.cs_n.value) & ((1 << NUM_CS) - 1)
        for i, dev in self.devices.items():
            if asserted >> i & 1:
                return dev
        return None

    @property
    def cpol(self):
        dev = self._selected()
        return dev.cpol if dev else self.idle_cpol

    @property
    def cpha(self):
        dev = self._selected()
        return dev.cpha if dev else 0

    def driver_selected(self, drv):
        self.idle_cpol = drv.cpol


class InferredCpol:
    """Checker `cfg` for when a CPOL change races the engine and the test
    can't know which CPOL a frame will use: take it from the bus (SCLK's level
    as the frame starts). SPI-1 (no SCLK edge coincident with a CS edge),
    SPI-3 and SPI-5 stay fully armed."""

    def __init__(self, dut):
        self.dut = dut
        self.cpha = 0

    @property
    def cpol(self):
        return int(self.dut.sclk.value)


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_unmapped_access_no_side_effects(dut):
    """An access that returns PSLVERR must not touch any register or FIFO.

    (Regression: the original decode only looked at paddr[4:2] for the write
    strobes, so e.g. a write to 0x20 was flagged PSLVERR *and* aliased onto
    CTRL, silently enabling the engine.)
    """
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = start_slave(dut)
    bfm.tx_byte = 0x3C

    alias = 0x20
    assert await apb.write(alias | REG_CTRL, CTRL_EN) == 1
    assert await apb.write(alias | REG_CONFIG, 0x7F) == 1
    assert await apb.write(alias | REG_DIVIDER, 0xFF) == 1
    assert await apb.write(alias | REG_CS_CTRL, CS_HOLD | 3) == 1
    assert await apb.write(alias | REG_TXDATA, 0x55) == 1
    for reg in (REG_CTRL, REG_CONFIG, REG_DIVIDER, REG_CS_CTRL):
        val, err = await apb.read(reg)
        assert val == 0 and err == 0, f"reg 0x{reg:02X} changed by an errored write: 0x{val:X}"
    st = await status_of(apb)
    assert _tx_count(st) == 0 and not st & ST_CS_ACTIVE

    # One real transfer so RXDATA has something an aliased read could pop.
    await apb.write(REG_TXDATA, 0xA5)
    await apb.write(REG_CTRL, CTRL_EN)
    await wait_rx_count(dut, apb, 1)
    val, err = await apb.read(alias | REG_RXDATA)
    assert err == 1 and val == 0
    assert _rx_count(await status_of(apb)) == 1, "errored read popped the RX FIFO"
    rx, _ = await apb.read(REG_RXDATA)
    assert rx == 0x3C
    await apb.write(REG_CTRL, 0)


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_cs_select_lines(dut):
    """Each CS_SEL value drives exactly its own chip-select line."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfms = [start_slave(dut, cs_index=i, checker=(i == 0)) for i in range(NUM_CS)]
    for i, b in enumerate(bfms):
        b.tx_byte = 0x10 + i
    await apb.write(REG_CTRL, CTRL_EN)

    for line in (2, 0, 3, 1):
        before = [b.frames for b in bfms]
        await apb.write(REG_CS_CTRL, line)
        await apb.write(REG_TXDATA, 0xC0 | line)
        await wait_rx_count(dut, apb, 1)
        await wait_engine_idle(dut, apb)
        rx, _ = await apb.read(REG_RXDATA)
        assert rx == 0x10 + line, f"line {line}: got 0x{rx:X} from the wrong device"
        for i, b in enumerate(bfms):
            exp = before[i] + (1 if i == line else 0)
            assert b.frames == exp, f"CS_SEL={line}: device {i} saw {b.frames - before[i]} frames"
        assert bfms[line].received == 0xC0 | line
        COV.sample_apb(cs_line=line)
    await apb.write(REG_CTRL, 0)


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_cs_hold_multiword_frame(dut):
    """CS_HOLD keeps one chip-select asserted across many words (longer than
    the FIFOs), then releases it; an empty hold frame is harmless."""
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH, cs_index=3)
    cocotb.start_soon(bfm.run())
    checker = SPIProtocolChecker(dut, WIDTH, cfg=bfm, allow_empty_frames=True)
    cocotb.start_soon(checker.run())
    dev = SpiFlashDriver(dut, apb, cs=3)
    await dev.init()

    for n in (12, 3, 1):
        words = [(0x31 * i + n) & 0xFF for i in range(n)]
        bfm.tx_queue = [(0xA0 + i) & 0xFF for i in range(n)]
        frames0, cframes0 = bfm.frames, checker.frames
        rx = await dev.xfer(words)
        assert bfm.frames == frames0 + 1 and checker.frames == cframes0 + 1, \
            "CS toggled inside a held frame"
        assert bfm.frame_words == words, f"slave saw {bfm.frame_words}, sent {words}"
        assert rx == [(0xA0 + i) & 0xFF for i in range(n)], f"master got {rx}"
        COV.sample_apb(frame_words=n)

    # Empty frame: hold set and released with nothing sent.
    await apb.write(REG_CS_CTRL, CS_HOLD | 3)
    for _ in range(3):
        await RisingEdge(dut.clk)
    assert (await status_of(apb)) & ST_CS_ACTIVE
    assert int(dut.cs_n.value) == 0b0111
    await apb.write(REG_CS_CTRL, 3)
    for _ in range(3):
        await RisingEdge(dut.clk)
    assert not (await status_of(apb)) & ST_CS_ACTIVE
    assert int(dut.cs_n.value) == 0b1111
    COV.sample_apb(reg="cs_ctrl")
    await apb.write(REG_CTRL, 0)


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_mode_switch_timing_sweep(dut):
    """Flip CPOL (mode 0 <-> 2) at every clock offset across two back-to-back
    queued words, so the CONFIG write lands on every cycle of the engine's
    word boundary. Whichever mode each word ends up using, the bus must stay
    protocol-clean (no SCLK edge on a CS edge) and the data must be intact.
    """
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)
    bfm = SPISlaveBFM(dut, WIDTH, cs_index=0)
    cocotb.start_soon(bfm.run())
    bfm.tx_byte = 0xC3
    checker = SPIProtocolChecker(dut, WIDTH, cfg=InferredCpol(dut))
    cocotb.start_soon(checker.run())

    cpol = 0
    for offset in range(44):
        await apb.write(REG_CONFIG, mode_bits(cpol, 0))
        words = [offset, 0xFF - offset]
        for w in words:
            await apb.write(REG_TXDATA, w)
        await apb.write(REG_CTRL, CTRL_EN)
        while int(dut.cs_n.value) & 1:
            await RisingEdge(dut.clk)
        for _ in range(offset):
            await RisingEdge(dut.clk)
        cpol ^= 1
        await apb.write(REG_CONFIG, mode_bits(cpol, 0))
        await wait_rx_count(dut, apb, 2)
        await wait_engine_idle(dut, apb)
        await apb.write(REG_CTRL, 0)
        for _ in range(2):
            rx, _ = await apb.read(REG_RXDATA)
            assert rx == 0xC3
        assert bfm.history[-2:] == words, f"offset {offset}: slave saw {bfm.history[-2:]}"
    assert checker.frames == 88


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_flash_end_to_end(dut):
    """Firmware-style driver -> APB -> controller -> pins -> SPI NOR flash
    model, sharing the bus with a second device in a different SPI mode.

    Expectations come from an image the test keeps itself, not from the model.
    """
    await _start_clock(dut)
    await _reset(dut)
    apb = APBMaster(dut)

    flash = SPIFlashModel(dut, cs_index=1)                 # mode 0
    cocotb.start_soon(flash.run())
    other = SPISlaveBFM(dut, WIDTH, cs_index=2)            # mode 3 register device
    other.cpol, other.cpha, other.tx_byte = 1, 1, 0x5A
    cocotb.start_soon(other.run())
    bus = BusMode(dut, {1: flash, 2: other})
    checker = SPIProtocolChecker(dut, WIDTH, cfg=bus, allow_empty_frames=True)
    cocotb.start_soon(checker.run())

    drv = SpiFlashDriver(dut, apb, cs=1, cpol=0, cpha=0, divider=1,
                         on_select=bus.driver_selected)
    dev2 = SpiFlashDriver(dut, apb, cs=2, cpol=1, cpha=1, on_select=bus.driver_selected)
    await drv.init()
    rng = random.Random(int(os.environ.get("SPI_SEED", "1")))

    assert await drv.jedec_id() == [0xEF, 0x40, 0x18]
    COV.sample_apb(flash_op="jedec_id")

    base = 0x012000                       # a 4 KB sector
    image = {}                            # what the flash *should* hold

    def expect(addr, n):
        return [image.get(addr + i, 0xFF) for i in range(n)]

    await drv.sector_erase(base)
    COV.sample_apb(flash_op="sector_erase")
    COV.sample_apb(flash_op="wren")
    assert await drv.read(base, 16) == expect(base, 16)
    COV.sample_apb(flash_op="read")

    # Page program without WREN must be ignored by the device.
    await drv.page_program(base, [0x00] * 4, wren=False)
    assert (CMD_PP, "WEL not set") in flash.ignored
    assert await drv.read(base, 4) == [0xFF] * 4
    COV.sample_apb(flash_op="no_wel_ignored")

    # 32-byte program starting 16 bytes before the end of a page: the last
    # 16 bytes wrap to the *start of the same page* (datasheet behaviour).
    data = [rng.randrange(256) for _ in range(32)]
    start = base + 0xF0
    await drv.page_program(start, data)
    for i, v in enumerate(data):
        image[base + ((0xF0 + i) & 0xFF)] = v
    assert await drv.read(start, 16) == expect(start, 16)
    assert await drv.fast_read(base, 16) == expect(base, 16)
    COV.sample_apb(flash_op="page_program")
    COV.sample_apb(flash_op="page_wrap")
    COV.sample_apb(flash_op="fast_read")

    # Talk to the mode-3 device in between: CS line and mode both switch.
    assert await dev2.xfer([0x11, 0x22, 0x33]) == [0x5A] * 3
    assert other.frame_words == [0x11, 0x22, 0x33]

    # NOR semantics: programming can only clear bits.
    await drv.page_program(base + 0x100, [0xF0])
    await drv.page_program(base + 0x100, [0x3F])
    image[base + 0x100] = 0xF0 & 0x3F
    assert await drv.read(base + 0x100, 1) == [0x30]
    COV.sample_apb(flash_op="nor_and")

    # A 48-byte streaming read across the page boundary: 6x the FIFO depth,
    # so it only works if the driver's flow control keeps up.
    assert await drv.read(base + 0xE0, 48) == expect(base + 0xE0, 48)
    COV.sample_apb(flash_op="long_stream")

    # Erase again and confirm everything reads blank.
    await drv.sector_erase(base)
    image.clear()
    assert await drv.fast_read(base + 0xF8, 16) == [0xFF] * 16

    assert flash.busy_polls > 0, "driver never observed BUSY -- polling untested"
    COV.sample_apb(flash_op="busy_poll")
    assert sum(1 for c, _ in flash.executed if c == CMD_SE) == 2
    dut._log.info(f"flash: {len(flash.executed)} commands executed, "
                  f"{flash.busy_polls} BUSY polls, {checker.frames} CS frames checked")
    await apb.write(REG_CTRL, 0)


@cocotb.test(timeout_time=TEST_TIMEOUT_US, timeout_unit="us")
async def test_apb_coverage_report(dut):
    """Persist + report APB-side coverage (mirrors test_constrained_random's role)."""
    await _start_clock(dut)
    await _reset(dut)
    COV.merge_file(COV_FILE)
    report, overall = COV.report()
    dut._log.info(report)
    dut._log.info(f"APB coverage after merge: {overall:.1f}%")
