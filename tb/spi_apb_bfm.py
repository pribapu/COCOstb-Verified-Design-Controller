"""
spi_apb_bfm.py -- minimal APB4 master driver for spi_apb_wrapper.

Drives the classic two-phase APB transaction (SETUP with penable=0, then
ACCESS with penable=1) and returns PSLVERR (write) or (data, PSLVERR) (read).
The DUT ties PREADY high, so every access completes in exactly one ACCESS
cycle -- no wait-state handling is needed here.
"""

from cocotb.triggers import RisingEdge

# ---- register map (mirrors the header comment in rtl/spi_apb_wrapper.sv) ----
REG_CTRL = 0x00
REG_CONFIG = 0x04
REG_DIVIDER = 0x08
REG_STATUS = 0x0C
REG_TXDATA = 0x10
REG_RXDATA = 0x14
REG_IRQ_STATUS = 0x18
REG_CS_CTRL = 0x1C

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
ST_CS_ACTIVE = 1 << 1
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

CS_HOLD = 1 << 8


def tx_count(status):
    return (status >> 8) & 0xFF


def rx_count(status):
    return (status >> 16) & 0xFF


def mode_bits(cpol, cpha, lsb=0):
    return (CFG_CPOL if cpol else 0) | (CFG_CPHA if cpha else 0) | (CFG_LSB if lsb else 0)


class APBMaster:
    def __init__(self, dut):
        self.dut = dut

    def idle(self):
        self.dut.psel.value = 0
        self.dut.penable.value = 0
        self.dut.pwrite.value = 0
        self.dut.paddr.value = 0
        self.dut.pwdata.value = 0

    async def write(self, addr, data):
        dut = self.dut
        await RisingEdge(dut.clk)
        dut.psel.value = 1
        dut.penable.value = 0
        dut.pwrite.value = 1
        dut.paddr.value = addr
        dut.pwdata.value = data
        await RisingEdge(dut.clk)
        dut.penable.value = 1
        await RisingEdge(dut.clk)
        assert int(dut.pready.value) == 1, "PREADY not high during ACCESS"
        err = int(dut.pslverr.value)
        self.idle()
        return err

    async def read(self, addr):
        dut = self.dut
        await RisingEdge(dut.clk)
        dut.psel.value = 1
        dut.penable.value = 0
        dut.pwrite.value = 0
        dut.paddr.value = addr
        await RisingEdge(dut.clk)
        dut.penable.value = 1
        await RisingEdge(dut.clk)
        assert int(dut.pready.value) == 1, "PREADY not high during ACCESS"
        data = int(dut.prdata.value)
        err = int(dut.pslverr.value)
        self.idle()
        return data, err
