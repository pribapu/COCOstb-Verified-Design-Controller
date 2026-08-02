"""
spi_apb_bfm.py -- minimal APB4 master driver for spi_apb_wrapper.

Drives the classic two-phase APB transaction (SETUP with penable=0, then
ACCESS with penable=1) and returns PSLVERR (write) or (data, PSLVERR) (read).
The DUT ties PREADY high, so every access completes in exactly one ACCESS
cycle -- no wait-state handling is needed here.
"""

from cocotb.triggers import RisingEdge

REG_CTRL = 0x00
REG_CONFIG = 0x04
REG_DIVIDER = 0x08
REG_STATUS = 0x0C
REG_TXDATA = 0x10
REG_RXDATA = 0x14
REG_IRQ_STATUS = 0x18


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
