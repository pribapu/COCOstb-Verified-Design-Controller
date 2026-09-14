"""
spi_flash_driver.py -- "firmware" SPI NOR flash driver that talks to the
flash exclusively through spi_apb_wrapper's APB registers, the way a CPU
would: select a chip-select line + mode, hold CS for the whole command, stream
bytes through the TX/RX FIFOs, release CS.

The streaming loop never lets more than FIFO_DEPTH words be outstanding
(pushed to TXDATA but not yet popped from RXDATA), so the RX FIFO can never
overrun no matter how long the transfer is -- the same flow-control rule a
real driver needs. It also asserts the controller never reports a dropped TX
word or an RX overrun.
"""

from cocotb.triggers import RisingEdge
from spi_apb_bfm import (
    CS_HOLD,
    CTRL_EN,
    REG_CONFIG,
    REG_CS_CTRL,
    REG_CTRL,
    REG_DIVIDER,
    REG_RXDATA,
    REG_STATUS,
    REG_TXDATA,
    ST_CS_ACTIVE,
    ST_RX_OVERRUN,
    ST_TX_DROPPED,
    ST_TX_FULL,
    mode_bits,
    rx_count,
)
from spi_flash_model import (
    CMD_FAST_READ,
    CMD_JEDEC,
    CMD_PP,
    CMD_RDSR,
    CMD_READ,
    CMD_SE,
    CMD_WREN,
    SR_BUSY,
)


class SpiFlashDriver:
    def __init__(self, dut, apb, cs, cpol=0, cpha=0, divider=0, fifo_depth=8, on_select=None):
        self.dut = dut
        self.apb = apb
        self.cs = cs
        self.cpol = cpol
        self.mode = mode_bits(cpol, cpha)
        self.on_select = on_select      # tells a bus monitor the configured mode
        self.divider = divider
        self.fifo_depth = fifo_depth
        self.frames = 0
        self.status_polls = 0

    async def init(self):
        await self.apb.write(REG_CTRL, CTRL_EN)

    async def _status(self):
        st, err = await self.apb.read(REG_STATUS)
        assert err == 0
        assert not st & (ST_TX_DROPPED | ST_RX_OVERRUN), f"controller lost data: STATUS=0x{st:X}"
        return st

    async def select(self):
        """Program this device's mode/divider and chip-select (no CS yet)."""
        await self.apb.write(REG_CONFIG, self.mode)
        if self.on_select:
            self.on_select(self)
        await self.apb.write(REG_DIVIDER, self.divider)
        await self.apb.write(REG_CS_CTRL, self.cs)

    async def xfer(self, out):
        """One CS-held frame: shift out `out`, return the bytes shifted in."""
        await self.select()
        await self.apb.write(REG_CS_CTRL, CS_HOLD | self.cs)
        rx, sent = [], 0
        while len(rx) < len(out):
            st = await self._status()
            if sent < len(out) and not st & ST_TX_FULL and sent - len(rx) < self.fifo_depth:
                await self.apb.write(REG_TXDATA, out[sent])
                sent += 1
            elif rx_count(st):
                val, err = await self.apb.read(REG_RXDATA)
                assert err == 0
                rx.append(val)
        await self.apb.write(REG_CS_CTRL, self.cs)          # release CS
        timeout = 100
        while (await self._status()) & ST_CS_ACTIVE:
            timeout -= 1
            assert timeout > 0, "CS never deasserted after release"
        self.frames += 1
        return rx

    # ------------------------------------------------------------ flash ops
    @staticmethod
    def _addr(a):
        return [(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF]

    async def jedec_id(self):
        return (await self.xfer([CMD_JEDEC, 0, 0, 0]))[1:]

    async def read_status(self):
        return (await self.xfer([CMD_RDSR, 0]))[1]

    async def wait_ready(self, timeout_polls=200):
        while True:
            sr = await self.read_status()
            self.status_polls += 1
            if not sr & SR_BUSY:
                return sr
            timeout_polls -= 1
            assert timeout_polls > 0, "flash stayed BUSY"
            for _ in range(20):
                await RisingEdge(self.dut.clk)

    async def write_enable(self):
        await self.xfer([CMD_WREN])

    async def read(self, addr, n):
        return (await self.xfer([CMD_READ, *self._addr(addr)] + [0] * n))[4:]

    async def fast_read(self, addr, n):
        return (await self.xfer([CMD_FAST_READ, *self._addr(addr), 0] + [0] * n))[5:]

    async def page_program(self, addr, data, wren=True):
        if wren:
            await self.write_enable()
        await self.xfer([CMD_PP, *self._addr(addr), *data])
        await self.wait_ready()

    async def sector_erase(self, addr, wren=True):
        if wren:
            await self.write_enable()
        await self.xfer([CMD_SE, *self._addr(addr)])
        await self.wait_ready()
