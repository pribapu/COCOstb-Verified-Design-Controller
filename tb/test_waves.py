import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from spi_bfm import SPISlaveBFM

@cocotb.test()
async def gen_waves(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst_n.value=0; dut.start.value=0; dut.cpol.value=0; dut.cpha.value=0
    dut.lsb_first.value=0; dut.clk_div.value=1; dut.tx_data.value=0; dut.miso.value=0
    for _ in range(5): await RisingEdge(dut.clk)
    dut.rst_n.value=1; await RisingEdge(dut.clk)
    bfm=SPISlaveBFM(dut,len(dut.tx_data)); cocotb.start_soon(bfm.run())
    async def xfer(cpol,cpha,tx,stx):
        dut.cpol.value=cpol; dut.cpha.value=cpha; dut.lsb_first.value=0
        dut.clk_div.value=1; dut.tx_data.value=tx
        bfm.cpol=cpol; bfm.cpha=cpha; bfm.lsb_first=0; bfm.tx_byte=stx
        await RisingEdge(dut.clk); dut.start.value=1
        await RisingEdge(dut.clk); dut.start.value=0
        while int(dut.done.value)==0: await RisingEdge(dut.clk)
        for _ in range(4): await RisingEdge(dut.clk)
    await xfer(0,0,0xA5,0x3C)   # mode 0
    await xfer(1,1,0x5A,0xC3)   # mode 3
    for _ in range(4): await RisingEdge(dut.clk)
