"""
spi_bfm.py -- SPI slave Bus Functional Model (BFM) and a matching software
reference model, used to verify the `spi_master` DUT.

The BFM behaves as a standard Motorola-SPI slave:
  * it watches CS_n / SCLK / MOSI and drives MISO,
  * it samples MOSI on the same edges the master samples MISO, and drives MISO
    on the opposite (shift) edges,
  * it is configured per-transfer (cpol, cpha, lsb_first, width) to match the
    master's latched configuration.

Because the BFM is written independently of the RTL, "master.rx == slave.tx"
and "slave.rx == master.tx" is a genuine cross-check of protocol timing, not a
tautology.
"""

import cocotb
from cocotb.triggers import Edge, FallingEdge, RisingEdge


def bit_phys(width, order_i, lsb_first):
    """Physical bit position of the order_i-th transmitted bit (0 = first)."""
    return order_i if lsb_first else (width - 1 - order_i)


def get_bit(word, width, order_i, lsb_first):
    return (word >> bit_phys(width, order_i, lsb_first)) & 1


def set_bit(word, width, order_i, lsb_first, value):
    pos = bit_phys(width, order_i, lsb_first)
    if value:
        return word | (1 << pos)
    return word & ~(1 << pos)


class SPISlaveBFM:
    """Reactive SPI slave model driven off the DUT's SPI pins."""

    def __init__(self, dut, width):
        self.dut = dut
        self.width = width
        # per-transfer configuration (set by the test before each transfer)
        self.cpol = 0
        self.cpha = 0
        self.lsb_first = 0
        self.tx_byte = 0          # what the slave sends to the master
        # results captured from the last completed transfer
        self.received = None      # what the slave saw from the master (== master.tx)
        self.edge_count = 0       # number of SCLK toggles observed
        self.idle_ok = True       # SCLK observed idling at CPOL

    async def run(self):
        """Main loop: handle one transfer per CS assertion, forever."""
        self.dut.miso.value = 0
        while True:
            await FallingEdge(self.dut.cs_n)
            await self._transfer()

    async def _transfer(self):
        w = self.width
        cpha = self.cpha
        lsb = self.lsb_first
        recv = 0
        self.edge_count = 0

        # Sanity: SCLK should currently sit at its idle (CPOL) level.
        self.idle_ok = (int(self.dut.sclk.value) == self.cpol)

        # CPHA=0: first MISO bit must be valid before the first edge.
        if cpha == 0:
            self.dut.miso.value = get_bit(self.tx_byte, w, 0, lsb)

        sample_i = 0   # next order-index to sample from MOSI
        drive_i = 0    # next order-index to drive on MISO (CPHA=1)

        for e in range(2 * w):
            await Edge(self.dut.sclk)
            self.edge_count += 1
            leading = (e % 2 == 0)

            if cpha == 0:
                if leading:
                    b = int(self.dut.mosi.value)
                    recv = set_bit(recv, w, sample_i, lsb, b)
                    sample_i += 1
                else:
                    if sample_i < w:
                        self.dut.miso.value = get_bit(self.tx_byte, w, sample_i, lsb)
            else:  # cpha == 1
                if leading:
                    self.dut.miso.value = get_bit(self.tx_byte, w, drive_i, lsb)
                    drive_i += 1
                else:
                    b = int(self.dut.mosi.value)
                    recv = set_bit(recv, w, sample_i, lsb, b)
                    sample_i += 1

        self.received = recv


def spi_reference(tx_master, tx_slave, width, cpol, cpha, lsb_first):
    """Pure-software model of a full-duplex SPI exchange.

    Returns (master_rx_expected, slave_rx_expected). For an ideal bit-exchange
    the master receives exactly the slave's tx word and vice-versa, independent
    of mode/order -- which is the property the scoreboard asserts against the
    RTL + BFM.
    """
    return tx_slave & ((1 << width) - 1), tx_master & ((1 << width) - 1)
