"""
spi_bfm.py -- SPI slave Bus Functional Model (BFM) and a matching software
reference model, used to verify the `spi_master` DUT.

The BFM behaves as a standard Motorola-SPI slave:
  * it watches CS_n / SCLK / MOSI and drives MISO,
  * it samples MOSI on the same edges the master samples MISO, and drives MISO
    on the opposite (shift) edges,
  * it is configured per-frame (cpol, cpha, lsb_first) to match the master's
    latched configuration,
  * a frame (one CS assertion) may carry any number of words -- as with a
    real shift-register slave, the next word's first bit appears on MISO as
    soon as the previous word's last edge has passed.

Device models (e.g. spi_flash_model.SPIFlashModel) subclass it and override
the hooks: on_frame_start(), next_tx(i), on_word(i, word), on_frame_end().

Because the BFM is written independently of the RTL, "master.rx == slave.tx"
and "slave.rx == master.tx" is a genuine cross-check of protocol timing, not a
tautology.
"""

from cocotb.triggers import FallingEdge, First, RisingEdge, ValueChange


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
    """Reactive SPI slave model driven off the DUT's SPI pins.

    cs_index=None: `dut.cs_n` is a single chip-select line.
    cs_index=i:    `dut.cs_n` is a vector; this slave sits on line i.
    """

    def __init__(self, dut, width, cs_index=None):
        self.dut = dut
        self.width = width
        self.cs_index = cs_index
        # per-frame configuration (set by the test before each frame)
        self.cpol = 0
        self.cpha = 0
        self.lsb_first = 0
        self.tx_byte = 0          # what the slave sends back for every word...
        self.tx_queue = []        # ...unless per-word responses are queued here
        # results
        self.received = None      # last word the slave saw from the master
        self.frame_words = []     # every word of the current / last frame
        self.history = []         # every word ever received, in order
        self.edge_count = 0       # SCLK toggles observed in the last word
        self.idle_ok = True       # SCLK observed idling at CPOL at frame start
        self.frames = 0
        self.aborted = False      # last frame ended mid-word (e.g. reset)

    # ---------------------------------------------------------- hooks
    def on_frame_start(self):
        pass

    def next_tx(self, index):
        """Word to shift out as word `index` of the current frame."""
        return self.tx_queue.pop(0) if self.tx_queue else self.tx_byte

    def on_word(self, index, word):
        pass

    def on_frame_end(self):
        pass

    # ---------------------------------------------------------- pins
    def _selected(self):
        v = int(self.dut.cs_n.value)
        if self.cs_index is None:
            return v == 0
        return ((v >> self.cs_index) & 1) == 0

    async def _wait_select(self):
        if self.cs_index is None:
            await FallingEdge(self.dut.cs_n)
            return
        while not self._selected():
            await ValueChange(self.dut.cs_n)

    def _deselect_trigger(self):
        if self.cs_index is None:
            return RisingEdge(self.dut.cs_n)
        return ValueChange(self.dut.cs_n)

    async def _edge_or_deselect(self):
        """Wait for the next SCLK edge; return False if CS went away instead."""
        while True:
            sclk_edge = ValueChange(self.dut.sclk)
            fired = await First(sclk_edge, self._deselect_trigger())
            if not self._selected():
                return False
            if fired is sclk_edge:
                return True
            # a different chip-select line changed -- not our business

    # ---------------------------------------------------------- behaviour
    async def run(self):
        """Main loop: handle one frame per CS assertion, forever."""
        self.dut.miso.value = 0
        while True:
            await self._wait_select()
            await self._frame()

    async def _frame(self):
        self.frames += 1
        self.aborted = False
        self.frame_words = []
        # Sanity: SCLK should currently sit at its idle (CPOL) level.
        self.idle_ok = (int(self.dut.sclk.value) == self.cpol)
        cpha, lsb = self.cpha, self.lsb_first          # latched for the frame
        self.on_frame_start()
        while True:
            tx = self.next_tx(len(self.frame_words))
            word = await self._word(tx, cpha, lsb)
            if word is None:
                break
            self.received = word
            self.on_word(len(self.frame_words), word)
            self.frame_words.append(word)
            self.history.append(word)
        self.on_frame_end()

    async def _word(self, tx, cpha, lsb):
        w = self.width
        recv = 0

        # CPHA=0: first MISO bit must be valid before the first edge.
        if cpha == 0:
            self.dut.miso.value = get_bit(tx, w, 0, lsb)

        sample_i = 0   # next order-index to sample from MOSI
        drive_i = 0    # next order-index to drive on MISO (CPHA=1)

        for e in range(2 * w):
            if not await self._edge_or_deselect():
                if e != 0:
                    self.aborted = True
                return None
            self.edge_count = e + 1
            leading = (e % 2 == 0)

            if cpha == 0:
                if leading:
                    recv = set_bit(recv, w, sample_i, lsb, int(self.dut.mosi.value))
                    sample_i += 1
                elif sample_i < w:
                    self.dut.miso.value = get_bit(tx, w, sample_i, lsb)
            else:  # cpha == 1
                if leading:
                    self.dut.miso.value = get_bit(tx, w, drive_i, lsb)
                    drive_i += 1
                else:
                    recv = set_bit(recv, w, sample_i, lsb, int(self.dut.mosi.value))
                    sample_i += 1

        return recv


def spi_reference(tx_master, tx_slave, width, cpol, cpha, lsb_first):
    """Pure-software model of a full-duplex SPI exchange.

    Returns (master_rx_expected, slave_rx_expected). For an ideal bit-exchange
    the master receives exactly the slave's tx word and vice-versa, independent
    of mode/order -- which is the property the scoreboard asserts against the
    RTL + BFM.
    """
    return tx_slave & ((1 << width) - 1), tx_master & ((1 << width) - 1)
