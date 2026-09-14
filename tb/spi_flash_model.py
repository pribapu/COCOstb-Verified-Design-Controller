"""
spi_flash_model.py -- behavioural SPI NOR flash slave (Winbond W25Q-style).

Sits on one chip-select line of the bus and reacts at the pin level through
SPISlaveBFM, so every byte really crosses the DUT's SCLK/MOSI/MISO/CS pins.
Implements the parts of the datasheet behaviour a controller + driver is
most likely to get wrong:

  0x9F JEDEC ID          -> manufacturer / memory type / capacity
  0x05 READ STATUS-1     -> [0] BUSY  [1] WEL; repeats while CS stays low
  0x06 / 0x04 WREN/WRDI  -> set / clear the write-enable latch
  0x03 READ              -> 24-bit address, streams until CS rises
  0x0B FAST READ         -> 24-bit address + 1 dummy byte, then streams
  0x02 PAGE PROGRAM      -> needs WEL; address wraps *within* the 256-byte
                            page; NOR semantics (programming can only clear
                            bits); BUSY for t_pp afterwards; clears WEL
  0x20 SECTOR ERASE 4 KB -> needs WEL; bytes read 0xFF afterwards; BUSY for
                            t_se; clears WEL

Commands are executed at CS rising edge (as real parts do), only if the frame
ended on a byte boundary with the right length. While BUSY, everything except
READ STATUS is ignored. Ignored commands are logged with the reason so tests
can assert on them.
"""

from cocotb.utils import get_sim_time
from spi_bfm import SPISlaveBFM

CMD_WREN = 0x06
CMD_WRDI = 0x04
CMD_RDSR = 0x05
CMD_READ = 0x03
CMD_FAST_READ = 0x0B
CMD_PP = 0x02
CMD_SE = 0x20
CMD_JEDEC = 0x9F

SR_BUSY = 1 << 0
SR_WEL = 1 << 1

PAGE = 256
SECTOR = 4096


class SPIFlashModel(SPISlaveBFM):
    def __init__(self, dut, cs_index, size=1 << 24, jedec=(0xEF, 0x40, 0x18),
                 t_pp_ns=3000, t_se_ns=9000):
        super().__init__(dut, 8, cs_index=cs_index)
        self.size = size
        self.jedec = jedec
        self.t_pp_ns = t_pp_ns
        self.t_se_ns = t_se_ns
        self.mem = {}            # sparse array; never-programmed bytes read 0xFF
        self.wel = False
        self.busy_until = 0
        self.executed = []       # (cmd, addr) of every command that took effect
        self.ignored = []        # (cmd, reason)
        self.busy_polls = 0      # RDSR bytes returned with BUSY set
        self._reset_frame()

    # --------------------------------------------------------- state helpers
    def busy(self):
        return get_sim_time(unit="ns") < self.busy_until

    def status(self):
        return (SR_BUSY if self.busy() else 0) | (SR_WEL if self.wel else 0)

    def peek(self, addr):
        return self.mem.get(addr % self.size, 0xFF)

    def _reset_frame(self):
        self.cmd = None
        self.addr = 0
        self.pp_data = {}

    def _data_start(self):
        return {CMD_READ: 4, CMD_FAST_READ: 5}.get(self.cmd)

    # --------------------------------------------------------- BFM hooks
    def on_frame_start(self):
        self._reset_frame()

    def next_tx(self, index):
        """Byte driven on MISO during byte `index` of the frame."""
        if self.cmd is None:
            return 0xFF                                   # MISO high-Z -> reads 1s
        if self.cmd == CMD_RDSR:
            st = self.status()
            if st & SR_BUSY:
                self.busy_polls += 1
            return st
        if self.busy():
            return 0xFF
        if self.cmd == CMD_JEDEC:
            return self.jedec[index - 1] if 1 <= index <= 3 else 0xFF
        start = self._data_start()
        if start is not None and index >= start:
            return self.peek(self.addr + index - start)
        return 0xFF

    def on_word(self, index, word):
        if index == 0:
            self.cmd = word
            return
        if self.cmd in (CMD_READ, CMD_FAST_READ, CMD_PP, CMD_SE) and index <= 3:
            self.addr = ((self.addr << 8) | word) & (self.size - 1)
        elif self.cmd == CMD_PP and index >= 4:
            page_base = self.addr & ~(PAGE - 1)
            column = (self.addr + index - 4) & (PAGE - 1)   # wraps inside the page
            self.pp_data[page_base | column] = word

    def on_frame_end(self):
        n = len(self.frame_words)
        cmd = self.cmd
        if cmd is None:
            return
        if self.aborted:
            self.ignored.append((cmd, "frame ended mid-byte"))
            return
        if cmd in (CMD_RDSR, CMD_JEDEC, CMD_READ, CMD_FAST_READ):
            if not self.busy():
                self.executed.append((cmd, self.addr))
            return
        if self.busy():
            self.ignored.append((cmd, "busy"))
            return
        if cmd == CMD_WREN and n == 1:
            self.wel = True
        elif cmd == CMD_WRDI and n == 1:
            self.wel = False
        elif cmd in (CMD_PP, CMD_SE):
            if not self.wel:
                self.ignored.append((cmd, "WEL not set"))
                return
            if cmd == CMD_PP and n >= 5:
                for a, v in self.pp_data.items():
                    self.mem[a] = self.peek(a) & v           # NOR: only 1 -> 0
                self.busy_until = get_sim_time(unit="ns") + self.t_pp_ns
            elif cmd == CMD_SE and n == 4:
                base = self.addr & ~(SECTOR - 1)
                for a in [a for a in self.mem if base <= a < base + SECTOR]:
                    del self.mem[a]
                self.busy_until = get_sim_time(unit="ns") + self.t_se_ns
            else:
                self.ignored.append((cmd, f"bad length {n}"))
                return
            self.wel = False
        else:
            self.ignored.append((cmd, f"unsupported / bad length {n}"))
            return
        self.executed.append((cmd, self.addr))
