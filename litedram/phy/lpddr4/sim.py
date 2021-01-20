from operator import or_
from functools import reduce

from migen import *

from litex.soc.interconnect.stream import SyncFIFO, AsyncFIFO

from litedram.common import TappedDelayLine


class LPDDR4Sim(Module):
    def __init__(self, pads):
        self.submodules.cmd = ClockDomainsRenamer("sys8x_90")(CommandsSim(pads))
        self.submodules.data = ClockDomainsRenamer("sys8x_ddr")(DataSim(pads, self.cmd))


class CombLogger(Module):
    # Allows to use Display inside FSM, allows to filter log messages by level
    DEBUG = 0
    INFO  = 1
    WARN  = 2
    ERROR = 3
    NONE  = 4

    def __init__(self, log_level=INFO):
        self.ops = []
        self.level = Signal(reset=log_level, max=self.NONE)

    def debug(self, fmt, *args, **kwargs):
        return self.log("[DEBUG] " + fmt, *args, level=self.DEBUG, **kwargs)

    def info(self, fmt, *args, **kwargs):
        return self.log("[INFO] " + fmt, *args, level=self.INFO, **kwargs)

    def warn(self, fmt, *args, **kwargs):
        return self.log("[WARN] " + fmt, *args, level=self.WARN, **kwargs)

    def error(self, fmt, *args, **kwargs):
        return self.log("[ERROR] " + fmt, *args, level=self.ERROR, **kwargs)

    def log(self, fmt, *args, level=DEBUG, once=True):
        cond = Signal()
        if once:  # make the condition be triggered only on rising edge
            cond_d = Signal()
            self.sync += cond_d.eq(cond)
            condition = ~cond_d & cond
        else:
            condition = cond

        self.ops.append((level, condition, fmt, args))
        return cond.eq(1)

    def do_finalize(self):
        for level, cond, fmt, args in self.ops:
            self.sync += If((level >= self.level) & cond, Display(fmt, *args))


class FSMWrapper(Module):
    def __init__(self, *ops):
        self.submodules.fsm = fsm = FSM()
        fsm.act("ONLY", *ops)
        fsm.act("UNUSED")  # FIXME


class CommandsSim(Module):  # clock domain: clk_p
    def __init__(self, pads):
        self.submodules.log = log = CombLogger(log_level=CombLogger.DEBUG)

        # for DataSim
        self.active_banks = active_banks = Array([Signal() for _ in range(8)])
        self.active_rows = active_rows = Array([Signal(17) for _ in range(8)])
        self.data_en = data_en = TappedDelayLine(ntaps=20)
        self.data = data = AsyncFIFO([("we", 1), ("bank", 3), ("row", 17), ("col", 10)], depth=4)

        # More Registers storage
        mode_regs = Memory(8, depth=64)
        mr_port = mode_regs.get_port(write_capable=True, async_read=True)
        self.specials += mode_regs, mr_port

        # CS/CA shift registers
        cs = TappedDelayLine(pads.cs, ntaps=2)
        ca = TappedDelayLine(pads.ca, ntaps=2)
        self.submodules += cs, ca

        self.cs_low  = Signal(6)
        self.cs_high = Signal(6)

        handle_cmd = Signal()
        handlers = [
            self.mrw_handler(mr_port),
            self.refresh_handler(),
            self.activate_handler(),
            self.precharge_handler(),
            self.read_write_handler(),
        ]
        any_handler = Signal()
        for cond, handler in handlers:
            handler = CEInserter()(handler)
            self.submodules += handler

            handler.ce.eq(handle_cmd)
            any_handler = any_handler | cond

        self.comb += If(handle_cmd & ~any_handler,
            self.log.error("Unexpected command: cs_high=0b%06b cs_low=0b%06b", self.cs_high, self.cs_low)
        )

        reset_n_d = Signal()
        self.sync += reset_n_d.eq(pads.reset_n)

        self.submodules.fsm = fsm = FSM()
        fsm.act("RESET",
            If(~reset_n_d & pads.reset_n,
                self.log.info("RESET released"),
                NextState("ON")
            )
        )
        fsm.act("ON",
            If(Cat(cs.taps) == 0b10,
                handle_cmd.eq(1),
                self.cs_high.eq(ca.taps[1]),
                self.cs_low.eq(ca.taps[0]),
            )
        )

    def mrw_handler(self, mr_port):
        cond = self.cs_high[:5] == 0b00110

        ma  = Signal(6)
        op7 = Signal()
        op  = Signal(8)

        fsm = FSM()
        fsm.act("MRW-1",
            If(cond,
                self.log.debug("MRW-1"),
                NextValue(ma, self.cs_low),
                NextValue(op7, self.cs_high[5]),
                NextState("MRW-2")
            )
        )
        fsm.act("MRW-2",
            If(self.cs_high[:5] == 0b10110,
                self.log.debug("MRW-2"),
                self.log.info("MRW: MR[%d] = 0x%02x", ma, op),
                op.eq(Cat(self.cs_low, self.cs_high[5], op7)),
                mr_port.adr.eq(ma),
                mr_port.dat_w.eq(op),
                mr_port.we.eq(1),
            ).Else(
                self.log.error("Waiting for MRW-2 but not found: cs_high=0b%06b", self.cs_high)
            ),
            NextState("MRW-1")
        )

        return cond, fsm

    def refresh_handler(self):
        cond = self.cs_high[:5] == 0b01000
        return cond, FSMWrapper(If(cond,
            self.log.debug("REFRESH"),
            If(reduce(or_, self.active_banks),
                self.log.error("Not all banks precharged during REFRESH")
            )
        ))

    def activate_handler(self):
        cond = self.cs_high[:2] == 0b01

        bank = Signal(3)
        row1 = Signal(7)
        row2 = Signal(10)
        row  = Signal(17)

        self.comb += row.eq(Cat(row2, row1))

        fsm = FSM()
        fsm.act("ACTIVATE-1",
            If(cond,
                self.log.debug("ACTIVATE-1"),
                NextValue(bank, self.cs_low[:3]),
                NextValue(row1, Cat(self.cs_low[4:6], self.cs_high[2:6], self.cs_low[3])),
                NextState("ACTIVATE-2")
            )
        )
        fsm.act("ACTIVATE-2",
            If(self.cs_high[:2] == 0b11,
                self.log.info("ACT: bank=%d row=%d", bank, row),
                row2.eq(Cat(self.cs_low, self.cs_high[2:])),
                NextValue(self.active_banks[bank], 1),
                NextValue(self.active_rows[bank], row),
                If(self.active_banks[bank], self.log.warn("Bank already active"))
            ).Else(
                self.log.error("Waiting for ACTIVATE-2 but not found: cs_high=0b%06b", self.cs_high)
            ),
            NextState("ACTIVATE-1")
        )
        return cond, fsm

    def read_write_handler(self):
        cond = (self.cs_high[:5] == 0b00100) | (self.cs_high[:5] == 0b00010)

        bank           = Signal(3)
        col            = Signal(10)
        burst_len      = Signal()
        auto_precharge = Signal()

        fsm = FSM()
        fsm.act("CAS-1",
            If(cond,
                self.log.debug("CAS-1"),
                NextValue(bank, self.cs_low[:3]),
                NextValue(col[9], self.cs_low[4]),
                NextValue(burst_len, self.cs_high[5]),
                NextValue(auto_precharge, self.cs_low[5]),
                If(self.cs_high[:5] == 0b00100,
                    NextState("WRITE-2")
                ).Elif(self.cs_high[:5] == 0b00010,
                    NextState("READ-2")
                ),
            )
        )
        fsm.act("READ-2",
            If(self.cs_high[:5] == 0b10010,
                self.log.debug("READ-2"),
            ).Else(
                self.log.error("Waiting for CAS-2 but not found: cs_high=0b%06b", self.cs_high)
            ),
            NextState("CAS-1")
        )
        fsm.act("WRITE-2",
            If(self.cs_high[:5] == 0b10010,
                self.log.debug("WRITE-2"),
            ).Else(
                self.log.error("Waiting for CAS-2 but not found: cs_high=0b%06b", self.cs_high)
            ),
            NextState("CAS-1")
        )
        return cond, fsm

    def precharge_handler(self):
        cond = self.cs_high[:5] == 0b10000
        bank = Signal(3)

        return cond, FSMWrapper(If(cond,
            If(self.cs_high[5],
                self.log.debug("PRECHARGE: all banks"),
                bank.eq(2**len(bank) - 1),
                *[NextValue(self.active_banks[b], 0) for b in range(2**len(bank))],
            ).Else(
                self.log.debug("PRECHself.ARGE: bank = %d", bank),
                bank.eq(self.cs_low[:3]),
                NextValue(self.active_banks[bank], 0),
            ),
        ))

class DataSim(Module):  # clock domain: ddr
    def __init__(self, pads, cmds_sim):
        bl = 16

        bank = Signal(3)
        row = Signal(17)
        col = Signal(10)
        col_burst = Signal(10)

        # Per-bank memory
        nrows, ncols = 32768, 1024
        mems = [Memory(len(pads.dq), depth=nrows * ncols) for _ in range(8)]
        ports = [mem.get_port(write_capable=True, async_read=True) for mem in mems]
        self.specials += [*mems, *ports]
        ports = Array(ports)

        counter = Signal(max=32)
        bank_addr = Signal(max=nrows * ncols)
        self.comb += col_burst.eq(col + col_burst)
        self.comb += bank_addr.eq(row * ncols + col_burst)

        self.submodules.fsm = fsm = FSM()
        fsm.act("IDLE",
            NextValue(counter, 0),
            If(cmds_sim.data_en.output,
                NextValue(bank, cmds_sim.data.source.bank),
                NextValue(row, cmds_sim.data.source.row),
                NextValue(col, cmds_sim.data.source.col),
                If(cmds_sim.data.source.we,
                    NextState("WRITE"),
                ).Else(
                    NextState("READ"),
                )
            )
        )
        fsm.act("WRITE",
            ports[bank].we.eq(1),
            ports[bank].adr.eq(bank_addr),
            ports[bank].dat_w.eq(pads.dq),
            NextValue(counter, counter + 1),
            If(counter == bl,
                NextState("IDLE")
            ),
        )
        fsm.act("READ",
            ports[bank].we.eq(0),
            ports[bank].adr.eq(bank_addr),
            pads.dq.eq(ports[bank].dat_r),
            NextValue(counter, counter + 1),
            If(counter == bl,
                NextState("IDLE")
            ),
        )

        self.sync += [
            If(fsm.ongoing("WRITE"),
                Display("WRITE: bank=%d, row=%d, col=%d, data=0x%04x", bank, row, col_burst, pads.dq)
            ).Elif(fsm.ongoing("READ"),
                Display("READ: bank=%d, row=%d, col=%d, data=0x%04x", bank, row, col_burst, pads.dq)
            )
        ]
