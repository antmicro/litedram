from operator import or_
from functools import reduce

from migen import *

from litedram.common import TappedDelayLine


class LPDDR4Sim(Module):
    def __init__(self, pads):
        self.submodules.cmd = ClockDomainsRenamer("sys8x_90")(CommandsSim(pads))
        self.submodules.data = ClockDomainsRenamer("sys8x")(DataSim(pads, self.cmd))


class CommandsSim(Module):  # clock domain: clk_p
    def __init__(self, pads):
        from litex.soc.interconnect.stream import SyncFIFO, AsyncFIFO

        # for DataSim
        self.active_banks = active_banks = Array([Signal() for _ in range(8)])
        self.active_rows = active_rows = Array([Signal(17) for _ in range(8)])
        self.data_en = data_en = TappedDelayLine(ntaps=20)
        self.data = data = AsyncFIFO([("we", 1), ("bank", 3), ("row", 17), ("col", 10)], depth=4)

        # More Registers
        mode_regs = Memory(8, depth=64)
        mr_port = mode_regs.get_port(write_capable=True, async_read=True)
        self.specials += mode_regs, mr_port

        # Level 1 - "small" commands
        small_cmds = SyncFIFO([("cs_high", 6), ("cs_low", 6)], 2)
        cs = TappedDelayLine(pads.cs, ntaps=2)
        ca = TappedDelayLine(pads.ca, ntaps=2)
        self.submodules += cs, ca, small_cmds

        self.comb += If(Cat(cs.taps) == 0b10,
            small_cmds.sink.valid.eq(1),
            small_cmds.sink.cs_high.eq(ca.taps[1]),
            small_cmds.sink.cs_low.eq(ca.taps[0]),
            # ignore ready
        )
        self.sync += If(small_cmds.sink.valid & ~small_cmds.sink.ready,
            Display("'Small' commands overflow")
        )

        error = Signal()

        # Level 2 - "big" commands
        mrw1   = Signal()
        mrw2   = Signal()
        mrw_ma = Signal(6)
        mrw_op7 = Signal()
        mrw_op = Signal(8)

        act1    = Signal()
        act2    = Signal()
        act     = Signal()
        pre     = Signal()
        write   = Signal()
        read    = Signal()
        refresh = Signal()

        bank = Signal(3)
        row  = Signal(17)
        col  = Signal(10)
        burst_len = Signal()
        auto_precharge = Signal()

        cs_low = small_cmds.source.cs_low
        cs_high = small_cmds.source.cs_high
        self.submodules.fsm = fsm = FSM()
        fsm.act("IDLE",
            If(small_cmds.source.valid,
                small_cmds.source.ready.eq(1),
                # Dispatch command
                If(cs_high[:5] == 0b00110,  # MRW-1
                    NextValue(mrw_ma, cs_low),
                    NextValue(mrw_op7, cs_high[5]),
                    NextValue(mrw1, 1),
                ).Elif((cs_high[:5] == 0b10110) & mrw1,  # MRW-2
                    mrw_op.eq(Cat(cs_low, cs_high[5], mrw_op7)),
                    mr_port.adr.eq(mrw_ma),
                    mr_port.dat_w.eq(mrw_op),
                    mr_port.we.eq(1),
                    mrw2.eq(1),
                    NextValue(mrw1, 0),
                ).Elif(cs_high[:5] == 0b01000,  # REFRESH
                    refresh.eq(1),
                    If(reduce(or_, active_banks), error.eq(1))
                ).Elif(cs_high[:2] == 0b01,  # ACTIVATE-1
                    NextValue(bank, cs_low[:3]),
                    NextValue(row, Cat(Replicate(0, 10), cs_low[4:6], cs_high[2:6], cs_low[3])),
                    NextValue(act1, 1),
                ).Elif((cs_high[:2] == 0b11) & act1,  # ACTIVATE-2
                    NextValue(row[:10], Cat(cs_low, cs_high[2:])),
                    NextValue(act1, 0),
                    act2.eq(1),
                    NextValue(act, 1),
                    NextState("UPDATE-ACTIVE"),
                ).Elif((cs_high[:5] == 0b00100) | (cs_high[:5] == 0b00010),  # "CAS-1"
                    If(cs_high[:5] == 0b00100,  # WRITE-1
                        NextValue(write, 1)
                    ).Elif(cs_high[:5] == 0b00010,  # READ-1
                        NextValue(read, 1)
                    ),
                    NextValue(bank, cs_low[:3]),
                    NextValue(col[9], cs_low[4]),
                    NextValue(burst_len, cs_high[5]),
                    NextValue(auto_precharge, cs_low[5]),
                ).Elif((cs_high[:5] == 0b10010) & (write | read),  # CAS-2
                    NextValue(col[:9], Cat(Replicate(0, 2), cs_low, cs_high[5])),
                    NextState("DATA-COMMAND"),
                ).Elif(cs_high[:5] == 0b10000,  # PRECHARGE
                    If(cs_high[5],
                        NextValue(bank, 2**len(bank) - 1)
                    ).Else(
                        NextValue(bank, cs_low[:3])
                    ),
                    NextValue(pre, 1),
                    NextState("UPDATE-ACTIVE"),
                ).Else(
                    error.eq(1),
                )
            )
        )
        fsm.act("UPDATE-ACTIVE",
            If(act,
                NextValue(active_banks[bank], 1),
                NextValue(active_rows[bank], row),
            ).Elif(pre,
                If(bank == 2**len(bank) - 1,
                    [NextValue(active_banks[b], 0) for b in range(8)],
                ).Else(
                    NextValue(active_banks[bank], 0),
                )
            ),
            NextState("IDLE"),
            NextValue(act, 0),
            NextValue(pre, 0),
        )
        fsm.act("DATA-COMMAND",
            data_en.input.eq(1),
            data.sink.valid.eq(1),
            data.sink.bank.eq(bank),
            data.sink.row.eq(active_rows[bank]),
            data.sink.col.eq(col),
            error.eq(data.sink.ready == 0),  # ignore overflows
            NextValue(write, 0),
            NextValue(read, 0),
            If(auto_precharge,  # FIXME: should be done after write/read
                NextValue(pre, 1),
                NextValue(bank, 2**len(bank) - 1),
                NextState("UPDATE-ACTIVE"),
            ).Else(
                NextState("IDLE"),
            ),
        )

        # Display
        self.sync += [
            If(mrw2, Display("MRW: MR[%d] = 0x%02x", mrw_ma, mrw_op), mrw1.eq(0)),
            If(act, Display("ACT: bank = %d, row = %d", bank, row), act1.eq(0)),
            # If(refresh, Display("REF")),
            If(pre,
                If(bank == 2**len(bank) - 1,
                    # avoid display when no banks are active
                    If(reduce(or_, active_banks), Display("PRE: all banks"))
                ).Else(
                    Display("PRE: bank = %d", bank),
                )
            ),
            If(error, Display("ERROR")),
        ]


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
