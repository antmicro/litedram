import math
from operator import or_
from functools import reduce

from migen import *

from litex.soc.interconnect.stream import SyncFIFO, AsyncFIFO

from litedram.common import TappedDelayLine, tXXDController
from litedram.phy.lpddr4.utils import delayed, once
from litedram.phy.lpddr4.commands import MPC


class LPDDR4Sim(Module):
    def __init__(self, pads, sys_clk_freq, log_level):
        if isinstance(log_level, str):
            log_level = getattr(CombLogger, log_level.upper())
        cmd = CommandsSim(pads, clk_freq=8*sys_clk_freq, log_level=log_level)
        self.submodules.cmd = ClockDomainsRenamer("sys8x_90")(cmd)
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


class CommandsSim(Module):  # clock domain: clk_p
    def __init__(self, pads, clk_freq, log_level=CombLogger.INFO, init_delays=False):
        self.submodules.log = log = CombLogger(log_level=log_level)

        # for DataSim
        self.active_banks = active_banks = Array([Signal() for _ in range(8)])
        self.active_rows = active_rows = Array([Signal(17) for _ in range(8)])
        self.data_en = data_en = TappedDelayLine(ntaps=20)
        self.data = data = AsyncFIFO([("we", 1), ("bank", 3), ("row", 17), ("col", 10)], depth=4)

        # Mode Registers storage
        mode_regs = Memory(8, depth=64)
        self.mr_port = mode_regs.get_port(write_capable=True, async_read=True)
        self.specials += mode_regs, self.mr_port

        # CS/CA shift registers
        cs = TappedDelayLine(pads.cs, ntaps=2)
        ca = TappedDelayLine(pads.ca, ntaps=2)
        self.submodules += cs, ca

        self.cs_low     = Signal(6)
        self.cs_high    = Signal(6)
        self.handle_cmd = Signal()
        self.mpc_op     = Signal(7)

        cmds_enabled = Signal()
        cmd_handlers = {
            "MRW": self.mrw_handler(),
            "REF": self.refresh_handler(),
            "ACT": self.activate_handler(),
            "PRE": self.precharge_handler(),
            "CAS": self.cas_handler(),
            "MPC": self.mpc_handler(),
        }
        self.comb += [
            If(cmds_enabled,
                If(Cat(cs.taps) == 0b10,
                    self.handle_cmd.eq(1),
                    self.cs_high.eq(ca.taps[1]),
                    self.cs_low.eq(ca.taps[0]),
                )
            ),
            If(self.handle_cmd & ~reduce(or_, cmd_handlers.values()),
                self.log.error("Unexpected command: cs_high=0b%06b cs_low=0b%06b", self.cs_high, self.cs_low)
            ),
        ]

        def ck(t):
            return math.ceil(t * clk_freq)

        self.submodules.tinit0 = tXXDController(ck(20e-3))
        self.submodules.tinit1 = tXXDController(ck(200e-6))
        self.submodules.tinit2 = tXXDController(ck(10e-9))
        self.submodules.tinit3 = tXXDController(ck(2e-3))
        self.submodules.tinit4 = tXXDController(5)  # TODO: would require counting pads.clk_p ticks
        self.submodules.tinit5 = tXXDController(ck(2e-6))
        self.submodules.tzqcal = tXXDController(ck(1e-6))
        self.submodules.tzqlat = tXXDController(max(8, ck(30e-9)))

        self.comb += [
            If(~delayed(self, pads.reset_n) & pads.reset_n,
                self.log.info("RESET released"),
            ),
            If(delayed(self, pads.reset_n) & ~pads.reset_n,
                self.log.info("RESET asserted"),
            ),
            If(delayed(self, pads.cke) & ~pads.cke,
                self.log.info("CKE falling edge"),
            ),
            If(~delayed(self, pads.cke) & pads.cke,
                self.log.info("CKE rising edge"),
            ),
        ]

        self.submodules.fsm = fsm = FSM()
        fsm.act("POWER-RAMP",
            self.tinit0.valid.eq(1),
            If(~pads.reset_n,
                If(self.tinit0.ready,  # tINIT0 is MAX, so should be not ready
                    self.log.warn("tINIT0 violated")
                ),
                NextState("RESET")  # Tb
            )
        )
        fsm.act("RESET",
            self.tinit1.valid.eq(1),
            self.tinit2.valid.eq(~pads.cke),
            If(pads.reset_n,
                If(~self.tinit1.ready,
                    self.log.warn("tINIT1 violated")
                ),
                If(~self.tinit2.ready,
                    self.log.warn("tINIT2 violated")
                ),
                NextState("WAIT-PD"),  # Tc
            )
        )
        fsm.act("WAIT-PD",
            self.tinit3.valid.eq(1),
            If(self.tinit3.ready | (not init_delays),
                NextState("EXIT-PD")  # Td
            )
        )
        fsm.act("EXIT-PD",
            self.tinit5.valid.eq(1),
            If(self.tinit5.ready | (not init_delays),
                NextState("MRW")  # Te
            )
        )
        fsm.act("MRW",
            cmds_enabled.eq(1),
            If(self.handle_cmd & ~cmd_handlers["MRW"] & ~cmd_handlers["MPC"],
                self.log.warn("Only MRW/MRR commands expected before ZQ calibration")
            ),
            If(cmd_handlers["MPC"],
                If(self.mpc_op != MPC["ZQC-START"],
                    self.log.error("ZQC-START expected, got op=0b%07b", self.mpc_op)
                ).Else(
                    NextState("ZQC")  # Tf
                )
            ),
        )
        fsm.act("ZQC",
            self.tzqcal.valid.eq(1),
            cmds_enabled.eq(1),
            If(self.handle_cmd,
                If(~(cmd_handlers["MPC"] & (self.mpc_op == MPC["ZQC-LATCH"])),
                    self.log.error("Expected ZQC-LATCH")
                ).Else(
                    If(~self.tzqcal.ready,
                        self.log.warn("tZQCAL violated")
                    ),
                    NextState("NORMAL")  # Tg
                )
            ),
        )
        # TODO: Bus training currently is not performed in the simulation
        fsm.act("NORMAL",
            cmds_enabled.eq(1),
            self.tzqlat.valid.eq(1),
            once(self, self.handle_cmd & ~self.tzqlat.ready,
                self.log.warn("tZQLAT violated")
            ),
        )

        # Log state transitions
        fsm.finalize()
        prev_state = delayed(self, fsm.state)
        self.comb += If(prev_state != fsm.state,
            Case(prev_state, {
                state: Case(fsm.state, {
                    next_state: self.log.info(f"FSM: {state_name} -> {next_state_name}")
                    for next_state, next_state_name in fsm.decoding.items()
                })
                for state, state_name in fsm.decoding.items()
            })
        )

    def cmd_one_step(self, name, cond, comb, sync=None):
        matched = Signal()
        self.comb += If(self.handle_cmd & cond,
            self.log.debug(name),
            matched.eq(1),
            *comb
        )
        if sync is not None:
            self.sync += If(self.handle_cmd & cond,
                *sync
            )
        return matched

    def cmd_two_step(self, name, cond1, body1, cond2, body2):
        state1, state2 = f"{name}-1", f"{name}-2"
        matched = Signal()

        fsm = FSM()
        fsm.act(state1,
            If(self.handle_cmd & cond1,
                self.log.debug(state1),
                matched.eq(1),
                *body1,
                NextState(state2)
            )
        )
        fsm.act(state2,
            If(self.handle_cmd,
                If(cond2,
                    self.log.debug(state2),
                    matched.eq(1),
                    *body2
                ).Else(
                    self.log.error(f"Waiting for {state2} but got unexpected cs_high=0b%06b cs_low=0b%06b", self.cs_high, self.cs_low)
                ),
                NextState(state1)  # always back to first
            )
        )
        self.submodules += fsm

        return matched

    def mrw_handler(self):
        ma  = Signal(6)
        op7 = Signal()
        op  = Signal(8)
        return self.cmd_two_step("MRW",
            cond1 = self.cs_high[:5] == 0b00110,
            body1 = [
                NextValue(ma, self.cs_low),
                NextValue(op7, self.cs_high[5]),
            ],
            cond2 = self.cs_high[:5] == 0b10110,
            body2 = [
                self.log.info("MRW: MR[%d] = 0x%02x", ma, op),
                op.eq(Cat(self.cs_low, self.cs_high[5], op7)),
                self.mr_port.adr.eq(ma),
                self.mr_port.dat_w.eq(op),
                self.mr_port.we.eq(1),
            ]
        )

    def refresh_handler(self):
        return self.cmd_one_step("REFRESH",
            cond = self.cs_high[:5] == 0b01000,
            comb = [
                If(reduce(or_, self.active_banks),
                    self.log.error("Not all banks precharged during REFRESH")
                )
            ]
        )

    def activate_handler(self):
        bank = Signal(3)
        row1 = Signal(7)
        row2 = Signal(10)
        row  = Signal(17)
        return self.cmd_two_step("ACTIVATE",
            cond1 = self.cs_high[:2] == 0b01,
            body1 = [
                NextValue(bank, self.cs_low[:3]),
                NextValue(row1, Cat(self.cs_low[4:6], self.cs_high[2:6], self.cs_low[3])),
            ],
            cond2 = self.cs_high[:2] == 0b11,
            body2 = [
                self.log.info("ACT: bank=%d row=%d", bank, row),
                row2.eq(Cat(self.cs_low, self.cs_high[2:])),
                row.eq(Cat(row2, row1)),
                NextValue(self.active_banks[bank], 1),
                NextValue(self.active_rows[bank], row),
            ]
        )

    def precharge_handler(self):
        bank = Signal(3)
        return self.cmd_one_step("PRECHARGE",
            cond = self.cs_high[:5] == 0b10000,
            comb = [
                If(self.cs_high[5],
                    self.log.info("PRE: all banks"),
                    bank.eq(2**len(bank) - 1),
                ).Else(
                    self.log.info("PRE: bank = %d", bank),
                    bank.eq(self.cs_low[:3]),
                ),
            ],
            sync = [
                If(self.cs_high[5],
                    *[self.active_banks[b].eq(0) for b in range(2**len(bank))]
                ).Else(
                    self.active_banks[bank].eq(0)
                )
            ]
        )

    def mpc_handler(self):
        cases = {value: self.log.info(f"MPC: {name}") for name, value in MPC.items()}
        cases["default"] = self.log.error("Invalid MPC op=0b%07b", self.mpc_op)
        return self.cmd_one_step("MPC",
            cond = self.cs_high[:5] == 0b00000,
            comb = [
                self.mpc_op.eq(Cat(self.cs_low, self.cs_high[5])),
                If(self.cs_high[5] == 0,
                    self.log.info("MPC: NOOP")
                ).Else(
                    Case(self.mpc_op, cases)
                )
            ],
        )

    def cas_handler(self):
        cas1   = Signal(5)
        write1 = 0b00100
        read1  = 0b00010
        cas2   = 0b10010

        bank           = Signal(3)
        row            = Signal(17)
        col9           = Signal()
        col            = Signal(10)
        burst_len      = Signal()
        auto_precharge = Signal()

        return self.cmd_two_step("CAS",
            cond1 = (self.cs_high[:5] == write1) | (self.cs_high[:5] == read1),
            body1 = [
                NextValue(cas1, self.cs_high[:5]),
                NextValue(bank, self.cs_low[:3]),
                NextValue(col9, self.cs_low[4]),
                NextValue(burst_len, self.cs_high[5]),
                NextValue(auto_precharge, self.cs_low[5]),
            ],
            cond2 = self.cs_high[:5] == cas2,
            body2 = [
                row.eq(self.active_rows[bank]),
                If(cas1 == write1,
                    self.log.info("WRITE: bank=%d row=%d col=%d", bank, row, col),
                ).Elif(cas1 == read1,
                    self.log.info("READ: bank=%d row=%d col=%d", bank, row, col),
                ),
                If(auto_precharge,
                    self.log.info("AUTO-PRECHARGE: bank=%d row=%d", bank, row)
                ),
                col.eq(Cat(Replicate(0, 2), self.cs_low, self.cs_high[5])),
            ],
        )

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
