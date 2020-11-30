import re
from functools import reduce
from operator import or_

import math

from migen import *

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *


def _chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def bitpattern(s):
    if len(s) > 8:
        return [bitpattern(si) for si in _chunks(s, 8)]
    assert len(s) == 8
    s = s.translate(s.maketrans("_-", "01"))
    return int(s[::-1], 2)  # LSB first, so reverse the string

class ShiftRegister(Module):
    def __init__(self, n, i=None):
        if i is None:
            i = Signal()
        assert len(i) == 1

        self.i = i
        self.sr = sr = Signal(n)
        last = Signal.like(sr)

        self.comb += sr.eq(Cat(i, last))
        self.sync += last.eq(sr)

    def __getitem__(self, key):
        return self.sr[key]

class ConstBitSlip(Module):
    def __init__(self, dw, i=None, o=None, slp=None, cycles=1):
        self.i   = Signal(dw, name='i') if i is None else i
        self.o   = Signal(dw, name='o') if o is None else o
        assert cycles >= 1
        assert 0 <= slp <= cycles*dw-1
        slp = (cycles*dw-1) - slp

        # # #

        r = Signal((cycles+1)*dw, reset_less=True)
        self.sync += r.eq(Cat(r[dw:], self.i))
        cases = {}
        for i in range(cycles*dw):
            cases[i] = self.o.eq(r[i+1:dw+i+1])
        self.comb += Case(slp, cases)

class LPDDR4PHY(Module, AutoCSR):
    def __init__(self, pads, *,
                 sys_clk_freq, write_ser_latency, read_des_latency, phytype,
                 cmd_latency=1, cmd_delay=None):
        self.pads        = pads
        self.memtype     = memtype     = "LPDDR4"
        self.nranks      = nranks      = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        self.databits    = databits    = len(pads.dq)
        self.addressbits = addressbits = 17  # for activate row address
        self.bankbits    = bankbits    = 3
        self.nphases     = nphases     = 8
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)
        assert databits % 8 == 0

        # Parameters -------------------------------------------------------------------------------
        def get_cl_cw(memtype, tck):
            # MT53E256M16D1, No DBI, Set A
            f_to_cl_cwl = OrderedDict()
            f_to_cl_cwl[ 532e6] = ( 6,  4)
            f_to_cl_cwl[1066e6] = (10,  6)
            f_to_cl_cwl[1600e6] = (14,  8)
            f_to_cl_cwl[2132e6] = (20, 10)
            f_to_cl_cwl[2666e6] = (24, 12)
            f_to_cl_cwl[3200e6] = (28, 14)
            f_to_cl_cwl[3732e6] = (32, 16)
            f_to_cl_cwl[4266e6] = (36, 18)
            for f, (cl, cwl) in f_to_cl_cwl.items():
                if tck >= 2/f:
                    return cl, cwl
            raise ValueError

        cl, cwl         = get_cl_cw(memtype, tck)
        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)
        rdphase         = get_sys_phase(nphases, cl_sys_latency,   cl + cmd_latency)
        wrphase         = get_sys_phase(nphases, cwl_sys_latency, cwl + cmd_latency)

        # Registers --------------------------------------------------------------------------------
        self._rst             = CSRStorage()

        self._dly_sel         = CSRStorage(databits//8)

        self._wlevel_en     = CSRStorage()
        self._wlevel_strobe = CSR()

        self._dly_sel = CSRStorage(databits//8)

        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip     = CSR()

        self._wdly_dq_bitslip_rst = CSR()
        self._wdly_dq_bitslip     = CSR()

        self._rdphase = CSRStorage(int(math.log2(nphases)), reset=rdphase)
        self._wrphase = CSRStorage(int(math.log2(nphases)), reset=wrphase)

        # PHY settings -----------------------------------------------------------------------------
        self.settings = PhySettings(
            phytype       = phytype,
            memtype       = memtype,
            databits      = databits,
            dfi_databits  = 2*databits,
            nranks        = nranks,
            nphases       = nphases,
            rdphase       = self._rdphase.storage,
            wrphase       = self._wrphase.storage,
            cl            = cl,
            cwl           = cwl,
            read_latency  = cl_sys_latency + 6,
            write_latency = cwl_sys_latency - 1,
            cmd_latency   = cmd_latency,
            cmd_delay     = cmd_delay,
        )

        # DFI Interface ----------------------------------------------------------------------------
        # Due to the fact that LPDDR4 has 16n prefetch we use 8 phases to be able to read/write a
        # whole burst during a single controller clock cycle. PHY should use sys8x clock.
        self.dfi = dfi = Interface(addressbits, bankbits, nranks, 2*databits, nphases=8)

        # # #

        adapters = [DFIPhaseAdapter(phase) for phase in self.dfi.phases]
        self.submodules += adapters

        # Now prepare the data by converting the sequences on adapters into sequences on the pads.
        # We have to ignore overlapping commands, and module timings have to ensure that there are
        # no overlapping commands anyway.
        # Pads: reset_n, CS, CKE, CK, CA[5:0], DMI[1:0], DQ[15:0], DQS[1:0], ODT_CA
        self.ck_cke   = Signal(nphases)
        self.ck_odt   = Signal(nphases)
        self.ck_clk   = Signal(2*nphases)
        self.ck_cs    = Signal(nphases)
        self.ck_ca    = [Signal(nphases)   for _ in range(6)]
        self.ck_dmi_o = [Signal(2*nphases) for _ in range(2)]
        self.ck_dmi_i = [Signal(2*nphases) for _ in range(2)]
        self.dmi_oe   = Signal()
        self.ck_dq_o  = [Signal(2*nphases) for _ in range(databits)]
        self.ck_dq_i  = [Signal(2*nphases) for _ in range(databits)]
        self.dq_oe    = Signal()
        self.ck_dqs_o = [Signal(2*nphases) for _ in range(2)]
        self.ck_dqs_i = [Signal(2*nphases) for _ in range(2)]
        self.dqs_oe   = Signal()

        # Clocks -----------------------------------------------------------------------------------
        self.comb += self.ck_clk.eq(bitpattern("-_-_-_-_"))  # FIXME: length

        # Commands ---------------------------------------------------------------------------------
        # Each command can span several phases (up to 4), so we must ignore overlapping commands,
        # but in general, module timings should be set in a way that overlapping will never happen.

        cs_bitslips = []
        for phase, adapter in enumerate(adapters):
            cs_bs = ConstBitSlip(dw=nphases, cycles=1, slp=phase)
            self.submodules += cs_bs
            self.comb += cs_bs.i.eq(Cat(adapter.cs)),
            cs_bitslips.append(cs_bs)

        # bitslip introduces 2 cycle latency
        valids    = Signal(nphases)
        valids_d1 = Signal(nphases)
        valids_d2 = Signal(nphases)
        self.comb += valids.eq(Cat(a.valid for a in adapters))
        self.sync += valids_d1.eq(valids)
        self.sync += valids_d2.eq(valids_d1)
        valids_hist = Signal(2*nphases)
        self.comb += valids_hist.eq(Cat(valids_d2, valids_d1))

        cs_masked = []
        for phase, adapter in enumerate(adapters):
            cs = cs_bitslips[phase].o
            # we can set the signal if there was no command on 3 previous cycles
            valids_prev = valids_hist[nphases-3 + phase:nphases + phase]
            print(f'{phase=} {nphases-3 + phase}:{nphases + phase} len({len(valids_prev)})')
            any_valid = reduce(or_, valids_prev)
            allowed = ~any_valid
            mask = Replicate(allowed, len(cs))
            cs_masked.append(cs & mask)

        self.comb += self.ck_cs.eq(reduce(or_, cs_masked))



class LPDDR4Pads(Module):
    def __init__(self, databits=16):
        self.clk_p   = Signal()
        self.clk_n   = Signal()
        self.cke     = Signal()
        self.cs      = Signal()
        self.ca      = Signal(6)
        self.odt     = Signal()
        self.dq      = Signal(databits)
        self.dqs     = Signal(databits//8)
        self.dmi     = Signal(databits//8)
        self.reset_n = Signal()

class SimulationPHY(LPDDR4PHY):
    def __init__(self, sys_clk_freq=100e6):
        pads = LPDDR4Pads()
        self.submodules += pads
        super().__init__(pads, sys_clk_freq=sys_clk_freq,
                         write_ser_latency=1, read_des_latency=1, phytype="SimulationPHY")

        # Serialization
        sd_sys   = self.sync.sys
        sd_sys8x = self.sync.sys8x
        self.submodules += Serializer(sd_sys, sd_sys8x, i_dw=8, o_dw=1, i=self.ck_cs, o=self.pads.cs, name='ser_cs')

class Serializer(Module):
    def __init__(self, sd_clk, sd_clkdiv, i_dw, o_dw, i=None, o=None, reset=None, name=None):
        assert i_dw % o_dw == 0
        ratio = i_dw // o_dw

        if i is None: i = Signal(i_dw)
        if o is None: o = Signal(o_dw)
        if reset is None: reset = Signal()

        self.i = i
        self.o = o
        self.reset = reset

        cnt = Signal(max=ratio, name='{}_cnt'.format(name) if name is not None else None)
        sd_clkdiv += If(reset, cnt.eq(0)).Else(cnt.eq(cnt + 1))

        i_d = Signal.like(self.i)
        sd_clk += i_d.eq(self.i)
        i_array = Array([self.i[n*o_dw:(n+1)*o_dw] for n in range(ratio)])
        self.comb += self.o.eq(i_array[cnt])


class DFIPhaseAdapter(Module):
    # We must perform mapping of DFI commands to the LPDDR4 commands set on CA bus.
    # LPDDR4 "small command" consists of 2 words CA[5:0] sent on the bus in 2 subsequent
    # cycles. First cycle is marked with CS high, second with CS low.
    # Then most "big commands" consist of 2 "small commands" (e.g. ACTIVATE-1, ACTIVATE-2).
    # If a command uses 1 "small command", then it shall go as cmd2 so that all command
    # timings can be counted from the same moment (cycle of cmd2 CS low).
    def __init__(self, dfi_phase):
        # CS/CA values for 4 SDR cycles
        self.cs = Signal(4)
        self.ca = Array([Signal(6) for _ in range(4)])
        self.valid = Signal()

        # # #

        self.submodules.cmd1 = Command(dfi_phase)
        self.submodules.cmd2 = Command(dfi_phase)
        self.comb += [
            self.cs[:2].eq(self.cmd1.cs),
            self.cs[2:].eq(self.cmd2.cs),
            self.ca[0].eq(self.cmd1.ca[0]),
            self.ca[1].eq(self.cmd1.ca[1]),
            self.ca[2].eq(self.cmd2.ca[0]),
            self.ca[3].eq(self.cmd2.ca[1]),
        ]

        dfi_cmd = Signal(3)
        self.comb += dfi_cmd.eq(Cat(~dfi_phase.we_n, ~dfi_phase.ras_n, ~dfi_phase.cas_n)),
        _cmd = {  # cas, ras, we
            "NOP": 0b000,
            "ACT": 0b010,
            "RD":  0b100,
            "WR":  0b101,
            "PRE": 0b011,
            "REF": 0b110,
            "ZQC": 0b001,
            "MRS": 0b111,
        }

        def cmds(cmd1, cmd2, valid=1):
            return self.cmd1.set(cmd1) + self.cmd2.set(cmd2) + [self.valid.eq(valid)]

        self.comb += Case(dfi_cmd, {
            _cmd["ACT"]: cmds("ACTIVATE-1", "ACTIVATE-2"),
            _cmd["RD"]:  cmds("READ-1",     "CAS-2"),
            _cmd["WR"]:  cmds("WRITE-1",    "CAS-2"),
            _cmd["PRE"]: cmds("DESELECT",   "PRECHARGE"),
            _cmd["REF"]: cmds("DESELECT",   "REFRESH"),
            # TODO: ZQC init/short/long? start/latch?
            # _cmd["ZQC"]: [
            #     *cmds("DESELECT", "MPC"),
            #     self.cmd2.mpc.eq(0b1001111),
            # ],
            _cmd["MRS"]: cmds("MRW-1",      "MRW-2"),
            "default":   cmds("DESELECT",   "DESELECT", valid=0),
        })

class Command(Module):
    # String description of 1st and 2nd edge of each command, later parsed to construct
    # the value. CS is assumed to be H for 1st edge and L for 2nd edge.
    TRUTH_TABLE = {
        "MRW-1":        ["L H H L L OP7",       "MA0 MA1 MA2 MA3 MA4 MA5"],
        "MRW-2":        ["L H H L H OP6",       "OP0 OP1 OP2 OP3 OP4 OP5"],
        "MRR-1":        ["L H H H L V",         "MA0 MA1 MA2 MA3 MA4 MA5"],
        "REFRESH":      ["L L L H L AB",        "BA0 BA1 BA2 V V V"],
        "ACTIVATE-1":   ["H L R12 R13 R14 R15", "BA0 BA1 BA2 R16 R10 R11"],
        "ACTIVATE-2":   ["H H R6 R7 R8 R9",     "R0 R1 R2 R3 R4 R5"],
        "WRITE-1":      ["L H L L BL",          "BA0 BA1 BA2 V C9 AP"],
        "MASK WRITE-1": ["L L H H L BL",        "BA0 BA1 BA2 V C9 AP"],
        "READ-1":       ["L H L L L BL",        "BA0 BA1 BA2 V C9 AP"],
        "CAS-2":        ["L H L L H C8",        "C2 C3 C4 C5 C6 C7"],
        "PRECHARGE":    ["L L L L H AB",        "BA0 BA1 BA2 V V V"],
        "MPC":          ["L L L L L OP6",       "OP0 OP1 OP2 OP3 OP4 OP5"],
        "DESELECT":     ["X X X X X X",         "X X X X X X"],
    }

    def __init__(self, dfi_phase):
        self.cs = Signal(2)
        self.ca = Array([Signal(6), Signal(6)])  # CS high, CS low
        self.mpc = Signal(7)  # special OP values for multipurpose command
        self.dfi = dfi_phase

    def set(self, cmd):
        desc = self.TRUTH_TABLE[cmd]
        ops = []
        for i, description in enumerate(self.TRUTH_TABLE[cmd]):
            for j, bit in enumerate(description.split()):
                ops.append(self.ca[i][j].eq(self.parse_bit(bit, is_mpc=cmd == "MPC")))
        if cmd != "DESELECT":
            ops.append(self.cs[0].eq(1))
        return ops

    def parse_bit(self, bit, is_mpc=False):
        rules = {
            "H":       lambda: 1,  # high
            "L":       lambda: 0,  # low
            "V":       lambda: 0,  # defined logic
            "X":       lambda: 0,  # don't care
            "BL":      lambda: 0,  # on-the-fly burst length, not using
            "AP":      lambda: self.dfi.address[10],  # auto precharge
            "AB":      lambda: self.dfi.address[10],  # all banks
            "BA(\d+)": lambda i: self.dfi.bank[i],
            "R(\d+)":  lambda i: self.dfi.address[i],  # row
            "C(\d+)":  lambda i: self.dfi.address[i],  # column
            "MA(\d+)": lambda i: self.dfi.address[8+i],  # mode register address
            # mode register value, or op code for MPC
            "OP(\d+)": lambda i: self.mpc[i] if is_mpc else self.dfi.address[i],
        }
        for pattern, value in rules.items():
            m = re.match(pattern, bit)
            if m:
                args = [int(g) for g in m.groups()]
                return value(*args)
        raise ValueError(bit)
