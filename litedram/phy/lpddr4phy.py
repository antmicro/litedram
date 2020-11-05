import re
from functools import reduce
from operator import or_

import math

from migen import *

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *


class LPDDR4PHY(Module, AutoCSR):
    def __init__(self, pads, *,
                 sys_clk_freq, write_ser_latency, read_des_latency, phytype
                 cmd_latency=1):
        self.pads        = pads
        self.memtype     = memtype     = "LPDDR4"
        self.nranks      = nranks      = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        self.databits    = databits    = len(pads.dq)
        self.addressbits = addressbits = 16  # for activate row address
        self.bankbits    = bankbits    = 3
        self.nphases     = nphases     = 8
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)
        assert databits % 8 == 0

        # Parameters -------------------------------------------------------------------------------
        iodelay_tap_average = {
            200e6: 78e-12,
            300e6: 52e-12,
            400e6: 39e-12,  # Only valid for -3 and -2/2E speed grades
        }
        half_sys8x_taps = math.floor(tck/(4*iodelay_tap_average[iodelay_clk_freq]))

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
        self._half_sys8x_taps = CSRStorage(5, reset=half_sys8x_taps)

        self._wlevel_en     = CSRStorage()
        self._wlevel_strobe = CSR()

        if with_odelay:
            self._cdly_rst = CSR()
            self._cdly_inc = CSR()

        self._dly_sel = CSRStorage(databits//8)

        self._rdly_dq_rst         = CSR()
        self._rdly_dq_inc         = CSR()
        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip     = CSR()

        if with_odelay:
            self._wdly_dq_rst   = CSR()
            self._wdly_dq_inc   = CSR()
            self._wdly_dqs_rst  = CSR()
            self._wdly_dqs_inc  = CSR()

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
        self.dfi = dfi = Interface(addressbits, bankbits, nranks, 2*databits, nphases=8)

        # # #

        adapters = [DFIPhaseAdapter(phase) for phase in self.dfi.phases]
        self.submodules += adapters


class DFIPhaseAdapter(Module):
    # We must perform mapping of DFI commands to the LPDDR4 commands set on CA bus
    # LPDDR4 "small command" consists of 2 words CA[5:0] sent on the bus in 2 subsequent
    # cycles. First cycle is marked with CS high, second with CS low.
    # Then most "big commands" consists of 2 "small commands".
    # If a command uses 1 "small command", then it shall go as cmd2 so that all command
    # timings can be counted from the same moment (cycle of cmd2 CS low).
    def __init__(self, dfi_phase):
        # CS/CA values for 4 SDR cycles
        self.cs = Signal(4)
        self.ca = Array([Signal(6) for _ in range(4)])

        # # #

        self.submodules.cmd1 = Command(dfi_phase)
        self.submodules.cmd2 = Command(dfi_phase)
        self.comb += [
            self.cs[:2].eq(self.cmd1.cs),
            self.cs[2:].eq(self.cmd2.cs),
            self.ca[0].eq(self.cmd1.ca[0])
            self.ca[1].eq(self.cmd1.ca[1])
            self.ca[2].eq(self.cmd2.ca[0])
            self.ca[3].eq(self.cmd2.ca[1])
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

        def cmds(cmd1, cmd2):
            return self.cmd1.set(cmd1) + self.cmd2.set(cmd2)

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
            "default":   cmds("DESELECT",   "DESELECT"),
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
