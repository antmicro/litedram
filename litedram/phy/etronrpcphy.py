# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
# Etron RPC DRAM PHY

from math import ceil
from operator import and_

from migen import *
from migen.fhdl.specials import Tristate
from migen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *
from litedram.modules import SDRAMModule, _TechnologyTimings, _SpeedgradeTimings

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

# Etron RPC Module ---------------------------------------------------------------------------------

class EM6GA16L(SDRAMModule):
    memtype = "RPC"
    # geometry
    nbanks = 4     # 64MBits per bank => 256Mb
    ncols = 1024   # most probably? and we have 10-bit column address, so no more than that
    nrows = 4096  # 64M / 1024 / 16 = 4k
    # timings TODO: real timings
    technology_timings = _TechnologyTimings(
        # Refresh: needs to refresh every row once per 64ms
        # Is tREFI in RPC the same as in normal DRAM?
        #   tREFI = 100ns (FST refresh) or 3.2us (LP refresh)
        tREFI=64e6/8192,
        # # It seems that we need to calculate tCCD from tBESL by assuming BC=1, then it is:
        # # RL=WL + burst_time + tBESL
        # tCCD=(13+1 + 8 + 7, None),
        # # CCD enforces tBESL for read commands, we use tWTR to ensure it for writes
        # # 11 for tBESL + 2 for STB low before a command
        # tWTR=(11 + 2, None),
        # tRRD=(None, 7.5),
        # tZQCS=(None, 90)

        # longer timings for testing to make sure the timings are not the problem
        tCCD=(13+1 + 8 + 7 + 10, None),
        tWTR=(11 + 2 + 10, None),
        tRRD=(None, 7.5 + 5),
        tZQCS=None  # disable ZQCS
    )
    speedgrade_timings = {
        # FIXME: we're increasing tWR by 1 sysclk to compensate for long write
        # Should we use tRFQSd for tRFC?
        # "1600": _SpeedgradeTimings(tRP=13.75, tRCD=13.75, tWR=15 + (1/50e6 * 2), tRFC=(3*100, None), tFAW=None, tRAS=35),
        "1866": _SpeedgradeTimings(tRP=13.91, tRCD=13.91, tWR=15 + (1/10e6 * 2), tRFC=(3*100, None), tFAW=None, tRAS=34),
        # longer timings for testing to make sure the timings are not the problem
        "1600": _SpeedgradeTimings(tRP=20, tRCD=20, tWR=40 + (1/10e6 * 8), tRFC=(5*100, None), tFAW=None, tRAS=50),
    }
    speedgrade_timings["default"] = speedgrade_timings["1600"]

# RPC Commands -------------------------------------------------------------------------------------

class ModeRegister:
    """RPC Mode Register encoding (RPC has only 1 mode register)"""
    def __init__(self):
        self.cl      = Signal(3)
        # TODO: in LPDDR3 nWR is the number of clock cycles determining when to start internal
        # precharge for a write burst when auto-precharge is enabled (ceil(tRW/tCK) ?)
        self.nwr     = Signal(3)
        self.zout    = Signal(4)
        self.odt     = Signal(3)
        self.odt_stb = Signal(1)
        self.csr_fx  = Signal(1)
        self.odt_pd  = Signal(1)
        self.tm      = Signal(1)

    CL = {
        8:  0b000,  # default
        10: 0b001,
        11: 0b010,
        13: 0b011,
        3:  0b110,
    }
    NWR = {
        4:  0b000,
        6:  0b001,
        7:  0b010,
        8:  0b011,  # default
        10: 0b100,
        12: 0b101,
        14: 0b110,
        16: 0b111,
    }
    ZOUT = {  # resistance in Ohms
        120:     0b0010,
        90:      0b0100,
        51.4:    0b0110,
        60:      0b1000,
        40:      0b1010,
        36:      0b1100,
        27.7:    0b1110,
        "short": 0b0001,  # 0bxxx1
        "open":  0b0000,  # output disabled, default
    }
    ODT = {
        60:     0b001,
        45:     0b010,
        25.7:   0b011,
        30:     0b100,
        20:     0b101,
        18:     0b110,
        13.85:  0b111,
        "open": 0b000,
    }

    # Encode mode register information in DFI address/bank
    DFI_ENCODING = {
        # field: (dfi_signal, width, offset)
        "cl":      ("address", 3,  0),
        "nwr":     ("address", 3,  3),
        "zout":    ("address", 4,  6),
        "odt":     ("address", 3, 10),
        "csr_fx":  ("address", 1, 13),
        "odt_stb": ("bank",    1,  0),
        "odt_pd":  ("bank",    1,  1),
        "tm":      None,
    }

    @classmethod
    def dfi_encode(cls, **kwargs):
        address = 0
        bank = 0
        for field, encoding in cls.DFI_ENCODING.items():
            if encoding is None:
                continue
            sig, width, offset = encoding
            value = (kwargs[field] & (2**width - 1)) << offset
            if sig == "address":
                address |= value
            elif sig == "bank":
                bank |= value
            else:
                raise ValueError(sig)
        return address, bank

    def dfi_decode(self, dfi_phase):
        r = []
        for field, encoding in self.DFI_ENCODING.items():
            if encoding is None:
                continue
            sig, width, offset = encoding
            r += [getattr(self, field).eq(getattr(dfi_phase, sig)[offset:offset+width])]
        return r

class DFIAdapter(Module):
    """Translate DFI commands into RPC parallel commands format (Request Packet)

    Maps DFI commands to RPC parallel packet commands (sent over DB lines). Some commands cannot
    be represented by cas_n/ras_n/we_n combinations, for that reason when reset_n=0, some DFI
    commands are interpreted specially:
    - ACT -> perform a RESET sequence
    - MRS -> write UTR, utr_op and utr_en are encoded in phase.address (see `dfi_utr_encode`)
    - ZQC -> will perform di_ferent types of ZQ calibration: "init" when auto_precharge=1,
             "reset" if auto_precharge=0
    """
    ZQC_OP = {
        "init":  0b00,  # calibration after initialization
        "long":  0b01,
        "short": 0b10,
        "reset": 0b11,  # ZQ reset
    }
    REF_OP = {
        "FST": 0b00,  # FST refresh: tREFi = 100ns
        "LP":  0b00,  # LP refresh:  tREFi = 3.2us
    }
    UTR_OP = {  # Utility Register read pattern
        "0101": 0b00,
        "1100": 0b01,
        "0011": 0b10,
        "1010": 0b11,
    }

    SPECIAL_CMDS = {
        "RESET":    "ACT",
        "UTR":      "MRS",
        "ZQC_INIT": "ZQC",
    }

    @classmethod
    def dfi_utr_encode(cls, utr_op, utr_en):
        if isinstance(utr_op, str):
            utr_op = cls.UTR_OP[utr_op]
        address = 0
        address |= (utr_en &  0b1) << 0
        address |= (utr_op & 0b11) << 1
        return address

    def __init__(self, phase):
        self.db_p = Signal(16)  # on positive edge
        self.db_n = Signal(16)  # on negative edge

        self.cmd_valid = Signal()
        self.do_reset  = Signal()  # force sending RESET command
        self.bc        = Signal(6)  # burst count (bs+1 = number of 32-byte words in the transfer)
        self.ref_op    = Signal(2)

        # # #

        self.phase_cmd = phase_cmd = Signal(3)
        self.comb += phase_cmd.eq(Cat(phase.cas_n, phase.ras_n, phase.we_n))

        def _cmd(cas, ras, we):
            assert cas in [0, 1] and ras in [0, 1] and we in [0, 1]
            return ((1 - cas) << 0) | ((1 - ras) << 1) | ((1 - we) << 2)

        self.dfi_cmds = dfi_cmds = {
            "NOP": _cmd(cas=0, ras=0, we=0),
            "ACT": _cmd(cas=0, ras=1, we=0),
            "RD":  _cmd(cas=1, ras=0, we=0),
            "WR":  _cmd(cas=1, ras=0, we=1),
            "PRE": _cmd(cas=0, ras=1, we=1),
            "REF": _cmd(cas=1, ras=1, we=0),
            "ZQC": _cmd(cas=0, ras=0, we=1),
            "MRS": _cmd(cas=1, ras=1, we=1),
        }

        # precharge/refresh use a bank bitmask, so PRECHARGE ALL uses 0b1111
        self.special_cmds = special_cmds = Signal()
        self.utr_en       = utr_en       = phase.address[0]
        utr_op         = phase.address[1:3]
        bk             = Signal(4)
        zqc_op         = Signal(2)
        auto_precharge = Signal()
        mr             = ModeRegister()

        self.comb += mr.dfi_decode(phase)
        self.comb += [
            special_cmds.eq(~phase.reset_n),
            auto_precharge.eq(phase.address[10]),
            If(auto_precharge,
                bk.eq(0b1111),
            ).Else(  # binary to one-hot encoded
                Case(phase.bank[:2], {i: bk.eq(1 << i) for i in range(4)}),
            ),
            If(special_cmds,
                If(auto_precharge,
                    zqc_op.eq(self.ZQC_OP["init"]),
                ).Else(
                    zqc_op.eq(self.ZQC_OP["reset"]),
                )
            ).Else(
                If(auto_precharge,
                    zqc_op.eq(self.ZQC_OP["long"]),
                ).Else(
                    zqc_op.eq(self.ZQC_OP["short"]),
                )
            ),
        ]

        cases = {
            "NOP": [
                self.db_p.eq(0),
                self.db_n.eq(0),
            ],
            "ACT": [
                self.db_p[0:2  +1].eq(0b101),
                self.db_p[3:4  +1].eq(phase.bank[:2]),
                self.db_n[0      ].eq(0),
                self.db_n[1:12 +1].eq(phase.address[:12]),  # row address
            ],
            "RD": [
                self.db_p[0:2   +1].eq(0b000),
                self.db_p[3:4   +1].eq(phase.bank[:2]),
                self.db_p[5:10  +1].eq(self.bc),
                self.db_p[13:15 +1].eq(phase.address[4:6 +1]),
                self.db_n[0       ].eq(0),
                self.db_n[13:15 +1].eq(phase.address[7:9 +1]),
            ],
            "WR": [
                self.db_p[0:2   +1].eq(0b001),
                self.db_p[3:4   +1].eq(phase.bank[:2]),
                self.db_p[5:10  +1].eq(self.bc),
                self.db_p[13:15 +1].eq(phase.address[4:6 +1]),
                self.db_n[0       ].eq(0),
                self.db_n[13:15 +1].eq(phase.address[7:9 +1]),
            ],
            "PRE": [
                self.db_p[6:9+1].eq(bk),
                self.db_p[0:2+1].eq(0b100),
                self.db_n[0    ].eq(0),
            ],
            "REF": [
                self.db_p[6:9+1].eq(bk),
                self.db_p[0:2+1].eq(0b110),
                self.db_n[1:2+3].eq(self.ref_op),
                self.db_n[0    ].eq(0),
            ],
            "ZQC": [
                self.db_p[0:2  +1].eq(0b001),
                self.db_p[14:15+1].eq(zqc_op),
                self.db_n[0      ].eq(1),
            ],
            "MRS": [
                self.db_p[0:2  +1].eq(0b010),
                self.db_p[3:15 +1].eq(Cat(mr.cl, mr.nwr, mr.zout, mr.odt)),
                self.db_n[0      ].eq(0),
                self.db_n[12:15+1].eq(Cat(mr.odt_stb, mr.csr_fx, mr.odt_pd, mr.tm)),
            ],
            "UTR": [
                self.db_p[0:2  +1].eq(0b111),
                self.db_p[3      ].eq(utr_en),
                self.db_p[4:5  +1].eq(utr_op),
                self.db_n[0      ].eq(0),
            ],
            "RESET": [
                self.db_p.eq(0),
                self.db_n.eq(1),
            ],
        }

        self.comb += [
            If(self._is_cmd("RESET"),
                cases["RESET"],
                self.cmd_valid.eq(1),
            ).Elif(self._is_cmd("UTR"),
                cases["UTR"],
                self.cmd_valid.eq(1),
            ).Else(
                self.cmd_valid.eq(~self._is_cmd("NOP")),
                Case(phase_cmd, {dfi_cmds[cmd]: cases[cmd] for cmd in dfi_cmds.keys()}),
            ),
        ]

    def _is_cmd(self, cmd):
        if isinstance(cmd, list):
            return reduce(or_, (self._is_cmd(c) for c in cmd))
        if cmd in ["RESET", "UTR", "ZQC_INIT"]:
            return self.special_cmds & (self.phase_cmd == self.dfi_cmds[self.SPECIAL_CMDS[cmd]])
        return self.phase_cmd == self.dfi_cmds[cmd]

    def is_cmd(self, cmd):
        return self._is_cmd(cmd) & self.cmd_valid

# Etron RPC DRAM PHY Base --------------------------------------------------------------------------

class RPCPads:
    _layout = [
        ("clk_p",  1),
        ("clk_n",  1),
        ("cs_n",   1),
        ("dqs_p",  1),  # may be 2 (hardware option; 2-bit DQS strobes DB by bytes: [0:7], [8:15])
        ("dqs_n",  1),  # may be 2
        ("stb",    1),
        ("db",    16),
    ]

    def __init__(self, pads):
        self.map(pads)
        for pad, width in self._layout:
            assert len(getattr(self, pad)) >= width, \
                "Pad {} has width {} < {}".format(pad, len(getattr(self, pad)), width)

    # reimplement if a specific mapping is needed
    def map(self, pads):
        for pad, _ in self._layout:
            setattr(self, pad, getattr(pads, pad))

class BasePHY(Module, AutoCSR):
    def __init__(self, pads, sys_clk_freq, write_ser_latency, read_des_latency, phytype):
        # TODO: pads groups for multiple chips
        #  pads = PHYPadsCombiner(pads)
        if not isinstance(pads, RPCPads):
            pads = RPCPads(pads)
        self.pads = pads

        self.memtype     = memtype     = "RPC"
        self.nranks      = nranks      = 1
        self.databits    = databits    = 16
        self.addressbits = addressbits = 14
        self.bankbits    = bankbits    = 2
        self.nphases     = nphases     = 4
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)

        # CSRs -------------------------------------------------------------------------------------
        self._dly_sel             = CSRStorage(len(self.pads.dqs_p))
        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip     = CSR()

        self._reset_done = CSRStatus()
        self._init_done  = CSRStatus()

        # PHY settings -----------------------------------------------------------------------------
        def get_cl(tck):
            # FIXME: for testing it's easier to use CL=8; read/write will be on phase 3; max sys_clk_freq=100e6
            assert tck >= 2/800e6
            return 8
            # tck is for DDR frequency
            f_to_cl = OrderedDict()
            f_to_cl[533e6]  =  3
            f_to_cl[800e6]  =  8
            f_to_cl[1200e6] =  8
            f_to_cl[1333e6] = 10
            f_to_cl[1600e6] = 11
            f_to_cl[1866e6] = 13
            for f, cl in f_to_cl.items():
                if tck >= 2/f:
                    return cl
            raise ValueError(tck)

        # RPC always has AL=1 and both read and write latencies are equal: RL=WL=AL+CL
        al = 1
        cwl = cl = get_cl(tck) + al

        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)

        rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
        wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)

        # Read latency
        db_cmd_dly   = 2  # (need 1 cycle to insert STB preamble + 1 more to always meet tCSS)
        cmd_ser_dly  = write_ser_latency
        read_mux_dly = 1
        bitslip_dly  = 3  # ncycles + 1
        # Time until first data is available on DB
        read_db_dly = db_cmd_dly + cmd_ser_dly + cl_sys_latency
        # Time until data is set on DFI
        read_dfi_dly = read_des_latency + read_mux_dly + bitslip_dly
        # Final latency
        read_latency = read_db_dly + read_dfi_dly

        # Write latency for the controller. We must send 1 cycles of data mask before the
        # data, and we serialize data over 2 sysclk cycles due to minimal BL=16, so we
        # are writing in the 2 cycles following the cycle when we obtain data on DFI.
        # Other PHYs can send everything in 1 sysclk. Because of this spcific delay, we
        # have to increate tWR in the RPC SDRModule definition to meet tWR requirements.
        # +1 cycle needed to insert CS before command
        write_latency = cwl_sys_latency + 1

        self.settings = PhySettings(
            phytype       = phytype,
            memtype       = memtype,
            databits      = databits,
            dfi_databits  = 4*databits,
            nranks        = nranks,
            nphases       = nphases,
            rdphase       = rdphase,
            wrphase       = wrphase,
            rdcmdphase    = rdcmdphase,
            wrcmdphase    = wrcmdphase,
            cl            = cl,
            cwl           = cwl,
            read_latency  = read_latency,
            write_latency = write_latency
        )

        # DFI Interface ----------------------------------------------------------------------------
        # minimal BL=16, which gives 16*16=256 bits; with 4 phases we need 16/4=4 data widths
        dfi_params = dict(addressbits=addressbits, bankbits=bankbits, nranks=nranks,
                          databits=4*databits, nphases=nphases)

        # Register DFI history (from newest to oldest), as we need to operate on 3 subsequent cycles
        # hist[0] = dfi[N], hist[1] = dfi[N-1], ...
        self.dfi = dfi = Interface(**dfi_params)
        dfi_hist = [dfi, Interface(**dfi_params), Interface(**dfi_params)]
        self.sync += dfi_hist[0].connect(dfi_hist[1], omit={"rddata", "rddata_valid"})
        self.sync += dfi_hist[1].connect(dfi_hist[2], omit={"rddata", "rddata_valid"})

        # Serialization ----------------------------------------------------------------------------
        # We have the following signals that have to be serialized:
        # - CLK (O)  - full-rate clock
        # - CS  (O)  - chip select
        # - STB (O)  - serial commands, serial preamble
        # - DB  (IO) - transmits data in/out, data mask, parallel commands
        # - DQS (IO) - strobe for commands/data/mask on DB pins
        # DQS is edge-aligned to CLK, while DB and STB are center-aligned to CLK (phase = -90).
        # Sending a parallel command (on DB pins):
        #  CLK: ____----____----____----____----____----____
        #  STB: ----------________________------------------
        #  DQS: ....................----____----____........
        #  DB:  ..........................PPPPnnnn..........
        # The signals prepared by BasePHY will all be phase-aligned. The concrete PHY should shift
        # them so that DB/STB/CS are delayed by 90 degrees in relation to CLK/DQS.

        # Signal values (de)serialized during 1 sysclk.
        # These signals must be populated in specific PHY implementations.
        self.clk_1ck_out  = clk_1ck_out  = Signal(2*nphases)
        self.stb_1ck_out  = stb_1ck_out  = Signal(2*nphases)
        self.cs_n_1ck_out = cs_n_1ck_out = Signal(2*nphases)

        self.dqs_1ck_out = dqs_1ck_out = Signal(2*nphases)
        self.dqs_1ck_in  = dqs_1ck_in  = Signal(2*nphases)
        self.dqs_oe      = dqs_oe      = Signal()

        self.db_1ck_out  = db_1ck_out  = [Signal(2*nphases) for _ in range(databits)]
        self.db_1ck_in   = db_1ck_in   = [Signal(2*nphases) for _ in range(databits)]
        self.db_oe       = db_oe       = Signal()

        # Clocks -----------------------------------------------------------------------------------
        self.comb += clk_1ck_out.eq(bitpattern("-_-_-_-_"))

        # DB muxing --------------------------------------------------------------------------------
        # Muxed cmd/data/mask
        db_1ck_data = [Signal(2*nphases) for _ in range(databits)]
        db_1ck_mask = [Signal(2*nphases) for _ in range(databits)]
        db_1ck_cmd  = [Signal(2*nphases) for _ in range(databits)]
        dq_data_en  = Signal()
        dq_mask_en  = Signal()
        dq_cmd_en   = Signal()
        dq_read_stb = Signal()

        # Output enable when writing cmd/data/mask
        # Mask is being send during negative half of sysclk
        self.comb += db_oe.eq(dq_data_en | dq_mask_en | dq_cmd_en)

        # Mux between cmd/data/mask
        for i in range(databits):
            self.comb += \
                If(dq_data_en,
                    db_1ck_out[i].eq(db_1ck_data[i])
                ).Elif(dq_mask_en,
                    db_1ck_out[i].eq(db_1ck_mask[i])
                ).Else(
                    db_1ck_out[i].eq(db_1ck_cmd[i])
                )

        # Parallel commands ------------------------------------------------------------------------
        # We need to insert 2 full-clk cycles of STB=0 before any command, to mark the beginning of
        # Request Packet. For that reason we use the previous values of DFI commands. To always be
        # able to meet tCSS, we have to add a delay of 1 more sysclk.
        # list from oldest to newest: dfi[N-1][p0], dfi[N-1][p1], ..., dfi[N][p0], dfi[N][p1], ...
        dfi_adapters = []
        for phase in dfi_hist[2].phases + dfi_hist[1].phases + dfi_hist[0].phases:
            adapter = DFIAdapter(phase)
            self.submodules += adapter
            dfi_adapters.append(adapter)
            self.comb += [
                # We always send one WORD, which consists of 32 bytes.
                adapter.bc.eq(0),
                # Always use fast refresh (equivalent to auto refresh) instead of low-power refresh
                # (equivalent to self refresh).
                adapter.ref_op.eq(adapter.REF_OP["FST"]),
            ]

        # Serialize commands to DB pins
        for i in range(databits):
            # A list of differential DB values using previous DFI coomand:
            # db_p[p][i], db_n[p][i], db_p[p+1][i], db_n[p+1][i], ...
            bits = [db for a in dfi_adapters[:nphases] for db in [a.db_p[i], a.db_n[i]]]
            self.comb += db_1ck_cmd[i].eq(Cat(*bits))

        # Commands go on the 2nd cycle, so use previous DFI
        self.comb += dq_cmd_en.eq(reduce(or_, [a.cmd_valid for a in dfi_adapters[:nphases]]))

        # Power Up Reset ---------------------------------------------------------------------------
        # During Power Up, after stabilizing clocks, Power Up Reset must be done. It consists of a
        # a Parallel Reset followed by two Serial Resets (2x8=16 full-rate cycles = 4 sys cycles).
        # We use an FSM to make sure that we pass only the commands from the controller that are
        # supported in the current state.
        t_reset          = 5e-6
        t_zqcinit        = 1e-6
        serial_reset_len = 4

        cmd_valid          = Signal()
        stb_reset_seq      = Signal()
        serial_reset_count = Signal(max=serial_reset_len + 1)

        # prolong the cmd_valid for the cmd latency (length of history)
        self.submodules.cmd_valid_sr = ShiftRegister(len(dfi_adapters) // nphases)
        self.comb += cmd_valid.eq(reduce(or_, self.cmd_valid_sr))

        self.submodules.reset_timer = WaitTimer(ceil(t_reset * sys_clk_freq))
        self.submodules.zqcinit_timer = WaitTimer(ceil(t_zqcinit * sys_clk_freq))

        self.submodules.reset_fsm = fsm = FSM()
        fsm.act("IDLE",
            NextValue(serial_reset_count, 0),
            NextValue(self._reset_done.status, 0),
            self.cmd_valid_sr.i.eq(dfi_adapters[2*nphases+0].is_cmd("RESET")),
            If(self.cmd_valid_sr.i,
                NextState("RESET_RECEIVED")
            ),
        )
        fsm.act("RESET_RECEIVED",
            NextState("SERIAL_RESET")
        )
        fsm.act("SERIAL_RESET",
            self.reset_timer.wait.eq(1),
            If(serial_reset_count != serial_reset_len,
               stb_reset_seq.eq(1),
               NextValue(serial_reset_count, serial_reset_count + 1),
            ),
            If(self.reset_timer.done,
                NextValue(self._reset_done.status, 1),
                NextState("RESET_DONE")
            )
        )
        fsm.act("RESET_DONE",
            self.cmd_valid_sr.i.eq(dfi_adapters[2*nphases+0].is_cmd(["PRE", "MRS", "ZQC_INIT"])),
            If(dfi_adapters[2*nphases+0].is_cmd("ZQC_INIT"),
                NextState("ZQC_INIT")
            )
        )
        fsm.act("ZQC_INIT",
            self.zqcinit_timer.wait.eq(1),
            If(self.zqcinit_timer.done,
                NextState("READY")
            )
        )
        fsm.act("READY",
            self._init_done.status.eq(1),
            self.cmd_valid_sr.i.eq(1),
            If(dfi_adapters[2*nphases+0].is_cmd("UTR") & (dfi_adapters[2*nphases+0].utr_en == 1),
                NextState("UTR_MODE")
            )
        )
        fsm.act("UTR_MODE",
            self._init_done.status.eq(1),
            self.cmd_valid_sr.i.eq(reduce(or_, [dfi_adapters[p].is_cmd(["UTR", "RD"])
                                                for p in [2*nphases, 2*nphases+self.settings.rdphase]])),
            If(dfi_adapters[2*nphases+0].is_cmd("UTR") & (dfi_adapters[2*nphases+0].utr_en == 0),
                NextState("READY")
            )
        )

        # STB --------------------------------------------------------------------------------------
        # Currently not sending any serial commands, but the STB pin must be held low for 2 full
        # rate cycles before writing a parallel command to activate the DRAM.
        stb_bits = []
        for p in range(nphases):
            # Use cmd from current and prev cycle, depending on which phase the command appears on
            preamble = (dfi_adapters[p+2].cmd_valid | dfi_adapters[p+1].cmd_valid) & cmd_valid
            # We only want to use STB to start parallel commands, serial reset or to send NOPs. NOP
            # is indicated by the first two bits being high (0b11, and other as "don't care"), so
            # we can simply hold STB high all the time and reset is zeros for 8 cycles (1 sysclk).
            stb_bits += 2 * [~(preamble | stb_reset_seq)]
        self.comb += stb_1ck_out.eq(Cat(*stb_bits))

        # Chip Select ------------------------------------------------------------------------------
        # RPC has quite high required time of CS# low before sending a command (tCSS), this means
        # that we would need 1 more cmd_latency to support it for all standard frequencies.
        tCSS = 10e-9
        tCSH =  5e-9
        # CS# is held for 2 sysclks before any command, and 1 sysclk after any command
        assert 2 * 1/sys_clk_freq >= tCSS, "tCSS not met for commands on phase 0"
        assert 1 * 1/sys_clk_freq >= tCSH, "tCSH not met for commands on phase 3"

        cs        = Signal()
        cs_cond   = Signal()
        cs_cond_d = Signal()
        self.sync += cs_cond_d.eq(cs_cond)
        self.comb += [
            cs_cond.eq(cmd_valid & reduce(or_, (a.cmd_valid for a in dfi_adapters))),
            cs.eq(cs_cond | cs_cond_d),
            cs_n_1ck_out.eq(Replicate(~cs, len(cs_n_1ck_out))),
        ]

        # Data IN ----------------------------------------------------------------------------------
        # Synchronize the deserializer because we deserialize over 2 cycles.
        dq_in_cnt = Signal()
        self.sync += If(dq_read_stb, dq_in_cnt.eq(~dq_in_cnt)).Else(dq_in_cnt.eq(0))

        # Deserialize read data
        # sys_clk:    ------------____________------------____________
        # sysx4_clk:  ---___---___---___---___---___---___---___---___
        # DB num:     <0><1><2><3><4><5><6><7><8><9><a><b><c><d><e><f>
        for i in range(databits):
            # BL=16 -> 2ck
            n_1ck = 2*nphases
            rbits_2ck = Signal(2*n_1ck)
            rbits_1ck = Signal(n_1ck)

            self.comb += rbits_1ck.eq(db_1ck_in[i])
            self.sync += Case(dq_in_cnt, {
                0: rbits_2ck[:n_1ck].eq(rbits_1ck),
                1: rbits_2ck[n_1ck:].eq(rbits_1ck),
            })

            bs = BitSlip(len(rbits_2ck), cycles=2,
                rst = self.dly_sel_for_bit(i) & self._rdly_dq_bitslip_rst.re,
                slp = self.dly_sel_for_bit(i) & self._rdly_dq_bitslip.re,
            )
            self.submodules += bs
            self.comb += bs.i.eq(rbits_2ck)

            for p in range(nphases):
                self.comb += [
                    dfi.phases[p].rddata[i+0*databits].eq(bs.o[p*nphases+0]),
                    dfi.phases[p].rddata[i+1*databits].eq(bs.o[p*nphases+1]),
                    dfi.phases[p].rddata[i+2*databits].eq(bs.o[p*nphases+2]),
                    dfi.phases[p].rddata[i+3*databits].eq(bs.o[p*nphases+3]),
                ]

        # Data OUT ---------------------------------------------------------------------------------
        # TODO: add 1 to tWR bacause we need 2 cycles to send data from 1 cycle

        # Before sending the actual data we have to send 2 32-bit data masks (4 DDR cycles). In the
        # mask each 0 bit means "write byte" and 1 means "mask byte". The 1st 32-bits mask the first
        # data WORD (32 bytes), and the 2nd 32-bits mask the last data WORD. Because we always send
        # 1 WORD of data (BC=0), we don't care about the 2nd mask (can send 1).
        #
        # Write data
        # DFI valid:  xxxxxxxxxxxxxxxxxx
        # sys_clk:    ------____________------------____________------------____________
        # sysx4_clk:  ---___---___---___---___---___---___---___---___---___---___---___
        # DB num:           <M><M><M><M><0><1><2><3><4><5><6><7><8><9><a><b><c><d><e><f>

        # Synchronize to 2 cycles, reset counting when dq_data_en turns high.
        db_cnt = Signal()
        self.sync += If(dq_data_en, db_cnt.eq(~db_cnt)).Else(db_cnt.eq(0))

        for i in range(databits):
            # Write data ---------------------------------------------------------------------------
            wbits = []
            for p in range(nphases):
                _dfi = dfi_hist[1] if p < nphases//2 else dfi_hist[2]
                wbits += [
                    _dfi.phases[p].wrdata[i+0*databits],
                    _dfi.phases[p].wrdata[i+1*databits],
                    _dfi.phases[p].wrdata[i+2*databits],
                    _dfi.phases[p].wrdata[i+3*databits],
                ]

            # Mux datas from 2 cycles to always serialize single cycle.
            wbits_2ck = [Cat(*wbits[:2*nphases]), Cat(*wbits[2*nphases:])]
            wbits_1ck = Signal(2*nphases)
            self.comb += wbits_1ck.eq(Array(wbits_2ck)[db_cnt])
            self.comb += db_1ck_data[i].eq(Cat(*wbits_1ck))

            # Data mask ----------------------------------------------------------------------------
            mask = [
                Constant(0),                                    # phase 0
                Constant(0),                                    # phase 0
                Constant(0),                                    # phase 1
                Constant(0),                                    # phase 1
                dfi_hist[0].phases[i//8 + 0].wrdata_mask[i%8],  # WL-2
                dfi_hist[0].phases[i//8 + 2].wrdata_mask[i%8],  # WL-2
                Constant(1),                                    # WL-1
                Constant(1),                                    # WL-1
            ]
            self.comb += db_1ck_mask[i].eq(Cat(*mask))

        # DQS --------------------------------------------------------------------------------------
        # Strobe pattern can go over 2 sysclk (do not count while transmitting mask)
        dqs_cnt = Signal()
        self.sync += If(dqs_oe & (~dq_mask_en), dqs_cnt.eq(~dqs_cnt)).Else(dqs_cnt.eq(0))

        pattern_2ck = [Signal(2*nphases), Signal(2*nphases)]
        self.comb += dqs_1ck_out.eq(Array(pattern_2ck)[dqs_cnt])

        # To avoid having to serialize dqs_oe, we serialize predefined pattern on dqs_out
        # All the patterns will be shifted back 90 degrees!
        data_pattern = bitpattern("-_-_-_-_-_-_-_-_")
        mask_pattern = bitpattern("__-_-_-_-_-_-_-_")
        phase_patterns = {
            0: bitpattern("______-_-_______"),
            1: bitpattern("________-_-_____"),
            2: bitpattern("__________-_-___"),
            3: bitpattern("____________-_-_"),
        }

        pattern_cases = \
            If(dq_mask_en,
                pattern_2ck[0].eq(mask_pattern[0]),
                pattern_2ck[1].eq(mask_pattern[1]),
            ).Elif(dq_data_en,
                pattern_2ck[0].eq(data_pattern[0]),
                pattern_2ck[1].eq(data_pattern[1]),
            ).Else(
                pattern_2ck[0].eq(0),
                pattern_2ck[1].eq(0),
            )
        any_phase_valid = 0
        for p in range(nphases):
            phase_valid = dfi_adapters[p].cmd_valid | dfi_adapters[nphases+p].cmd_valid
            pattern_cases = \
                If(phase_valid,
                    pattern_2ck[0].eq(phase_patterns[p][0]),
                    pattern_2ck[1].eq(phase_patterns[p][1]),
                ).Else(pattern_cases)
            any_phase_valid = any_phase_valid | phase_valid

        self.comb += pattern_cases
        self.comb += dqs_oe.eq(any_phase_valid | dq_mask_en | dq_data_en)

        # Read Control Path ------------------------------------------------------------------------
        # Creates a shift register of read commands coming from the DFI interface. This shift
        # register is used to indicate to the DFI interface that the read data is valid.
        self.submodules.rddata_en = rddata_en = ShiftRegister(self.settings.read_latency)
        self.comb += rddata_en.i.eq(dfi.phases[self.settings.rdphase].rddata_en)
        # -1 because of syncronious assignment
        self.sync += [phase.rddata_valid.eq(rddata_en[-1]) for phase in dfi.phases]
        # Strobe high when data from DRAM is available, before we can send it to DFI.
        self.sync += dq_read_stb.eq(rddata_en[read_db_dly-1] | rddata_en[read_db_dly-1 + 1])

        # Write Control Path -----------------------------------------------------------------------
        # Creates a shift register of write commands coming from the DFI interface. This shift
        # register is used to control DQ/DQS tristates.
        self.submodules.wrdata_en = wrdata_en = ShiftRegister(self.settings.write_latency + 2 + 1)
        self.comb += wrdata_en.i.eq(dfi.phases[self.settings.wrphase].wrdata_en & cmd_valid)
        # DQS Preamble and data mask are transmitted 1 cycle before data, then 2 cycles of data
        self.comb += dq_mask_en.eq(wrdata_en[write_latency])
        self.comb += dq_data_en.eq(wrdata_en[write_latency + 1] | wrdata_en[write_latency + 2])

        # Additional variables for LiteScope -------------------------------------------------------
        variables = ["dq_data_en", "dq_mask_en", "dq_cmd_en", "dq_read_stb", "dfi_adapters",
                     "dq_in_cnt", "db_cnt", "dqs_cnt", "rddata_en", "wrdata_en"]
        for v in variables:
            setattr(self, v, locals()[v])

    def dly_sel_for_bit(self, i):
        return self._dly_sel.storage[i // (self.databits//len(self.pads.dqs_p))]

    def do_finalize(self):
        self.do_clock_serialization(self.clk_1ck_out, self.pads.clk_p, self.pads.clk_n)
        self.do_stb_serialization(self.stb_1ck_out, self.pads.stb)
        self.do_dqs_serialization(self.dqs_1ck_out, self.dqs_1ck_in, self.dqs_oe,
                                  self.pads.dqs_p, self.pads.dqs_n)
        self.do_db_serialization(self.db_1ck_out, self.db_1ck_in, self.db_oe, self.pads.db)
        self.do_cs_serialization(self.cs_n_1ck_out, self.pads.cs_n)

    # I/O implementation ---------------------------------------------------------------------------

    def do_clock_serialization(self, clk_1ck_out, clk_p, clk_n):
        raise NotImplementedError("Serialize the full-rate clock with 90 deg phase delay")

    def do_stb_serialization(self, stb_1ck_out, stb):
        raise NotImplementedError("Serialize the STB line")

    def do_dqs_serialization(self, dqs_1ck_out, dqs_1ck_in, dqs_oe, dqs_p, dqs_n):
        raise NotImplementedError("Tristate and (de)serialize DQS with 90 deg phase delay")

    def do_db_serialization(self, db_1ck_out, db_1ck_in, db_oe, db):
        raise NotImplementedError("Tristate and (de)serialize DB")

    def do_cs_serialization(self, cs_n_1ck_out, cs_n):
        raise NotImplementedError("Serialize the chip select line (CS#)")

# Etron RPC Simulation PHY -------------------------------------------------------------------------

class SimulationPHY(BasePHY):
    def __init__(self, *args, generate_read_data=True, **kwargs):
        kwargs.update(dict(
            write_ser_latency = 1,
            read_des_latency  = 1,
            phytype           = "RPC" + self.__class__.__name__,
        ))
        super().__init__(*args, **kwargs)

        self.generate_read_data = generate_read_data

        # For simulation purpose (serializers/deserializers)
        self.sd_ddr_90  = getattr(self.sync, "sys4x_90_ddr")
        self.sd_ddr_180 = getattr(self.sync, "sys4x_180_ddr")

    def do_clock_serialization(self, clk_1ck_out, clk_p, clk_n):
        ser = Serializer(self.sync, self.sd_ddr_180, clk_1ck_out, delay=1)
        self.submodules += ser
        self.comb += clk_p.eq(ser.o)
        self.comb += clk_n.eq(~ser.o)

    def do_stb_serialization(self, stb_1ck_out, stb):
        ser = Serializer(self.sync, self.sd_ddr_90, stb_1ck_out, delay=1)
        self.submodules += ser
        self.comb += stb.eq(ser.o)

    def do_dqs_serialization(self, dqs_1ck_out, dqs_1ck_in, dqs_oe, dqs_p, dqs_n):
        # Delay dqs_oe by 1 cycle (Serializer latency) and register it on output domain
        dqs_oe_d   = Signal()
        dqs_oe_cdc = Signal()
        self.sync += dqs_oe_d.eq(dqs_oe)
        self.sync.sys4x_180 += dqs_oe_cdc.eq(dqs_oe_d)

        for i in range(len(dqs_p)):
            dqs_out = Signal()
            dqs_in  = Signal()  # TODO: use it for reading

            ser = Serializer(self.sync, self.sd_ddr_180, dqs_1ck_out, delay=1)
            self.submodules += ser
            self.comb += dqs_out.eq(ser.o)

            if i == 0:
                des = Deserializer(self.sd_ddr_180, dqs_in, dqs_1ck_in)
                self.submodules += des

            self.specials += Tristate(dqs_p[i],  dqs_out, dqs_oe_cdc, dqs_in)
            self.specials += Tristate(dqs_n[i], ~dqs_out, dqs_oe_cdc)

    def do_db_serialization(self, db_1ck_out, db_1ck_in, db_oe, db):
        if self.generate_read_data:
            # Dummy read data generator for simulation purpose
            dq_in_dummy = Signal(self.databits)
            gen = DummyReadGenerator(dq_in=db, dq_out=dq_in_dummy, stb_in=self.pads.stb,
                                     cl=self.settings.cl)
            self.submodules += ClockDomainsRenamer({"sys": "sys4x_180_ddr"})(gen)

        # Delay db_oe by 1 cycle (Serializer latency) and register it on output domain
        db_oe_d   = Signal()
        db_oe_cdc = Signal()
        self.sync += db_oe_d.eq(db_oe)
        self.sync.sys4x_90 += db_oe_cdc.eq(db_oe_d)

        for i in range(self.databits):
            # To/from tristate
            dq_out = Signal()
            dq_in = Signal()

            ser = Serializer(self.sync, self.sd_ddr_90, db_1ck_out[i], delay=1)
            self.submodules += ser
            self.comb += dq_out.eq(ser.o)

            # Use sd_ddr_90 in simulation, in general leveling would be needed.
            if self.generate_read_data:
                des = Deserializer(self.sd_ddr_90, dq_in_dummy[i], db_1ck_in[i])
            else:
                des = Deserializer(self.sd_ddr_90, dq_in, db_1ck_in[i])
            self.submodules += des

            self.specials += Tristate(db[i], dq_out, db_oe_cdc, dq_in)

    def do_cs_serialization(self, cs_n_1ck_out, cs_n):
        ser = Serializer(self.sync, self.sd_ddr_90, cs_n_1ck_out, delay=1)
        self.submodules += ser
        self.comb += cs_n.eq(ser.o)

class DummyReadGenerator(Module):
    def __init__(self, stb_in, dq_in, dq_out, cl):
        # self.sync should be a 4x DDR clock phase-aligned with CLK

        data_counter = Signal(max=16)
        cmd_counter = Signal(max=16)
        # count STB zeros, 2 full-rate cycles (4 DDR) mean STB preamble, but more mean RESET
        stb_zero_counter = Signal(max=4 + 2)
        self.sync += \
            If(stb_in == 0,
                If(stb_zero_counter != 2**len(stb_zero_counter) - 1,
                    stb_zero_counter.eq(stb_zero_counter + 1)
                )
            ).Else(
                stb_zero_counter.eq(0)
            )

        # Because the generator clock domain is phase shifted in relation to DB, we have to
        # register the value to hold it until the end of our clock, so that we get correct FSM
        # transitions
        # FIXME: would cause problems if clocks were aligned
        dq_in_r = Signal.like(dq_in)
        self.sync += dq_in_r.eq(dq_in)

        # generate DQS just for viewing signal dump
        dqs_out = Signal()
        dqs_oe  = Signal()

        data = Array([
            0x0000,  # 0x0110,
            0x1111,  # 0x1221,
            0x2222,  # 0x2332,
            0x3333,  # 0x3443,
            0x4444,  # 0x4554,
            0x5555,  # 0x5665,
            0x6666,  # 0x6776,
            0x7777,  # 0x7887,
            0x8888,  # 0x8998,
            0x9999,  # 0x9aa9,
            0xaaaa,  # 0xabba,
            0xbbbb,  # 0xbccb,
            0xcccc,  # 0xcddc,
            0xdddd,  # 0xdeed,
            0xeeee,  # 0xeffe,
            0xffff,  # 0xf00f,
        ])

        self.submodules.fsm = fsm = FSM()
        fsm.act("IDLE",
            NextValue(data_counter, 0),
            If(stb_zero_counter == 4,
                NextState("CHECK_CMD_P")
            )
        )
        fsm.act("CHECK_CMD_P",
            If(dq_in_r[:3] == 0,
                NextState("CHECK_CMD_N")
            ).Else(
                NextState("IDLE")
            )
        )
        fsm.act("CHECK_CMD_N",
            If(dq_in_r[0] == 0,
                NextState("CL_WAIT")
            ).Else(
                NextState("IDLE")
            )
        )
        # 2x for DDR, -1 for CHECK_CMD_*, -2 for preamble
        fsm.delayed_enter("CL_WAIT", "PREAMBLE_P", 2*(cl - 1) - 2)
        fsm.act("PREAMBLE_P",
            dqs_oe.eq(1),
            dqs_out.eq(0),
            NextState("PREAMBLE_N")
        )
        fsm.act("PREAMBLE_N",
            dqs_oe.eq(1),
            dqs_out.eq(0),
            NextState("SEND_DATA")
        )
        fsm.act("SEND_DATA",
            dqs_oe.eq(1),
            dqs_out.eq(data_counter[0] == 0),
            dq_out.eq(data[data_counter + cmd_counter]),
            NextValue(data_counter, data_counter + 1),
            If(data_counter == 16 - 1,
                NextValue(cmd_counter, cmd_counter + 1),
                NextState("IDLE")
            )
        )

# I/O Primitives -----------------------------------------------------------------------------------

class Serializer(Module):
    """Serialize input signals into one output with 1 sd_clk clock latency"""
    def __init__(self, sd_clk, sd_clkdiv, inputs, reset=0, name=None, delay=0):
        # `delay` must be used to get correct data_cntr=0 for phase delayed signals
        assert(len(inputs) > 0)
        assert(len(s) == len(inputs[0]) for s in inputs)

        data_width = len(inputs)
        signal_width = len(inputs[0])
        cntr_max = 2 * data_width

        if not isinstance(inputs, Array):
            inputs = Array(inputs)

        self.o = Signal(signal_width)
        data_cntr = Signal(max=cntr_max, name=name, reset=(0 - delay) % cntr_max)
        sd_clkdiv += If(reset, data_cntr.eq(0)).Else(data_cntr.eq(data_cntr+1))

        # If we used inputs combinatorically, then we will have a problem when the serialization
        # clock is not phase-aligned with the `inputs` clock, e.g.
        #  inputs clk 1:      | 0 : 1 : 2 : 3 | 4 : 5 : 6 : 7 |
        #  serialized 90 deg:   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
        # Here input 3 becomes invalid before serialized output moves to 4, so it would actually
        # send 3 for the 1st half and 7 for the 2nd half. To avoid this we introduce 1 clock
        # latency of the inputs and use 2 registers to hold the values of subsequent inputs.
        inputs_cnt = Signal()
        inputs_d   = Array([Signal.like(i) for i in [*inputs, *inputs]])
        sd_clk += [
            inputs_cnt.eq(inputs_cnt + 1),
            Case(inputs_cnt, {
                0: [i_d.eq(i) for i_d, i in zip(inputs_d[data_width:], inputs)],
                1: [i_d.eq(i) for i_d, i in zip(inputs_d[:data_width], inputs)],
            })
        ]

        self.comb += self.o.eq(inputs_d[data_cntr])

class Deserializer(Module):
    """Deserialize an input signal into outputs in the `sd` clock domain"""
    def __init__(self, sd, input, outputs, reset=0, name=None):
        assert(len(outputs) > 0)
        assert(len(s) == len(outputs[0]) for s in outputs)
        assert(len(outputs[0]) == len(input))

        data_width = len(outputs)
        signal_width = len(outputs[0])

        if not isinstance(outputs, Array):
            outputs = Array(outputs)

        data_cntr = Signal(log2_int(data_width), name=name)
        sd += If(reset, data_cntr.eq(0)).Else(data_cntr.eq(data_cntr+1))
        sd += outputs[data_cntr].eq(input)
