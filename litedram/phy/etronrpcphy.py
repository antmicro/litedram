# Etron RPC DRAM PHY

from migen import *

from litedram.common import *
from litedram.phy.dfi import *
from litedram.modules import SDRAMModule, _TechnologyTimings, _SpeedgradeTimings

# RPC Commands -------------------------------------------------------------------------------------

class EM6GA16L(SDRAMModule):
    memtype = "RPC"
    # geometry
    nbanks = 4     # 64MBits per bank => 256Mb
    ncols = 1024   # most probably? and we have 10-bit column address, so no more than that
    nrows = 65536  # 64M / 1024 = 64k
    # timings TODO: real timings
    technology_timings = _TechnologyTimings(tREFI=64e6/8192, tWTR=(10, None), tCCD=(30, None), tRRD=(30, None), tZQCS=(64, 80))
    speedgrade_timings = {
        "1600": _SpeedgradeTimings(tRP=15, tRCD=15, tWR=15, tRFC=(280, None), tFAW=(None, 40), tRAS=40),
    }
    speedgrade_timings["default"] = speedgrade_timings["1600"]

# RPC Commands -------------------------------------------------------------------------------------

class ModeRegister:
    CL = {
        8:  0b000,  # default
        10: 0b001,
        11: 0b010,
        13: 0b011,
        3:  0b110,
    }
    nWR = {
        4:  0b000,
        6:  0b001,
        7:  0b010,
        8:  0b011,  # default
        10: 0b100,
        12: 0b101,
        14: 0b110,
        16: 0b111,
    }
    # resistance in Ohms
    Zout = {
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

    def __init__(self):
        # CAS latency, what is the encoding?
        self.cl      = Signal(3)
        # in LPDDR3 nWR is the number of clock cycles determining when to start internal precharge
        # for a write burst when auto-precharge is enabled (ceil(tRW/tCK) ?)
        # but in RPC we don't seem to b able to specify auto-precharge in any way, right?
        self.nwr     = Signal(3)
        self.zout    = Signal(4)
        # on-die-termination resistance configuration
        self.odt     = Signal(3)
        # as we have no ODT pin, these is probably used to enable/disable ODT
        self.odt_stb = Signal(1)
        # ??
        self.csr_fx  = Signal(1)
        # probably like odt_stb
        self.odt_pd  = Signal(1)
        self.tm      = Signal(1)

class DFIAdapter(Module):
    # Translate DFI controls to RPC versions
    # It seems that the encoding is different when we use STB serial pin and CD data pins
    # For now we want to focus on CD encoding

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

    # dfi: single DFI phase
    def __init__(self, dfi):
        # Request Packet: data for positive and negative edge
        self.db_p = Signal(16)
        self.db_n = Signal(16)
        # Serial Packet
        self.stb  = Signal(16)

        self.mr = ModeRegister()

        # use it to send PRE on STB
        auto_precharge = Signal()
        self.comb += auto_precharge.eq(dfi.address[10])

        # burst count - specifies the number of 32-byte bursts in the transfer
        self.bc = Signal(6)
        # Refresh
        self.ref_op = Signal(2)
        # ZQ Calibration
        self.zqc_op = Signal(2)
        # utility register read
        self.utr_en = Signal()
        self.utr_op = Signal(2)

        # for WR and RD, it seems to be a banks mask, so PRECHARGE ALL would be 0b1111
        bk = Signal(4)
        self.comb += [
            If(auto_precharge,
                bk.eq(0b1111)
            ).Else(
                # binary to one-hot encoded
                Case(dfi.bank[:2], {i: bk.eq(1 << i) for i in range(4)})
            )
        ]

        def cmd_sig(dfi):
            return Cat(dfi.cas_n, dfi.ras_n, dfi.we_n)

        def cmd(cas, ras, we):
            assert cas in [0, 1] and ras in [0, 1] and we in [0, 1]
            return ((1 - cas) << 0) | ((1 - ras) << 1) | ((1 - we) << 2)

        dfi_cmd = {
            "NOP": cmd(cas=0, ras=0, we=0),
            "ACT": cmd(cas=0, ras=1, we=0),
            "RD":  cmd(cas=1, ras=0, we=0),
            "WR":  cmd(cas=1, ras=0, we=1),
            "PRE": cmd(cas=0, ras=1, we=1),
            "REF": cmd(cas=1, ras=1, we=0),
            "ZQC": cmd(cas=0, ras=0, we=1),
            "MRS": cmd(cas=1, ras=1, we=1),
        }

        # command encoding for CD data lines
        parallel_cmd = {
            "NOP": [
                self.db_p.eq(0),
                self.db_n.eq(0),
            ],
            "ACT": [
                self.db_p[0:2  +1].eq(0b101),
                self.db_p[3:4  +1].eq(dfi.bank[:2]),
                self.db_n[0      ].eq(0),
                self.db_n[1:12 +1].eq(dfi.address[:12]),  # row address
            ],
            "RD": [
                self.db_p[0:2   +1].eq(0b000),
                self.db_p[3:4   +1].eq(dfi.bank[:2]),
                self.db_p[5:10  +1].eq(self.bc),
                self.db_p[13:15 +1].eq(dfi.address[4:6 +1]),
                self.db_n[0       ].eq(0),
                self.db_n[13:15 +1].eq(dfi.address[7:9 +1]),
            ],
            "WR": [
                self.db_p[0:2   +1].eq(0b001),
                self.db_p[3:4   +1].eq(dfi.bank[:2]),
                self.db_p[5:10  +1].eq(self.bc),
                self.db_p[13:15 +1].eq(dfi.address[4:6 +1]),
                self.db_n[0       ].eq(0),
                self.db_n[13:15 +1].eq(dfi.address[7:9 +1]),
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
                self.db_p[14:15+1].eq(self.zqc_op),
                self.db_n[0      ].eq(1),
            ],
            "MRS": [
                self.db_p[0:2  +1].eq(0b010),
                self.db_p[3:15 +1].eq(Cat(self.mr.cl, self.mr.nwr, self.mr.zout, self.mr.odt)),
                self.db_n[0      ].eq(0),
                self.db_n[12:15+1].eq(Cat(self.mr.odt_stb, self.mr.csr_fx, self.mr.odt_pd, self.mr.tm)),
            ],
            "UTR": [
                self.db_p[0:2  +1].eq(0b111),
                self.db_p[3      ].eq(self.utr_en),
                self.db_p[4:5  +1].eq(self.utr_op),
                self.db_n[0      ].eq(0),
            ],
        }

        # command encoding for STB serial line
        stb_op   = Signal(2)
        stb_addr = Signal(14)
        self.comb += [
            self.stb[:2].eq(stb_op),
            self.stb[2:].eq(stb_addr),
        ]
        rd_wr = 0  # TODO: which one is 0 and which is 1
        serial_cmd = {
            "NOP": [
                stb_op.eq(0b11),
            ],
            "ACT": [
                stb_op.eq(0b10),
                stb_addr.eq(Cat(dfi.bank[:2], dfi.address[:12])),
            ],
            "RD": [
                stb_op.eq(0b01),  # burst, TODO: may require "Toggle RW" before
                stb_addr.eq(Cat(dfi.bank[:2], rd_wr, dfi.address[4:9+1])),
            ],
            "WR": [
                stb_op.eq(0b01),  # burst, TODO: may require "Toggle RW" before
                stb_addr.eq(Cat(dfi.bank[:2], rd_wr, dfi.address[4:9+1])),
            ],
            "PRE": [
                stb_op.eq(0b00),  # utility
            ],
            # ZQC: not available
            # MRS: not available
            # TODO: the rest is a bit more compicated, rework STB encoding
            "REF": [],
            "TOGGLE_RW": [
                stb_op.eq(0b00),
                stb_addr[0].eq(1),
            ],
            # Burst Stop
        }

        parallel_cases = {dfi_cmd[cmd]: parallel_cmd[cmd] for cmd in dfi_cmd.keys()}
        #  serial_cases   = {dfi_cmd[cmd]: serial_cmd[cmd]   for cmd in dfi_cmd.keys()}
        self.comb += [
            Case(cmd_sig(dfi), parallel_cases),
            #  Case(cmd_sig(dfi), serial_cases),
        ]

# ECP5 Etron RPC DRAM PHY --------------------------------------------------------------------------

class ECP5RPCPHY(Module):
    def __init__(self, pads):
        # TODO: multiple chips?
        # TODO: we should be able to use both DDR3 pads and RPC-specific pads
        pads = PHYPadsCombiner(pads)

        # TODO: verify DDR3 compatibility
        if hasattr(pads, "stb"):
            stb = pads.stb
        else:
            stb = pads.a[0]

        phytype = self.__class__.__name__
        memtype = "RPC"
        tck     = 2 / (4*sys_clk_freq)  # 2 for DDR, 4 for DDR3 frequency ratio

        databits = len(pads.dq)
        assert databits == 16
        addressbits = 14
        bankbits = 2
        nranks = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        nphases = 4

        # PHY settings -----------------------------------------------------------------------------
        # RPC always has AL=1 and both read and write latencies are equal: RL = WL = AL + CL
        def get_cl_cw(tck):  # TODO: add to litedram.common
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
                    cwl = cl
                    return cl + 1, cwl + 1
            raise ValueError(tck)

        cl, cwl         = get_cl_cw(tck)
        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)

        rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
        wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)

        read_latency = cl_sys_latency
        # TODO: we must additionally transmit write mask before every burst, so +1?
        write_latency = cwl_sys_latency

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
        # minimal BL=16, which gives 16*16=256 bits of data, with 4 phases we need 16/4=4 data widths
        self.dfi = dfi = Interface(addressbits, bankbits, nranks, 4*databits, nphases)
