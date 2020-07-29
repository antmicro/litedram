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

class DFIAdapter(Module):
    # Translate DFI controls to RPC versions
    # It seems that the encoding is different when we use STB serial pin and CD data pins
    # For now we want to focus on CD encoding
    # dfi: single DFI phase
    def __init__(self, dfi):
        # data for positive and negative edge
        self.db_p = Signal(16)
        self.db_n = Signal(16)
        # serial
        self.stb  = Signal(16)

        class ModeRegister:
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
        self.mr = ModeRegister()

        # use it to send PRE on STB
        auto_precharge = Signal()
        self.comb += auto_precharge.eq(dfi.address[10])

        # TODO: what are these signals
        # byte count? burst count?
        bc = Signal(6)
        # refresh what?
        ref_op = Signal(2)
        # zqc what?
        zqc_op = Signal(2)

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
                self.db_p[5:10  +1].eq(bc),
                self.db_p[13:15 +1].eq(dfi.address[4:6 +1]),
                self.db_n[0       ].eq(0),
                self.db_n[13:15 +1].eq(dfi.address[7:9 +1]),
            ],
            "WR": [
                self.db_p[0:2   +1].eq(0b001),
                self.db_p[3:4   +1].eq(dfi.bank[:2]),
                self.db_p[5:10  +1].eq(bc),
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
                self.db_n[1:2+3].eq(ref_op),
                self.db_n[0    ].eq(0),
            ],
            "ZQC": [
                self.db_p[0:2  +1].eq(0b001),
                self.db_p[14:15+1].eq(zqc_op),
                self.db_n[0      ].eq(1),
            ],
            "MRS": [
                self.db_p[0:2  +1].eq(0b010),
                self.db_p[3:15 +1].eq(Cat(self.mr.cl, self.mr.nwr, self.mr.zout, self.mr.odt)),
                self.db_n[0      ].eq(0),
                self.db_n[12:15+1].eq(Cat(self.mr.odt_stb, self.mr.csr_fx, self.mr.odt_pd, self.mr.tm)),
            ],
        }

        # command encoding for STB serial line
        # TODO: need more info on command encoding
        serial_cmd = {
            "NOP": [
                self.stb.eq(0),
            ],
            "ACT": [
                self.stb[14:15+1].eq(0b10),
                self.stb[0:13 +1].eq(Cat(dfi.address[:12], dfi.bank[:2])),
            ],
            "RD": [
                self.stb[14:15+1].eq(0b01),  # burst, TODO: may require "Toggle RW" before
                self.stb[0:13 +1].eq(Cat(dfi.address[:12], dfi.bank[:2])),  # ?
            ],
            "WR": [
                self.stb[14:15+1].eq(0b01),  # burst, TODO: may require "Toggle RW" before
                self.stb[0:13 +1].eq(Cat(dfi.address[:12], dfi.bank[:2])),  # ?
            ],
            "PRE": [],
            "REF": [],
            "ZQC": [],
            "MRS": [],
            # there should be also:
            # - Toggle RW
            # - Burst Stop
        }

        parallel_cases = {dfi_cmd[cmd]: parallel_cmd[cmd] for cmd in dfi_cmd.keys()}
        serial_cases   = {dfi_cmd[cmd]: serial_cmd[cmd]   for cmd in dfi_cmd.keys()}
        self.comb += [
            Case(cmd_sig(dfi), parallel_cases),
            Case(cmd_sig(dfi), serial_cases),
        ]

# ECP5 Etron RPC DRAM PHY --------------------------------------------------------------------------

class ECP5RPCPHY(Module):
    def __init__(self, pads):
        # we should be able to use both DDR3 pads and RPC-specific pads
        # so we must omit:
        # - pads.a (besides 1 line used for STB - ?pads.a[0]?)
        # - pads.ba
        # - pads.ras_n
        # - pads.cas_n
        # - pads.we_n
        # - pads.dm
        # - pads.cke
        # - pads.odt
        # - pads.reset_n
        # FIXME: multiple chips?
        pads = PHYPadsCombiner(pads)

        # TODO: verify DDR3 compatibility
        stb = pads.a[0]

        phytype = self.__class__.__name__
        memtype = "RPC"

        databits = len(pads.dq)
        assert databits == 16
        addressbits = 14
        bankbits = 2
        nranks   = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        nphases = 4

        # TODO
        rdphase = 0
        wrphase = 0
        rdcmdphase = 0
        wrcmdphase = 0
        cl = 0
        cwl = 0
        read_latency = 0
        write_latency = 0

        # PHY settings -----------------------------------------------------------------------------
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
