# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
# Etron RPC DRAM PHY

from migen import *
from migen.fhdl.specials import Tristate

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
        parallel_cases["default"] = parallel_cmd["NOP"]
        serial_cases   = {dfi_cmd[cmd]: serial_cmd["NOP"]   for cmd in dfi_cmd.keys()}
        #  serial_cases["default"] = serial_cmd["NOP"]
        self.comb += [
            Case(cmd_sig(dfi), parallel_cases),
            Case(cmd_sig(dfi), serial_cases),
        ]

# Etron RPC DRAM PHY model -------------------------------------------------------------------------

class RPCPHY(Module):
    def __init__(self, pads, sys_clk_freq):
        # TODO: multiple chips?
        # TODO: we should be able to use both DDR3 pads and RPC-specific pads
        # TODO: pads groups
        #  pads = PHYPadsCombiner(pads)

        # TODO: verify DDR3 compatibility
        if hasattr(pads, "stb"):
            stb = pads.stb
        else:
            stb = pads.a[0]

        phytype = self.__class__.__name__
        memtype = "RPC"
        tck     = 2 / (2*4*sys_clk_freq)

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

        # DFI Interface Adaptation -----------------------------------------------------------------
        self.adapters = []
        for phase in dfi.phases:
            adapter = DFIAdapter(phase)
            self.submodules += adapter
            self.adapters.append(adapter)

        # Clocks -----------------------------------------------------------------------------------
        sd_sys  = getattr(self.sync, "sys")
        sd_half = getattr(self.sync, "sys2x")
        sd_full = getattr(self.sync, "sys4x")

        sd_sys_clk  = ClockSignal("sys")
        sd_half_clk = ClockSignal("sys2x")
        sd_full_clk = ClockSignal("sys4x")

        # current phase number
        phase_sel = Signal(max=nphases)
        sd_full += phase_sel.eq(phase_sel + 1)

        # Commands ---------------------------------------------------------------------------------
        # send commands on DQ if we are not reading/writing
        dq_oe = Signal()
        dq_wr_en = Signal()
        dq_rd_en  = Signal()
        dq_cmd_en = Signal()
        self.comb += dq_cmd_en.eq(~dq_wr_en & ~dq_rd_en)
        self.comb += dq_oe.eq(dq_wr_en | dq_cmd_en),

        for i in range(databits):
            # parallel command output
            dq_cmd = Signal()
            # data output
            dq_data = Signal()
            # to tristate
            dq_o  = Signal()
            dq_i  = Signal()

            # Parallel command + address
            db = []
            for p in range(nphases):
                db.append(self.adapters[p].db_p[i])
                db.append(self.adapters[p].db_n[i])
            ser = Serializer(getattr(self.sync, "sys4x_ddr"), 0, db)
            self.submodules += ser
            self.comb += dq_cmd.eq(ser.o)

            # Data out
            data = []
            for p in range(nphases):
                data += [
                    dfi.phases[p].wrdata[i+0*databits],
                    dfi.phases[p].wrdata[i+1*databits],
                    dfi.phases[p].wrdata[i+2*databits],
                    dfi.phases[p].wrdata[i+3*databits],
                ]
            ser = Serializer(getattr(self.sync, "sys4x_ddr"), 0, data)
            self.submodules += ser
            self.comb += dq_data.eq(ser.o)

            # Mux cmd/data
            self.comb += Case(dq_wr_en, {
                1: dq_o.eq(dq_data),
                0: dq_o.eq(dq_cmd),
            })

            # Tristate
            self.specials += Tristate(pads.dq[i], dq_o, dq_oe, dq_i)

        # Write Control Path -----------------------------------------------------------------------
        # Creates a shift register of write commands coming from the DFI interface. This shift register
        # is used to control DQ/DQS tristates.
        wrdata_en = Signal(cwl_sys_latency + 2)
        wrdata_en_last = Signal.like(wrdata_en)
        self.comb += wrdata_en.eq(Cat(dfi.phases[self.settings.wrphase].wrdata_en, wrdata_en_last))
        self.sync += wrdata_en_last.eq(wrdata_en)
        self.comb += dq_wr_en.eq(wrdata_en[cwl_sys_latency])
        #  self.comb += If(self._wlevel_en.storage, dqs_oe.eq(1)).Else(dqs_oe.eq(dq_oe))


# I/O Primitives -----------------------------------------------------------------------------------

class DDRClockGen(Module):
    """Generate sync.ddr_pos and sync.ddr_neg for simulation purpose"""
    def __init__(self, clk):
        self.clock_domains.cd_ddr_pos = ClockDomain("ddr_pos", reset_less=True)
        self.clock_domains.cd_ddr_neg = ClockDomain("ddr_neg", reset_less=True)
        self.comb += [
            self.cd_ddr_pos.clk.eq(clk),
            self.cd_ddr_neg.clk.eq(~clk),
        ]

class DDROut(Module):
    """Output on both edges of the clock (1 cycle latency)"""
    def __init__(self, i1, i2, o):
        # delay the second input to maintain correct order
        i2_r = Signal()
        self.sync.ddr_pos += i2_r.eq(i2)
        # drive the output pin
        self.sync.ddr_pos += o.eq(i1)
        self.sync.ddr_neg += o.eq(i2_r)

class DDRIn(Module):
    """Read on both edges of the clock (1 cycle latency)"""
    # TODO: test if it's correct
    def __init__(self, i, o1, o2):
        i1 = Signal()
        i2 = Signal()
        # register the inputs
        self.sync.ddr_pos += i1.eq(i)
        self.sync.ddr_neg += i2.eq(i)
        # drive outputs on positive edge
        self.sync.ddr_pos += [
            o1.eq(i1),
            o2.eq(i2),
        ]

class Serializer(Module):
    """Serialize input signals into one output in the `sd` clock domain, reset with `strb`"""
    def __init__(self, sd, strb, inputs):
        assert(len(inputs) > 0)
        assert(len(s) == len(inputs[0]) for s in inputs)

        data_width = len(inputs)
        signal_width = len(inputs[0])

        if not isinstance(inputs, Array):
            inputs = Array(inputs)

        self.o = Signal(signal_width)
        data_cntr = Signal(log2_int(data_width), reset=strb)
        sd += data_cntr.eq(data_cntr+1)
        self.comb += self.o.eq(inputs[data_cntr])

class Deserializer(Module):
    """Deserialize an input signal into outputs in the `sd` clock domain, reset by `strb`"""
    def __init__(self, sd, strb, input, outputs):
        assert(len(outputs) > 0)
        assert(len(s) == len(outputs[0]) for s in outputs)
        assert(len(outputs[0]) == len(input))

        data_width = len(outputs)
        signal_width = len(outputs[0])

        if not isinstance(outputs, Array):
            outputs = Array(outputs)

        data_cntr = Signal(log2_int(data_width), reset=strb)
        sd += data_cntr.eq(data_cntr+1)
        sd += outputs[data_cntr].eq(input)
