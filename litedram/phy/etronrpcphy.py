# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
# Etron RPC DRAM PHY

from migen import *
from migen.fhdl.specials import Tristate

from litedram.common import *
from litedram.phy.dfi import *
from litedram.modules import SDRAMModule, _TechnologyTimings, _SpeedgradeTimings

# Etron RPC Module ---------------------------------------------------------------------------------

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

        # 1 when not in NOP
        self.db_valid = Signal(reset=1)

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
                self.db_valid.eq(0),
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

        # TODO: STB is currently not used, this has to be rewritten
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
    def __init__(self, pads, sys_clk_freq, bitslip=False):
        # TODO: pads groups for multiple chips
        #  pads = PHYPadsCombiner(pads)

        # TODO: verify DDR3 compatibility
        # RPC DRAM is compatible with DDR3 pads, so if stb is not present, use address[0].
        stb = Signal()
        stb_pad = pads.stb if hasattr(pads, "stb") else pads.a[0]
        self.comb += stb_pad.eq(stb)

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
        def get_cl(tck):
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

        # RPC always has AL=1 and both read and write latencies are equal: RL = WL = AL + CL
        al = 1
        cwl = cl = get_cl(tck) + al

        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)

        rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
        wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)

        # +1 for both for sending STB before parallel cmd
        # +2 to assemble the data and put on DFI
        # +3 for bitslip
        read_latency = cl_sys_latency + 1 + 2 + (3 if bitslip else 0)
        # This write latency is only for LiteDRAMController.
        # Actually we must send data mask 1 cycle before sending data, and we must send
        # a full burst of BL=16, which takes 2 cycles, so we spend 3 cycles on writing,
        # instead of 1 cycle as for other PHYs. This means that we have to include these
        # 2 additional cycles while setting tWR for the module.
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
        # minimal BL=16, which gives 16*16=256 bits; with 4 phases we need 16/4=4 data widths
        dfi_params = dict(addressbits=addressbits, bankbits=bankbits, nranks=nranks,
                          databits=4*databits, nphases=nphases)

        # Register DFI history, as we need to operate on 3 subsequent cycles (write data,
        # only 2 needed for commands)
        # hist[0] = dfi[N], hist[1] = dfi[N-1], ...
        self.dfi = dfi = Interface(**dfi_params)
        dfi_hist = [dfi, Interface(**dfi_params), Interface(**dfi_params)]
        self.sync += dfi_hist[0].connect(dfi_hist[1])
        self.sync += dfi_hist[1].connect(dfi_hist[2])

        # Clocks -----------------------------------------------------------------------------------
        sd_sys  = getattr(self.sync, "sys")
        sd_half = getattr(self.sync, "sys2x")
        sd_full = getattr(self.sync, "sys4x")

        # For simulation purpose (serializers/deserializers)
        sd_ddr = getattr(self.sync, "sys4x_ddr")

        sd_sys_clk  = ClockSignal("sys")
        sd_half_clk = ClockSignal("sys2x")
        sd_full_clk = ClockSignal("sys4x")

        # Parallel commands ------------------------------------------------------------------------
        # We need to insert 2 full-clk cycles of STB=0 before any command, to mark the beginning of
        # Request Packet. For that reason we use the previous values of DFI commands.
        # list: dfi[N-1][p0], dfi[N-1][p1], ..., dfi[N][p0], dfi[N][p1], ...
        dfi_adapters = []
        for phase in dfi_hist[1].phases + dfi_hist[0].phases:
            adapter = DFIAdapter(phase)
            self.submodules += adapter
            dfi_adapters.append(adapter)
            # We always send one WORD, which consists of 32 bytes.
            self.comb += adapter.bc.eq(1)

        # Serialize commands to DB pins:
        # CLK: ____----____----____----____----____----____
        # STB: ----------________________------------------
        # DQS: ....................----____----____........
        # DB:  ..........................PPPPnnnn..........
        # TODO: it must be center-aligned to the full-rate clk
        dq_cmd = Signal(databits)
        for i in range(databits):
            # Serialize a list of differential DB values using previous DFI coomand:
            # db_p[p][i], db_n[p][i], db_p[p+1][i], db_n[p+1][i], ...
            bits = [db for a in dfi_adapters[:nphases] for db in [a.db_p[i], a.db_n[i]]]
            ser = Serializer(sd_ddr, bits)
            self.submodules += ser
            self.comb += dq_cmd[i].eq(ser.o)

        # STB --------------------------------------------------------------------------------------
        # Currently not sending any serial commands, but the STB pin must be held low for 2 full
        # rate cycles before writing a parallel command to activate the DRAM.
        stb_preamble_en = Signal()

        stb_preamble = []
        for p in range(nphases):
            # Use cmd from current and prev cycle, depending on which phase the command appears on.
            en = dfi_adapters[p + 2].db_valid | dfi_adapters[p + 1].db_valid
            stb_preamble += [en, en]
        ser = Serializer(sd_ddr, stb_preamble)
        self.submodules += ser
        self.comb += stb_preamble_en.eq(ser.o)

        # We only want to use STB to start parallel commands or to send NOPs. NOP is indicated by
        # the first two bits being high (0b11, and other as "don't care") so we can simply hold STB
        # high all the time.
        self.comb += stb.eq(~stb_preamble_en)

        # Data IN ----------------------------------------------------------------------------------
        # Dummy read data generator for simulation purpose
        dq_i_dummy = Signal(databits)
        gen = DummyReadGenerator(dq_in=pads.dq, dq_out=dq_i_dummy, stb_in=stb_pad, cl=cl)
        self.submodules += ClockDomainsRenamer({"sys": "sys4x_ddr"})(gen)

        # Synchronize the deserializer as we deserialize more than 1 sysclk
        rd_strb = Signal()

        # Deserialize read data (TODO: add phase shift)
        # sys_clk:    ------------____________------------____________
        # sysx4_clk:  ---___---___---___---___---___---___---___---___
        # DB num:     <0><1><2><3><4><5><6><7><8><9><a><b><c><d><e><f>
        for i in range(databits):
            # BL=16
            rbits = Signal(16)
            des = Deserializer(sd_ddr, dq_i_dummy[i], rbits, reset=~rd_strb)
            self.submodules += des

            if bitslip:
                bs = BitSlip(16, cycles=2)
                self.submodules += bs
                self.comb += bs.i.eq(rbits)
                rbits = bs.o

            for p in range(nphases):
                # Register the values from 1st cycle.
                domain = self.sync if p < nphases//2 else self.comb
                domain += [  # TODO: change to dfi_hist[0]
                    dfi.phases[p].rddata[i+0*databits].eq(rbits[p*nphases+0]),
                    dfi.phases[p].rddata[i+1*databits].eq(rbits[p*nphases+1]),
                    dfi.phases[p].rddata[i+2*databits].eq(rbits[p*nphases+2]),
                    dfi.phases[p].rddata[i+3*databits].eq(rbits[p*nphases+3]),
                ]

        # Data OUT ---------------------------------------------------------------------------------
        # TODO: add 1 to tWR bacause we need 2 cycles to send data from 1 cycle

        # Before sending the actual data we have to send 2 32-bit data masks (4 DDR cycles). In the
        # mask each 0 bit means "write byte" and 1 means "mask byte". The 1st 32-bits mask the first
        # data WORD (32 bytes), and the 2nd 32-bits mask the last data WORD. Because we always send
        # 1 WORD of data (BC=1), we don't care about the 2nd mask (can send 1).
        #
        # Write data (TODO: add phase shift):
        # DFI valid:  xxxxxxxxxxxxxxxxxx
        # sys_clk:    ------____________------------____________------------____________
        # sysx4_clk:  ---___---___---___---___---___---___---___---___---___---___---___
        # DB num:           <M><M><M><M><0><1><2><3><4><5><6><7><8><9><a><b><c><d><e><f>
        dq_data = Signal(databits)
        dq_mask = Signal(databits)

        dq_data_stb = Signal()

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
            # Reset counting when dq_data_en turns high as we serialize over 2 sysclk cycles.
            ser = Serializer(sd_ddr, wbits, reset=~dq_data_stb)
            self.submodules += ser
            self.comb += dq_data[i].eq(ser.o)

            # Data mask ----------------------------------------------------------------------------
            mask = [
                dfi_hist[0].phases[i//8 + 0].wrdata_mask[i%8],  # WL-2
                dfi_hist[0].phases[i//8 + 2].wrdata_mask[i%8],  # WL-2
                Constant(1),                                    # WL-1
                Constant(1),                                    # WL-1
            ]
            ser = Serializer(sd_ddr, mask)
            self.submodules += ser
            self.comb += dq_mask[i].eq(ser.o)

        # DB tristate and muxing -------------------------------------------------------------------
        dq_out = Signal(databits)
        dq_in  = Signal(databits)
        dq_oe  = Signal(databits)

        dq_data_en = Signal()
        dq_mask_en = Signal()
        dq_cmd_en  = Signal()

        # Tristate
        for i in range(databits):
            self.specials += Tristate(pads.dq[i], dq_out[i], dq_oe, dq_in[i])

        # Mux cmd/data/data_mask
        self.comb += \
            If(dq_data_en,
                dq_out.eq(dq_data)
            ).Elif(dq_mask_en,
                dq_out.eq(dq_mask)
            ).Else(
                dq_out.eq(dq_cmd)
            )

        # Output enable when writing cmd/data/mask
        self.comb += [
            dq_data_stb.eq(dq_data_en),
            # Commands go on the 2nd cycle, so use previous DFI
            dq_cmd_en.eq(reduce(or_, [a.db_valid for a in dfi_adapters[:nphases]])),
            # Mask is being send during negative half of sysclk
            dq_oe.eq(dq_data_en | (dq_mask_en & ~sd_sys_clk) | dq_cmd_en),
        ]

        # DQS --------------------------------------------------------------------------------------
        dqs_oe      = Signal()
        dqs_in      = Signal()
        dqs_out     = Signal()
        dqs_cmd_oe  = Signal()
        dqs_wpre_en = Signal()

        def pattern(s):
            assert len(s) == 8
            s = s.translate(s.maketrans("_-", "01"))
            return int(s[::-1], 2)  # LSB first, so reverse the string

        # Serialize strobe pattern
        dqs_pattern = Signal(8)
        self.comb += Case(dqs_wpre_en, {
            1: dqs_pattern.eq(pattern("__-_-_-_")),
            0: dqs_pattern.eq(pattern("-_-_-_-_")),
        })
        ser = Serializer(getattr(self.sync, "sys4x_ddr"), dqs_pattern)
        self.submodules += ser
        self.comb += dqs_out.eq(ser.o)

        # Serialize output-enable for commands (as commands can go on any phase)
        # TODO: we know wrphase, so we can generate fixed patter!
        dqs_cmd_oe_bits = []
        for p in range(nphases):
            # Strobe starts 1 cycle before the command
            oe = dfi_adapters[p+1].db_valid | dfi_adapters[p].db_valid
            dqs_cmd_oe_bits += [oe, oe]
        ser = Serializer(getattr(self.sync, "sys4x_ddr"), dqs_cmd_oe_bits)
        self.submodules += ser
        self.comb += dqs_cmd_oe.eq(ser.o)

        # Send strobe before cmd/write and during write
        self.comb += dqs_oe.eq(dqs_cmd_oe | dq_data_en | dqs_wpre_en)

        # Tristate
        # RPC DRAM uses only 1 differential line for strobe
        self.specials += Tristate(pads.dqs_p[0],  dqs_out, dqs_oe, dqs_in)  # TODO: use input strobe
        self.specials += Tristate(pads.dqs_n[0], ~dqs_out, dqs_oe)

        # Read Control Path ------------------------------------------------------------------------
        # Creates a shift register of read commands coming from the DFI interface. This shift
        # register is used to indicate to the DFI interface that the read data is valid.
        rddata_en      = Signal(self.settings.read_latency)
        rddata_en_last = Signal.like(rddata_en)
        self.comb += rddata_en.eq(Cat(dfi.phases[self.settings.rdphase].rddata_en, rddata_en_last))
        self.sync += rddata_en_last.eq(rddata_en)
        self.sync += [phase.rddata_valid.eq(rddata_en[-1]) for phase in dfi.phases]
        # Strobe high when data from DRAM is available, before we can send it to DFI.
        if bitslip:
            self.sync += rd_strb.eq(rddata_en[-5] | rddata_en[-6])
        else:
            self.sync += rd_strb.eq(rddata_en[-2] | rddata_en[-3])

        # Write Control Path -----------------------------------------------------------------------
        # Creates a shift register of write commands coming from the DFI interface. This shift
        # register is used to control DQ/DQS tristates.
        wrdata_en = Signal(write_latency + 2 + 1)
        wrdata_en_last = Signal.like(wrdata_en)
        self.comb += wrdata_en.eq(Cat(dfi.phases[self.settings.wrphase].wrdata_en, wrdata_en_last))
        self.sync += wrdata_en_last.eq(wrdata_en)
        self.comb += dq_data_en.eq(wrdata_en[write_latency + 1] | wrdata_en[write_latency + 2])
        # DQS Preamble and data mask are transmitted 1 cycle before data
        self.comb += dqs_wpre_en.eq(wrdata_en[write_latency])
        self.comb += dq_mask_en.eq(wrdata_en[write_latency])

class DummyReadGenerator(Module):
    def __init__(self, dq_in, dq_out, stb_in, cl):
        # self.sync should be sys4x_ddr
        # sys4x:     ----____----____
        # sys4x_ddr: --__--__--__--__
        # pos:       1 0 1 0 1 0 1 0
        pos = Signal(reset=1)
        self.sync += pos.eq(~pos)

        stb_zero_counter = Signal(max=4)
        data_counter = Signal(max=16)
        cmd_counter = Signal(max=16)

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
            If(stb_in == 0,
                NextValue(stb_zero_counter, stb_zero_counter + 1),
                If(stb_zero_counter == 4 - 1,
                    If(pos,
                        Display("ERROR: end of STB preable on positive edge!")
                    ),
                    NextState("CHECK_CMD_P")
                )
            ).Else(
                NextValue(stb_zero_counter, 0)
            )
        )
        fsm.act("CHECK_CMD_P",
            If(dq_in[:3] == 0,
                NextState("CHECK_CMD_N")
            ).Else(
                NextState("IDLE")
            )
        )
        fsm.act("CHECK_CMD_N",
            If(dq_in[0] == 0,
                NextState("CL_WAIT")
            ).Else(
                NextState("IDLE")
            )
        )
        fsm.delayed_enter("CL_WAIT", "SEND_DATA", 2*(cl - 1))  # 2x for DDR, -1 for CHECK_CMD_*
        fsm.act("SEND_DATA",
            NextValue(data_counter, data_counter + 1),
            dq_out.eq(data[data_counter + cmd_counter]),
            If(data_counter == 16 - 1,
                NextValue(cmd_counter, cmd_counter + 1),
                NextState("IDLE")
            )
        )

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
    """Serialize input signals into one output in the `sd` clock domain"""
    def __init__(self, sd, inputs, reset=0):
        assert(len(inputs) > 0)
        assert(len(s) == len(inputs[0]) for s in inputs)

        data_width = len(inputs)
        signal_width = len(inputs[0])

        if not isinstance(inputs, Array):
            inputs = Array(inputs)

        self.o = Signal(signal_width)
        data_cntr = Signal(log2_int(data_width))
        sd += If(reset, data_cntr.eq(0)).Else(data_cntr.eq(data_cntr+1))
        self.comb += self.o.eq(inputs[data_cntr])

class Deserializer(Module):
    """Deserialize an input signal into outputs in the `sd` clock domain"""
    def __init__(self, sd, input, outputs, reset=0):
        assert(len(outputs) > 0)
        assert(len(s) == len(outputs[0]) for s in outputs)
        assert(len(outputs[0]) == len(input))

        data_width = len(outputs)
        signal_width = len(outputs[0])

        if not isinstance(outputs, Array):
            outputs = Array(outputs)

        data_cntr = Signal(log2_int(data_width))
        sd += If(reset, data_cntr.eq(0)).Else(data_cntr.eq(data_cntr+1))
        sd += outputs[data_cntr].eq(input)
