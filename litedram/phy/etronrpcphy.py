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
        read_latency = cl_sys_latency + 1 + 2 + 3
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
        self.dfi = dfi = Interface(addressbits, bankbits, nranks, 4*databits, nphases)

        # register DFI commands to have a 2 cycles window of 8 phases to choose from
        dfi_r = Interface(addressbits, bankbits, nranks, 4*databits, nphases)
        self.sync += dfi.connect(dfi_r)
        dfi_r2 = Interface(addressbits, bankbits, nranks, 4*databits, nphases)
        self.sync += dfi_r.connect(dfi_r2)
        dfi_phases_x3 = dfi_r2.phases + dfi_r.phases + dfi.phases

        # DFI Interface Adaptation -----------------------------------------------------------------
        # hold the commands for 2 more cycles so that we can insert STB, DQS and DM before the command
        self.adapters = []
        for phase in dfi_phases_x3:
            adapter = DFIAdapter(phase)
            self.submodules += adapter
            self.adapters.append(adapter)
            # always send one WORD
            self.comb += adapter.bc.eq(1)

        # Clocks -----------------------------------------------------------------------------------
        sd_sys  = getattr(self.sync, "sys")
        sd_half = getattr(self.sync, "sys2x")
        sd_full = getattr(self.sync, "sys4x")

        sd_sys_clk  = ClockSignal("sys")
        sd_half_clk = ClockSignal("sys2x")
        sd_full_clk = ClockSignal("sys4x")

        # Logic ------------------------------------------------------------------------------------

        # send commands on DQ if we are not reading/writing
        dq_oe = Signal()
        dq_wr_en = Signal()
        dq_dm_en = Signal()
        dq_rd_en  = Signal()
        dq_cmd_en = Signal()
        self.comb += dq_cmd_en.eq(reduce(or_, [adapter.db_valid for adapter in self.adapters]))
        self.comb += dq_oe.eq(dq_wr_en | (dq_dm_en & ~sd_sys_clk) | dq_cmd_en),

        stb_preamble_en = Signal()
        stb_preamble = []
        for p in range(nphases):
            # before sending parallel command on DB pins we need to send 2 full-rate clk cycles
            # of STB low, so we delay the DB signals by 2
            p += nphases
            en = self.adapters[p + 2].db_valid | self.adapters[p + 1].db_valid
            stb_preamble += [en, en]
        ser = Serializer(getattr(self.sync, "sys4x_ddr"), stb_preamble)
        self.submodules += ser
        self.comb += stb_preamble_en.eq(ser.o)

        self.comb += If(stb_preamble_en, stb.eq(0)).Else(stb.eq(1))  # 0b11 means NOP

        dq_i_dummy = Signal(16)
        rddata_available = Signal()

        for i in range(databits):
            # parallel command output
            dq_cmd = Signal()
            # data output
            dq_data = Signal()
            dq_mask = Signal()
            # to tristate
            dq_o  = Signal()
            dq_i  = Signal()

            # Parallel command
            # CLK: ____----____----____----____----____----____
            # STB: ----------________________------------------
            # DQS: ....................----____----____........
            # DB:  ..........................PPPPnnnn..........
            # TODO: it must be center-aligned to the full-rate clk
            db = []
            for p in range(nphases):
                p += nphases
                db_p = self.adapters[p].db_p[i]
                db_n = self.adapters[p].db_n[i]
                db += [db_p, db_n]
            ser = Serializer(getattr(self.sync, "sys4x_ddr"), db)
            self.submodules += ser
            self.comb += dq_cmd.eq(ser.o)

            # Data out
            # TODO: add 1 to tWR bacause we need 2 cycles to send data from 1 cycle
            wrdata = []
            for p in range(nphases):
                # In first sys_clk cycle we send from current phases, in the second
                # we use the data registered in the previous cycle.
                p += nphases
                if p < nphases//2:
                    p += nphases
                wrdata += [
                    dfi_phases_x3[p].wrdata[i+0*databits],
                    dfi_phases_x3[p].wrdata[i+1*databits],
                    dfi_phases_x3[p].wrdata[i+2*databits],
                    dfi_phases_x3[p].wrdata[i+3*databits],
                ]
            # Start counting when dq_wr_en turns high (required as we serialize over 2 sys_clk cycles).
            ser = Serializer(getattr(self.sync, "sys4x_ddr"), wrdata, reset=~dq_wr_en)
            self.submodules += ser
            self.comb += dq_data.eq(ser.o)

            # Data mask
            # Two 32-bit masks (4 DDR cycles) are sent before write data.
            # In the mask each 0 bit means "write" and 1 means "mask".
            # The 1st 32-bits mask the first data WORD (32 bytes), and
            # the 2nd 32-bits mask the last data WORD. Because we always
            # send 1 WORD of data (BC=1), we don't care for the 2nd mask.
            # TODO: increase tWR by 1
            mask = [
                dfi_phases_x3[2*nphases + i//8 + 0].wrdata_mask[i%8], # WL-2
                dfi_phases_x3[2*nphases + i//8 + 2].wrdata_mask[i%8], # WL-2
                Constant(1), # WL-1,
                Constant(1), # WL-1,
            ]
            ser = Serializer(getattr(self.sync, "sys4x_ddr"), mask)
            self.submodules += ser
            self.comb += dq_mask.eq(ser.o)

            # Data in
            # TODO: synchronize deserializer to rd start
            rddata = Signal(16)
            des = Deserializer(getattr(self.sync, "sys4x_ddr"), dq_i_dummy[i], rddata,
                               reset=~rddata_available)
            self.submodules += des

            bitslip = BitSlip(16, cycles=2)
            self.submodules += bitslip
            self.comb += bitslip.i.eq(rddata)
            self.comb += bitslip.i.eq(rddata)
            for p in range(nphases):
                domain = self.sync if p < nphases//2 else self.comb
                domain += [
                    dfi_phases_x3[nphases+p].rddata[i+0*databits].eq(bitslip.o[p*nphases+0]),
                    dfi_phases_x3[nphases+p].rddata[i+1*databits].eq(bitslip.o[p*nphases+1]),
                    dfi_phases_x3[nphases+p].rddata[i+2*databits].eq(bitslip.o[p*nphases+2]),
                    dfi_phases_x3[nphases+p].rddata[i+3*databits].eq(bitslip.o[p*nphases+3]),
                ]

            # Mux cmd/data/data_mask
            self.comb += \
                If(dq_wr_en,
                    dq_o.eq(dq_data)
                ).Elif(dq_dm_en,
                    dq_o.eq(dq_mask)
                ).Else(
                    dq_o.eq(dq_cmd)
                )

            # Tristate
            self.specials += Tristate(pads.dq[i], dq_o, dq_oe, dq_i)

        # Data strobe
        dqs_oe = Signal()
        dqs_i  = Signal()  # width 1
        dqs_o  = Signal()  # width 1
        dqs_pattern = Signal(8)
        dqs_wr_preamble = Signal()

        self.comb += \
            If(dqs_wr_preamble,  # (transmitted LSB first)
                dqs_pattern.eq(0b01010100)
            ).Else(
                dqs_pattern.eq(0b01010101)
            )

        dqs_cmd_oes = []
        dqs_cmd_oe = Signal()

        for p in range(nphases):
            # strobe starts 1 cycle before command
            p += nphases
            oe = self.adapters[p + 1].db_valid | self.adapters[p].db_valid
            dqs_cmd_oes += [oe, oe]

        ser = Serializer(getattr(self.sync, "sys4x_ddr"), dqs_pattern)
        self.submodules += ser
        self.comb += dqs_o.eq(ser.o)

        ser = Serializer(getattr(self.sync, "sys4x_ddr"), dqs_cmd_oes)
        self.submodules += ser
        self.comb += dqs_cmd_oe.eq(ser.o)

        self.comb += dqs_oe.eq(dqs_cmd_oe | dq_wr_en | dqs_wr_preamble)

        # Tristate
        self.specials += Tristate(pads.dqs_p[0], dqs_o, dqs_oe, dqs_i)
        self.specials += Tristate(pads.dqs_n[0], ~dqs_o, dqs_oe)

        # Read Control Path ------------------------------------------------------------------------
        # Creates a shift register of read commands coming from the DFI interface. This shift register
        # is used to indicate to the DFI interface that the read data is valid.
        #
        # The read data valid is asserted for 1 sys_clk cycle when the data is available on the DFI
        # interface, the latency is the sum of the OSERDESE2, CAS, ISERDESE2 and Bitslip latencies.
        rddata_en      = Signal(self.settings.read_latency)
        rddata_en_last = Signal.like(rddata_en)
        self.comb += rddata_en.eq(Cat(dfi.phases[self.settings.rdphase].rddata_en, rddata_en_last))
        self.sync += rddata_en_last.eq(rddata_en)
        self.sync += rddata_available.eq(rddata_en[-5] | rddata_en[-6])
        self.sync += [phase.rddata_valid.eq(rddata_en[-1]) for phase in dfi.phases]

        # Write Control Path -----------------------------------------------------------------------
        # Creates a shift register of write commands coming from the DFI interface. This shift register
        # is used to control DQ/DQS tristates.
        wrdata_en = Signal(write_latency + 2 + 1)
        wrdata_en_last = Signal.like(wrdata_en)
        self.comb += wrdata_en.eq(Cat(dfi.phases[self.settings.wrphase].wrdata_en, wrdata_en_last))
        self.sync += wrdata_en_last.eq(wrdata_en)
        self.comb += dq_wr_en.eq(wrdata_en[write_latency + 1] | wrdata_en[write_latency + 2])
        self.comb += dq_dm_en.eq(wrdata_en[write_latency])
        #  self.comb += If(self._wlevel_en.storage, dqs_oe.eq(1)).Else(dqs_oe.eq(dq_oe))

        #  # Write DQS Postamble/Preamble Control Path ------------------------------------------------
        #  # Generates DQS Preamble 1 cycle before the first write and Postamble 1 cycle after the last
        #  # write. During writes, DQS tristate is configured as output for at least 3 sys_clk cycles:
        #  # 1 for Preamble, 1 for the Write and 1 for the Postamble.
        self.comb += dqs_wr_preamble.eq(wrdata_en[write_latency] & ~wrdata_en[write_latency + 1])
        #  self.comb += dqs_pattern.postamble.eq(wrdata_en[write_latency + 1] & ~wrdata_en[write_latency])

        # # #
        dummy_read = DummyReadGenerator(pads.dq, dq_i_dummy, stb_pad, cl=cl)
        self.submodules += ClockDomainsRenamer({"sys": "sys4x_ddr"})(dummy_read)

class DummyReadGenerator(Module):
    def __init__(self, dq_in, dq_out, stb, cl):
        # self.sync should be sys4x_ddr
        # sys4x:     ----____----____
        # sys4x_ddr: --__--__--__--__
        # pos:       1 0 1 0 1 0 1 0
        pos = Signal(reset=1)
        self.sync += pos.eq(~pos)

        stb_zero_counter = Signal(max=4)
        data_counter = Signal(max=16)

        data = Array([
            0x0000, # 0x0110,
            0x1111, # 0x1221,
            0x2222, # 0x2332,
            0x3333, # 0x3443,
            0x4444, # 0x4554,
            0x5555, # 0x5665,
            0x6666, # 0x6776,
            0x7777, # 0x7887,
            0x8888, # 0x8998,
            0x9999, # 0x9aa9,
            0xaaaa, # 0xabba,
            0xbbbb, # 0xbccb,
            0xcccc, # 0xcddc,
            0xdddd, # 0xdeed,
            0xeeee, # 0xeffe,
            0xffff, # 0xf00f,
        ])

        self.submodules.fsm = fsm = FSM()
        fsm.act("IDLE",
            If(stb == 0,
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
            If(dq_in[:2] == 0,
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
            dq_out.eq(data[data_counter]),
            If(data_counter == 16 - 1,
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
