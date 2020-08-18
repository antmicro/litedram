# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
# Etron RPC DRAM PHY

from migen import *
from migen.fhdl.specials import Tristate

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *
from litedram.modules import SDRAMModule, _TechnologyTimings, _SpeedgradeTimings

def bitpattern(s):
    assert len(s) == 8
    s = s.translate(s.maketrans("_-", "01"))
    return int(s[::-1], 2)  # LSB first, so reverse the string

# Etron RPC Module ---------------------------------------------------------------------------------

class EM6GA16L(SDRAMModule):
    memtype = "RPC"
    # geometry
    nbanks = 4     # 64MBits per bank => 256Mb
    ncols = 1024   # most probably? and we have 10-bit column address, so no more than that
    nrows = 65536  # 64M / 1024 = 64k
    # timings TODO: real timings
    technology_timings = _TechnologyTimings(
        # Refresh: needs to refresh every row once per 64ms
        # Is tREFI in RPC the same as in normal DRAM?
        #   tREFI = 100ns (FST refresh) or 3.2us (LP refresh)
        tREFI=64e6/8192,
        # It seems that we need to calculate tCCD from tBESL by assuming BC=1, then it is:
        # RL=WL + burst_time + tBESL
        tCCD=(13+1 + 8 + 7, None),
        # CCD enforces tBESL for read commands, we use tWTR to ensure it for writes
        # 11 for tBESL + 2 for STB low before a command
        tWTR=(11 + 2, None),
        tRRD=(None, 7.5),
        tZQCS=(None, 90)
    )
    speedgrade_timings = {
        # FIXME: we're increasing tWR by 1 sysclk to compensate for long write
        # Should we use tRFQSd for tRFC?
        "1600": _SpeedgradeTimings(tRP=13.75, tRCD=13.75, tWR=15 + (1/100e6 * 2), tRFC=(3*100, None), tFAW=None, tRAS=35),
        "1866": _SpeedgradeTimings(tRP=13.91, tRCD=13.91, tWR=15 + (1/100e6 * 2), tRFC=(3*100, None), tFAW=None, tRAS=34),
    }
    speedgrade_timings["default"] = speedgrade_timings["1600"]

# RPC Commands -------------------------------------------------------------------------------------

class ModeRegister:
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

class DFIAdapter(Module):
    # Translate DFI controls to RPC versions
    # It seems that the encoding is different when we use STB serial pin and CD data pins
    # For now we want to focus on CD encoding
    # TODO: STB is currently not used, this has to be rewritten

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

        # force sending RESET command
        self.do_reset = Signal()

        self.mr = ModeRegister()

        # use it to send PRE on STB
        auto_precharge = Signal()
        self.comb += auto_precharge.eq(dfi.address[10])

        # burst count - specifies the number of 32-byte bursts in the transfer
        self.bc = Signal(6)
        # Refresh
        self.ref_op = Signal(2)
        # utility register read
        self.utr_en = Signal()
        self.utr_op = Signal(2)
        # ZQ Calibration
        zqc_op = Signal(2)

        # for WR and RD, it seems to be a banks mask, so PRECHARGE ALL would be 0b1111
        bk = Signal(4)
        self.comb += [
            If(auto_precharge,
                bk.eq(0b1111),
                # LiteDRAM will try to perform "long" calibration during init and then
                # do only "short" ones, so we have to send "init" instead of "long" for RPC
                zqc_op.eq(self.ZQC_OP["init"]),
            ).Else(
                # binary to one-hot encoded
                Case(dfi.bank[:2], {i: bk.eq(1 << i) for i in range(4)}),
                zqc_op.eq(self.ZQC_OP["short"]),
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
                self.db_p[14:15+1].eq(zqc_op),
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
            "RESET": [
                self.db_p.eq(0),
                self.db_n.eq(1),
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
            If(self.do_reset,
                parallel_cmd["RESET"]
            ).Else(
                Case(cmd_sig(dfi), parallel_cases),
            ),
            Case(cmd_sig(dfi), serial_cases),
        ]

# Etron RPC DRAM PHY Base --------------------------------------------------------------------------

class BasePHY(Module, AutoCSR):
    def __init__(self, pads, sys_clk_freq, write_ser_latency, read_des_latency, phytype):
        self.memtype     = memtype = "RPC"
        self.tck         = tck     = 2 / (2*4*sys_clk_freq)
        self.databits    = databits = len(pads.dq)
        assert databits == 16
        self.addressbits = addressbits = 14
        self.bankbits    = bankbits = 2
        self.nranks      = nranks = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        self.nphases     = nphases = 4

        # TODO: pads groups for multiple chips
        #  pads = PHYPadsCombiner(pads)
        self.pads = pads

        # TODO: verify DDR3 compatibility
        # RPC DRAM is compatible with DDR3 pads, so if stb is not present, use address[0].
        self.stb = pads.stb if hasattr(pads, "stb") else pads.a[0]

        # CSRs -------------------------------------------------------------------------------------
        self._dly_sel             = CSRStorage(databits//8)
        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip     = CSR()

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

        # RPC always has AL=1 and both read and write latencies are equal: RL=WL=AL+CL
        al = 1
        cwl = cl = get_cl(tck) + al

        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)

        rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
        wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)

        # Read latency
        db_cmd_dly   = 1  # (need +1 cycle to insert STB preamble)
        cmd_ser_dly  = write_ser_latency
        read_mux_dly = 1
        bitslip_dly  = 3  # ncycles + 1
        # Time until first data is available on DQ
        read_dq_dly = db_cmd_dly + cmd_ser_dly + cl_sys_latency
        # Time until data is set on DFI
        read_dfi_dly = read_des_latency + read_mux_dly + bitslip_dly
        # Final latency
        read_latency = read_dq_dly + read_dfi_dly

        # Write latency for the controller. We must send 1 cycles of data mask before the
        # data, and we serialize data over 2 sysclk cycles due to minimal BL=16, so we
        # are writing in the 2 cycles following the cycle when we obtain data on DFI.
        # Other PHYs can send everything in 1 sysclk. Because of this spcific delay, we
        # have to increate tWR in the RPC SDRModule definition to meet tWR requirements.
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

        # Register DFI history, as we need to operate on 3 subsequent cycles (write data, only
        # 2 needed for commands)
        # hist[0] = dfi[N], hist[1] = dfi[N-1], ...
        self.dfi = dfi = Interface(**dfi_params)
        dfi_hist = [dfi, Interface(**dfi_params), Interface(**dfi_params)]
        self.sync += dfi_hist[0].connect(dfi_hist[1])
        self.sync += dfi_hist[1].connect(dfi_hist[2])

        # Serialization ----------------------------------------------------------------------------
        # We have the following signals that have to be serialized:
        # - DB  (IO) - transmits data in/out, data mask, parallel commands
        # - STB (O)  - serial commands, serial preamble
        # - CLK (O)  - full-rate clock
        # - DQS (IO) - strobe for commands/data/mask on DB pins
        # DQS is edge-aligned to CLK, while DB and STB are center-aligned to CLK (phase = -90).
        # Sending a parallel command (on DB pins):
        #  CLK: ____----____----____----____----____----____
        #  STB: ----------________________------------------
        #  DQS: ....................----____----____........
        #  DB:  ..........................PPPPnnnn..........
        # This PHY sends DB/STB phase aligned to sysclk and DQS/CLK are assumed to be shifted
        # back by 90 degrees (serializers clocked with sys4x+90deg).

        # Signal values (de)serialized during 1 sysclk.
        # These signals must be populated in specific PHY implementations.
        self.clk_1ck_out = clk_1ck_out = Signal(2*nphases)
        self.stb_1ck_out = stb_1ck_out = Signal(2*nphases)

        self.dqs_1ck_out = dqs_1ck_out = Signal(2*nphases)
        self.dqs_1ck_in  = dqs_1ck_in  = Signal(2*nphases)
        self.dqs_oe      = dqs_oe      = Signal()

        self.db_1ck_out  = db_1ck_out  = [Signal(2*nphases) for _ in range(databits)]
        self.db_1ck_in   = db_1ck_in   = [Signal(2*nphases) for _ in range(databits)]
        self.db_oe       = db_oe       = Signal()

        # Clocks -----------------------------------------------------------------------------------
        # Serialize clock (pattern will be shifted back 90 degrees!)
        self.comb += clk_1ck_out.eq(bitpattern("_-_-_-_-"))

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
        # Request Packet. For that reason we use the previous values of DFI commands.
        # list: dfi[N-1][p0], dfi[N-1][p1], ..., dfi[N][p0], dfi[N][p1], ...
        dfi_adapters = []
        for phase in dfi_hist[1].phases + dfi_hist[0].phases:
            adapter = DFIAdapter(phase)
            self.submodules += adapter
            dfi_adapters.append(adapter)
            self.comb += [
                # We always send one WORD, which consists of 32 bytes.
                adapter.bc.eq(0),
                # Always use fast refresh (equivalent to auto refresh) instead of low-power refresh
                # (equivalent to self refresh).
                adapter.ref_op.eq(adapter.REF_OP["FST"]),
                # Format mode register from addressbits and bankbits as defined in litedram/init.py
                adapter.mr.cl.eq(phase.address[0:3]),
                adapter.mr.nwr.eq(phase.address[3:6]),
                adapter.mr.zout.eq(phase.address[6:10]),
                adapter.mr.odt.eq(phase.address[10:13]),
                adapter.mr.csr_fx.eq(phase.address[13]),
                adapter.mr.odt_stb.eq(phase.bank[0]),
                adapter.mr.odt_pd.eq(phase.bank[1]),
                adapter.mr.tm.eq(0),
            ]

        # Serialize commands to DB pins
        for i in range(databits):
            # A list of differential DB values using previous DFI coomand:
            # db_p[p][i], db_n[p][i], db_p[p+1][i], db_n[p+1][i], ...
            bits = [db for a in dfi_adapters[:nphases] for db in [a.db_p[i], a.db_n[i]]]
            self.comb += db_1ck_cmd[i].eq(Cat(*bits))

        # Commands go on the 2nd cycle, so use previous DFI
        self.comb += dq_cmd_en.eq(reduce(or_, [a.db_valid for a in dfi_adapters[:nphases]]))

        # Power Up Reset ---------------------------------------------------------------------------
        # During Power Up, after stabilizing clocks, Power Up Reset must be done. It consists of a
        # single Parallel Reset followed by two Serial Resets (2x8=16 full-rate cycles).
        # FIXME: Perform Power Up RESET sequence when controller sends ZQC with address=0x123 as
        # there is no RESET command. Controller will send it on phase 0.
        def is_reset(p):
            return p.cas_n & p.ras_n & ~p.we_n & (p.address == 0x0123)

        # Shift register for reset timing (reset_hist[0] = current)
        reset_hist = Signal(1 + 4)
        reset_hist_last = Signal.like(reset_hist)
        self.comb += reset_hist.eq(Cat(is_reset(dfi_hist[0].phases[0]), reset_hist_last))
        self.sync += reset_hist_last.eq(reset_hist)
        # Always send reset on phase 0 to easily track the required 16 cycles of STB reset (stb=0)
        self.comb += dfi_adapters[0].do_reset.eq(reset_hist[1])
        stb_reset_seq = reduce(or_, reset_hist[1:])

        # STB --------------------------------------------------------------------------------------
        # Currently not sending any serial commands, but the STB pin must be held low for 2 full
        # rate cycles before writing a parallel command to activate the DRAM.
        stb_bits = []
        for p in range(nphases):
            # Use cmd from current and prev cycle, depending on which phase the command appears on
            preamble = dfi_adapters[p + 2].db_valid | dfi_adapters[p + 1].db_valid
            # We only want to use STB to start parallel commands, serial reset or to send NOPs. NOP
            # is indicated by the first two bits being high (0b11, and other as "don't care"), so
            # we can simply hold STB high all the time and reset is zeros for 8 cycles (1 sysclk).
            stb_bits += 2 * [~(preamble | stb_reset_seq)]
        self.comb += stb_1ck_out.eq(Cat(*stb_bits))

        # Data IN ----------------------------------------------------------------------------------
        # Synchronize the deserializer as we deserialize over 2 cycles.
        dq_in_cnt = Signal()
        self.sync += If(dq_read_stb, dq_in_cnt.eq(~dq_in_cnt)).Else(dq_in_cnt.eq(0))

        # Deserialize read data (TODO: add phase shift)
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
                rst = self._dly_sel.storage[i//8] & self._rdly_dq_bitslip_rst.re,
                slp = self._dly_sel.storage[i//8] & self._rdly_dq_bitslip.re,
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
        # 1 WORD of data (BC=1), we don't care about the 2nd mask (can send 1).
        #
        # Write data (TODO: add phase shift):
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
        data_pattern = [bitpattern("_-_-_-_-")] * 2
        mask_pattern = [bitpattern("___-_-_-"), bitpattern("_-_-_-_-")]
        phase_patterns = {
            0: [bitpattern("_______-"), bitpattern("_-______")],
            1: [bitpattern("________"), bitpattern("_-_-____")],
            2: [bitpattern("________"), bitpattern("___-_-__")],
            3: [bitpattern("________"), bitpattern("_____-_-")],
        }

        pattern_cases = \
            If(dq_mask_en,
                pattern_2ck[0].eq(mask_pattern[0]),
                pattern_2ck[1].eq(mask_pattern[1]),
            ).Else(
                pattern_2ck[0].eq(data_pattern[0]),
                pattern_2ck[1].eq(data_pattern[1]),
            )
        for p in range(nphases):
            phase_valid = dfi_adapters[p].db_valid | dfi_adapters[nphases+p].db_valid
            pattern_cases = \
                If(phase_valid,
                    pattern_2ck[0].eq(phase_patterns[p][0]),
                    pattern_2ck[1].eq(phase_patterns[p][1]),
                ).Else(pattern_cases)

        self.comb += pattern_cases
        self.comb += dqs_oe.eq(reduce(or_, phase_valid) | dq_mask_en | dq_data_en)

        # Read Control Path ------------------------------------------------------------------------
        # Creates a shift register of read commands coming from the DFI interface. This shift
        # register is used to indicate to the DFI interface that the read data is valid.
        rddata_en = Signal(self.settings.read_latency)
        rddata_en_last = Signal.like(rddata_en)
        self.comb += rddata_en.eq(Cat(dfi.phases[self.settings.rdphase].rddata_en, rddata_en_last))
        self.sync += rddata_en_last.eq(rddata_en)
        # -1 because of syncronious assignment
        self.sync += [phase.rddata_valid.eq(rddata_en[-1]) for phase in dfi.phases]
        # Strobe high when data from DRAM is available, before we can send it to DFI.
        self.sync += dq_read_stb.eq(rddata_en[read_dq_dly-1] | rddata_en[read_dq_dly-1 + 1])

        # Write Control Path -----------------------------------------------------------------------
        # Creates a shift register of write commands coming from the DFI interface. This shift
        # register is used to control DQ/DQS tristates.
        wrdata_en = Signal(write_latency + 2 + 1)
        wrdata_en_last = Signal.like(wrdata_en)
        self.comb += wrdata_en.eq(Cat(dfi.phases[self.settings.wrphase].wrdata_en, wrdata_en_last))
        self.sync += wrdata_en_last.eq(wrdata_en)
        # DQS Preamble and data mask are transmitted 1 cycle before data, then 2 cycles of data
        self.comb += dq_mask_en.eq(wrdata_en[write_latency])
        self.comb += dq_data_en.eq(wrdata_en[write_latency + 1] | wrdata_en[write_latency + 2])

    def do_finalize(self):
        self.do_clock_serialization(self.clk_1ck_out, self.pads.clk_p, self.pads.clk_n)
        self.do_stb_serialization(self.stb_1ck_out, self.stb)
        self.do_dqs_serialization(self.dqs_1ck_out, self.dqs_1ck_in, self.dqs_oe,
                                  # RPC DRAM uses 1 differential DQS pair
                                  self.pads.dqs_p[0], self.pads.dqs_n[0])
        self.do_db_serialization(self.db_1ck_out, self.db_1ck_in, self.db_oe, self.pads.dq)

    # I/O implementation ---------------------------------------------------------------------------

    def do_clock_serialization(self, clk_1ck_out, clk_p, clk_n):
        raise NotImplementedError("Serialize the full-rate clock with 90 deg phase delay")

    def do_stb_serialization(self, stb_1ck_out, stb):
        raise NotImplementedError("Serialize the STB line")

    def do_dqs_serialization(self, dqs_1ck_out, dqs_1ck_in, dqs_oe, dqs_p, dqs_n):
        raise NotImplementedError("Tristate and (de)serialize DQS with 90 deg phase delay")

    def do_db_serialization(self, db_1ck_out, db_1ck_in, db_oe, dq):
        raise NotImplementedError("Tristate and (de)serialize DB")

# Etron RPC Simulation PHY -------------------------------------------------------------------------

class SimulationPHY(BasePHY):
    def __init__(self, *args, generate_read_data=True, **kwargs):
        kwargs.update(dict(
            write_ser_latency = 0,
            read_des_latency  = 1,
            phytype           = self.__class__.__name__,
        ))
        super().__init__(*args, **kwargs)

        self.generate_read_data = generate_read_data

        # For simulation purpose (serializers/deserializers)
        self.sd_ddr    = getattr(self.sync, "sys4x_ddr")
        self.sd_ddr_90 = getattr(self.sync, "sys4x_90_ddr")

    def do_clock_serialization(self, clk_1ck_out, clk_p, clk_n):
        ser = Serializer(self.sd_ddr_90, clk_1ck_out)
        self.submodules += ser
        self.comb += clk_p.eq(ser.o)
        self.comb += clk_n.eq(~ser.o)

    def do_stb_serialization(self, stb_1ck_out, stb):
        ser = Serializer(self.sd_ddr, stb_1ck_out)
        self.submodules += ser
        self.comb += stb.eq(ser.o)

    def do_dqs_serialization(self, dqs_1ck_out, dqs_1ck_in, dqs_oe, dqs_p, dqs_n):
        dqs_out = Signal()
        dqs_in  = Signal()  # TODO: use it for reading

        ser = Serializer(self.sd_ddr_90, dqs_1ck_out)
        self.submodules += ser
        self.comb += dqs_out.eq(ser.o)

        des = Deserializer(self.sd_ddr_90, dqs_in, dqs_1ck_in)
        self.submodules += des

        self.specials += Tristate(dqs_p,  dqs_out, dqs_oe, dqs_in)
        self.specials += Tristate(dqs_n, ~dqs_out, dqs_oe)

    def do_db_serialization(self, db_1ck_out, db_1ck_in, db_oe, dq):
        if self.generate_read_data:
            # Dummy read data generator for simulation purpose
            dq_in_dummy = Signal(self.databits)
            gen = DummyReadGenerator(dq_in=dq, dq_out=dq_in_dummy, stb_in=self.stb_pad,
                                     cl=self.settings.cl)
            self.submodules += ClockDomainsRenamer({"sys": "sys4x_ddr"})(gen)

        for i in range(self.databits):
            # To/from tristate
            dq_out = Signal()
            dq_in = Signal()

            ser = Serializer(self.sd_ddr, db_1ck_out[i])
            self.submodules += ser
            self.comb += dq_out.eq(ser.o)

            # Use sd_ddr_90 in simulation, in general leveling would be needed.
            if self.generate_read_data:
                des = Deserializer(self.sd_ddr_90, dq_in_dummy[i], db_1ck_in[i])
            else:
                des = Deserializer(self.sd_ddr_90, dq_in, db_1ck_in[i])
            self.submodules += des

            self.specials += Tristate(dq[i], dq_out, db_oe, dq_in)

class DummyReadGenerator(Module):
    def __init__(self, stb_in, dq_in, dq_out, cl):
        # self.sync should be sys4x_ddr
        # sys4x:     ----____----____
        # sys4x_ddr: --__--__--__--__
        # pos:       1 0 1 0 1 0 1 0
        pos = Signal(reset=1)
        self.sync += pos.eq(~pos)

        data_counter = Signal(max=16)
        cmd_counter = Signal(max=16)
        # count STB zeros, 2 full-rate cycles (4 DDR) mean STB preamble, but more mean RESET
        stb_zero_counter = Signal(max=4 + 1)
        self.sync += \
            If(stb_in == 0,
                If(stb_zero_counter != 4,
                    stb_zero_counter.eq(stb_zero_counter + 1)
                )
            ).Else(
                stb_zero_counter.eq(0)
            )

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
            If(stb_zero_counter == 4 - 1,
                If(pos,
                    Display("ERROR: end of STB preable on positive edge!")
                ),
                NextState("CHECK_CMD_P")
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
    """Serialize input signals into one output in the `sd` clock domain"""
    def __init__(self, sd, inputs, reset=0, name=None):
        assert(len(inputs) > 0)
        assert(len(s) == len(inputs[0]) for s in inputs)

        data_width = len(inputs)
        signal_width = len(inputs[0])

        if not isinstance(inputs, Array):
            inputs = Array(inputs)

        self.o = Signal(signal_width)
        data_cntr = Signal(log2_int(data_width), name=name)
        sd += If(reset, data_cntr.eq(0)).Else(data_cntr.eq(data_cntr+1))
        self.comb += self.o.eq(inputs[data_cntr])

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
