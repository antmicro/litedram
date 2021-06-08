#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import or_, and_
from functools import reduce
from typing import Tuple
from dataclasses import dataclass

from migen import *

from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSR

from litedram.common import BitSlip, get_sys_latency, get_sys_phase, PhySettings, TappedDelayLine
from litedram.phy.dfi import Interface as DFIInterface
from litedram.phy.utils import CommandsPipeline, bitpattern, delayed
from litedram.phy.lpddr5.commands import DFIPhaseAdapter


class LPDDR5Output:
    """Unserialized output of LPDDR5PHY. Has to be serialized by concrete implementation."""
    def __init__(self, nphases, databits):
        assert databits % 8 == 0
        self.reset_n = Signal(nphases)
        self.ck      = Signal(2*nphases)
        self.cs      = Signal(nphases)  # CK SDR
        self.ca      = [Signal(2*nphases)   for _ in range(7)]  # CK DDR
        # WCK DDR
        self.dq_o    = [Signal(2*2*nphases) for _ in range(databits)]
        self.dq_i    = [Signal(2*2*nphases) for _ in range(databits)]
        self.dq_oe   = Signal()
        self.wck     = [Signal(2*2*nphases)   for _ in range(databits//8)]
        self.rdqs_o  = [Signal(2*2*nphases)   for _ in range(databits//8)]
        self.rdqs_i  = [Signal(2*2*nphases)   for _ in range(databits//8)]
        self.rdqs_oe = Signal()
        self.dmi_o   = [Signal(2*2*nphases) for _ in range(databits//8)]
        self.dmi_i   = [Signal(2*2*nphases) for _ in range(databits//8)]
        self.dmi_oe  = Signal()


@dataclass
class FreqRange:
    mr:                 int                  # MR2[3:0] value
    data_rate:          Tuple[int, int]      # (> Mbps, <= Mbps)
    wl:                 Tuple[int, int]      # (Set A, Set B)
    t_wckenl_wr:        Tuple[int, int]      # (Set A, Set B)
    t_wckpre_static:    int
    t_wckpre_toggle_wr: int
    rl:                 Tuple[int, int, int] # (Set 0, Set 1, Set 2)
    t_wckenl_rd:        int
    t_wckpre_toggle_rd: int
    n_rbtp:             int

    @property
    def ck_freq(self):
        low, high = self.data_rate
        return (round(low / 4), round(hi / 4))

# Taken from Tables 182, 183, 201 of JEDEC specification for LPDDR5
# WCK:CK=2:1, DVFSC diabled, Read Link ECC off
FREQUENCY_RANGES = [
    #         MR       DR            WL                     RL         nRBTP
    FreqRange(0b0000, (40,   533),  (4,  4),  (1, 1), 1, 3, ( 6,  6,  6), 0,  6, 0),
    FreqRange(0b0001, (533,  1067), (4,  6),  (0, 2), 2, 3, ( 8,  8,  8), 0,  7, 0),
    FreqRange(0b0010, (1067, 1600), (6,  8),  (1, 3), 2, 4, (10, 10, 10), 1,  8, 0),
    FreqRange(0b0011, (1600, 2133), (8,  10), (2, 4), 3, 4, (12, 14, 14), 2,  8, 0),
    FreqRange(0b0100, (2133, 2750), (8,  14), (1, 7), 4, 4, (16, 16, 16), 3, 10, 2),
    FreqRange(0b0101, (2750, 3200), (10, 16), (3, 9), 4, 4, (18, 20, 20), 5, 10, 2),
]


def get_cl_cwl(tck, wl_set, rl_set):
    data_rate = 2 * 1/tck
    for frange in FREQUENCY_RANGES:
        dr_min, dr_max = frange.data_rate
        if dr_min < data_rate/1e6 <= dr_max:
            cl = frange.rl[rl_set]
            cwl = frange.wl[{'A': 0, 'B': 1}[wl_set]]
            return cl, cwl
    raise ValueError

def get_frange(tck):
    data_rate = 2 * 1/tck
    for frange in FREQUENCY_RANGES:
        dr_min, dr_max = frange.data_rate
        if dr_min < data_rate/1e6 <= dr_max:
            return frange
    raise ValueError



class LPDDR5PHY(Module, AutoCSR):
    # TODO: doc
    # LPDDR5, WCK:CK=2:1, 16n
    # Clocks:
    # * controller = sys
    # * CK (diff) = sys4x
    # * CS = SDR @ sys4x
    # * CA = DDR @ sys4x
    # * commands span 1-2 sys4x clocks
    # * WCK (RDQS, diff) = sys8x
    # * DQ/DMI = DDR @ sys8x
    # * on Series 7:
    #     * commands:
    #         * OSERDESE2 DDR @ sys->sys4x (8:1)
    #         * CA serialized normally
    #         * CS doubled (per-bit) before serialization
    #     * data:
    #         * in-FPGA serialization from sys to sys2x
    #         * OSERDESE2 DDR @ sys2x->sys8x (8:1)
    #         * DQ/DMI serialized normally
    # * controller timings might be a bit more complicated to get right
    def __init__(self, pads, *, sys_clk_freq, phytype, ser_latency, des_latency, cmd_delay=None,
            masked_write=True, extended_overlaps_check=False):
        self.pads        = pads
        self.memtype     = memtype     = "LPDDR5"
        self.nranks      = nranks      = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        self.databits    = databits    = len(pads.dq)
        self.addressbits = addressbits = 18  # for activate row address
        self.bankbits    = bankbits    = 7  # 4, but 7 bits needed for Mode Register address
        self.nphases     = nphases     = 4
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)
        self.ser_latency = ser_latency
        self.des_latency = des_latency
        assert databits % 8 == 0

        # Parameters -------------------------------------------------------------------------------
        frange = get_frange(tck)
        wl_set = "A"
        rl_set = 0

        # Bitslip introduces latency from 1 up to `cycles + 1` (sys)
        bitslip_cycles  = 1
        bitslip_range   = 1
        # Commands read from adapters are delayed on ConstBitSlips (sys)
        ca_latency      = 1
        # Commands are sent over 2 CK (sys4x) and we count cl/cwl from last bit
        cmd_latency     = 2

        cl, cwl = frange.rl[rl_set], frange.wl[{'A': 0, 'B': 1}[wl_set]]  # measured with respect to CK
        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)
        # For reads we need to account for ser+des latency to make sure we get the data in-phase with sys clock
        rdphase = get_sys_phase(nphases, cl_sys_latency, cl + cmd_latency + ser_latency.sys4x%4 + des_latency.sys4x%4)
        # No need to modify wrphase, because ser_latency applies the same to both CA and DQ
        wrphase = get_sys_phase(nphases, cwl_sys_latency, cwl + cmd_latency)

        # When the calculated phase is negative, it means that we need to increase sys latency
        def updated_latency(phase, sys_latency):
            while phase < 0:
                phase += nphases
                sys_latency += 1
            return phase, sys_latency

        wrphase, cwl_sys_latency = updated_latency(wrphase, cwl_sys_latency)
        rdphase, cl_sys_latency = updated_latency(rdphase, cl_sys_latency)

        # Read latency
        read_data_delay = ca_latency + ser_latency.sys4x//4 + cl_sys_latency  # DFI cmd -> read data on DQ
        read_des_delay  = des_latency.sys4x//4 + bitslip_cycles+bitslip_range  # data on DQ -> data on DFI rddata
        read_latency    = read_data_delay + read_des_delay

        # Write latency
        write_latency = cwl_sys_latency

        # Registers --------------------------------------------------------------------------------
        self._rst             = CSRStorage()

        self._wlevel_en     = CSRStorage()
        self._wlevel_strobe = CSR()

        self._dly_sel = CSRStorage(databits//8)

        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip     = CSR()

        self._wdly_dq_bitslip_rst = CSR()
        self._wdly_dq_bitslip     = CSR()

        self._rdphase = CSRStorage(log2_int(nphases), reset=rdphase)
        self._wrphase = CSRStorage(log2_int(nphases), reset=wrphase)

        # PHY settings -----------------------------------------------------------------------------
        self.settings = PhySettings(
            phytype       = phytype,
            memtype       = memtype,
            databits      = databits,
            dfi_databits  = 4*databits,
            nranks        = nranks,
            nphases       = nphases,
            rdphase       = self._rdphase.storage,
            wrphase       = self._wrphase.storage,
            cl            = cl,
            cwl           = cwl,
            read_latency  = read_latency,
            write_latency = write_latency,
            cmd_latency   = cmd_latency,
            cmd_delay     = cmd_delay,
        )

        # DFI Interface ----------------------------------------------------------------------------
        # We are using 16n WCK:CK=2:1, so during a period of single data burst there can be 4
        # "full" commands issued (= 4 DFI commands). For this reason we use 4 phases and extend
        # per-phase data width to be able to transfer all the data for WCK.
        self.dfi = dfi = DFIInterface(
            addressbits, bankbits, nranks, self.settings.dfi_databits, nphases)

        # # #

        adapters = [DFIPhaseAdapter(phase, masked_write=masked_write) for phase in self.dfi.phases]
        self.submodules += adapters

        self.out = LPDDR5Output(nphases, databits)

        # CK ---------------------------------------------------------------------------------------
        self.comb += self.out.ck.eq(bitpattern("-_-_-_-_"))

        # Commands ---------------------------------------------------------------------------------
        # Commands are sent with SDR CS and DDR CA[6:0] clocked by CK. DFI command can translate to
        # 1 or 2 LPDDR5 commands. For this reason there could be an overlap if e.g. ACT is presented
        # on two following DFI phases. This should not happen, guaranteed by module timings but we
        # include a check here too.
        # TODO: WCK sync logic, here and/or in commands.py, we can do a single sync during
        # initialization and keep WCK always enabled or try to do proper sync before each burst, but
        # with FPGA, power consumption is probably not very important
        self.submodules.commands = CommandsPipeline(adapters,
            cs_ser_width = len(self.out.cs),
            ca_ser_width = len(self.out.ca[0]),
            ca_nbits     = len(self.out.ca),
            cmd_nphases_span = 2,
            extended_overlaps_check = extended_overlaps_check
        )

        # reset_n=0 on any phase will result in reset
        self.comb += self.out.reset_n.eq(delayed(self, Cat(p.reset_n for p in self.dfi.phases)))

        self.comb += self.out.cs.eq(self.commands.cs)
        for bit in range(7):
            self.comb += self.out.ca[bit].eq(self.commands.ca[bit])

        # Write Control Path -----------------------------------------------------------------------
        wrtap = cwl_sys_latency - 1
        assert wrtap >= 0

        # Create a delay line of write commands coming from the DFI interface. This taps are used to
        # control DQ/DQS tristates.
        wrdata_en = TappedDelayLine(
            signal = reduce(or_, [dfi.phases[i].wrdata_en for i in range(nphases)]),
            ntaps  = wrtap + 2
        )
        self.submodules += wrdata_en

        dq_oe = Signal()
        wck_sync = Signal()
        self.comb += dq_oe.eq(wrdata_en.taps[wrtap])
        # # Always enabled in write leveling mode, else during transfers
        # self.comb += dqs_oe.eq(self._wlevel_en.storage | (dqs_preamble | dq_oe | dqs_postamble))

        # # Write DQS Postamble/Preamble Control Path ------------------------------------------------
        # # Generates DQS Preamble 1 cycle before the first write and Postamble 1 cycle after the last
        # # write. During writes, DQS tristate is configured as output for at least 3 sys_clk cycles:
        # # 1 for Preamble, 1 for the Write and 1 for the Postamble.
        # def wrdata_en_tap(i):  # allows to have wrtap == 0
        #     return wrdata_en.input if i == -1 else wrdata_en.taps[i]
        # self.comb += dqs_preamble.eq( wrdata_en_tap(wrtap - 1)  & ~wrdata_en_tap(wrtap + 0))
        # self.comb += dqs_postamble.eq(wrdata_en_tap(wrtap + 1)  & ~wrdata_en_tap(wrtap + 0))



        # DQ ---------------------------------------------------------------------------------------
        self.comb += self.out.dq_oe.eq(delayed(self, dq_oe))

        for bit in range(self.databits):
            # output
            wrdata = [
                self.dfi.phases[i//4].wrdata[i%4 * self.databits + bit]
                for i in range(2*2*nphases)
            ]
            self.submodules += BitSlip(
                dw     = 2*2*nphases,
                cycles = bitslip_cycles,
                rst    = self.get_rst(bit//8, self._wdly_dq_bitslip_rst.re),
                slp    = self.get_inc(bit//8, self._wdly_dq_bitslip.re),
                i      = Cat(*wrdata),
                o      = self.out.dq_o[bit],
            )

            # input
            dq_i_bs = Signal(2*2*nphases)
            self.submodules += BitSlip(
                dw     = 2*2*nphases,
                cycles = bitslip_cycles,
                rst    = self.get_rst(bit//8, self._rdly_dq_bitslip_rst.re),
                slp    = self.get_inc(bit//8, self._rdly_dq_bitslip.re),
                i      = self.out.dq_i[bit],
                o      = dq_i_bs,
            )
            for i in range(2*2*nphases):
                self.comb += self.dfi.phases[i//4].rddata[i%4 * self.databits + bit].eq(dq_i_bs[i])

        # WCK --------------------------------------------------------------------------------------
        # WCK can be enabled/disabled. When enabling, it has to be synchronized with CK. To do so,
        # CAS (alone or followed by WR/RD) must be issued. Synchronization is done after tCKSENL_x
        # after CAS command (CK rising edge), by keeping WCK static for tWCKPRE_Static, then
        # toggling it for tWCKPRE_Toggle_x. If using WCK:CK=4:1, then the first CK of toggling
        # should be with half WCK frequency.
        # Timings are in relation to WL and RL as:
        # WL = tWCKENL_WR - 1 + tWCKPRE_Static + tWCKPRE_Toggle_WR
        wck_oe = Signal()
        wck = Signal(2*2*nphases)
        wck_pattern = {
            "disabled":    bitpattern("________"),  # could be High-Z
            "static":      bitpattern("--------"),
            "toggle_half": bitpattern("--__--__"),
            "toggle_full": bitpattern("-_-_-_-_"),
        }
        wck_sync_timeline = [
            (frange.t_wckenl_wr, wck_pattern["disabled"]),
            (frange.t_wckpre_static, wck_pattern["static"]),
            (frange.t_wckpre_toggle_wr, wck_pattern["toggle_half"]),
        ]

        # # DMI --------------------------------------------------------------------------------------
        # # DMI signal is used for Data Mask or Data Bus Invertion depending on Mode Registers values.
        # # With DM and DBI disabled, this signal is a Don't Care.
        # # With DM enabled, masking is performed only when the command used is WRITE-MASKED.
        # # We don't support DBI, DM support is configured statically with `masked_write`.
        # for byte in range(self.databits//8):
        #     if isinstance(masked_write, Signal) or masked_write:
        #         self.comb += self.out.dmi_oe.eq(self.out.dq_oe)
        #         wrdata_mask = [
        #             self.dfi.phases[i//2] .wrdata_mask[i%2 * self.databits//8 + byte]
        #             for i in range(2*nphases)
        #         ]
        #         self.submodules += BitSlip(
        #             dw     = 2*nphases,
        #             cycles = bitslip_cycles,
        #             rst    = self.get_rst(byte, self._wdly_dq_bitslip_rst.re),
        #             slp    = self.get_inc(byte, self._wdly_dq_bitslip.re),
        #             i      = Cat(*wrdata_mask),
        #             o      = self.out.dmi_o[byte],
        #         )
        #     else:
        #         self.comb += self.out.dmi_o[byte].eq(0)
        #         self.comb += self.out.dmi_oe.eq(0)

        # # Read Control Path ------------------------------------------------------------------------
        # # Creates a delay line of read commands coming from the DFI interface. The output is used to
        # # signal a valid read data to the DFI interface.
        # #
        # # The read data valid is asserted for 1 sys_clk cycle when the data is available on the DFI
        # # interface, the latency is the sum of the OSERDESE2, CAS, ISERDESE2 and Bitslip latencies.
        # rddata_en = TappedDelayLine(
        #     signal = reduce(or_, [dfi.phases[i].rddata_en for i in range(nphases)]),
        #     ntaps  = self.settings.read_latency
        # )
        # self.submodules += rddata_en
        #
        # self.comb += [
        #     phase.rddata_valid.eq(rddata_en.output | self._wlevel_en.storage)
        #     for phase in dfi.phases
        # ]

    def get_rst(self, byte, rst):
        return (self._dly_sel.storage[byte] & rst) | self._rst.storage

    def get_inc(self, byte, inc):
        return self._dly_sel.storage[byte] & inc


