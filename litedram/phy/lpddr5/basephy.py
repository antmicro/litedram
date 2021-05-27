#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from typing import Tuple
from dataclasses import dataclass

from migen import *

from litedram.phy.lpddr5.commands import DFIPhaseAdapter


class LPDDR5Output:
    """Unserialized output of LPDDR5PHY. Has to be serialized by concrete implementation."""
    def __init__(self, nphases, databits):
        assert databits % 8 == 0
        self.reset_n = Signal()
        self.ck      = Signal(nphases)
        self.cs      = Signal(nphases//2)  # CK SDR
        self.ca      = [Signal(nphases)   for _ in range(7)]  # CK DDR
        # WCK DDR
        self.dq_o    = [Signal(2*nphases) for _ in range(databits)]
        self.dq_i    = [Signal(2*nphases) for _ in range(databits)]
        self.dq_oe   = Signal()
        self.wck     = [Signal(2*nphases)   for _ in range(databits//8)]
        self.wck_oe  = Signal()
        self.rdqs_o  = [Signal(2*nphases)   for _ in range(databits//8)]
        self.rdqs_i  = [Signal(2*nphases)   for _ in range(databits//8)]
        self.rdqs_oe = Signal()
        self.dmi_o   = [Signal(2*nphases) for _ in range(databits//8)]
        self.dmi_i   = [Signal(2*nphases) for _ in range(databits//8)]
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
    #         MR       WL                                   RL         nRBTP
    FreqRange(0b0000, (40,   533),  (4,  4),  (1, 1), 1, 3,  6, 0,  6, 0),
    FreqRange(0b0001, (533,  1067), (4,  6),  (0, 2), 2, 3,  8, 0,  7, 0),
    FreqRange(0b0010, (1067, 1600), (6,  8),  (1, 3), 2, 4, 10, 1,  8, 0),
    FreqRange(0b0011, (1600, 2133), (8,  10), (2, 4), 3, 4, 12, 2,  8, 0),
    FreqRange(0b0100, (2133, 2750), (8,  14), (1, 7), 4, 4, 16, 3, 10, 2),
    FreqRange(0b0101, (2750, 3200), (10, 16), (3, 9), 4, 4, 18, 5, 10, 2),
]

def get_cl_cwl(tck, wl_set, rl_set):
    data_rate = 2 * 1/tck
    for frange in FREQUENCY_RANGES.items():
        dr_min, dr_max = frange.data_rate
        if dr_min < data_rate <= dr_max:
            cl = frange.rl[rl_set]
            cwl = frange.wl[{'A': 0, 'B': 1}[wl_set]]
            return cl, cwl
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
    def __init__(self, pads, *, sys_clk_freq, phytype, cmd_delay, masked_write=True):
        self.pads        = pads
        self.memtype     = memtype     = "LPDDR5"
        self.nranks      = nranks      = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        self.databits    = databits    = len(pads.dq)
        self.addressbits = addressbits = 17  # for activate row address
        self.bankbits    = bankbits    = 7  # 4, but 7 bits needed for Mode Register address
        self.nphases     = nphases     = 8
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)
        assert databits % 8 == 0

        # Parameters -------------------------------------------------------------------------------
        # TODO

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
        # TODO

        # DFI Interface ----------------------------------------------------------------------------
        # DDR 8 phases to be able to process whole 16n burst in a single controller clock cycle.
        self.dfi = dfi = Interface(addressbits, bankbits, nranks, 2*databits, nphases=8)

        # # #

        adapters = [DFIPhaseAdapter(phase, masked_write=masked_write) for phase in self.dfi.phases]
        self.submodules += adapters

        self.out = LPDDR5Output(nphases, databits)

        # Clocks -----------------------------------------------------------------------------------
        self.comb += self.out.ck.eq(bitpattern("-_-_-_-_"))
        for wck in self.out.wck:
            self.comb += wck.eq(bitpattern("-_-_-_-_" * 2))
        self.comb += self.out.wck_oe.eq(1)  # TODO: enable only on burst

