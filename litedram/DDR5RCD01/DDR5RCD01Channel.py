#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
# migen
from migen import *
# RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
# Submodules
from litedram.DDR5RCD01.DDR5RCD01ControlCenter import DDR5RCD01ControlCenter
from litedram.DDR5RCD01.DDR5RCD01Error import DDR5RCD01Error
from litedram.DDR5RCD01.DDR5RCD01InputBuffer import DDR5RCD01InputBuffer
from litedram.DDR5RCD01.DDR5RCD01RankBuffer import DDR5RCD01RankBuffer


class DDR5RCD01Channel(Module):
    """TODO DDR5 RCD01 Channel
        The RCD Channel implements 2 Rank Buffers

    Module
    ------
    <interface> : CS,CA,etc.
    dck, dck_pll
    """

    def __init__(self,
                 if_ibuf,
                 if_clks_i,
                 if_obuf,
                 if_sdram,
                 if_bcom,
                 if_ctrl_global,
                 if_ctrl_common,
                 is_master=True,
                 ):
        """
            Master is the one providing global settings
        """
        if is_master:
            # Drive the ctrl and common intefaces
            # TODO
            pass
        else:
            pass

        """
            Input Buffer
        """
        if_ctrl_ibuf = If_ctrl_ibuf()
        if_ibuf_o = If_ibuf()

        xibuf = DDR5RCD01InputBuffer(
            if_ib_i=if_ibuf,
            if_ib_o=if_ibuf_o,
            if_ctrl=if_ctrl_ibuf
        )
        self.submodules += xibuf

        if_ibuf_2_ranks = If_bus_csca()
        self.comb += if_ibuf_2_ranks.cs_n.eq(if_ibuf_o.dcs_n)
        self.comb += if_ibuf_2_ranks.ca.eq(if_ibuf_o.dca)

        """
             Rank A
        """
        if_obuf_csca_row_A_rankA = If_bus_csca_o()
        if_obuf_csca_row_B_rankA = If_bus_csca_o()

        if_ctrl_lbuf_row_A_rankA = If_ctrl_lbuf()
        if_ctrl_lbuf_row_B_rankA = If_ctrl_lbuf()
        if_ctrl_obuf_csca_row_A_rankA = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_csca_row_B_rankA = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_clks_row_A_rankA = If_ctrl_obuf_CLKS()
        if_ctrl_obuf_clks_row_B_rankA = If_ctrl_obuf_CLKS()
        
        if_obuf_clks_row_A_rankA = If_ck()
        if_obuf_clks_row_B_rankA = If_ck()

        xrankA = DDR5RCD01RankBuffer(
            if_ibuf=if_ibuf_2_ranks,
            if_clk_row_A=if_clks_i,
            if_clk_row_B=if_clks_i,
            if_obuf_csca_row_A=if_obuf_csca_row_A_rankA,
            if_obuf_csca_row_B=if_obuf_csca_row_B_rankA,
            if_obuf_clks_row_A=if_obuf_clks_row_A_rankA,
            if_obuf_clks_row_B=if_obuf_clks_row_B_rankA,
            if_ctrl_lbuf_row_A=if_ctrl_lbuf_row_A_rankA,
            if_ctrl_lbuf_row_B=if_ctrl_lbuf_row_B_rankA,
            if_ctrl_obuf_csca_row_A=if_ctrl_obuf_csca_row_A_rankA,
            if_ctrl_obuf_csca_row_B=if_ctrl_obuf_csca_row_B_rankA,
            if_ctrl_obuf_clks_row_A=if_ctrl_obuf_clks_row_A_rankA,
            if_ctrl_obuf_clks_row_B=if_ctrl_obuf_clks_row_B_rankA,
        )
        self.submodules += xrankA

        self.comb += if_obuf.qacs_a_n.eq(if_obuf_csca_row_A_rankA.qcs_n)
        self.comb += if_obuf.qaca_a.eq(if_obuf_csca_row_A_rankA.qca)
        self.comb += if_obuf.qacs_b_n.eq(if_obuf_csca_row_B_rankA.qcs_n)
        self.comb += if_obuf.qaca_b.eq(if_obuf_csca_row_B_rankA.qca)

        self.comb += if_obuf.qack_t.eq(if_obuf_clks_row_A_rankA.ck_t)
        self.comb += if_obuf.qack_c.eq(if_obuf_clks_row_A_rankA.ck_c)

        self.comb += if_obuf.qbck_t.eq(if_obuf_clks_row_B_rankA.ck_t)
        self.comb += if_obuf.qbck_c.eq(if_obuf_clks_row_B_rankA.ck_c)

        """
            Rank B
        """

        if_ctrl_lbuf_row_A_rankB = If_ctrl_lbuf()
        if_ctrl_lbuf_row_B_rankB = If_ctrl_lbuf()
        if_ctrl_obuf_csca_row_A_rankB = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_csca_row_B_rankB = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_clks_row_A_rankB = If_ctrl_obuf_CLKS()
        if_ctrl_obuf_clks_row_B_rankB = If_ctrl_obuf_CLKS()

        if_obuf_csca_row_A_rankB = If_bus_csca_o()
        if_obuf_csca_row_B_rankB = If_bus_csca_o()


        if_obuf_clks_row_A_rankB = If_ck()
        if_obuf_clks_row_B_rankB = If_ck()

        xrankB = DDR5RCD01RankBuffer(
            if_ibuf=if_ibuf_2_ranks,
            if_clk_row_A=if_clks_i,
            if_clk_row_B=if_clks_i,
            if_obuf_csca_row_A=if_obuf_csca_row_A_rankB,
            if_obuf_csca_row_B=if_obuf_csca_row_B_rankB,
            if_obuf_clks_row_A=if_obuf_clks_row_A_rankB,
            if_obuf_clks_row_B=if_obuf_clks_row_B_rankB,
            if_ctrl_lbuf_row_A=if_ctrl_lbuf_row_A_rankB,
            if_ctrl_lbuf_row_B=if_ctrl_lbuf_row_B_rankB,
            if_ctrl_obuf_csca_row_A=if_ctrl_obuf_csca_row_A_rankB,
            if_ctrl_obuf_csca_row_B=if_ctrl_obuf_csca_row_B_rankB,
            if_ctrl_obuf_clks_row_A=if_ctrl_obuf_clks_row_A_rankB,
            if_ctrl_obuf_clks_row_B=if_ctrl_obuf_clks_row_B_rankB,
        )
        self.submodules += xrankB

        self.comb += if_obuf.qbcs_a_n.eq(if_obuf_csca_row_A_rankB.qcs_n)
        self.comb += if_obuf.qbca_a.eq(if_obuf_csca_row_A_rankB.qca)
        self.comb += if_obuf.qbcs_b_n.eq(if_obuf_csca_row_B_rankB.qcs_n)
        self.comb += if_obuf.qbca_b.eq(if_obuf_csca_row_B_rankB.qca)

        self.comb += if_obuf.qcck_t.eq(if_obuf_clks_row_A_rankB.ck_t)
        self.comb += if_obuf.qcck_c.eq(if_obuf_clks_row_A_rankB.ck_c)

        self.comb += if_obuf.qdck_t.eq(if_obuf_clks_row_B_rankB.ck_t)
        self.comb += if_obuf.qdck_c.eq(if_obuf_clks_row_B_rankB.ck_c)
        """
            Control Center
        """
        # xcontrol_center = DDR5RCD01ControlCenter(
        #     if_ibuf_2_lbuf=if_ibuf_o,
        #     if_ctrl_ibuf=if_ctrl_ibuf,
        #     # Rank A
        #     if_ctrl_lbuf_row_X_rankA=if_ctrl_lbuf_row_X_rankA,
        #     if_ctrl_obuf_csca_row_A_rankA=if_ctrl_obuf_csca_row_A_rankA,
        #     if_ctrl_obuf_clks_row_B_rankA=if_ctrl_obuf_clks_row_B_rankA,
        #     if_ctrl_obuf_csca_row_A_rankA=if_ctrl_obuf_csca_row_A_rankA,
        #     if_ctrl_obuf_clks_row_B_rankA=if_ctrl_obuf_clks_row_B_rankA,
        #     # Rank B
        #     if_ctrl_lbuf_row_A_rankB=if_ctrl_lbuf_row_A_rankB,
        #     if_ctrl_lbuf_row_B_rankB=if_ctrl_lbuf_row_B_rankB,
        #     if_ctrl_obuf_csca_row_A_rankB=if_ctrl_obuf_csca_row_A_rankB,
        #     if_ctrl_obuf_csca_row_B_rankB=if_ctrl_obuf_csca_row_B_rankB,
        #     if_ctrl_obuf_clks_row_A_rankB=if_ctrl_obuf_clks_row_A_rankB,
        #     if_ctrl_obuf_clks_row_B_rankB=if_ctrl_obuf_clks_row_B_rankB,
        #     # Common
        #     if_ctrl_global=if_ctrl_global,
        #     if_ctrl_common=if_ctrl_common,
        #     is_channel_A=is_master,
        # )

        # self.submodules += xcontrol_center

        # TODO implement error handler
        # xerror = DDR5RCD01Error(iif_err=if_sdram, oif_err=if_err)
        # self.submodules += xerror


class TestBed(Module):
    def __init__(self):
        self.submodules.dut = DDR5RCD01Channel()


def run_test(dut):
    logging.debug('Write test')
    yield
    logging.debug('Yield from write test.')


def behav_write_word(data):
    yield


if __name__ == "__main__":
    raise NotSupportedException
