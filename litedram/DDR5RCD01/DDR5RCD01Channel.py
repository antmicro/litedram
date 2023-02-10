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
from litedram.DDR5RCD01.DDR5RCD01CommandLogic import DDR5RCD01CommandLogic
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
                 if_config_global,
                 if_common,
                 if_ctrl_common,
                 if_config_common,
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
            Split clock inputs
        """
        if_clk_rowA_rankA = If_ck()
        self.comb += if_clk_rowA_rankA.ck_t.eq(if_clks_i.ck_t[3])
        self.comb += if_clk_rowA_rankA.ck_c.eq(if_clks_i.ck_c[3])

        if_clk_rowB_rankA = If_ck()
        self.comb += if_clk_rowB_rankA.ck_t.eq(if_clks_i.ck_t[2])
        self.comb += if_clk_rowB_rankA.ck_c.eq(if_clks_i.ck_c[2])

        if_clk_rowA_rankB = If_ck()
        self.comb += if_clk_rowA_rankB.ck_t.eq(if_clks_i.ck_t[1])
        self.comb += if_clk_rowA_rankB.ck_c.eq(if_clks_i.ck_c[1])

        if_clk_rowB_rankB = If_ck()
        self.comb += if_clk_rowB_rankB.ck_t.eq(if_clks_i.ck_t[0])
        self.comb += if_clk_rowB_rankB.ck_c.eq(if_clks_i.ck_c[0])
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

        """
            Command Logic : Rank A
        """
        if_csca_o = If_ibuf()
        if_csca_o_rank_A = If_ibuf()
        if_csca_o_rank_B = If_ibuf()

        if_ctrl_lbuf_row_A_rankA = If_ctrl_lbuf()
        if_ctrl_lbuf_row_B_rankA = If_ctrl_lbuf()

        if_ctrl_lbuf_row_A_rankB = If_ctrl_lbuf()
        if_ctrl_lbuf_row_B_rankB = If_ctrl_lbuf()

        tmp_rw_is_output_inversion_enabled=Signal()
        self.comb += tmp_rw_is_output_inversion_enabled.eq(1)

        tmp_rw_is_parity_checking_enabled=Signal()
        self.comb += tmp_rw_is_parity_checking_enabled.eq(0)
        tmp_parity_error=Signal()
        tmp_reserved_if_mrw_actor=Signal()

        xcmd_logic = DDR5RCD01CommandLogic(
            if_ibuf_i=if_ibuf_o,
            if_csca_o=if_csca_o,
            if_csca_o_rank_A=if_csca_o_rank_A,
            if_csca_o_rank_B=if_csca_o_rank_B,
            if_ctrl_lbuf_rank_A_row_A=if_ctrl_lbuf_row_A_rankA,
            if_ctrl_lbuf_rank_A_row_B=if_ctrl_lbuf_row_B_rankA,
            if_ctrl_lbuf_rank_B_row_A=if_ctrl_lbuf_row_A_rankB,
            if_ctrl_lbuf_rank_B_row_B=if_ctrl_lbuf_row_B_rankB,
            rw_is_output_inversion_enabled=tmp_rw_is_output_inversion_enabled,
            rw_is_parity_checking_enabled=tmp_rw_is_parity_checking_enabled,
            parity_error=tmp_parity_error,
            reserved_if_mrw_actor=tmp_reserved_if_mrw_actor,
        )
        self.submodules += xcmd_logic

        """
             Rank Buffer A
        """
        if_obuf_csca_row_A_rankA = If_bus_csca_o()
        if_obuf_csca_row_B_rankA = If_bus_csca_o()

        if_ctrl_obuf_csca_row_A_rankA = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_csca_row_B_rankA = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_clks_row_A_rankA = If_ctrl_obuf_CLKS()
        if_ctrl_obuf_clks_row_B_rankA = If_ctrl_obuf_CLKS()

        if_obuf_clks_row_A_rankA = If_ck()
        if_obuf_clks_row_B_rankA = If_ck()

        if_csca_rank_A = If_bus_csca()
        self.comb += if_csca_rank_A.cs_n.eq(if_csca_o_rank_A.dcs_n)
        self.comb += if_csca_rank_A.ca.eq(if_csca_o_rank_A.dca)

        xrankA = DDR5RCD01RankBuffer(
            if_ibuf=if_csca_rank_A,
            if_clk_row_A=if_clk_rowA_rankA,
            if_clk_row_B=if_clk_rowB_rankA,
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

        if_ctrl_obuf_csca_row_A_rankB = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_csca_row_B_rankB = If_ctrl_obuf_CSCA()
        if_ctrl_obuf_clks_row_A_rankB = If_ctrl_obuf_CLKS()
        if_ctrl_obuf_clks_row_B_rankB = If_ctrl_obuf_CLKS()

        if_obuf_csca_row_A_rankB = If_bus_csca_o()
        if_obuf_csca_row_B_rankB = If_bus_csca_o()

        if_obuf_clks_row_A_rankB = If_ck()
        if_obuf_clks_row_B_rankB = If_ck()

        if_csca_rank_B = If_bus_csca()
        self.comb += if_csca_rank_B.cs_n.eq(if_csca_o_rank_B.dcs_n)
        self.comb += if_csca_rank_B.ca.eq(if_csca_o_rank_B.dca)

        xrankB = DDR5RCD01RankBuffer(
            if_ibuf=if_csca_rank_B,
            if_clk_row_A=if_clk_rowA_rankB,
            if_clk_row_B=if_clk_rowB_rankB,
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
        xcontrol_center = DDR5RCD01ControlCenter(
            if_ctrl_ibuf=if_ctrl_ibuf,
            if_ctrl_lbuf_row_A_rankA=if_ctrl_lbuf_row_A_rankA,
            if_ctrl_lbuf_row_B_rankA=if_ctrl_lbuf_row_B_rankA,
            if_ctrl_obuf_csca_row_A_rankA=if_ctrl_obuf_csca_row_A_rankA,
            if_ctrl_obuf_csca_row_B_rankA=if_ctrl_obuf_csca_row_B_rankA,
            if_ctrl_obuf_clks_row_A_rankA=if_ctrl_obuf_clks_row_A_rankA,
            if_ctrl_obuf_clks_row_B_rankA=if_ctrl_obuf_clks_row_B_rankA,
            if_ctrl_lbuf_row_A_rankB=if_ctrl_lbuf_row_A_rankB,
            if_ctrl_lbuf_row_B_rankB=if_ctrl_lbuf_row_B_rankB,
            if_ctrl_obuf_csca_row_A_rankB=if_ctrl_obuf_csca_row_A_rankB,
            if_ctrl_obuf_csca_row_B_rankB=if_ctrl_obuf_csca_row_B_rankB,
            if_ctrl_obuf_clks_row_A_rankB=if_ctrl_obuf_clks_row_A_rankB,
            if_ctrl_obuf_clks_row_B_rankB=if_ctrl_obuf_clks_row_B_rankB,
            if_ctrl_global=if_ctrl_global,
            if_config_global=if_config_global,
            if_common=if_common,
            if_ctrl_common=if_ctrl_common,
            if_config_common=if_config_common,
            is_channel_A=is_master,
        )

        self.submodules += xcontrol_center



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
