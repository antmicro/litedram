#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
# migen
from operator import xor
from migen import *
from migen.fhdl import verilog
# RCD
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
# Simulation Pads
from litedram.DDR5RCD01.DDR5RCD01CoreEgressSimulationPads import DDR5RCD01CoreEgressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01CoreIngressSimulationPads import DDR5RCD01CoreIngressSimulationPads
# Submodules
from litedram.DDR5RCD01.DDR5RCD01Core import DDR5RCD01Core

class DDR5RCD01CoreWrapper(Module):
    """DDR5 RCD01 Core Wrapper
    TODO Connects the simulations pads to the core
    """

    def __init__(self, pads_ingress, pads_sideband, aligned_reset_zero=False, dq_dqs_ratio=8,
                 nranks=1, is_dual_channel=False, **kwargs):
        self.submodules.pads_ingress = pads_ingress
        self.submodules.pads_sideband = pads_sideband

        pads_egress = DDR5RCD01CoreEgressSimulationPads(
            is_dual_channel=is_dual_channel)

        self.submodules.pads_egress = pads_egress
        # Internal implementation
        # TODO Integrate simulation pads and interfaces
        if_sdram_A_rst_n = If_rst_n()
        self.comb += if_sdram_A_rst_n.rst_n.eq(self.pads_egress.A_qrst_n)

        # Host <-> common
        if_host_ck = If_ck()
        self.comb += if_host_ck.ck_t.eq(self.pads_ingress.dck_t)
        self.comb += if_host_ck.ck_c.eq(self.pads_ingress.dck_c)
        if_host_rst_n = If_rst_n()
        self.comb += if_host_rst_n.rst_n.eq(self.pads_ingress.drst_n)
        if_host_err = If_error()
        self.comb += if_host_err.err_n.eq(self.pads_ingress.alert_n)
        if_host_lb = If_lb()
        self.comb += if_host_lb.lbd.eq(self.pads_ingress.qlbd)
        self.comb += if_host_lb.lbs.eq(self.pads_ingress.qlbd)
        # Host <-> channel A
        if_ibuf_A = If_channel_ibuf()
        self.comb += if_ibuf_A.dca.eq(self.pads_ingress.A_dca)
        self.comb += if_ibuf_A.dcs_n.eq(self.pads_ingress.A_dcs_n)
        self.comb += if_ibuf_A.dpar.eq(self.pads_ingress.A_dpar)
        # Channel A <-> SDRAM
        if_obuf_csca_A = If_channel_obuf_csca()
        self.comb += if_obuf_csca_A.qacs_a_n.eq(self.pads_egress.A_qacs_a_n)
        self.comb += if_obuf_csca_A.qaca_a.eq(self.pads_egress.A_qaca_a)
        self.comb += if_obuf_csca_A.qacs_b_n.eq(self.pads_egress.A_qacs_b_n)
        self.comb += if_obuf_csca_A.qaca_b.eq(self.pads_egress.A_qaca_b)
        if_obuf_clks_A = If_channel_obuf_clks()
        self.comb += if_obuf_clks_A.qack_t.eq(self.pads_egress.A_qack_t)
        self.comb += if_obuf_clks_A.qack_c.eq(self.pads_egress.A_qack_c)
        self.comb += if_obuf_clks_A.qbck_t.eq(self.pads_egress.A_qbck_t)
        self.comb += if_obuf_clks_A.qbck_c.eq(self.pads_egress.A_qbck_c)
        self.comb += if_obuf_clks_A.qcck_t.eq(self.pads_egress.A_qcck_t)
        self.comb += if_obuf_clks_A.qcck_c.eq(self.pads_egress.A_qcck_c)
        self.comb += if_obuf_clks_A.qdck_t.eq(self.pads_egress.A_qdck_t)
        self.comb += if_obuf_clks_A.qdck_c.eq(self.pads_egress.A_qdck_c)

        if_sdram_A = If_error()
        self.comb += if_sdram_A.err_n.eq(self.pads_egress.A_derror_in_n)

        if_sdram_B_rst_n = If_rst_n()
        self.comb += if_sdram_B_rst_n.rst_n.eq(self.pads_egress.B_qrst_n)

        
        xcore = DDR5RCD01Core()
        self.submodules += xcore

        # TODO feed egress from interfaces


if __name__ == "__main__":
    raise NotSupportedException