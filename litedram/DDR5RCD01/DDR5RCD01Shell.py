#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# RCD
from litedram.DDR5RCD01.DDR5RCD01BCOMSimulationPads import DDR5RCD01BCOMSimulationPads
from litedram.DDR5RCD01.DDR5RCD01CoreEgressSimulationPads import DDR5RCD01CoreEgressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01SidebandSimulationPads import DDR5RCD01SidebandSimulationPads

from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *

class DDR5RCD01Shell(Module):
    """The DDR5RCD01Shell is a module, which:
          - implements the pin-out (similarly) as the DDR5RCD01_model
          - connect relevant outputs to inputs.
          - the purpose of this block is to quickly develop testbenches
    """

    def __init__(self,
                 pads_ingress_A,
                 pads_ingress_B,
                 pads_ingress_common,
                 pads_sideband,
                 **kwargs):

        self.submodules += pads_sideband
        self.submodules += pads_ingress_A
        self.submodules += pads_ingress_B

        pads_egress_A = DDR5RCD01CoreEgressSimulationPads()
        self.submodules += pads_egress_A
        if pads_ingress_B is not None:
            pads_egress_B = DDR5RCD01CoreEgressSimulationPads()
            self.submodules += pads_egress_B

        pads_bcom_A = DDR5RCD01BCOMSimulationPads()
        self.submodules += pads_bcom_A
        pads_bcom_B = DDR5RCD01BCOMSimulationPads()
        self.submodules += pads_bcom_B

        # Pass-through
        self.comb += pads_ingress_common.qlbd.eq(0)
        self.comb += pads_ingress_common.qlbs.eq(0)
        self.comb += pads_ingress_common.alert_n.eq(1)

        self.comb += pads_egress_A.qrst_n.eq(pads_ingress_common.drst_n)

        self.comb += pads_egress_A.qacs_a_n.eq(pads_ingress_A.dcs_n)
        self.comb += pads_egress_A.qaca_a.eq(pads_ingress_A.dca)
        self.comb += pads_egress_A.qacs_b_n.eq(pads_ingress_A.dcs_n)
        self.comb += pads_egress_A.qaca_b.eq(pads_ingress_A.dca)

        self.comb += pads_egress_A.qack_t.eq(pads_ingress_common.dck_t)
        self.comb += pads_egress_A.qack_c.eq(pads_ingress_common.dck_c)
        self.comb += pads_egress_A.qbck_t.eq(pads_ingress_common.dck_t)
        self.comb += pads_egress_A.qbck_c.eq(pads_ingress_common.dck_c)
        self.comb += pads_egress_A.qcck_t.eq(pads_ingress_common.dck_t)
        self.comb += pads_egress_A.qcck_c.eq(pads_ingress_common.dck_c)
        self.comb += pads_egress_A.qdck_t.eq(pads_ingress_common.dck_t)
        self.comb += pads_egress_A.qdck_c.eq(pads_ingress_common.dck_c)

        if pads_ingress_B is not None:
            self.comb += pads_egress_B.qrst_n.eq(pads_ingress_common.drst_n)

            self.comb += pads_egress_B.qacs_a_n.eq(pads_ingress_B.dcs_n)
            self.comb += pads_egress_B.qaca_a.eq(pads_ingress_B.dca)
            self.comb += pads_egress_B.qacs_b_n.eq(pads_ingress_B.dcs_n)
            self.comb += pads_egress_B.qaca_b.eq(pads_ingress_B.dca)

            self.comb += pads_egress_B.qack_t.eq(pads_ingress_common.dck_t)
            self.comb += pads_egress_B.qack_c.eq(pads_ingress_common.dck_c)
            self.comb += pads_egress_B.qbck_t.eq(pads_ingress_common.dck_t)
            self.comb += pads_egress_B.qbck_c.eq(pads_ingress_common.dck_c)
            self.comb += pads_egress_B.qcck_t.eq(pads_ingress_common.dck_t)
            self.comb += pads_egress_B.qcck_c.eq(pads_ingress_common.dck_c)
            self.comb += pads_egress_B.qdck_t.eq(pads_ingress_common.dck_t)
            self.comb += pads_egress_B.qdck_c.eq(pads_ingress_common.dck_c)


if __name__ == "__main__":
    pads_ingress_A = DDR5RCD01ChannelIngressSimulationPads()
    pads_ingress_B = DDR5RCD01ChannelIngressSimulationPads()
    pads_ingress_common = DDR5RCD01CommonIngressSimulationPads()
    pads_sideband = DDR5RCD01SidebandSimulationPads()

    shell_dc = DDR5RCD01Shell(
        pads_ingress_A=pads_ingress_A,
        pads_ingress_B=pads_ingress_B,
        pads_ingress_common=pads_ingress_common,
        pads_sideband=pads_sideband,
    )

    shell_sc = DDR5RCD01Shell(
        pads_ingress_A=pads_ingress_A,
        pads_ingress_B=None,
        pads_ingress_common=pads_ingress_common,
        pads_sideband=pads_sideband,
    )
