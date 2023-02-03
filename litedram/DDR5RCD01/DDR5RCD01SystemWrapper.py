#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# LiteDRAM : RCD
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads

from litedram.DDR5RCD01.DDR5GlueRCD import DDR5GlueRCDCommon, DDR5GlueRCDChannel, DDR5GlueRCDDataBuffer

from litedram.DDR5RCD01.DDR5RCD01System import DDR5RCD01System
from litedram.DDR5RCD01.DDR5RCD01SidebandSimulationPads import DDR5RCD01SidebandSimulationPads

from litedram.DDR5RCD01.RCD_definitions import sideband_type as sb_enum
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *

class DDR5RCD01SystemWrapper(Module):
    """
        DDR5RCD01SystemWrapper
        ----------------------
        Module
        ------
        Parameters
        ----------
    """
    def __init__(self, phy_pads, pads_sideband, rcd_passthrough, sideband_type):
        _pads_sideband = None
        if pads_sideband is not None and sideband_type is not None:
            _pads_sideband = DDR5RCD01SidebandSimulationPads()

        common_pads = DDR5RCD01CommonIngressSimulationPads()
        self.submodules += DDR5GlueRCDCommon(phy_pads, common_pads)

        pads_A      = DDR5RCD01ChannelIngressSimulationPads(
            dcs_n_w = len(phy_pads.A_cs_n),
            dca_w   = len(phy_pads.A_ca),
        )
        self.submodules += DDR5GlueRCDChannel(phy_pads, pads_A, "A_")

        pads_B      = DDR5RCD01ChannelIngressSimulationPads(
            dcs_n_w = len(phy_pads.B_cs_n),
            dca_w   = len(phy_pads.B_ca),
        )
        self.submodules += DDR5GlueRCDChannel(phy_pads, pads_B, "B_")

        data_pads_A = DDR5RCD01DataBufferSimulationPads(
            dq_w    = len(phy_pads.A_dq),
            cb_w    = 0,
            dqs_w   = len(phy_pads.A_dqs_t),
        )
        self.submodules += DDR5GlueRCDDataBuffer(phy_pads, data_pads_A, "A_")

        data_pads_B = DDR5RCD01DataBufferSimulationPads(
            dq_w    = len(phy_pads.B_dq),
            cb_w    = 0,
            dqs_w   = len(phy_pads.B_dqs_t),
        )
        self.submodules += DDR5GlueRCDDataBuffer(phy_pads, data_pads_B, "B_")


        xRCDSystem = DDR5RCD01System(
            pads_ingress_dq_A   = data_pads_A,
            pads_ingress_dq_B   = data_pads_B,
            pads_ingress_A      = pads_A,
            pads_ingress_B      = pads_B,
            pads_ingress_common = common_pads,
            pads_sideband   = _pads_sideband,
            sideband_type   = sideband_type,
            rcd_passthrough = rcd_passthrough,
        )
        self.submodules += xRCDSystem

        self.A_DRAM_pads = xRCDSystem.pads_egress_dq_A
        self.B_DRAM_pads = xRCDSystem.pads_egress_dq_B
