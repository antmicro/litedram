#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# LiteDRAM : MODULE PADS
from litedram.phy.ddr5.simphy import DDR5SimulationPads
# LiteDRAM : RCD
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads

from litedram.DDR5RCD01.DDR5GlueRCD import DDR5GlueRCDCommon, DDR5GlueRCDChannel, DDR5GlueRCDDataBuffer
from litedram.DDR5RCD01.RCDGlueDDR5 import RCDGlueDDR5Channel, RCDGlueDDR5DataBuffer

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

        len_dq_A  = len(phy_pads.A_dq)
        len_cb_A  = len(phy_pads.A_cb) if hasattr(phy_pads, 'A_cb') else 0
        len_dqs_A = len(phy_pads.A_dqs_t)
        data_pads_A = DDR5RCD01DataBufferSimulationPads(
            dq_w  = len_dq_A,
            cb_w  = len_cb_A,
            dqs_w = len_dqs_A,
        )
        self.submodules += DDR5GlueRCDDataBuffer(phy_pads, data_pads_A, "A_")

        len_dq_B  = len(phy_pads.B_dq)
        len_cb_B  = len(phy_pads.B_cb) if hasattr(phy_pads, 'B_cb') else 0
        len_dqs_B = len(phy_pads.B_dqs_t)
        data_pads_B = DDR5RCD01DataBufferSimulationPads(
            dq_w  = len_dq_B,
            cb_w  = len_cb_B,
            dqs_w = len_dqs_B,
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
        self.submodules.xRCDSystem = xRCDSystem

        quarters = {
            "A": "front_top",
            "B": "front_bottom",
            "C": "back_top",
            "D": "back_bottom",
        }
        quarter_to_offset = {
            "A": 1,
            "B": 0,
            "C": 1,
            "D": 0,
        }

        dq_dqs_ratio_A = (len_dq_A + len_cb_A)//len_dqs_A
        dq_dqs_ratio_B = (len_dq_B + len_cb_B)//len_dqs_B
        constans = {
            "A_": (len_dq_A, dq_dqs_ratio_A, xRCDSystem.pads_egress_A, xRCDSystem.pads_egress_dq_A),
            "B_": (len_dq_B, dq_dqs_ratio_B, xRCDSystem.pads_egress_B, xRCDSystem.pads_egress_dq_B),
        }

        for prefix in ["A_", "B_"]:
            dq, dq_dqs_ratio, egress, egress_dq = constans[prefix]
            top = False
            if (dq//dq_dqs_ratio) % 2 == 0:
                dq /= 2
                top = True
            pads = []
            for quarter, val in quarters.items():
                if "top" in val and top:
                    setattr(self, prefix+val,
                        DDR5SimulationPads(
                            databits=dq,
                            dq_dqs_ratio=dq_dqs_ratio,
                        )
                    )
                    pads.append(getattr(self, prefix+val))
                else:
                    setattr(self, prefix+val,
                        DDR5SimulationPads(
                            databits=dq,
                            dq_dqs_ratio=dq_dqs_ratio,
                        )
                    )
                    pads.append((quarter, getattr(self, prefix+val)))
            for quarter, pad in pads:
                self.submodules += RCDGlueDDR5Channel(egress, pad, quarter)
                self.submodules += RCDGlueDDR5DataBuffer(
                    egress_dq,
                    pad,
                    interleave=top,
                    offset=quarter_to_offset[quarter]
                )
            self.comb += egress.derror_in_n.eq(reduce(or_, [pad[1].alert_n for pad in pads]))
