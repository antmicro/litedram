#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# LiteDRAM : RCD
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads


class DDR5RCD01DataBufferShell(Module):
    """
    DRAM Data bus pass-through
    """

    def __init__(self, pads_ingress, **kwargs):
        # self.submodules.pads_ingress = pads_ingress

        self.pads_egress = DDR5RCD01DataBufferSimulationPads()
        # self.submodules.pads_egress = pads_egress

        self.comb += self.pads_egress.dq.eq(pads_ingress.dq)
        self.comb += self.pads_egress.cb.eq(pads_ingress.cb)
        self.comb += self.pads_egress.dqs_t.eq(pads_ingress.dqs_t)
        self.comb += self.pads_egress.dqs_c.eq(pads_ingress.dqs_c)


if __name__ == "__main__":
    pads_ingress = DDR5RCD01DataBufferSimulationPads()
    xShell = DDR5RCD01DataBufferShell(
        pads_ingress=pads_ingress
    )
