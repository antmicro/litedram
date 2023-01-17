#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# LiteDRAM : RCD
from litedram.DDR5RCD01.DDR5RCD01Chip import DDR5RCD01Chip
from litedram.DDR5RCD01.DDR5RCD01DataBuffer import DDR5RCD01DataBuffer
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01CoreEgressSimulationPads import DDR5RCD01CoreEgressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01SidebandSimulationPads import DDR5RCD01SidebandSimulationPads
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads
from litedram.DDR5RCD01.DDR5RCD01Shell import DDR5RCD01Shell

from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *


class DDR5RCD01System(Module):
    """The DDR5 RCD01 System encapsulates:
        - the RCD chip
        - the Data Buffer chips
    The System may be configured for RDIMM or LRDIMM type.
    In the RDIMM mode BCOM is unused and Data Buffer signals
    are passed through. The LRDIMM is not yet implemented.
    TODO enable BCOM support
    TODO attach a data buffer model

    The system is structured as follows:
    System:
      -> RCD Shell or RCD Chip
      -> Data Buffer Shell or Data Buffer Chip

    The "shell" is a view, which only implementes pass-through function.
    The "chip" is a view, which implements the physical function.
    """

    def __init__(self,
                 pads_ingress_dq,
                 pads_ingress_A,
                 pads_ingress_B,
                 pads_ingress_common,
                 pads_sideband,
                 rcd_passthrough=True,
                 sideband_type=sideband_type.I2C,
                 ):

        self.submodules += pads_ingress_dq
        self.submodules += pads_ingress_A
        self.submodules += pads_ingress_common
        self.submodules += pads_sideband

        pads_egress_dq = DDR5RCD01DataBufferSimulationPads()
        pads_egress_A = DDR5RCD01CoreEgressSimulationPads()
        self.submodules += pads_egress_dq
        self.submodules += pads_egress_A

        if pads_ingress_B is not None:
            pads_egress_B = DDR5RCD01CoreEgressSimulationPads()
            self.submodules += pads_ingress_B
            self.submodules += pads_egress_B

        if rcd_passthrough == True:
            if pads_ingress_B is not None:
                xRCD = DDR5RCD01Shell(
                    pads_ingress_A=pads_ingress_A,
                    pads_ingress_B=pads_ingress_B,
                    pads_ingress_common=pads_ingress_common,
                    pads_sideband=pads_sideband,
                )
            else:
                xRCD = DDR5RCD01Shell(
                    pads_ingress_A=pads_ingress_A,
                    pads_ingress_B=None,
                    pads_ingress_common=pads_ingress_common,
                    pads_sideband=pads_sideband,
                )
        else:
            if pads_ingress_B is not None:
                xRCD = DDR5RCD01Chip(
                    pads_ingress_A=pads_ingress_A,
                    pads_ingress_B=pads_ingress_B,
                    pads_ingress_common=pads_ingress_common,
                    pads_sideband=pads_sideband,
                )
            else:
                xRCD = DDR5RCD01Chip(
                    pads_ingress_A=pads_ingress_A,
                    pads_ingress_B=None,
                    pads_ingress_common=pads_ingress_common,
                    pads_sideband=pads_sideband,
                )
        self.submodules += xRCD
        # self.submodules += xRCD.pads_egress

        # Data Buffer
        xDB = DDR5RCD01DataBuffer(
            pads_ingress=pads_ingress_dq,
            dimm_type=dimm_type.RDIMM
        )
        self.submodules += xDB
        # self.submodules += xDB.pads_egress


# if __name__ == "__main__":
    # pads_ingress_dq = DDR5RCD01DataBufferSimulationPads()
    # pads_ingress_A = DDR5RCD01ChannelIngressSimulationPads()
    # pads_ingress_B = DDR5RCD01ChannelIngressSimulationPads()
    # pads_ingress_common = DDR5RCD01CommonIngressSimulationPads()
    # pads_sideband = DDR5RCD01SidebandSimulationPads()

    # xSystem_dc = DDR5RCD01System(
    #     pads_ingress_dq=pads_ingress_dq,
    #     pads_ingress_A=pads_ingress_A,
    #     pads_ingress_B=pads_ingress_B,
    #     pads_ingress_common=pads_ingress_common,
    #     pads_sideband=pads_sideband,
    #     rcd_passthrough=True,
    #     sideband_type=sideband_type.I2C,
    # )

    # xSystem_sc = DDR5RCD01System(
    #     pads_ingress_dq=pads_ingress_dq,
    #     pads_ingress_A=pads_ingress_A,
    #     pads_ingress_B=None,
    #     pads_ingress_common=pads_ingress_common,
    #     pads_sideband=pads_sideband,
    #     rcd_passthrough=True,
    #     sideband_type=sideband_type.I2C,
    # )

    # xSystem_Core_dc = DDR5RCD01System(
    #     pads_ingress_dq=pads_ingress_dq,
    #     pads_ingress_A=pads_ingress_A,
    #     pads_ingress_B=pads_ingress_B,
    #     pads_ingress_common=pads_ingress_common,
    #     pads_sideband=pads_sideband,
    #     rcd_passthrough=False,
    #     sideband_type=sideband_type.I2C,
    # )

    # xSystem_Core_sc = DDR5RCD01System(
    #     pads_ingress_dq=pads_ingress_dq,
    #     pads_ingress_A=pads_ingress_A,
    #     pads_ingress_B=None,
    #     pads_ingress_common=pads_ingress_common,
    #     pads_sideband=pads_sideband,
    #     rcd_passthrough=False,
    #     sideband_type=sideband_type.I2C,
    # )

class TestBed(Module):
    def __init__(self):
        self.pads_ingress_dq = DDR5RCD01DataBufferSimulationPads()
        self.pads_ingress_A = DDR5RCD01ChannelIngressSimulationPads()
        self.pads_ingress_B = DDR5RCD01ChannelIngressSimulationPads()
        self.pads_ingress_common = DDR5RCD01CommonIngressSimulationPads()
        self.pads_sideband = DDR5RCD01SidebandSimulationPads()

        xSystem_dc = DDR5RCD01System(
            pads_ingress_dq=self.pads_ingress_dq,
            pads_ingress_A=self.pads_ingress_A,
            pads_ingress_B=self.pads_ingress_B,
            pads_ingress_common=self.pads_ingress_common,
            pads_sideband=self.pads_sideband,
            rcd_passthrough=True,
            sideband_type=sideband_type.I2C,
        )
        self.submodules.dut = xSystem_dc

def run_test(tb):
    logging.debug('Write test')
    for i in range(5):
        yield

if __name__ == "__main__":  
    eT = EngTest(level=logging.INFO)
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready. Simulating with migen...")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
