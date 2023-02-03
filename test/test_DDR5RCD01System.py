#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python tools
import unittest
from functools import partial
from typing import Mapping
from collections import defaultdict
import sys
# Simulator
from migen import *
# from migen.fhdl import verilog
from migen.fhdl.verilog import convert

# LiteDRAM
from litedram.common import *
from litedram.phy.ddr5.simphy import DDR5SimPHY
from litedram.phy.ddr5.simsoc import DDR5Sim
# from phy.utils import
from litedram.phy.utils import Serializer, Deserializer

import test.phy_common
from test.phy_common import DFISequencer, PadChecker


# sys.path.insert(1, '/home/mczyz/workspace/rowhammer-tester/third_party/litedram/litedram/')
# sys.path.insert(1, '/home/mczyz/workspace/rowhammer-tester/third_party/litedram/litedram/test')
# /home/mczyz/workspace/rowhammer-tester/third_party/litedram/test/phy_common.py
# from ....test.phy_common import DFISequencer, PadChecker
# import ....test.phy_common
# import test.phy_common
# import test.phy_common

# RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
# Submodules
from litedram.DDR5RCD01.DDR5RCD01System import DDR5RCD01System
from litedram.DDR5RCD01.DDR5GlueRCD import DDR5GlueRCD
from litedram.DDR5RCD01.DDR5GlueRCDData import DDR5GlueRCDData
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01SidebandSimulationPads import DDR5RCD01SidebandSimulationPads
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads

"""
    Simulation clocks
"""
sim_clocks = {
    "sys":            (64, 31),
    "sys_rst":        (64, 30),
    "sys2x":          (32, 15),
    "sys4x":          (16,  7),
    "sys4x_ddr":      (8,  3),
    "sys4x_90":       (16,  3),
    "sys4x_90_ddr":   (8,  7),
    "sys4x_180":      (16, 15),
    "sys4x_180s_ddr": (8,  5),
}
run_simulation = partial(test.phy_common.run_simulation, clocks=sim_clocks)


class TestBed(Module):
    """
        DDR System Test Bed

        Testbed setup
        -------------

        The testbench contains:

            1. DDR5 command sequencer (phy, dfi) [HOST]

            2. DDR5 RCD 01 system [RDIMM]

            3. SDRAM model [RDIMM]


        ---------------  CS/CA BUS    -------    CS/CA BUS  ---------
        |DDR5Commands | ------------> | RCD | ------------> | SDRAM |
        ---------------               -------               ---------
                    |                                           ^
                    |                 DATA BUS                  |
                    ---------------------------------------------

        Note, the Data Bus is routed through the RCD System to create
        a unified environment for both RDIMM and LRDIMM solutions;
        however, in the RDIMM use case the signals should appear
        unchanged on the egress.

        DDR5 Command Sequencer
        ----------------------

        This device can theoretically be derived from a full host model,
        a memory controller model or a PHY model. In this implementation,
        the DFISequencer connected to a SimPHY was selected to create
        a simple and fast test environment.

        DFISequence -> DFISerializer -> SimPHY -- to RCD ->
                                            -- to RAM ->


        DDR5 RCD 01 System
        ------------------

        In the beginning phase of the project, a shell view was used to
        validate the test bed. The 'shell' refers to the fact that no
        internal logic or features are yet implemented and I/O signals
        are passed through the block. As development progresses, it
        shall be replaced by the RCD model.

        SDRAM model
        -----------

        A simulation view which is capable of receiving DDR5 commands and
        interacting with the ADR/CMD and DATA bus is neccessary to validate
        the RCD model.
    """

    def __init__(self):
        """
            Phy
        """
        self.SYS_CLK_FREQ = 50e6
        self.DATABITS = 8
        self.BURST_LENGTH = 8
        self.NPHASES = 4
        self.DQ_DQS_RATIO=4

        self.xPHY = DDR5SimPHY(
            sys_clk_freq=self.SYS_CLK_FREQ,
            aligned_reset_zero=True,
            masked_write=False,
            dq_dqs_ratio=self.DQ_DQS_RATIO,
            with_sub_channels=False,
            direct_control=False,
        )
        self.submodules += self.xPHY

        # CLK for sdram module, originates from phy
        setattr(self.clock_domains, "cd_sys4x_p_dimm",
                ClockDomain("sys4x_p_dimm"))
        setattr(self.clock_domains, "cd_sys4x_n_dimm",
                ClockDomain("sys4x_n_dimm"))
        self.comb += [
            ClockSignal("sys4x_p_dimm").eq(self.ddrphy.pads.ck_t),
            ResetSignal("sys4x_p_dimm").eq(~self.ddrphy.pads.reset_n),
            ClockSignal("sys4x_n_dimm").eq(self.ddrphy.pads.ck_c),
            ResetSignal("sys4x_n_dimm").eq(~self.ddrphy.pads.reset_n),
        ]


        """
            RCD System
        """
        self.pads_ingress_dq = DDR5RCD01DataBufferSimulationPads()
        self.pads_ingress_A = DDR5RCD01ChannelIngressSimulationPads()
        self.pads_ingress_B = DDR5RCD01ChannelIngressSimulationPads()
        self.pads_ingress_common = DDR5RCD01CommonIngressSimulationPads()
        self.pads_sideband = DDR5RCD01SidebandSimulationPads()

        xRCDSystem = DDR5RCD01System(
            pads_ingress_dq=self.pads_ingress_dq,
            pads_ingress_A=self.pads_ingress_A,
            pads_ingress_B=self.pads_ingress_B,
            pads_ingress_common=self.pads_ingress_common,
            pads_sideband=self.pads_sideband,
            rcd_passthrough=True,
            sideband_type=sideband_type.I2C,
        )
        self.submodules += xRCDSystem

        """
            SDRAM
        """



        xDRAM = DDR5Sim()
        self.submodules += xDRAM

        """
            PHY 2 RCD Glue
        """
        self.glue = DDR5GlueRCD(self.phy.pads)

        """
            RCD
        """
        pads_ingress_dq = DDR5RCD01DataBufferSimulationPads()
        pads_ingress_A = DDR5RCD01ChannelIngressSimulationPads()
        pads_ingress_B = DDR5RCD01ChannelIngressSimulationPads()
        pads_ingress_common = DDR5RCD01CommonIngressSimulationPads()
        pads_sideband = DDR5RCD01SidebandSimulationPads()

        xSystem_dc = DDR5RCD01System(
            pads_ingress_dq=pads_ingress_dq,
            pads_ingress_A=pads_ingress_A,
            pads_ingress_B=pads_ingress_B,
            pads_ingress_common=pads_ingress_common,
            pads_sideband=pads_sideband,
            rcd_passthrough=True,
            sideband_type=sideband_type.MOCK,
        )

        """
            RCD 2 SDRAM Glue
        """
        self.glue_data = DDR5GlueRCDData(self.rcd.pads_dram_data_egress)
        """
            SDRAM
        """
        # self.sdram = SDRAM()
        # self.submodules.sdram = SDRAM_model(**kwargs)
        # dut.connect(phy.pads) # Connect, alternatively
        # dut = RCD(phy.pads)

        # sdram_model = SDRAM() # Create SDRAM model
        # sdram_model.connect(dut.pads_o) # Connect

        # self.submodules.system = DDR5RCD01System(pads_sideband, self.phy.pads, rcd_passthrough=True, sideband_type='i2c',**kwargs)


class DDR5RDIMMIntegrationTests(unittest.TestCase):
    """
    DDR5RDIMMIntegrationTests
    -------------------------
    """

    def setUp(self):
        self.tb = TestBed()

    @staticmethod
    def process_ca(ca: str) -> int:
        """dfi_address is mapped 1:1 to CA"""
        ca = ca.replace(' ', '')  # remove readability spaces
        ca = ca[::-1]            # reverse bit order (also readability)
        return int(ca, 2)        # convert to int

    def run_test(self, dfi_sequence, pad_checkers: Mapping[str, Mapping[str, str]], pad_generators=None, **kwargs):
        """
            DFI, checkers, generators
        """
        # pad_checkers: {clock: {sig: values}}
        dfi = DFISequencer([{}, {}] + dfi_sequence)
        checkers = {clk: PadChecker(self.tb.phy.pads, pad_signals)
                    for clk, pad_signals in pad_checkers.items()}
        generators = defaultdict(list)
        generators["sys"].append(dfi.generator(self.tb.phy.dfi))
        generators["sys"].append(dfi.reader(self.tb.phy.dfi))
        for clock, checker in checkers.items():
            generators[clock].append(checker.run())
        pad_generators = pad_generators or {}
        for clock, gens in pad_generators.items():
            gens = gens if isinstance(gens, list) else [gens]
            for gen in gens:
                generators[clock].append(gen(self.tb.phy.pads))

        # Wrapper class to enable rst
        class CRG(Module):
            def __init__(self, dut):
                r = Signal(2)
                self.sync.sys_rst += [If(r < 3, r.eq(r+1))]
                self.submodules.dut = dut
                for clk in sim_clocks:
                    if clk == "sys_rst":
                        continue
                    setattr(self.clock_domains, "cd_{}".format(
                        clk), ClockDomain(clk))
                    cd = getattr(self, 'cd_{}'.format(clk))
                    self.comb += cd.rst.eq(~r[1])
        self.tb = CRG(self.tb)
        run_simulation(self.tb, generators, **kwargs)
        PadChecker.assert_ok(self, checkers)
        dfi.assert_ok(self)

    # --------------------------------------------------------------------------------
    # USE CASEs
    # --------------------------------------------------------------------------------

    def test_RCD_passthrough(self):
        """
            Test scenario
            -------------
            Verify that signals passthrough the RCD unchanged
        """

        """
            DFI Sequencer
        """
        dfi_data = [
            {
                0: dict(wrdata=0x1122),
                1: dict(wrdata=0x3344),
                2: dict(wrdata=0x5566),
                3: dict(wrdata=0x7788),
            },
            {
                0: dict(wrdata=0x99aa),
                1: dict(wrdata=0xbbcc),
                2: dict(wrdata=0xddee),
                3: dict(wrdata=0xff00),
            },
        ]

        write_0 = dict(cs_n=0, address=self.process_ca(
            '10110 0 00000 000'))  # WR p0
        write_1 = dict(cs_n=1, address=self.process_ca(
            '000000000 01100'))    # WR p1

        dfi_sequence = [
            {
                self.wrphase:     write_0 | dict(wrdata_en=1),
                self.wrphase + 1: write_1,
            },
            {
                self.wrphase:     write_0 | dict(wrdata_en=1),
                self.wrphase + 1: write_1,
            },
            *[{} for _ in range(self.write_latency - 2)],
            *dfi_data,
            {},
            {},
            {},
            {},
            {},
        ]
        """
            Run test
        """
        self.run_test(
            dfi_sequence=dfi_sequence,
            pad_checkers={},
            pad_generators={},
            vcd_name="DDR5RCD01System_passthrough.vcd"
        )
