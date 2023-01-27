#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import unittest
import logging
import subprocess
# migen
from migen import *
from migen.fhdl import verilog
# RCD
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
# Submodules


class TestBed(Module):
    def __init__(self):
        pass

    def scenario(self, py):
        try:
            subprocess.check_output(
                "python ../"+py, shell=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            if "UnderConstruction" in str(e.output):
                raise UnderConstruction
            if "NotSupportedException" in str(e.output):
                return
            raise ValueError


class DDR5RCD01System(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed()
        dir_name = "./compile"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"

    def tearDown(self):
        del self.tb

    def test_BusCSCAAgent(self):
        self.tb.scenario("BusCSCAAgent.py")

    def test_BusCSCACommand(self):
        self.tb.scenario("BusCSCACommand.py")

    def test_BusCSCADriver(self):
        self.tb.scenario("BusCSCADriver.py")

    def test_BusCSCAEnvironment(self):
        self.tb.scenario("BusCSCAEnvironment.py")

    def test_BusCSCAMonitor(self):
        self.tb.scenario("BusCSCAMonitor.py")

    def test_BusCSCASequencer(self):
        self.tb.scenario("BusCSCASequencer.py")

    def test_DDR5GlueRCDData(self):
        self.tb.scenario("DDR5GlueRCDData.py")

    def test_DDR5GlueRCD(self):
        self.tb.scenario("DDR5GlueRCD.py")

    def test_DDR5RCD01ActorMRR(self):
        self.tb.scenario("DDR5RCD01ActorMRR.py")

    def test_DDR5RCD01ActorMRW(self):
        self.tb.scenario("DDR5RCD01ActorMRW.py")

    def test_DDR5RCD01Actor(self):
        self.tb.scenario("DDR5RCD01Actor.py")

    def test_DDR5RCD01Alert(self):
        self.tb.scenario("DDR5RCD01Alert.py")

    def test_DDR5RCD01BCOMSimulationPads(self):
        self.tb.scenario("DDR5RCD01BCOMSimulationPads.py")

    def test_DDR5RCD01ChannelIngressSimulationPads(self):
        self.tb.scenario("DDR5RCD01ChannelIngressSimulationPads.py")

    def test_DDR5RCD01Channel(self):
        self.tb.scenario("DDR5RCD01Channel.py")

    def test_DDR5RCD01Chip(self):
        self.tb.scenario("DDR5RCD01Chip.py")

    def test_DDR5RCD01CommonIngressSimulationPads(self):
        self.tb.scenario("DDR5RCD01CommonIngressSimulationPads.py")

    def test_DDR5RCD01Common(self):
        self.tb.scenario("DDR5RCD01Common.py")

    def test_DDR5RCD01ControlCenter(self):
        self.tb.scenario("DDR5RCD01ControlCenter.py")

    def test_DDR5RCD01CoreEgressSimulationPads(self):
        self.tb.scenario("DDR5RCD01CoreEgressSimulationPads.py")

    def test_DDR5RCD01Core(self):
        self.tb.scenario("DDR5RCD01Core.py")

    def test_DDR5RCD01CoreWrapper(self):
        self.tb.scenario("DDR5RCD01CoreWrapper.py")

    def test_DDR5RCD01CSLogic(self):
        self.tb.scenario("DDR5RCD01CSLogic.py")

    def test_DDR5RCD01DataBufferChip(self):
        self.tb.scenario("DDR5RCD01DataBufferChip.py")

    def test_DDR5RCD01DataBuffer(self):
        self.tb.scenario("DDR5RCD01DataBuffer.py")

    def test_DDR5RCD01DataBufferShell(self):
        self.tb.scenario("DDR5RCD01DataBufferShell.py")

    def test_DDR5RCD01DataBufferSimulationPads(self):
        self.tb.scenario("DDR5RCD01DataBufferSimulationPads.py")

    def test_DDR5RCD01Decoder(self):
        self.tb.scenario("DDR5RCD01Decoder.py")

    def test_DDR5RCD01Error(self):
        self.tb.scenario("DDR5RCD01Error.py")

    def test_DDR5RCD01FetchDecode(self):
        self.tb.scenario("DDR5RCD01FetchDecode.py")

    def test_DDR5RCD01InputBuffer(self):
        self.tb.scenario("DDR5RCD01InputBuffer.py")

    def test_DDR5RCD01LineBuffer(self):
        self.tb.scenario("DDR5RCD01LineBuffer.py")

    def test_DDR5RCD01Loopback(self):
        self.tb.scenario("DDR5RCD01Loopback.py")

    def test_DDR5RCD01ModuleTemplate(self):
        self.tb.scenario("DDR5RCD01ModuleTemplate.py")

    def test_DDR5RCD01OutBuf(self):
        self.tb.scenario("DDR5RCD01OutBuf.py")

    def test_DDR5RCD01OutputBuffer_CLKS(self):
        self.tb.scenario("DDR5RCD01OutputBuffer_CLKS.py")

    def test_DDR5RCD01OutputBuffer_CSCA(self):
        self.tb.scenario("DDR5RCD01OutputBuffer_CSCA.py")

    def test_DDR5RCD01Page(self):
        self.tb.scenario("DDR5RCD01Page.py")

    def test_DDR5RCD01Pages(self):
        self.tb.scenario("DDR5RCD01Pages.py")

    def test_DDR5RCD01PLL(self):
        self.tb.scenario("DDR5RCD01PLL.py")

    def test_DDR5RCD01RankBuffer(self):
        self.tb.scenario("DDR5RCD01RankBuffer.py")

    def test_DDR5RCD01RegFile(self):
        self.tb.scenario("DDR5RCD01RegFile.py")

    def test_DDR5RCD01RowBuffer(self):
        self.tb.scenario("DDR5RCD01RowBuffer.py")

    def test_DDR5RCD01Shell(self):
        self.tb.scenario("DDR5RCD01Shell.py")

    def test_DDR5RCD01SidebandSimulationPads(self):
        self.tb.scenario("DDR5RCD01SidebandSimulationPads.py")

    def test_DDR5RCD01System(self):
        self.tb.scenario("DDR5RCD01System.py")

    def test_I2CSlave(self):
        self.tb.scenario("I2CSlave.py")

    def test_I3CSlave(self):
        self.tb.scenario("I3CSlave.py")

    def test___init__(self):
        self.tb.scenario("__init__.py")

    def test_RCD_definitions(self):
        self.tb.scenario("RCD_definitions.py")

    def test_RCD_interfaces_external(self):
        self.tb.scenario("RCD_interfaces_external.py")

    def test_RCD_interfaces(self):
        self.tb.scenario("RCD_interfaces.py")

    def test_RCD_utils(self):
        self.tb.scenario("RCD_utils.py")

    def test_SidebandMock(self):
        self.tb.scenario("SidebandMock.py")

    def test_SimCMDs(self):
        self.tb.scenario("SimCMDs.py")

    def test_SimCSCADriver(self):
        self.tb.scenario("SimCSCADriver.py")


if __name__ == '__main__':
    unittest.main()
