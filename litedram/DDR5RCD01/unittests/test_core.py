#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import unittest
import logging
# migen
from migen import *
from migen.fhdl import verilog
# RCD
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
# Submodules
from litedram.DDR5RCD01.DDR5RCD01Core import DDR5RCD01Core
from litedram.DDR5RCD01.BusCSCAEnvironment import BusCSCAEnvironment
from litedram.DDR5RCD01.BusCSCAEnvironment import EnvironmentScenarios
from litedram.DDR5RCD01.BusCSCAMonitor import BusCSCAMonitor


class TestBed(Module):
    def __init__(self, is_dual_channel=False):
        self.if_ck_rst = If_ck_rst()
        self.if_sdram_A = If_sdram()
        self.if_sdram_B = If_sdram()
        self.if_alert_n = If_alert_n()
        self.if_ibuf_A = If_ibuf()
        self.if_ibuf_B = If_ibuf()
        self.if_obuf_A = If_obuf()
        self.if_obuf_B = If_obuf()
        self.if_lb = If_lb()
        self.if_bcom_A = If_bcom()
        self.if_bcom_B = If_bcom()
        self.if_sideband = If_sideband()
        self.is_dual_channel = is_dual_channel

        self.submodules.xenvironment = BusCSCAEnvironment(
            if_ibuf_o=self.if_ibuf_A,
        )

        self.submodules.xmonitor_ingress = BusCSCAMonitor(
            if_ibuf_i=self.if_ibuf_A
        )

        self.submodules.xrcd_core = DDR5RCD01Core(
            if_ck_rst=self.if_ck_rst,
            if_sdram_A=self.if_sdram_A,
            if_sdram_B=self.if_sdram_B,
            if_alert_n=self.if_alert_n,
            if_ibuf_A=self.if_ibuf_A,
            if_ibuf_B=self.if_ibuf_B,
            if_obuf_A=self.if_obuf_A,
            if_obuf_B=self.if_obuf_B,
            if_lb=self.if_lb,
            if_bcom_A=self.if_bcom_A,
            if_bcom_B=self.if_bcom_B,
            if_sideband=self.if_sideband,
            is_dual_channel=self.is_dual_channel,
        )

        self.submodules.xmonitor_egress = BusCSCAMonitor(
            if_ibuf_i=self.if_obuf_A
        )

    def scenario(self):
        scenario_select = EnvironmentScenarios.SIMPLE_GENERIC
        yield from self.xenvironment.run_env(scenario_select=scenario_select)
        yield from self.xmonitor_ingress.post_process()


class DDR5RCD01CoreTests_SingleChannel(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed(is_dual_channel=False)
        dir_name = "./wave_sc"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"


    def tearDown(self):
        del self.tb

    def test_core_compile(self):
        run_simulation(self.tb,
                       self.tb.scenario(),
                       vcd_name=self.wave_file_name
                       )


if __name__ == '__main__':
    unittest.main()
