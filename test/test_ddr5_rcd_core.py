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
from test.CRG import CRG


class TestBed(Module):
    def __init__(self, is_dual_channel=False):
        RESET_TIME = 1
        self.clocks = {
            "sys":      (128, 63),
            "sysx2":    (64, 31),
            "sys_rst":  (128, 63+4),
        }
        self.submodules.xcrg = CRG(
            clocks=self.clocks,
            reset_cnt=RESET_TIME
        )
        self.generators = {}

        """
            Items on the bed
        """
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

        self.submodules.xenvironment = ClockDomainsRenamer("sys")(
            BusCSCAEnvironment(
                if_ibuf_o=self.if_ibuf_A,
            )
        )

        self.submodules.xmonitor_ingress = ClockDomainsRenamer("sys")(
            BusCSCAMonitor(
                if_ibuf_i=self.if_ibuf_A,
                is_sim_finished=self.xenvironment.agent.sequencer.is_sim_finished
            )
        )

        self.submodules.xrcd_core = ClockDomainsRenamer("sys")(
            DDR5RCD01Core(
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
        )

        self.if_bus_csca_o = If_bus_csca_o()
        self.comb += self.if_bus_csca_o.qcs_n.eq(self.if_obuf_A.qacs_a_n)
        self.comb += self.if_bus_csca_o.qca.eq(self.if_obuf_A.qaca_a)

        self.submodules.xmonitor_egress = ClockDomainsRenamer("sys")(
            BusCSCAMonitor(
                if_ibuf_i=self.if_bus_csca_o,
                is_sim_finished=self.xenvironment.agent.sequencer.is_sim_finished
            )
        )

        """

        """

        """
            Generators
        """
        self.add_generators(
            self.generators_dict()
        )

    def generators_dict(self):
        return {
            "sys":
            [
                self.xenvironment.run_env(
                    scenario_select=EnvironmentScenarios.SIMPLE_GENERIC),
                self.xmonitor_ingress.monitor(),
                self.xmonitor_egress.monitor(),
            ]
        }

    def add_generators(self, generators):
        for key, value in generators.items():
            if key not in self.generators:
                self.generators[key] = list()
            if not isinstance(value, list):
                value = list(value)
            self.generators[key].extend(value)

    def run_test(self):
        return self.generators


class DDR5RCD01CoreTests_SingleChannel(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed(is_dual_channel=False)
        """
            Waveform file
        """
        dir_name = "./wave_ut"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"
        """
            Logging
        """
        LOG_FILE_NAME = dir_name + '/' + file_name + ".log"
        FORMAT = "[%(module)s.%(funcName)s] %(message)s"
        fileHandler = logging.FileHandler(filename=LOG_FILE_NAME,mode='w')
        fileHandler.formatter = logging.Formatter(FORMAT)
        streamHandler = logging.StreamHandler()

        logger = logging.getLogger('root')
        logger.addHandler(fileHandler)
        logger.addHandler(streamHandler)
        logger.setLevel(logging.DEBUG)


    def tearDown(self):
        del self.tb

    def test_core(self):
        logger = logging.getLogger('root')
        logger.debug("-"*80)
        run_simulation(
            self.tb,
            generators=self.tb.run_test(),
            clocks=self.tb.xcrg.clocks,
            vcd_name=self.wave_file_name
        )
        # breakpoint()
        self.tb.xmonitor_ingress.post_process()
        self.tb.xmonitor_egress.post_process()        
        
        logger.debug(str(self.tb.xmonitor_ingress.monitor_q))
        logger.debug(str(self.tb.xmonitor_egress.monitor_q))
        # breakpoint()
        assert 1 == 1



if __name__ == '__main__':
    unittest.main()
