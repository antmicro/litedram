#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
import random
from collections import namedtuple
from operator import xor
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.BusCSCAAgent import BusCSCAAgent
from litedram.DDR5RCD01.BusCSCACommand import *


Payload = namedtuple('payload', ['mra', 'op', 'cw'])


@enum.unique
class EnvironmentScenarios(enum.IntEnum):
    NONE = 0
    TEST_CW_WR_RD = 1
    SIMPLE_GENERIC = 2


class BusCSCAEnvironment(Module):
    """
        Scenarios:
            Single MRW/MRR
            Pattern MRWs/MRRs
            Random Valid
            Random Random

        Write options:
            Randomized
            Range (i,i+i,...)
            Fixed

        # Perform MRWs from addr 23,46
        ["MRW_2_RCD_PATTERN_ITER", range=[23,46], payload="random|fixed|range"]
        # Perform MRWs at random addresses
        ["MRW_2_RCD_PATTERN_RND", n=20 writes, payload="random|fixed|range"]
        ["MRW_2_RCD_SINGLE", dest_register=0x5E, dest_val=0xFF]
        ["MRW_2_DRAM_SINGLE", 3]
    """

    def __init__(self,
                 if_ibuf_o
                 ):
        self.queue = []
        xBusCSCAAgent = BusCSCAAgent(
            if_ibuf_o=if_ibuf_o,
        )
        self.submodules.agent = xBusCSCAAgent
        self.scenarios = []

    def run_env(self, scenario_select=EnvironmentScenarios.NONE):
        self.build_scenario(scenario_select=scenario_select)
        yield from self.agent.run_agent(self.queue)

    def build_scenario(self, scenario_select=EnvironmentScenarios.TEST_CW_WR_RD):
        if scenario_select == EnvironmentScenarios.TEST_CW_WR_RD:
            self.queue = self.scenario_cw_wr_rd(
                inactive_pre_len=100,
                inactive_inter_len=1,
                inactive_post_len=1,
                pattern="consecutive",
                addr_begin=0x12,
                pattern_len=4
            )
        elif scenario_select == EnvironmentScenarios.SIMPLE_GENERIC:
            self.queue = self.simple_generic(
                inactive_pre_len=100,
                inactive_inter_len=4,
                inactive_post_len=1,
                pattern_len=4
            )
        else:
            self.queue = []

    def simple_generic(self,
                       inactive_pre_len=5,
                       inactive_inter_len=1,
                       inactive_post_len=1,
                       pattern_len=4
                       ):
        scenario = []
        for i in range(inactive_pre_len):
            scenario += [BusCSCAInactive().cmd]

        for i in range(pattern_len):
            scenario += self.generic_mix()
            for j in range(inactive_inter_len):
                scenario += [BusCSCAInactive().cmd]

        for i in range(inactive_post_len):
            scenario += [BusCSCAInactive().cmd]

        return scenario

    def scenario_cw_wr_rd(self, inactive_pre_len=2, inactive_inter_len=2, inactive_post_len=2, pattern="consecutive", addr_begin=0, pattern_len=3):
        scenario = []
        for i in range(inactive_pre_len):
            scenario += [BusCSCAInactive().cmd]

        if pattern == "consecutive":
            for i in range(pattern_len):
                scenario += self.CW_read(rw_addr=addr_begin+i)
                for j in range(inactive_inter_len):
                    scenario += [BusCSCAInactive().cmd]
        elif pattern == "random":
            random_addrs = random.sample(list(range(0, 128)), pattern_len)
            for i in random_addrs:
                scenario += self.CW_read(rw_addr=i)
                for j in range(inactive_inter_len):
                    scenario += [BusCSCAInactive().cmd]

        for i in range(inactive_post_len):
            scenario += [BusCSCAInactive().cmd]

        return scenario

    def generic_mix(self):
        flow = []
        flow += [
            BusCSCAGeneric1().cmd,
            BusCSCAGeneric1A().cmd,
            BusCSCAGeneric1B().cmd,
            BusCSCAGeneric1AB().cmd,
            BusCSCAGeneric2().cmd,
            BusCSCAGeneric2A().cmd,
            BusCSCAGeneric2B().cmd,
            BusCSCAGeneric2AB().cmd,
        ]
        return flow

    def CW_read(self, rw_addr=0x00):
        """
            8.1 Reading Control Words
            Table 90 - Control Word Read Sequence
        """
        flow = []
        flow += [
            BusCSCAMRW(payload=Payload(0x5E, rw_addr, 0x1),
                       is_padded=True).cmd,
            BusCSCAMRW(payload=Payload(0x3F, 0x5A, 0x0), is_padded=True).cmd,
            BusCSCAMRR(payload=Payload(0x3F, 0x00, 0x0), is_padded=True).cmd,
        ]
        return flow


class TestBed(Module):
    def __init__(self):
        if_ibuf_o = If_ibuf()
        self.submodules.dut = BusCSCAEnvironment(
            if_ibuf_o=if_ibuf_o,
        )


def run_test(tb):
    logging.debug('Write test')
    scenario_select = EnvironmentScenarios.TEST_CW_WR_RD
    yield from tb.dut.run_env(scenario_select=scenario_select)
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
