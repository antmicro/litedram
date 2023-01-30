#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
from operator import xor
from dataclasses import dataclass
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
#
from litedram.DDR5RCD01.BusCSCAEnvironment import BusCSCAEnvironment
from litedram.DDR5RCD01.BusCSCAEnvironment import EnvironmentScenarios
from litedram.DDR5RCD01.monitor_definitions import *


class BusCSCAScoreboard(Module):
    """
        DDR5 RCD01 Scoreboard

        Compare queues of 2 monitors. Currently meant for DDR and 1N

        Module
        ------

        Parameters
        ----------

    """

    def __init__(self, q1, q2):
        # self.q1 = q1.q
        # self.q2 = q2.q
        self.q1 = q1.q_filter_inactive
        self.q2 = q2.q_filter_inactive

        # assert self.compare()

    def compare(self):
        assert len(self.q1) == len(self.q2)
        #  TODO come back here, all of this is pseudo-code
        for id, item in enumerate(self.q1):
            """
                Compare
                    UI length:
                        DDR is 2x smaller than 1N
            """
            assert len(self.q1[id].cmd) == len(self.q2[id].cmd)
            """
            
                    CSCA Values
                        DCS: ==
                        DCA: 7bit to 14 bit compare

                        DPAR is not in output bus, so will be ignored
            """
            dca_1 = self.q1[id].cmd[0].dca
            dca_2 = self.q2[id].cmd[0].dca
            assert ((dca_1 << 7) & (dca_1)) != (dca_2)
            """        
                    cmd_type should match
            """
            assert self.q1[id].cmd[0].cmd_type == self.q2[id].cmd[0].cmd_type

            print(self.q1[id].cmd)
            print(self.q2[id].cmd)
            breakpoint()
        return True


class TestBed(Module):
    def __init__(self):
        pass


def run_test(tb):
    logging.debug('Write test')
    # scenario_select = EnvironmentScenarios.SIMPLE_GENERIC
    # yield from tb.env.run_env(scenario_select=scenario_select)
    # logging.debug(str(tb.monitor.monit_q))
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
