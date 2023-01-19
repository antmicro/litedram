#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *


class DDR5RCD01ModuleTemplate(Module):
    """
        DDR5 RCD01 Module Template

        Module
        ------


        Parameters
        ------
    """

    def __init__(self):
        pass


class TestBed(Module):
    def __init__(self):
        self.submodules.dut = DDR5RCD01ModuleTemplate()


def run_test(tb):
    logging.debug('Write test')
    for i in range(5):
        yield
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
