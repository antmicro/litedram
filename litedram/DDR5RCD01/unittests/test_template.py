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

class TestBed(Module):
    def __init__(self):
        # DUT
        pass

    def scenario(self):
        yield


class DDR5RCD01System(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed()
        dir_name = "./wave"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"

    def tearDown(self):
        del self.tb

    def test_template(self):
        run_simulation(self.tb,
                       self.tb.scenario(),
                       vcd_name=self.wave_file_name
                       )


if __name__ == '__main__':
    unittest.main()
