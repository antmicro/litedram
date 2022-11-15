#
# This file is part of LiteDRAM.
#
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest
import itertools
import collections

from migen import *

from litex.soc.interconnect import stream

from litedram.common import *
from test.rcd_common import RCDCS

from test.common import timeout_generator, CmdRequestRWDriver


class CSLogicDUT(Module):
    def __init__(self, **kwargs):
        self.submodules.cs = RCDCS(**kwargs)


class TestRCDCS(unittest.TestCase):
    def test_cs_lock(self):
        # Verify that CS logic allows for CA pass only if it's not locked by parity error
        def test():
            def main_generator(dut):
                yield dut.cs.rc_access.eq(1) # no parity
                yield dut.cs.cs_n.eq(0b11) # no rank selected
                yield
                self.assertEqual((yield dut.cs.ca_ce), 0b0) # block
                yield dut.cs.cs_n.eq(0b00) # both ranks selected
                yield
                self.assertEqual((yield dut.cs.ca_ce), 0b0) # block
                yield dut.cs.rc_access.eq(0) # ok parity
                yield
                self.assertEqual((yield dut.cs.ca_ce), 0b1) # pass
                yield dut.cs.cs_n.eq(0b11) # no chips selected - should block anyway?
                yield
                self.assertEqual((yield dut.cs.ca_ce), 0b0) # block

            dut = CSLogicDUT()
            run_simulation(dut, main_generator(dut))

        test()
