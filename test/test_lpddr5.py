#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
from typing import Mapping
from collections import defaultdict

from migen import *

from litedram.phy.lpddr5.simphy import LPDDR5SimPHY

from test.phy_common import DFISequencer, PadChecker, run_simulation as _run_simulation

# Clocks are set up such that the first rising edge is on tic 1 (not 0), just as in test_lpddr4.
CLOCKS = {
    "sys":          (64, 31),
    "sys2x":        (32, 15),
    "sys4x":        (16,  7),
    "sys4x_90":     (16,  3),
    "sys8x":        ( 8,  3),
    "sys8x_ddr":    ( 4,  1),
    "sys8x_90":     ( 8,  1),
    "sys8x_90_ddr": ( 4,  3),
}

def run_simulation(dut, generators, **kwargs):
    _run_simulation(dut, generators, CLOCKS, **kwargs)


class LPDDR5Tests(unittest.TestCase):
    SYS_CLK_FREQ = 100e6

    def run_test(self, dut, dfi_sequence, pad_checkers: Mapping[str, Mapping[str, str]], pad_generators=None, **kwargs):
        # pad_checkers: {clock: {sig: values}}
        dfi = DFISequencer(dfi_sequence)
        checkers = {clk: PadChecker(dut.pads, pad_signals) for clk, pad_signals in pad_checkers.items()}
        generators = defaultdict(list)
        generators["sys"].append(dfi.generator(dut.dfi))
        generators["sys"].append(dfi.reader(dut.dfi))
        for clock, checker in checkers.items():
            generators[clock].append(checker.run())
        pad_generators = pad_generators or {}
        for clock, gens in pad_generators.items():
            gens = gens if isinstance(gens, list) else [gens]
            for gen in gens:
                generators[clock].append(gen(dut.pads))
        run_simulation(dut, generators, **kwargs)
        PadChecker.assert_ok(self, checkers)
        dfi.assert_ok(self)

    def test_lpddr5_cs_phase_0(self):
        # Test that CS is serialized correctly when sending command on phase 0
        self.run_test(LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ),
            dfi_sequence = [
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # p0: READ
            ],
            pad_checkers = {"sys4x_90": {
                'cs': '0000 0000 1100',
            }},
            vcd_name='sim.vcd'
        )

