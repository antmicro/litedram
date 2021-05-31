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
    "sys4x_ddr":    ( 8,  3), # = sys8x
    "sys4x_90":     (16,  3),
    "sys4x_90_ddr": ( 8,  7),
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
        phy = LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        latency = '0000' + '0' * phy.ser_latency.sys4x
        self.run_test(phy,
            dfi_sequence = [
                {0: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1)},  # p0: ACT
                {0: dict(cs_n=0, cas_n=1, ras_n=0, we_n=0)},  # p0: PRE
            ],
            pad_checkers = {"sys4x_90": {
                'cs': latency + '1100 0100',
            }},
        )

    def test_lpddr5_ck(self):
        # Test clock serialization, first few cycles are undefined so ignore them
        phy = LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        latency = 'x' * (8 + phy.ser_latency.sys8x)
        self.run_test(phy,
            dfi_sequence = [
                {3: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},
            ],
            pad_checkers = {"sys4x_90_ddr": {
                'ck': latency + '01010101' * 3,
            }},
        )

    def test_lpddr5_cs_multiple_phases(self):
        # Test that CS is serialized on different phases and that overlapping commands are handled
        phy = LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        latency = '0' * (4 + phy.ser_latency.sys4x)
        self.run_test(phy,
            dfi_sequence = [
                {0: dict(cs_n=0, cas_n=1, ras_n=0, we_n=0)},  # PRE (1 command)
                {1: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1)},  # ACT (2 commands)
                {
                    0: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1),
                    1: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1),  # should be ignored
                },
                {
                    0: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1),
                    2: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1),  # should NOT be ignored
                },
                {3: dict(cs_n=0, cas_n=1, ras_n=0, we_n=1)},  # crosses cycle boundaries
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # should be ignored
                {2: dict(cs_n=1, cas_n=0, ras_n=1, we_n=1)},  # ignored due to cs_n=1
            ],
            pad_checkers = {"sys4x_90": {
                'cs': latency + ''.join([
                    '0100',  # p0
                    '0110',  # p1
                    '1100',  # p0, p1 ignored
                    '1111',  # p0, p2 not ignored
                    '0001',  # p3
                    '1000',  # p0 ignored
                    '0000',  # p2 ignored
                ])
            }},
        )

