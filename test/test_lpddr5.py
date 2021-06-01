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

    def test_lpddr5_ca_sequencing(self):
        # Test proper serialization of commands to CA pads and that overlapping commands are handled
        phy = LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        cs_latency = '0' * (4 + phy.ser_latency.sys4x)
        ca_latency = '0' * (8 + phy.ser_latency.sys8x)
        read = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)  # CAS+RD16
        precharge = dict(cs_n=0, cas_n=1, ras_n=0, we_n=0)
        self.run_test(phy,
            dfi_sequence = [
                {0: read, 3: read},
                {0: read, 2: read},  # p0 ignored
                {1: precharge},
            ],
            pad_checkers = {
                "sys4x_90": {
                    'cs':  cs_latency + '1 1 0 1  1 0 1 1  0 0 1 0 ', },
                "sys4x_90_ddr": {
                    'ca0': ca_latency + '00100000 10000010 00000000',
                    'ca1': ca_latency + '00000000 00000000 00000000',
                    'ca2': ca_latency + '10000010 00001000 00000000',
                    'ca3': ca_latency + '10000010 00001000 00001000',
                    'ca4': ca_latency + '00000000 00000000 00001000',
                    'ca5': ca_latency + '10000010 00001000 00001000',
                    'ca6': ca_latency + '00000000 00000000 00001000',
                }
            },
        )

    def test_lpddr5_ca_addressing(self):
        # Test that bank/address for different commands are correctly serialized to CA pads
        # LPDDR5 has only 64 columns, but uses optional 4-bit "burst address"
        read       = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1, bank=0b1111, address=0b110101)
        write_ap   = dict(cs_n=0, cas_n=0, ras_n=1, we_n=0, bank=0b1010, address=0b10000000000)
        activate   = dict(cs_n=0, cas_n=1, ras_n=0, we_n=1, bank=0b0010, address=0b111110000111100001)
        refresh_ab = dict(cs_n=0, cas_n=0, ras_n=0, we_n=1, bank=0b1001, address=0b10000000000)
        precharge  = dict(cs_n=0, cas_n=1, ras_n=0, we_n=0, bank=0b0111, address=0)
        mrw        = dict(cs_n=0, cas_n=0, ras_n=0, we_n=0, bank=0b1010011, address=0b10101010)  # bank=7-bit address, address=8-bit op code
        mrr        = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=1,     address=0b1101101)  # 7-bit address (bank=1 selects MRR)
        zqc_start  = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=0,     address=0b10000101)  # MPC with ZQCAL START operand
        zqc_latch  = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=0,     address=0b10000110)  # MPC with ZQCAL LATCH operand

        for masked_write in [True, False]:
            with self.subTest(masked_write=masked_write):
                phy = LPDDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ, masked_write=masked_write)
                cs_latency = '0' * (4 + phy.ser_latency.sys4x)
                ca_latency = '0' * (8 + phy.ser_latency.sys8x)
                mw = f"10{int(not masked_write)}0"
                self.run_test(phy,
                    dfi_sequence = [
                        {0: read, 2: write_ap},
                        {0: activate, 2: refresh_ab},
                        {0: precharge, 2: mrw},
                        {0: mrr},
                        {0: zqc_start, 2: zqc_latch},
                    ],
                    pad_checkers = {
                        "sys4x_90": {
                            'cs':  cs_latency + '1 1  1 1  1 1  0 1  0 1  1 1  1 1  0 0  0 1  0 1 ', },
                        "sys4x_90_ddr": {
                            'ca0': ca_latency + '0011 0000 1011 0001 0001 0100 0001 0000 0001 0000',
                            'ca1': ca_latency + '0001 0011 1110 0000 0001 0101 0000 0000 0000 0001',
                            'ca2': ca_latency +f'1001 {mw} 1000 0000 0001 0000 1001 0000 0001 0001',
                            'ca3': ca_latency + '1011 1001 1010 0010 0010 1011 1011 0000 0000 0000',
                            'ca4': ca_latency + '0000 1000 1010 0010 001x 1100 0010 0000 0010 0010',
                            'ca5': ca_latency + '1011 0000 1001 0010 001x 0001 0001 0000 0010 0010',
                            'ca6': ca_latency + '0010 0001 1101 0001 0010 1110 1001 0000 0010 0010',
                        }
                    },
                )
