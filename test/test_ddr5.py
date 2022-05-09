#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import re
import copy
import unittest
from typing import Mapping
from functools import partial
from collections import defaultdict

from migen import *

from litedram.phy.ddr5.simphy import DDR5SimPHY, DoubleRateDDR5SimPHY
from litedram.phy.ddr5 import simsoc
from litedram.phy.sim_utils import SimLogger

import test.phy_common
from test.phy_common import DFISequencer, PadChecker


# Migen simulator supports reset signals so we could add CRG to start all the signals
# in the same time, however the clock signals will still be visible in the VCD dump
# and the generators we assign to them will still work before reset. For this reason we
# use clocks set up in such a way that we have all the phase aligned clocks start in tick
# 1 (not zero), so that we avoid any issues with clock alignment.
#
# NOTE: On hardware proper reset must be ensured!
#
# The simulation should start like this:
#   sys          |_--------------
#   sys_11_25    |___------------
#   sys8x        |_----____----__
#   sys8x_ddr    |_--__--__--__--
#   sys8x_90     |___----____----
#   sys8x_90_ddr |-__--__--__--__
#
# sys8x_90_ddr does not trigger at the simulation start (not an edge),
# BUT a generator starts before first edge, so a `yield` is needed to wait until the first
# rising edge!
run_simulation = partial(test.phy_common.run_simulation, clocks={
    "sys":          (64, 31),
    "sys2x":        (32, 15),
    "sys8x":        ( 8,  3),
    "sys8x_ddr":    ( 4,  1),
    "sys8x_90":     ( 8,  1),
    "sys8x_90_ddr": ( 4,  3),
})

dfi_data_to_dq = partial(test.phy_common.dfi_data_to_dq, databits=8, nphases=8, burst=16)
dq_pattern = partial(test.phy_common.dq_pattern, databits=8, nphases=8, burst=16)


class DDR5Tests(unittest.TestCase):
    SYS_CLK_FREQ = 50e6
    CMD_LATENCY = 2

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

    def test_ddr5_empty_command_sequence(self):
        # Test CS_n/CA values for empty dfi commands sequence
        latency   = '00000000' * self.CMD_LATENCY
        latency_n = '11111111' * self.CMD_LATENCY

        self.run_test(DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ),
                      dfi_sequence = [],
                      pad_checkers = {"sys8x_90": {
                          'cs_n': latency_n,
                          'ca0':  latency,
                          'ca1':  latency,
                          'ca2':  latency,
                          'ca3':  latency,
                          'ca4':  latency,
                          'ca5':  latency,
                          'ca6':  latency,
                          'ca7':  latency,
                          'ca8':  latency,
                          'ca9':  latency,
                          'ca10': latency,
                          'ca11': latency,
                          'ca12': latency,
                          'ca13': latency,
                      }},
                      )

    def test_ddr5_ca_addressing(self):
        # Test that bank/address for different commands are correctly serialized to CA pads
        latency   = '00000000' * self.CMD_LATENCY
        latency_n = '11111111' * self.CMD_LATENCY

        read          = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1, bank=0b101,    address=0b1100110000)
        write_ap      = dict(cs_n=0, cas_n=0, ras_n=1, we_n=0, bank=0b111,    address=0b10000000000)
        activate      = dict(cs_n=0, cas_n=1, ras_n=0, we_n=1, bank=0b010,    address=0b11110000111100001)
        refresh_ab    = dict(cs_n=0, cas_n=0, ras_n=0, we_n=1, bank=0b100,    address=0)
        precharge_ab  = dict(cs_n=0, cas_n=1, ras_n=0, we_n=0, bank=0b011,    address=0)
        mrw           = dict(cs_n=0, cas_n=0, ras_n=0, we_n=0, bank=0b110011, address=0b10101010)  # bank=6-bit address, address=8-bit op code
        zqc_start     = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=0,        address=0b0000101)  # MPC with ZQCAL START operand
        zqc_latch     = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=0,        address=0b0000100)  # MPC with ZQCAL LATCH operand
        mrr           = dict(cs_n=0, cas_n=1, ras_n=1, we_n=0, bank=1,        address=0b101101)  # 6-bit address (bank=1 selects MRR)

        self.run_test(DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ),
                      dfi_sequence = [
                          {0: read, 4: write_ap},
                          {0: activate, 4: refresh_ab},
                          {0: precharge_ab, 4: mrw},
                          {0: zqc_start, 4: zqc_latch},
                          {0: mrr},
                      ],
                      pad_checkers = {"sys8x_90": {
                          #                    rd     wr       act    ref      pre    mrw      zqcs   zqcl     mrr
                          'cs_n': latency_n + '0111'+'0111' + '0111'+'0111' + '0111'+'0111' + '0111'+'0111' + '0111'+'1111',
                          'ca0':  latency   + '1000'+'1x00' + '0000'+'1000' + '1000'+'1000' + '1000'+'1000' + '1000'+'0000',
                          'ca1':  latency   + '0000'+'0000' + '0100'+'1000' + '1000'+'0100' + '1000'+'1000' + '0000'+'0000',
                          'ca2':  latency   + '1100'+'1000' + '1100'+'0000' + '0000'+'1000' + '1000'+'1000' + '1x00'+'0000',
                          'ca3':  latency   + '1100'+'1000' + '0100'+'0000' + '1000'+'0100' + '1000'+'1000' + '0x00'+'0000',
                          'ca4':  latency   + '1000'+'0000' + '0100'+'1000' + '0000'+'0000' + '0000'+'0000' + '1x00'+'0000',
                          'ca5':  latency   + '1000'+'1000' + '0000'+'x000' + 'x000'+'1100' + '1000'+'0000' + '1x00'+'0000',
                          'ca6':  latency   + '1100'+'1000' + '0000'+'x000' + 'x000'+'1000' + '0000'+'0000' + '0x00'+'0000',
                          'ca7':  latency   + '0100'+'1000' + '1000'+'x000' + 'x000'+'0100' + '1000'+'1000' + '1x00'+'0000',
                          'ca8':  latency   + '1000'+'1100' + '0000'+'x000' + 'x000'+'0x00' + '0000'+'0000' + '1x00'+'0000',
                          'ca9':  latency   + '0x00'+'0x00' + '0100'+'x000' + 'x000'+'1x00' + '0000'+'0000' + '0x00'+'0000',
                          'ca10': latency   + '0100'+'0100' + '0100'+'0000' + '0000'+'1x00' + '0000'+'0000' + '1x00'+'0000',
                          'ca11': latency   + 'xx00'+'x100' + 'x100'+'x000' + 'x000'+'0x00' + '0000'+'0000' + '0x00'+'0000',
                          'ca12': latency   + 'xx00'+'xx00' + 'x100'+'x000' + 'x000'+'0x00' + '0000'+'0000' + '0x00'+'0000',
                          'ca13': latency   + 'xx00'+'xx00' + 'x000'+'x000' + 'x000'+'xx00' + 'x000'+'x000' + 'xx00'+'0000',
                      }},
                      )

    def test_ddr5_dq_out(self):
        # Test serialization of dfi wrdata to DQ pads
        dut = DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        zero = '00000000' * 2  # zero for 1 sysclk clock in sys8x_ddr clock domain

        dfi_data = {
            0: dict(wrdata=0x1122),
            1: dict(wrdata=0x3344),
            2: dict(wrdata=0x5566),
            3: dict(wrdata=0x7788),
            4: dict(wrdata=0x99aa),
            5: dict(wrdata=0xbbcc),
            6: dict(wrdata=0xddee),
            7: dict(wrdata=0xff00),
        }
        dfi_wrdata_en = {0: dict(wrdata_en=1)}  # wrdata_en=1 required on any single phase

        self.run_test(dut,
            dfi_sequence = [dfi_wrdata_en, {}, dfi_data],
            pad_checkers = {"sys8x_90_ddr": {
                f'dq{i}': (self.CMD_LATENCY+1)*zero + zero + dq_pattern(i, dfi_data, "wrdata") + zero for i in range(8)
            }},
            vcd_name="ddr_dq_out.vcd"
        )

    def test_ddr5_dq_only_1cycle(self):
        # Test that DQ data is sent to pads only during expected cycle, on other cycles there is no data
        dut = DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ)
        zero = '00000000' * 2

        dfi_data = {
            0: dict(wrdata=0x1122),
            1: dict(wrdata=0x3344),
            2: dict(wrdata=0x5566),
            3: dict(wrdata=0x7788),
            4: dict(wrdata=0x99aa),
            5: dict(wrdata=0xbbcc),
            6: dict(wrdata=0xddee),
            7: dict(wrdata=0xff00),
        }
        dfi_wrdata_en = copy.deepcopy(dfi_data)
        dfi_wrdata_en[0].update(dict(wrdata_en=1))

        self.run_test(dut,
            dfi_sequence = [dfi_wrdata_en, dfi_data, dfi_data],
            pad_checkers = {"sys8x_90_ddr": {
                f'dq{i}': (self.CMD_LATENCY+1)*zero + zero + dq_pattern(i, dfi_data, "wrdata") + zero for i in range(8)
            }},
            vcd_name="ddr_dq_only_1cycle.vcd"
        )

    def test_ddr5_dq_in_rddata_valid(self):
        # Test that rddata_valid is set with correct delay
        read_latency = 9
        dfi_sequence = [
            {0: dict(rddata_en=1)},  # command is issued by MC (appears on next cycle)
            *[{p: dict(rddata_valid=0) for p in range(8)} for _ in range(read_latency - 1)],  # nothing is sent during write latency
            {p: dict(rddata_valid=1) for p in range(8)},
            {},
        ]

        self.run_test(DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ),
            dfi_sequence = dfi_sequence,
            pad_checkers = {},
            pad_generators = {},
            vcd_name="ddr5_dq_in_rddata_valid.vcd"
        )

    def test_ddr5_dq_in_rddata(self):
        # Test that data on DQ pads is deserialized correctly to DFI rddata.
        # We assume that when there are no commands, PHY will still deserialize the data,
        # which is generally true (tristate oe is 0 whenever we are not writing).
        dfi_data = {
            0: dict(rddata=0x1122),
            1: dict(rddata=0x3344),
            2: dict(rddata=0x5566),
            3: dict(rddata=0x7788),
            4: dict(rddata=0x99aa),
            5: dict(rddata=0xbbcc),
            6: dict(rddata=0xddee),
            7: dict(rddata=0xff00),
        }

        def sim_dq(pads):
            for _ in range(16 * 1):  # wait 1 sysclk cycle
                yield
            for cyc in range(16):  # send a burst of data on pads
                for bit in range(8):
                    yield pads.dq_i[bit].eq(int(dq_pattern(bit, dfi_data, "rddata")[cyc]))
                yield
            for bit in range(8):
                yield pads.dq_i[bit].eq(0)
            yield

        read_des_delay = 3  # phy.read_des_delay
        dfi_sequence = [
            {},  # wait 1 sysclk cycle
            *[{} for _ in range(read_des_delay)],
            dfi_data,
            {},
        ]

        self.run_test(DDR5SimPHY(sys_clk_freq=self.SYS_CLK_FREQ),
            dfi_sequence = dfi_sequence,
            pad_checkers = {},
            pad_generators = {
                "sys8x_90_ddr": sim_dq,
            },
            vcd_name="ddr_dq_in_rddata.vcd"
        )
