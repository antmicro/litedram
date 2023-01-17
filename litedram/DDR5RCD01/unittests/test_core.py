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


class TestBed(Module):
    def __init__(self, is_dual_channel=False):
        self.if_ck_rst = If_ck_rst()
        self.if_sdram_A = If_sdram()
        self.if_sdram_B = If_sdram()
        self.if_alert_n = If_alert_n()
        self.if_ibuf = If_ibuf()
        self.if_obuf_A = If_obuf()
        self.if_obuf_B = If_obuf()
        self.if_lb = If_lb()
        self.if_bcom_A = If_bcom()
        self.if_bcom_B = If_bcom()
        self.if_sideband = If_sideband()
        self.is_dual_channel = is_dual_channel

        self.submodules.dut = DDR5RCD01Core(
            if_ck_rst=self.if_ck_rst,
            if_sdram_A=self.if_sdram_A,
            if_sdram_B=self.if_sdram_B,
            if_alert_n=self.if_alert_n,
            if_ibuf=self.if_ibuf,
            if_obuf_A=self.if_obuf_A,
            if_obuf_B=self.if_obuf_B,
            if_lb=self.if_lb,
            if_bcom_A=self.if_bcom_A,
            if_bcom_B=self.if_bcom_B,
            if_sideband=self.if_sideband,
            is_dual_channel=self.is_dual_channel,
        )

    def n_ui_dram_command(self, nums, non_target_termination=False):
        """
        This function drives the interface with as in:
            "JEDEC 82-511 Figure 7
            One UI DRAM Command Timing Diagram"

        Nums can be any length to incroporate two, or more, UI commands

        The non target termination parameter extends the DCS assertion to the 2nd UI
        """
        SEQ_INACTIVE = [~0, 0]
        yield from self.drive_init()

        sequence = [SEQ_INACTIVE]
        for id, num in enumerate(nums):
            if non_target_termination:
                if id in [0, 1, 2, 3]:
                    sequence.append([0b00, num])
                else:
                    sequence.append([0b11, num])
            else:
                if id in [0, 1]:
                    sequence.append([0b00, num])
                else:
                    sequence.append([0b11, num])

        sequence.append(SEQ_INACTIVE)

        for seq_cs, seq_ca in sequence:
            logging.debug(str(seq_cs) + " " + str(seq_ca))
            yield from self.drive_cs_ca(seq_cs, seq_ca)
        for i in range(3):
            yield

    def seq_cmds(self):
        # TODO all commands are passed as if they were 2UIs long. To be fixed.
        # Single UI command
        yield from self.n_ui_dram_command(nums=[0x01, 0x02])
        # 2 UI commands
        yield from self.n_ui_dram_command(nums=[0x01, 0x02, 0x03, 0x04])
        yield from self.n_ui_dram_command(nums=[0x0A, 0x0B, 0x0C, 0x0D], non_target_termination=True)
        yield from self.n_ui_dram_command(nums=[0xDE, 0xAD, 0xBA, 0xBE], non_target_termination=True)
        yield from self.n_ui_dram_command(nums=[0xC0, 0xDE, 0xF0, 0x0D])

    def drive_init(self):
        yield self.if_ck_rst.drst_n.eq(1)
        yield from self.drive_cs_ca(~0, 0)

    def drive_cs_ca(self, cs, ca):
        yield self.if_ibuf.dcs_n.eq(cs)
        yield self.if_ibuf.dca.eq(ca)
        yield

    def scenario_compile(self):
        for i in range(2):
            yield

    def scenario_init_few_commands(self):
        INIT_CYCLES = CW_DA_REGS_NUM + 5
        yield from self.drive_init()
        for i in range(INIT_CYCLES):
            yield
        yield from self.seq_cmds()
        for i in range(3):
            yield

    def scenario_clock_distribution(self):
        INIT_CYCLES = CW_DA_REGS_NUM + 5
        yield from self.drive_init()
        for i in range(INIT_CYCLES):
            yield

        for b in [0, 1]*5:
            yield self.if_ck_rst.dck_t.eq(b)
            yield self.if_ck_rst.dck_c.eq(~b)
            yield


class DDR5RCD01CoreTests_SingleChannel(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed(is_dual_channel=False)
        dir_name = "./wave_sc"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"

    def tearDown(self):
        del self.tb

    def test_core_compile(self):
        run_simulation(self.tb,
                       self.tb.scenario_compile(),
                       vcd_name=self.wave_file_name
                       )

    def test_commands(self):
        run_simulation(self.tb,
                       self.tb.scenario_init_few_commands(),
                       vcd_name=self.wave_file_name
                       )


class DDR5RCD01CoreTests_DualChannel(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed(is_dual_channel=True)
        dir_name = "./wave_dc"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"

    def tearDown(self):
        del self.tb

    def test_core_compile(self):
        run_simulation(self.tb,
                       self.tb.scenario_compile(),
                       vcd_name=self.wave_file_name
                       )

    def test_clks(self):
        run_simulation(self.tb,
                       self.tb.scenario_clock_distribution(),
                       vcd_name=self.wave_file_name
                       )


if __name__ == '__main__':
    unittest.main()
