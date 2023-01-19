#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
from operator import xor
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *


class SimCSCADriver(Module):
    """
        DDR5 RCD01 Module Template

        Module
        ------


        Parameters
        ------
    """

    def __init__(self, if_ibuf_o, dcs_n_w=2, dca_w=7):
        self.dcs_n = Signal(dcs_n_w,reset=~0)
        self.dca = Signal(dca_w)

        self.dpar = Signal()
        self.comb += If(
            reduce(xor, [self.dca[bit] for bit in range(dca_w)]),
            self.dpar.eq(1)
        ).Else(
            self.dpar.eq(0)
        )
        self.comb += if_ibuf_o.dcs_n.eq(self.dcs_n)
        self.comb += if_ibuf_o.dca.eq(self.dca)
        self.comb += if_ibuf_o.dpar.eq(self.dpar)

    def seq_cmds(self):
        # TODO all commands are passed as if they were 2UIs long. To be fixed.
        # Single UI command
        yield from self.n_ui_dram_command(nums=[0x01, 0x02], sel_cs="rank_AB")
        # 2 UI commands
        yield from self.n_ui_dram_command(nums=[0x01, 0x02, 0x03, 0x04], sel_cs="rank_A")
        yield from self.n_ui_dram_command(nums=[0b00010101, 0xDE, 0xF0, 0x0D], sel_cs="rank_B") #MMR
        yield from self.n_ui_dram_command(nums=[0b00010100, 0xDE, 0xF0, 0x0D], sel_cs="rank_AB") #MMW
        yield from self.n_ui_dram_command(nums=[0x0A, 0x0B, 0x0C, 0x0D], non_target_termination=True)
        yield from self.n_ui_dram_command(nums=[0xDE, 0xAD, 0xBA, 0xBE], non_target_termination=True)
        yield from self.n_ui_dram_command(nums=[0xC0, 0xDE, 0xF0, 0x0D], sel_cs="rank_AB")

    def n_ui_dram_command(self, nums, sel_cs="rank_AB", non_target_termination=False):
        """
        This function drives the interface with as in:
            "JEDEC 82-511 Figure 7
            One UI DRAM Command Timing Diagram"

        Nums can be any length to incroporate two, or more, UI commands

        The non target termination parameter extends the DCS assertion to the 2nd UI
        """
        if sel_cs == "rank_A":
            cs = 0b10
        elif sel_cs == "rank_B":
            cs = 0b01
        elif sel_cs == "rank_AB":
            cs = 0b00
        else:
            cs = 0b11

        SEQ_INACTIVE = [~0, 0]
        yield from self.drive_init()

        sequence = [SEQ_INACTIVE]
        for id, num in enumerate(nums):
            if non_target_termination:
                if id in [0, 1, 2, 3]:
                    sequence.append([cs, num])
                else:
                    sequence.append([0b11, num])
            else:
                if id in [0, 1]:
                    sequence.append([cs, num])
                else:
                    sequence.append([0b11, num])

        sequence.append(SEQ_INACTIVE)

        for seq_cs, seq_ca in sequence:
            logging.debug("[CS CA] = " + str(seq_cs) + "    " + str(seq_ca))
            yield from self.drive_cs_ca(seq_cs, seq_ca)
        for i in range(1):
            yield

    def drive_init(self):
        yield from self.drive_cs_ca(~0, 0)

    def drive_cs_ca(self, cs, ca):
        yield self.dcs_n.eq(cs)
        yield self.dca.eq(ca)
        yield


class TestBed(Module):
    def __init__(self):
        if_ibuf_o = If_ibuf()
        self.submodules.dut = SimCSCADriver(if_ibuf_o=if_ibuf_o)


def run_test(tb):
    logging.debug('Write test')
    yield from tb.dut.seq_cmds()
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
