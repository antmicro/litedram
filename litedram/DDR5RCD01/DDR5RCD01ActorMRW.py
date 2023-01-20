#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
from operator import xor
import enum
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.SimCSCADriver import SimCSCADriver
from litedram.DDR5RCD01.DDR5RCD01Decoder import DDR5RCD01Decoder
from litedram.DDR5RCD01.DDR5RCD01Actor import DDR5RCD01Actor
from litedram.DDR5RCD01.DDR5RCD01Actor import DDR5Commands
from litedram.DDR5RCD01.DDR5RCD01Actor import DDR5Opcodes


class RCD01SpecialMRA(enum.IntEnum):
    MRW_5E = 0x5E
    MRW_3F = 0x3F


class DDR5RCD01ActorMRW(Module):
    """
        DDR5 RCD01 Actor is the module, which:

        acts on a command MRW with CW set to HIGH
        DRAMs ignore if CW is HIGH



        Module
        ------


        Parameters
        ------
        max_ui_num

        cs_n_w

        ca_w

    """

    def __init__(self,
                 trigger,
                 mrw_mra,
                 mrw_op,
                 mrw_cw,
                 RW5E_we,
                 RW5E_d,
                 RW5E_star_q,
                 mrw_op_o,
                 mrw_op_override,
                 
                 ):

        self.comb += If(
            mrw_mra == RCD01SpecialMRA.MRW_5E,
            RW5E_we.eq(1),
            RW5E_d.eq(mrw_op),
        ).Else(
            RW5E_we.eq(0),
            RW5E_d.eq(0),
        )

        self.comb += If(
            mrw_mra == RCD01SpecialMRA.MRW_3F,
            mrw_op_override.eq(1),
            mrw_op_o.eq(RW5E_star_q)
        ).Else(
            mrw_op_override.eq(0),
            mrw_op_o.eq(0)
        )


class TestBed(Module):
    def __init__(self):

        self.submodules.dut = DDR5RCD01ActorMRW()


def run_test(tb):
    logging.debug('Write test')
    for i in range(10):
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
