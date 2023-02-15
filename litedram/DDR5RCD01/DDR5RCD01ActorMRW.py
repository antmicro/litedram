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


class RCDSpecialMR(enum.IntEnum):
    MRW_5E = 0x5E
    MRW_5F = 0x5F
    MRW_3F = 0x3F


class RCDSpecialOpcode(enum.IntEnum):
    MRW = 0b00101


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
                 if_csca_i,
                 if_csca_o,
                 valid,
                 is_this_ui_odd,
                 is_cmd_beginning,
                 is_cw_bit_set,
                 reg_d,
                 reg_addr,
                 reg_we,
                 reg_q,
                 ):
        is_cw_bit_set_d = Signal()
        self.sync += is_cw_bit_set_d.eq(is_cw_bit_set)

        is_mrw_detected = Signal()
        mra = Signal(CW_REG_BIT_SIZE)
        opcode = Signal(CW_REG_BIT_SIZE)
        cw_bit = Signal()

        delay_len = 4
        is_cmd_beginning_d = Array(Signal() for _ in range(delay_len))
        for i in range(delay_len):
            if i == 0:
                self.sync += is_cmd_beginning_d[i].eq(is_cmd_beginning)
            else:
                self.sync += is_cmd_beginning_d[i].eq(is_cmd_beginning_d[i-1])

        self.sync += If(
            valid & is_cmd_beginning,
            If(
                if_csca_i.dca[0:5] == RCDSpecialOpcode.MRW,
                is_mrw_detected.eq(1),
                mra[0:2].eq(if_csca_i.dca[5:7]),
            ).Else(
                is_mrw_detected.eq(0),
            )
        )

        self.sync += If(
            valid & is_cmd_beginning_d[0],
            mra[2:8].eq(if_csca_i.dca[0:6])
        )

        self.sync += If(
            valid & is_cmd_beginning_d[1],
            opcode[0:7].eq(if_csca_i.dca[0:7]),
        )

        self.sync += If(
            valid & is_cmd_beginning_d[2],
            opcode[7].eq(if_csca_i.dca[0]),
            cw_bit.eq(if_csca_i.dca[3]),
        )


class TestBed(Module):
    def __init__(self):
        pass


def run_test(tb):
    logging.debug('Write test')
    for i in range(10):
        yield
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    raise UnderConstruction
