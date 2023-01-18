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
from litedram.DDR5RCD01.RCD_utils import *


class DDR5RCD01Error(Module):
    """
    DDR5 RCD01 Error
    ----------------

    1. If parity checking is enabled:
        - the module calculates the parity
        - raises alerts to Alert in Common (single-pulse)
    2. Alawys(?) passes derror_n_in signal
    3. Controls reads and writes to Error Log Registers (24:20)
        - Set CA Parity Error Status (RW24)
        - Clear bit in RW01 to disable parity checking (re-enable mode is possible)
    4. Send message to control center that the output should be blocked?
        or command center takes this information from rw20

    Module
    ------
    <interface>

    Parameters
    ----------
    <params>

    """

    def __init__(self,
                 if_ck_rst,
                 if_ibuf,
                 if_channel_sdram,
                 if_2_rws,
                 ):
        # if_ck_rst to handle drst_n

        # if_ibuf to calculate parity

        # if_channel_sdram to take derror_in_n

        # if_2_rws, whatever is required to write to error registers
        """
            Check parity
        """
        err_parity = Signal()
        dca_w = 7

        # If in first, second,.... UI
        self.comb += If(
            err_parity.eq(reduce(xor, [if_ibuf.dca[bit]
                          for bit in range(dca_w)]) ^ if_ibuf.dpar)
        )

        


class TestBed(Module):
    def __init__(self):

        self.submodules.dut = DDR5RCD01Error()


def run_test(dut):
    logging.debug('Run test')
    for i in range(5):
        yield
    logging.debug('Yield from run test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
