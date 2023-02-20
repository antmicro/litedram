#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *


class DDR5RCD01DCSTMAgent(Module):
    """
        DDR5 RCD DCS Training Mode Agent

        Module
        ------


        Parameters
        ------
        if_ctrl.enable
        if_ctrl.select_dcs_n
    """

    def __init__(self,
                 if_ibuf: If_ibuf,
                 sample_o: Signal,
                 if_ctrl: If_ctrl_dcstm_agent):
        """
           Select DCS_n line which is sampled (RW02)
        """
        dcs_n = Signal()
        self.comb += If(
            if_ctrl.select_dcs_n,
            dcs_n.eq(if_ibuf.dcs_n[1])
        ).Else(
            dcs_n.eq(if_ibuf.dcs_n[0])
        )

        """
            Hold 4 last samples
        """
        DCSTM_SAMPLE_NUM = 4
        dcs_n_samples = Array(Signal() for _ in range(DCSTM_SAMPLE_NUM-1))
        for i in range(DCSTM_SAMPLE_NUM-1):
            if i == 0:
                self.sync += dcs_n_samples[i].eq(dcs_n)
            else:
                self.sync += dcs_n_samples[i].eq(dcs_n_samples[i-1])
        """
            Count to 4 samples
        """
        ui_counter = Signal(2, reset=0)
        self.sync += If(
            if_ctrl.enable,
            ui_counter.eq(ui_counter+1)
        ).Else(
            ui_counter.eq(0)
        )

        """
            Table 16,17,18
            Output sample calculation logic
        """
        sample = Signal()
        self.sync += If(
            ui_counter == 3,
            If(
                (dcs_n == 0) &
                (dcs_n_samples[0] == 1) &
                (dcs_n_samples[1] == 0) &
                (dcs_n_samples[2] == 1),
                sample.eq(0)
            ).Else(
                sample.eq(1)
            )
        )

        """
            It is assumed that this block is connected to Alert block in Common.
            It is expected that Alert block is configured in static mode.
            The alert block expects positive logic.
        """
        self.comb += If(
            if_ctrl.enable,
            sample_o.eq(~sample),
        ).Else(
            sample_o.eq(0),
        )


class TestBed(Module):
    def __init__(self):
        self.submodules.dut = DDR5RCD01DCSTMAgent()


def run_test(tb):
    logging.debug('Write test')
    for i in range(5):
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
