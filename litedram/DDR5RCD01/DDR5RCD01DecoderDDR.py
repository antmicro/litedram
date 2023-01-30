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

from litedram.DDR5RCD01.BusCSCAEnvironment import BusCSCAEnvironment
from litedram.DDR5RCD01.BusCSCAEnvironment import EnvironmentScenarios


class DDR5RCD01DecoderDDR(Module):
    """
        DDR mode
        RCD model assumed that it operates on doubled frequency, compared to dck_t,dck_c inputs.
        This means that in DDR mode:
            (posedge self.sync) == ((posedge ck_t) or (posedge ck_c))
        
        This means that in SDR mode:???
            (every other posedge self.sync) == (posedge ck_t)
        
        Figure 7,8,9 

        Module
        ------


        Parameters
        ------
        max_ui_num

        cs_n_w

        ca_w

    """

    def __init__(self,
                 if_ibuf,
                 qvalid,
                 qcommands_cs_n,
                 qcommands_ca,
                 qcommands_par,
                 max_ui_num=4,
                 cs_n_w=2,
                 ca_w=7
                 ):
        """
            XOR edge detection
        """
        del_dcs_n = Signal(cs_n_w)
        self.sync += del_dcs_n.eq(if_ibuf.dcs_n)

        del_dca = Signal(ca_w)
        self.sync += del_dca.eq(if_ibuf.dca)

        del_dpar = Signal()
        self.sync += del_dpar.eq(if_ibuf.dpar)

        det_edge = Signal(2)
        self.comb += det_edge.eq(if_ibuf.dcs_n ^ del_dcs_n)

        det_posedge = Signal(2)
        self.comb += det_posedge.eq(det_edge & if_ibuf.dcs_n)

        det_negedge = Signal(2)
        self.comb += det_negedge.eq(det_edge & ~if_ibuf.dcs_n)
        
        cmd_active = Signal()
        is_1_ui_command = Signal()
        
        self.comb += is_1_ui_command.eq(if_ibuf.dca[1])

        ui_counter = Signal(8)

        """
            UI counter
        """
        ui_counter = Signal(8, reset=max_ui_num)
        self.sync += If(
            det_negedge != 0b00,
            ui_counter.eq(0)
        ).Else(
            If(ui_counter == max_ui_num,
               ui_counter.eq(ui_counter)
               ).Else(
                ui_counter.eq(ui_counter+1)
            )
        )

        self.comb += If(
            ui_counter != 0,
            cmd_active.eq(1)
        )
        """
        DDR Mode
        if is dcs asserted:
            command is begin
            FIRST-UI CAPTURE
            capture this ui
            if bit CA[1] in this ui is set:    
                this is 1 ui command
            else:
                this is 2 ui command
        
            SECOND-UI CAPTURE

            if cs is asserted or this is 2 ui command:
                capture this ui

            while cs is asserted:
                repeat steps above (to support multi-command)
        repeat all;
        """


class TestBed(Module):
    def __init__(self):

        if_ibuf = If_ibuf()
        self.submodules.env = BusCSCAEnvironment(
            if_ibuf_o=if_ibuf,
        )

        max_ui_num = 4
        cs_n_w = 2
        ca_w = 7
        qvalid = Signal()
        qcommands_cs_n = Array(Signal(cs_n_w) for y in range(max_ui_num))
        qcommands_ca = Array(Signal(ca_w) for y in range(max_ui_num))
        qcommands_par = Array(Signal() for y in range(max_ui_num))

        self.submodules.dut = DDR5RCD01DecoderDDR(
            if_ibuf=if_ibuf,
            qvalid=qvalid,
            qcommands_cs_n=qcommands_cs_n,
            qcommands_ca=qcommands_ca,
            qcommands_par=qcommands_par,
            max_ui_num=max_ui_num,
            cs_n_w=cs_n_w,
            ca_w=ca_w,
        )


def run_test(tb):
    logging.debug('Write test')
    scenario_select = EnvironmentScenarios.SIMPLE_GENERIC
    yield from tb.env.run_env(scenario_select=scenario_select)
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
