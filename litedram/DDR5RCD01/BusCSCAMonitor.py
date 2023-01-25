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
from migen.genlib.fifo import SyncFIFO
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
#
from litedram.DDR5RCD01.BusCSCAEnvironment import BusCSCAEnvironment


@enum.unique
class CMDTypes(enum.IntEnum):
    INACTIVE = 0
    SINGLE_UI = 1
    DOUBLE_UI = 2


class BusCSCAMonitor(Module):
    """
        DDR5 RCD01 Monitor

        cmd_len : bit width of this signal should be large enough
        to count all clock cycles of the simulation.
        Hard to assess how much array depth is required, this
        is related to environment setup. The longer the simulation,
        the longer the array.

        Module
        ------

        Parameters
        ------

        Sample output:
            - inactive for n clocks
            - 1 ui message
            - inactive for 1 clock
            - 2 ui message
            - 1 ui message
            - inactive

        Data structure to hold this:
        List would be best. migen style list is an Array

        I think that squashing inactives together is useful and would free some memory,
        array width {CSCA  bus capture, metadata}
        single entry
        is_active, cmd_len (cmd id),ui_n,{command_payload}
        []

        if cmd is inactive:
            increase inactive counter

        if cmd is active:
            save(inactive counter state)
            check if cmd is 1 ui or 2 ui 
            UI length is based on CA1:
                CA1==HIGH => CMD is 1 UI
                CA1==LOW  => CMD is 2 UI
            save(next 1/2ui)

        reset(counters)


        read():

    """

    def __init__(self,
                 if_ibuf_i,
                 dcs_n_w=2,
                 dca_w=7,
                 monit_arr_d=128):

        dcs_n = Signal(dcs_n_w)
        dca = Signal(dca_w)
        dpar = Signal()
        dpar_w = len(dpar)

        self.comb += dcs_n.eq(if_ibuf_i.dcs_n)
        self.comb += dca.eq(if_ibuf_i.dca)
        self.comb += dpar.eq(if_ibuf_i.dpar)

        CSCABus_w = dcs_n_w + dca_w + dpar_w

        """
            Signals that come into the array and are meant for post-processing
            cmd_type = {INACTIVE, SINGLE_UI, DOUBLE_UI}
        """
        is_active = Signal()
        cmd_len = Signal(16)
        cmd_len_w = len(cmd_len)
        cmd_type = Signal(2)

        monit_ctrl_w = len(is_active) + \
            cmd_len_w + len(cmd_type)

        monit_arr_w = CSCABus_w + monit_ctrl_w

        self.xarr = Array(Signal(monit_arr_w) for _ in range(monit_arr_d))
        xarr_ptr = Signal(monit_arr_w)

        cmd = Cat(is_active, cmd_len, cmd_type, dcs_n, dca, dpar)

        xarr_we = Signal()
        self.sync += If(
            xarr_we,
            self.xarr[xarr_ptr].eq(cmd)
        )
        self.sync += If(
            xarr_we,
            xarr_ptr.eq(xarr_ptr+1)
        )

        """
            Invalid counter
        """
        counter_invalid = Signal(cmd_len_w)
        counter_en = Signal()
        counter_rst = Signal()

        self.sync += If(
            counter_rst,
            counter_invalid.eq(0),
        ).Else(
            If(
                counter_en,
                counter_invalid.eq(counter_invalid + 1)
            )
        )

        """
            XOR edge detection
        """
        del_dcs_n = Signal(dcs_n_w)
        self.sync += del_dcs_n.eq(dcs_n)

        del_dca = Signal(dca_w)
        self.sync += del_dca.eq(dca)

        del_dpar = Signal()
        self.sync += del_dpar.eq(dpar)

        det_edge = Signal(2)
        self.comb += det_edge.eq(dcs_n ^ del_dcs_n)

        det_posedge = Signal(2)
        self.comb += det_posedge.eq(det_edge & dcs_n)

        det_negedge = Signal(2)
        self.comb += det_negedge.eq(det_edge & ~dcs_n)

        cmd_active = Signal(2)
        self.sync += If(
            det_negedge,
            cmd_active.eq(1)
        )


        is_1_ui_command = Signal()
        self.comb += is_1_ui_command.eq(dca[1])

        
        self.sync += If(
            det_posedge,
            cmd_active.eq(0)
        )
        """
            
        """
        self.comb += If(

        )
        """
            Simple decoding
        """

    def post_process(self):
        ds = yield self.xarr
        print(ds)


class TestBed(Module):
    def __init__(self):
        if_ibuf = If_ibuf()
        self.submodules.env = BusCSCAEnvironment(
            if_ibuf_o=if_ibuf,
        )
        self.submodules.monitor = BusCSCAMonitor(
            if_ibuf_i=if_ibuf
        )


def run_test(tb):
    logging.debug('Write test')
    scenario_select = "test_cw_wr_rd"
    yield from tb.env.run_env(scenario_select=scenario_select)
    yield from tb.monitor.post_process()
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
