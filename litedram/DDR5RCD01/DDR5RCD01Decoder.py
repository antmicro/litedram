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
from litedram.DDR5RCD01.SimCSCADriver import SimCSCADriver


class DDR5RCD01Decoder(Module):
    """
        DDR5 RCD01 Decoder implements XOR logic to
        detect positive and negative edges on dcs_n signals.
        If a negative edge is detected, then a command is present on
        the CSCA bus. It is assumed that the incoming messages are
        DDR and may take up to 4 UIs (max_ui_num=4). 4 UIs, starting
        with the one, when negedge occured, are captured in arrays:
        
        (Array) commands_cs_n = [CS_1UI, CS_2UI, CS_3UI, CS_4 UI]
        
        Similarly for CA and DPAR
        
        An internal counter (ui_counter) is used to count the 4 UIs.
        Counter states max_ui_num=4, ..., 2 are used to capture the last 3 UIs.
        Once the counter reaches state 1, the full message is captured
        into the qcommands and a valid signal is asserted for one clock cycle.
        The output is valid only in this one cycle.

        There is a delay from assertion of dcs_n to qvalid, which may be too
        long for scenarios, in which the RCD should react quickly (c.f. parity error
        blocking).

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
        MAX_UI_NUM = max_ui_num
        commands_cs_n = Array(Signal(cs_n_w) for y in range(MAX_UI_NUM))
        commands_ca = Array(Signal(ca_w) for y in range(MAX_UI_NUM))
        commands_par = Array(Signal() for y in range(MAX_UI_NUM))

        UI_counter = Signal(2)

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
        
        """
            UI counter
        """
        ui_counter = Signal(8)
        self.sync += If(
            det_negedge != 0b00,
            ui_counter.eq(MAX_UI_NUM)
        ).Else(
            If(ui_counter == 0,
               ui_counter.eq(ui_counter)
               ).Else(
                ui_counter.eq(ui_counter-1)
            )
        )
        
        self.sync += If(
            det_negedge != 0b00,
            commands_cs_n[0].eq(if_ibuf.dcs_n),
            commands_ca[0].eq(if_ibuf.dca),
            commands_par[0].eq(if_ibuf.dpar),
        )

        for i in [MAX_UI_NUM, MAX_UI_NUM-1, MAX_UI_NUM-2]:
            self.sync += If(
                ui_counter == i,
                commands_cs_n[MAX_UI_NUM+1-i].eq(if_ibuf.dcs_n),
                commands_ca[MAX_UI_NUM+1-i].eq(if_ibuf.dca),
                commands_par[MAX_UI_NUM+1-i].eq(if_ibuf.dpar),
            ).Elif(
                ui_counter == 0,
                commands_cs_n[MAX_UI_NUM+1-i].eq(0),
                commands_ca[MAX_UI_NUM+1-i].eq(0),
                commands_par[MAX_UI_NUM+1-i].eq(0),
            )
        
        """
            Capture output
        """
        qvalid = Signal()
        qcommands_cs_n = Array(Signal(cs_n_w) for y in range(MAX_UI_NUM))
        qcommands_ca = Array(Signal(ca_w) for y in range(MAX_UI_NUM))
        qcommands_par = Array(Signal() for y in range(MAX_UI_NUM))

        for i in range(MAX_UI_NUM):
            self.sync += If(
                ui_counter == 1,
                qcommands_cs_n[i].eq(commands_cs_n[i]),
                qcommands_ca[i].eq(commands_ca[i]),
                qcommands_par[i].eq(commands_par[i]),
                qvalid.eq(1),
            ).Else(
                qcommands_cs_n[i].eq(0),
                qcommands_ca[i].eq(0),
                qcommands_par[i].eq(0),
                qvalid.eq(0),
            )


class mem(Module):
    def __init__(self):

        pass


class TestBed(Module):
    def __init__(self):
        max_ui_num = 4
        cs_n_w = 2
        ca_w = 7
        if_ibuf = If_ibuf()
        qvalid = Signal()
        qcommands_cs_n = Array(Signal(cs_n_w) for y in range(max_ui_num))
        qcommands_ca = Array(Signal(ca_w) for y in range(max_ui_num))
        qcommands_par = Array(Signal() for y in range(max_ui_num))

        self.submodules.driver = SimCSCADriver(
            if_ibuf_o=if_ibuf,
        )

        self.submodules.dut = DDR5RCD01Decoder(
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
    yield from tb.driver.seq_cmds()
    for i in range(1):
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
