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
# RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_utils import *
# Submodules


class DDR5RCD01CSLogic(Module):
    """
        CS Logic
        The control center forwards the cmd to the deserializer by the
        If_ctrl_lbuf interface. The deserializer is placed inside of the lbuf.
        (Not optimal for synthesis).

        Commands (CA and CS_n) are forwarded in normal mode:
        1. Detect active CS_n
        2. Send the command to the DRAM interface
        Use Cases:

        1 UI command:
        - CS is low
        - CA on this edge and next is captured (c.f. RCD model clocking)

        2 UI commands:
        - CS is low only during 1st UI. Can be low if the non-target termination
        is being signalled. c.f. Table 4
        - CA must be captured on 2 more edges

        Parity error detected during a 1 UI command

        Parity error detected during a 2 UI command

        DRAM Interface Blocking Mode is enabled

        CA Pass-through Mode is enabled

        The decode portion should always listen
    """

    def __init__(self,
                 if_ibuf_o,
                 if_ctrl_lbuf,
                 inv_en=False,
                 cs_bit=0,
                 ):
        # Fix ibuf : if dcs_n == 0x00 should be if dcs_n[1] = 0 ,etc. for all cases
        if inv_en:
            self.comb += if_ctrl_lbuf.deser_cs_n_d_disable_state.eq(0xFFFF)
            self.comb += if_ctrl_lbuf.deser_ca_d_disable_state.eq(0x0000)
        else:
            self.comb += if_ctrl_lbuf.deser_cs_n_d_disable_state.eq(0xFFFF)
            self.comb += if_ctrl_lbuf.deser_ca_d_disable_state.eq(0xFFFF)

        """
          Drive deser if a command is sent
        """
        # Normal forward
        xfsm_cslogic = FSM(reset_state="RESET")
        self.submodules += xfsm_cslogic

        fetch_decode_en = Signal()
        self.comb += fetch_decode_en.eq(1)

        xfsm_cslogic.act(
            "RESET",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
            if_ctrl_lbuf.deser_ca_d_en.eq(0),
            if_ctrl_lbuf.deser_ca_q_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(0),
            If(
                fetch_decode_en,
                NextState("IDLE")
            )
        )
        xfsm_cslogic.act(
            "IDLE",
            If(
                if_ibuf_o.dcs_n[cs_bit] == 0x0,
                if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
                if_ctrl_lbuf.deser_ca_d_en.eq(1),
                if_ctrl_lbuf.deser_ca_q_en.eq(0),
                if_ctrl_lbuf.deser_cs_n_d_en.eq(1),
                if_ctrl_lbuf.deser_cs_n_q_en.eq(0),
                NextState("S_0a")
            ).Else(
                if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
                if_ctrl_lbuf.deser_ca_d_en.eq(0),
                if_ctrl_lbuf.deser_ca_q_en.eq(0),
                if_ctrl_lbuf.deser_cs_n_d_en.eq(0),
                if_ctrl_lbuf.deser_cs_n_q_en.eq(0),
            )
        )
        xfsm_cslogic.act(
            "S_0a",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(1),
            if_ctrl_lbuf.deser_ca_d_en.eq(1),
            if_ctrl_lbuf.deser_ca_q_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(0),
            NextState("S_0b")
        )
        xfsm_cslogic.act(
            "S_0b",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
            if_ctrl_lbuf.deser_ca_d_en.eq(1),
            if_ctrl_lbuf.deser_ca_q_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(1),
            NextState("S_1a")
        )
        xfsm_cslogic.act(
            "S_1a",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(1),
            if_ctrl_lbuf.deser_ca_d_en.eq(1),
            if_ctrl_lbuf.deser_ca_q_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(0),
            NextState("S_1b")
        )
        xfsm_cslogic.act(
            "S_1b",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
            if_ctrl_lbuf.deser_ca_d_en.eq(0),
            if_ctrl_lbuf.deser_ca_q_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(1),
            NextState("POST")
        )
        xfsm_cslogic.act(
            "POST",
            if_ctrl_lbuf.deser_sel_lower_upper.eq(0),
            if_ctrl_lbuf.deser_ca_d_en.eq(0),
            if_ctrl_lbuf.deser_ca_q_en.eq(1),
            if_ctrl_lbuf.deser_cs_n_d_en.eq(0),
            if_ctrl_lbuf.deser_cs_n_q_en.eq(1),
            NextState("IDLE")
        )


class TestBed(Module):
    def __init__(self):

        self.submodules.regfile = DDR5RCD01CSLogic()
        # print(verilog.convert(self.regfile))


def run_test(dut):
    logging.debug('Write test')
    yield from behav_write_word(0x0)
    yield from behav_write_word(0x1)
    yield from behav_write_word(0x0)
    yield from behav_write_word(0x1)
    yield from behav_write_word(0x0)

    logging.debug('Yield from write test.')


def behav_write_word(tb):
    #
    yield tb.d.eq(1)
    yield


if __name__ == "__main__":
    raise UnderConstruction
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
