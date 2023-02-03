#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *


class DDR5RCD01Decoder(Module):
    """
        DDR5 RCD01 Decoder
        ------------------

        DDR5 RCD01 Decoder implements XOR logic to detect positive and negative edges on dcs_n signals.

        1. If a negative edge is detected, then either:
             a single command is present on the CSCA bus
             a sequence of commands started

        Single command behavior
        -----------------------
        If bit CA[1] is high, then the command is 1UI, else a 2UI command.

            Single UI
            ---------
            qvalid will be set for the next 2 clock cycles.
            is_this_ui_odd will toggle once (0,1)
            is_cmd_beginning will produce a single pulse

            Double UI
            ---------
            qvalid will be set for the next 4 clock cycles.
            is_this_ui_odd will toggle twice (0,1,0,1)
            is_cmd_beginning will produce a single pulse

        Sequence of commands behavior
        -----------------------------
            qvalid will be set for z clock cycles:
                z=2k+4j,
                where k is the number of 1UI commands,
                where j is the number of 2UI commands,
            is_this_ui_odd will toggle as long as qvalid is asserted
            is_cmd_beginning will produce a single pulses. Their
            total number will be equal to the number of input commands (k+j)

        Module
        ------
        if_ibuf - The input CSCA bus interface. {CS_n[1:0], CA[6:0], DPAR} signals.
        qvalid - If this signal is asserted, then a valid UI is present on the qcs_n, qca ports.
        qca, qcs_n - CSCA bus
        is_this_ui_odd - This signal is asserted, every time an odd UI is present on the outputs.
        is_cmd_beginning - This signal is asserted every time a begining of a command is detected.

        Parameters
        ------
        N/A

    """

    def __init__(self,
                 if_ibuf,
                 qca,
                 qcs_n,
                 qvalid,
                 is_this_ui_odd,
                 is_cmd_beginning,
                 CS_BIT_SELECT=0
                 ):
        CA_IS_1UI_BIT = 1
        cs_n_w = len(if_ibuf.dcs_n[CS_BIT_SELECT])
        ca_w = len(if_ibuf.dca)

        """
            XOR edge detection
        """
        del_dcs_n = Signal(cs_n_w, reset=~0)
        self.sync += del_dcs_n.eq(if_ibuf.dcs_n[CS_BIT_SELECT])

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
        
        """
        is_cmd_active = Signal()

        is_1_ui_command = Signal()
        self.comb += is_1_ui_command.eq(if_ibuf.dca[CA_IS_1UI_BIT])

        force_active_high = Signal(2)

        del_is_1_ui_command = Signal()
        self.sync += del_is_1_ui_command.eq(is_1_ui_command)

        self.sync += If(
            is_cmd_active & (is_this_ui_odd == 0) & (del_is_1_ui_command == 0),
            force_active_high.eq(3),
        ).Else(
            If(
                force_active_high,
                force_active_high.eq(force_active_high-1),
            ).Else(
                force_active_high.eq(0),
            )

        )
        is_force_non_zero = Signal()
        self.comb += is_force_non_zero.eq(force_active_high > 0)

        self.sync += If(
            det_negedge != 0,
            is_cmd_active.eq(1),
        ).Else(
            If(
                det_posedge,
                is_cmd_active.eq(0),
            )
        )

        pseudo_clock = Signal()
        pseudo_clock_en = Signal()
        self.sync += If(
            det_negedge | is_cmd_active,
            pseudo_clock_en.eq(1),
        ).Else(
            pseudo_clock_en.eq(0),
        )

        self.sync += If(
            det_negedge,
            pseudo_clock.eq(0)
        ).Else(
            If(
                pseudo_clock_en,
                pseudo_clock.eq(~pseudo_clock)
            ).Else(
                pseudo_clock.eq(0)
            )
        )
        """
            if posedge came, but in previous cycle is_1_ui_command was low, then 
            delay deassertion of cmd_active
        """

        self.comb += is_this_ui_odd.eq(pseudo_clock)
        self.comb += qvalid.eq(is_cmd_active | is_force_non_zero)

        self.comb += is_cmd_beginning.eq(qvalid &
                                         (~is_this_ui_odd) & (is_force_non_zero == 0))

        self.comb += qcs_n.eq(del_dcs_n)
        self.comb += qca.eq(del_dca)


if __name__ == "__main__":
    NotSupportedException
