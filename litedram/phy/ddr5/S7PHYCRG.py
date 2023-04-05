#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.fhdl.module import Module
from migen.genlib.cdc import MultiReg

from operator import or_
from functools import reduce

class S7PHYCRG(Module):
    def __init__(self, reset_clock_domain,
                 source_4x, source_4x_90):

        self.rst                = Signal()
        self.rst_set            = False
        self.reset_clock_domain = reset_clock_domain
        self.domain_resets      = {}
        self.domain_CEs         = {}
        self.div_factors        = {}

        # BUFR with BUFMRCE reset sequence
        self.bufr_clr = bufr_clr = Signal()
        bufmrce_CE = Signal()
        bufmrce_90_CE = Signal()
        counter = Signal(8)

        # BUMRCE output
        self.intermediate = Signal()
        self.intermediate_90 = Signal()

        # BUFMRCE
        self.specials += Instance(
            "BUFMRCE",
            i_I=source_4x,
            o_O=self.intermediate,
            i_CE=bufmrce_CE,
        )

        self.specials += Instance(
            "BUFMRCE",
            i_I=source_4x_90,
            o_O=self.intermediate_90,
            i_CE=bufmrce_CE,
        )

        # Reset sequencer
        cd_reset = getattr(self.sync, reset_clock_domain)
        cd_reset += [
            If(self.rst,
                counter.eq(0),
                bufmrce_CE.eq(0),
                bufmrce_90_CE.eq(0),
            ),
            If(counter != 0xFF,
                counter.eq(counter+1)
            ),
            If(counter == 0x20,
                bufr_clr.eq(1),
            ),
            If(counter == 0x40,
                bufmrce_CE.eq(1),
                bufmrce_90_CE.eq(1),
            ),
            If(counter == 0x60,
                bufmrce_CE.eq(0),
                bufmrce_90_CE.eq(0),
            ),
            If(counter == 0x80,
                bufr_clr.eq(0),
            ),
            If(counter == 0xA0,
                bufmrce_CE.eq(1),
                bufmrce_90_CE.eq(1),
            ),
        ]


    def create_clock_domains(self, clock_domains, io_banks):
        for io_bank in io_banks:
            for clk_domain in clock_domains:
                div = 4
                buf_type = "BUFR"
                if "4x" in clk_domain:
                    buf_type="BUFIO"
                    div = None
                elif "2x" in clk_domain:
                    div = 2

                in_clk = self.intermediate
                if "90" in clk_domain:
                    in_clk = self.intermediate_90

                reset_less = True if div is None else False
                setattr(self.clock_domains,
                        f"cd_{clk_domain}_{io_bank}",
                        ClockDomain(reset_less=reset_less, name=f"{clk_domain}_{io_bank}")
                )
                clk = ClockSignal(f"{clk_domain}_{io_bank}")
                buffer_dict = dict(
                    i_I=in_clk,
                    o_O=clk,
                )
                if div is not None:
                    self.div_factors[f"{clk_domain}_{io_bank}"] = div
                    buffer_dict["p_BUFR_DIVIDE"] = str(div)
                    buffer_dict["i_CLR"] = self.bufr_clr

                special = Instance(
                    buf_type,
                    **buffer_dict
                )
                self.specials += special


    def get_rst(self, clock_domain):
        reset = Signal()

        if clock_domain not in self.domain_resets:
            _reset = Signal()
            counter = Signal(max=(64//self.div_factors[clock_domain]))
            _counter = Signal.like(counter)
            for i in range(len(counter)):
                self.specials += Instance(
                    "FDPE",
                    p_INIT  = 1,
                    i_PRE   = self.bufr_clr,
                    i_CE    = 1,
                    i_D     = _counter[i],
                    i_C     = ClockSignal(clock_domain),
                    o_Q     = counter[i],
                )
            self.specials += Instance(
                "FDPE",
                p_INIT  = 1,
                i_PRE   = self.bufr_clr,
                i_CE    = 1,
                i_D     = _reset,
                i_C     = ClockSignal(clock_domain),
                o_Q     = ResetSignal(clock_domain),
            )

            self.comb += [
                If(counter != 0,
                    _counter.eq(counter - 1),
                ),
                _reset.eq(reduce(or_, counter)),
            ]
            self.domain_resets[clock_domain] = _reset

        self.specials += Instance(
            "FDPE",
            p_INIT  = 1,
            i_PRE   = self.bufr_clr,
            i_CE    = 1,
            i_D     = self.domain_resets[clock_domain],
            i_C     = ClockSignal(clock_domain),
            o_Q     = reset,
        )
        return reset


    def get_ce(self, clock_domain):
        CE = Signal()

        if clock_domain not in self.domain_CEs:
            _CE = Signal()
            counter = Signal(max=(256//self.div_factors[clock_domain]))
            _counter = Signal.like(counter)
            for i in range(len(counter)):
                self.specials += Instance(
                    "FDPE",
                    p_INIT  = 1,
                    i_PRE   = self.bufr_clr,
                    i_CE    = 1,
                    i_D     = _counter[i],
                    i_C     = ClockSignal(clock_domain),
                    o_Q     = counter[i],
                )

            self.comb += [
                If(counter != 0,
                    _counter.eq(counter - 1),
                ),
                _CE.eq(~(reduce(or_, counter))),
            ]
            self.domain_CEs[clock_domain] = _CE

        self.specials += Instance(
            "FDCE",
            p_INIT  = 0,
            i_CLR   = self.bufr_clr,
            i_CE    = 1,
            i_D     = self.domain_CEs[clock_domain],
            i_C     = ClockSignal(clock_domain),
            o_Q     = CE,
        )

        return CE


    def add_rst(self, reset_signal):
        assert not self.rst_set
        self.specials += MultiReg(reset_signal, self.rst, self.reset_clock_domain)
        self.rst_set = True


    def do_finalize(self):
        assert self.rst_set
