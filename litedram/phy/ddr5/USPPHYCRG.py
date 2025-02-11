#
# This file is part of LiteDRAM.
#
# Copyright (c) 2024 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.fhdl.module import Module
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from operator import or_
from functools import reduce

class USPPHYCRG(Module):
    def __init__(
        self,
        reset_clock_domain,
        reset_clock_90_domain,
        source_4x,
        source_4x_90,
    ):

        self.rst = Signal(reset=1)
        self.rst_set = False
        self.reset_clock_domain = reset_clock_90_domain
        self.domain_resets = {}
        self.domain_load = {}
        self.div_factors = {}

        # Clock buffer control
        self.bufg_div_clr = bufg_div_clr = Signal()
        self.bufgce_CE = bufgce_CE = Signal()
        bufgce_90_CE = Signal()
        self.bufgce_90_CE_1 = bufgce_90_CE_1 = Signal()
        counter = Signal(8)

        # Fast clock
        self.source_4x = source_4x
        self.source_4x_90 = source_4x_90

        # IOSERDES, IODELAY and IDELAYCTRL
        self.vtc = Signal()
        self.iodelay_rst = Signal()
        self.serdes_rst = Signal()

        idelayctrl_rst = Signal()
        idelayctrl_rst_reg = Signal()
        idelayctrl_ready = Signal()
        idelayctrl_ready_1 = Signal()

        self.load_base_delay = Signal()

        attr = set()
        attr.add(("IODELAY_GROUP", "DDR5_PHY"))
        self.specials += Instance("IDELAYCTRL",
            attr = attr,
            p_SIM_DEVICE = "ULTRASCALE",
            i_REFCLK     = source_4x,
            i_RST        = idelayctrl_rst_reg,
            o_RDY        = idelayctrl_ready
        )

        # Reset sequencer
        cd_reset = getattr(self.sync, reset_clock_90_domain)
        cd_reset += [
            # Component mode apply reset sequence
            If(self.rst,
                counter.eq(0),
            # 1. Force EN_VTC HIGH
                self.vtc.eq(1),
            ).Elif(counter != 0xFF,
                counter.eq(counter+1)
            ),
            If(counter == 0x02,
            # 2. Reset MMCM, assuming that disabling clock is enough
                bufgce_90_CE.eq(0),
            ),
            If(counter == 0x10,
            # 3. Apply reset to all IO devices
                self.iodelay_rst.eq(1),
                self.serdes_rst.eq(1),
                idelayctrl_rst.eq(1),
            ),
            # Clear DIV counters
            If(counter == 0x20,
                bufg_div_clr.eq(1),
            ),
            If(counter == 0x24,
                bufg_div_clr.eq(0),
            ),
            # 4. Wait some time

            # Component reset removal procedure
            # 1. Force EN_VTC HIGH
            If(counter == 0x38,
                self.vtc.eq(1),
            ),
            # 2. a,b => release clock buffers
            If(counter == 0x40,
                bufgce_90_CE.eq(1),
            ),
            # 2. c Release IODELAY, IOSERDES resets
            If(counter == 0x50,
                self.iodelay_rst.eq(0),
                self.serdes_rst.eq(0),
            ),
            # 2. d Release IDELAYCTRL reset
            If(counter == 0x60,
               idelayctrl_rst.eq(0),
            ),
            # 2. e Ready state is not indicated to SW. so no step for ready check
            bufgce_90_CE_1.eq(bufgce_90_CE),
            idelayctrl_rst_reg.eq(idelayctrl_rst),
        ]
        self.sync += [
            idelayctrl_ready_1.eq(idelayctrl_ready),
        ]
        self.comb += [
            self.load_base_delay.eq(idelayctrl_ready & ~idelayctrl_ready_1)
        ]
        cd_reset = getattr(self.sync, reset_clock_domain)
        cd_reset += [
            bufgce_CE.eq(bufgce_90_CE),
        ]

    def create_clock_domains(self, clock_domains):
        for clk_domain in clock_domains:
            div = 4
            buf_type = "BUFGCE_DIV"
            if "4x" in clk_domain:
                buf_type="BUFGCE"
                div = None
            elif "2x" in clk_domain:
                div = 2

            in_clk = self.source_4x
            ce = self.bufgce_CE
            if "90" in clk_domain:
                in_clk = self.source_4x_90
                ce = self.bufgce_90_CE_1

            reset_less = True if div is None else False
            setattr(
                self.clock_domains,
                f"cd_{clk_domain}",
                ClockDomain(reset_less=reset_less, name=f"{clk_domain}")
            )
            clk = ClockSignal(f"{clk_domain}")
            buffer_dict = dict(
                i_I=in_clk,
                o_O=clk,
                i_CE=ce,
            )
            if div is not None:
                self.div_factors[f"{clk_domain}"] = div
                buffer_dict["p_BUFGCE_DIVIDE"] = str(div)
                buffer_dict["i_CLR"] = self.bufg_div_clr

            special = Instance(
                buf_type,
                **buffer_dict
            )
            self.specials += special

    def get_rst(self, clock_domain):
        if clock_domain == "sys":
            return self._raw_reset_signal
        if clock_domain not in self.domain_resets:
            reset_sig = Signal()
            self.specials += MultiReg(self.rst, reset_sig, clock_domain, reset=1)
            self.domain_resets[clock_domain] = reset_sig
        return self.domain_resets[clock_domain]

    def get_load_base(self, clock_domain):
        if clock_domain not in self.domain_load:
            psync = PulseSynchronizer("sys", clock_domain)
            self.submodules += psync
            load_cdc = Signal()
            self.comb += [
                psync.i.eq(self.load_base_delay),
                load_cdc.eq(psync.o)
            ]
            self.domain_load[clock_domain] = load_cdc
        return self.domain_load[clock_domain]

    def get_iodelay_vtc(self, clock_domain):
        return self.vtc

    def get_iodelay_rst(self, clock_domain):
        return self.iodelay_rst

    def get_serdes_rst(self, clock_domain):
        return self.serdes_rst

    def add_rst(self, reset_signal):
        assert not self.rst_set
        self._raw_reset_signal = reset_signal
        self.specials += MultiReg(reset_signal, self.rst, self.reset_clock_domain, reset=1)
        self.rst_set = True

    def do_finalize(self):
        assert self.rst_set
