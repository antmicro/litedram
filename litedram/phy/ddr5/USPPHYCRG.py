#
# This file is part of LiteDRAM.
#
# Copyright (c) 2024 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from litex.soc.cores.clock import USPIDELAYCTRL, USPMMCM
from migen import *
from migen.fhdl.module import Module
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from operator import or_, and_
from functools import reduce

class USPPHYCRG(Module):
    def __init__(
        self,
        sys_clk_freq,
        banks
    ):

        self.rst = Signal(reset=1)
        self.rst_set = False
        self.domain_resets = {}
        self.domain_load = {}
        self.div_factors = {}

        self.banks = banks
        self.sys_clk_freq = sys_clk_freq
        # Clock buffer control
        bufg_div_clr = Signal()
        self.bufg_div_clr = Signal()
        self.bufg_div_clr_90 = Signal()

        bufgdiv_CE = Signal()
        self.bufgdiv_CE = Signal()
        self.bufgdiv_90_CE = Signal()
        counter = Signal(8)
        self.stable_clk = Signal()

        self.clock_domains.cd_sys4x_raw = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_90_raw = ClockDomain(reset_less=True)
        self.submodules.mmcm = mmcm = USPMMCM(speedgrade=-2)
        mmcm.register_clkin(ClockSignal(), sys_clk_freq)
        mmcm.create_clkout(self.cd_sys4x_raw, 4 * sys_clk_freq, buf=None, with_reset=False)
        mmcm.create_clkout(
            self.cd_sys4x_90_raw, 4 * sys_clk_freq, phase=90, buf=None, with_reset=False
        )

        # Fast clock
        self.clock_domains.cd_sys4x_raw_buf = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_90_raw_buf = ClockDomain(reset_less=True)
        self.cd_sys4x_raw_buf.clk.attr.add(("DONT_TOUCH", "TRUE"))
        self.cd_sys4x_90_raw_buf.clk.attr.add(("DONT_TOUCH", "TRUE"))
        attr = set()
        attr.add(("DONT_TOUCH", "TRUE"))
        attr.add(("CLOCK_DELAY_GROUP", "PHY_CE"))
        self.specials += Instance(
            "BUFG",
            name="_sys4x_raw_buf",
            attr=attr,
            i_I=ClockSignal("sys4x_raw"),
            o_O=ClockSignal("sys4x_raw_buf"),
        )
        self.specials += Instance(
            "BUFG",
            name="_sys4x_90_raw_buf",
            attr=attr,
            i_I=ClockSignal("sys4x_90_raw"),
            o_O=ClockSignal("sys4x_90_raw_buf")
        )

        self.source_4x = ClockSignal("sys4x_raw_buf")
        self.source_4x_90 = ClockSignal("sys4x_90_raw_buf")

        self.clock_domains.cd_sys2x_ctrl = ClockDomain()
        self.clock_domains.cd_sys4x_ctrl = ClockDomain()
        self.clock_domains.cd_sys4x_90_ctrl = ClockDomain()
        attr = set()
        attr.add(("DONT_TOUCH", "TRUE"))
        attr.add(("CLOCK_DELAY_GROUP", "PHY_CE"))
        if self.sys_clk_freq > 150e6:
            self.specials += Instance(
                "BUFGCE_DIV",
                name="_sys2x_ctrl_buf",
                attr=attr,
                p_BUFGCE_DIVIDE="2",
                i_I=ClockSignal("sys4x_raw"),
                o_O=self.cd_sys2x_ctrl.clk
            )
        else:
            self.comb += [self.cd_sys2x_ctrl.clk.eq(self.cd_sys4x_ctrl.clk)]
        self.specials += Instance(
            "BUFG",
            name="_sys4x_ctrl_buf",
            attr=attr,
            i_I=ClockSignal("sys4x_raw"),
            o_O=self.cd_sys4x_ctrl.clk
        )
        self.specials += Instance(
            "BUFG",
            name="_sys4x_90_ctrl_buf",
            attr=attr,
            i_I=ClockSignal("sys4x_90_raw"),
            o_O=self.cd_sys4x_90_ctrl.clk
        )

        # IOSERDES, IODELAY and IDELAYCTRL
        self.vtc = Signal()
        self.iodelay_rst = Signal()
        self.serdes_rst = Signal()

        self.idelayctrl_rst = Signal()
        self.idelayctrl_ready = Signal()
        self.load_base_delay = Signal()

        halt = Signal(reset=0)

        # Reset sequencer
        self.sync += [
            # Component mode apply reset sequence
            If(self.rst,
                self.stable_clk.eq(0),
                counter.eq(0),
                halt.eq(0),
            # 1. Force EN_VTC HIGH
                self.vtc.eq(1),
            ).Elif((counter != 0xFF) & ~halt,
                counter.eq(counter+1)
            ),
            If(counter == 0x02,
            # 2. Reset MMCM
               mmcm.reset.eq(1),
            ),
            If(counter == 0x10,
            # 3. Apply reset to all IO devices
                self.iodelay_rst.eq(1),
                self.serdes_rst.eq(1),
                self.idelayctrl_rst.eq(1),
            ),
            # 4. Wait some time

            # Component reset removal procedure
            # 1. Force EN_VTC HIGH
            If(counter == 0x20,
                self.vtc.eq(1),
            ),
            # 2. a
            If(counter == 0x22,
               mmcm.reset.eq(0),
            ),
            # 2. b
            If((counter == 0x23) | (counter == 0x24),
               halt.eq(~mmcm.locked),
            ),
            # Disable Div
            If(counter == 0x28,
                bufgdiv_CE.eq(0),
            ),
            If(counter == 0x30,
                bufg_div_clr.eq(1),
            ),
            If(counter == 0x32,
                bufg_div_clr.eq(0),
            ),
            # Enable Div
            If(counter == 0x38,
                bufgdiv_CE.eq(1),
            ),
            # 2. c Release IODELAY, IOSERDES resets
            If(counter == 0x50,
                self.iodelay_rst.eq(0),
                self.serdes_rst.eq(0),
            ),
            # 2. d Release IDELAYCTRL reset
            If(counter == 0x60,
               self.idelayctrl_rst.eq(0),
            ),
            # 2. e Ready state is not indicated to SW. so no step for ready check
            If(counter == 0xFF,
                self.stable_clk.eq(1),
            ),
        ]

        bufgdiv_CE_1 = Signal()
        self.sync.sys4x_ctrl += [
            self.bufg_div_clr.eq(bufg_div_clr),
            self.bufgdiv_CE.eq(bufgdiv_CE_1),
        ]

        self.sync.sys4x_90_ctrl += [
            self.bufg_div_clr_90.eq(bufg_div_clr),
            bufgdiv_CE_1.eq(bufgdiv_CE),
            self.bufgdiv_90_CE.eq(bufgdiv_CE_1),
        ]

        # IDELAYCTRL setup

        idelayctrl_rst_regs = {}
        idelayctrl_readys = {}
        for bank in banks:
            idelayctrl_rst_regs[bank] = Signal()
            idelayctrl_readys[bank] = Signal()
        for bank in banks:
            attr = set()
            attr.add(("IODELAY_GROUP", f"DDR5_PHY_{bank}"))
            self.specials += Instance("IDELAYCTRL",
                attr = attr,
                p_SIM_DEVICE = "ULTRASCALE",
                i_REFCLK     = ClockSignal("sys2x_ctrl"),
                i_RST        = idelayctrl_rst_regs[bank],
                o_RDY        = idelayctrl_readys[bank]
            )

        _idelayctrl_rst_reg = Signal()
        _idelayctrl_rst_reg_1 = Signal()
        _idelayctrl_rst_reg_1.attr.add(("MAX_FANOUT", 1))
        self.specials += MultiReg(self.idelayctrl_rst, _idelayctrl_rst_reg, "sys2x_ctrl", reset=1)
        self.sync.sys2x_ctrl += [
            _idelayctrl_rst_reg_1.eq(_idelayctrl_rst_reg),
        ]
        for bank in banks:
            attr = set()
            attr.add(("KEEP", "TRUE"))
            attr.add(("MAX_FANOUT", 1))
            self.specials += Instance(
                "FDPE",
                attr=attr,
                p_INIT=1,
                i_PRE=0,
                i_CE=1,
                i_D=_idelayctrl_rst_reg_1,
                i_C=~ClockSignal("sys2x_ctrl"),
                o_Q=idelayctrl_rst_regs[bank],
            )

        self.comb += [
            self.idelayctrl_ready.eq(reduce(and_, [sig for _, sig in idelayctrl_readys.items()])),
        ]
        idelayctrl_ready_1 = Signal()
        idelayctrl_ready_2 = Signal()
        self.sync += [
            idelayctrl_ready_1.eq(self.idelayctrl_ready),
            idelayctrl_ready_2.eq(idelayctrl_ready_1),
        ]
        self.comb += [
            self.load_base_delay.eq(idelayctrl_ready_1 & ~idelayctrl_ready_2),
        ]

    def create_clock_domains(self, clock_domains):
        for clk_domain in clock_domains:
            div = 4
            buf_type = "BUFGCE_DIV"
            if "4x" in clk_domain:
                buf_type="BUFG"
                div = None
            elif "2x" in clk_domain:
                div = 2

            clr = None
            in_clk = self.source_4x
            ce = self.bufgdiv_CE
            if "90" in clk_domain:
                in_clk = self.source_4x_90
                ce = self.bufgdiv_90_CE

            reset_less = True if div is None else False
            setattr(
                self.clock_domains,
                f"cd_{clk_domain}",
                ClockDomain(reset_less=reset_less, name=f"{clk_domain}")
            )
            clk = ClockSignal(f"{clk_domain}")
            buffer_dict = dict(
                name=f"_{clk_domain}_buf",
                i_I=in_clk,
                o_O=clk,
            )
            if div is not None:
                clr = self.bufg_div_clr
                self.div_factors[f"{clk_domain}"] = div
                buffer_dict["p_BUFGCE_DIVIDE"] = str(div)
                buffer_dict["i_CLR"] = self.bufg_div_clr
                buffer_dict["i_CE"] = ce
                if "90" in clk_domain:
                    clr = self.bufg_div_clr_90
                    buffer_dict["i_CLR"] = self.bufg_div_clr_90

            attr = set()
            attr.add(("DONT_TOUCH", "TRUE"))
            special = Instance(
                buf_type,
                attr=attr,
                **buffer_dict
            )
            self.specials += special
            if clr is None:
                continue

            _reset = Signal()
            counter = Signal(max=(64//self.div_factors[clk_domain]))
            _counter = Signal.like(counter)
            _clr = Signal()
            self.specials += MultiReg(clr, _clr, clk_domain, reset=1)
            for i in range(len(counter)):
                self.specials += Instance(
                    "FDPE",
                    p_INIT  = 1,
                    i_PRE   = _clr,
                    i_CE    = self.stable_clk,
                    i_D     = _counter[i],
                    i_C     = ClockSignal(clk_domain),
                    o_Q     = counter[i],
                )
            self.specials += Instance(
                "FDPE",
                p_INIT  = 1,
                i_PRE   = _clr,
                i_CE    = self.stable_clk,
                i_D     = _reset,
                i_C     = ClockSignal(clk_domain),
                o_Q     = ResetSignal(clk_domain),
            )

            self.comb += [
                If(counter != 0,
                    _counter.eq(counter - 1),
                ),
                _reset.eq(reduce(or_, counter)),
            ]

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
        self.sync += self.rst.eq(reset_signal)
        self.rst_set = True

    def do_finalize(self):
        assert self.rst_set
