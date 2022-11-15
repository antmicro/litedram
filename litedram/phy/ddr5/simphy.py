#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect.csr import CSR

from litedram.phy.utils import delayed, Serializer, Deserializer, Latency, SimpleCDC
from litedram.phy.sim_utils import SimPad, SimulationPads, SimSerDesMixin
from litedram.phy.ddr5.basephy import DDR5PHY, DDR5Output


class DDR5SimulationPads(SimulationPads):
    def layout(self, databits=8, nranks=1, dq_dqs_ratio=8, with_sub_channels=False):
        common = [
            SimPad("ck_t", 1),
            SimPad("ck_c", 1),
            SimPad("reset_n", 1),
            SimPad("alert_n", 1),
        ]
        per_channel = [
            ('cs_n', nranks, False),
            ('ca', 14, False),
            ('par', 1, False),
            ('dq', databits, True),
            ('dm_n',  databits // dq_dqs_ratio, True),
            ('dqs_t',  databits // dq_dqs_ratio, True),
            ('dqs_c',  databits // dq_dqs_ratio, True),
        ]
        channels_prefix = [""] if not with_sub_channels else ["A_", "B_"]
        return common + \
                [SimPad(prefix+name, size, io) for prefix in channels_prefix for name, size, io in per_channel]


class DDR5SimPHY(SimSerDesMixin, DDR5PHY):
    """DDR5 simulation PHY with direct 16:1 serializers

    For simulation purpose two additional "DDR" clock domains are requires.
    """
    def __init__(self, aligned_reset_zero=False, dq_dqs_ratio=8, nranks=1, with_sub_channels=False,
                 with_delays=False, with_clock_odelay=False, with_address_odelay=False,
                 num_of_steps=0, step_interval_fs=0, **kwargs):
        databits = 0
        if dq_dqs_ratio == 8:
            databits=8
            pads = DDR5SimulationPads(databits=8, nranks=nranks, dq_dqs_ratio=8,
                                      with_sub_channels=with_sub_channels)
        elif dq_dqs_ratio == 4:
            databits=4
            # databits length taken from DDR5 Tester
            pads = DDR5SimulationPads(databits=4, nranks=nranks, dq_dqs_ratio=4,
                                      with_sub_channels=with_sub_channels)
        else:
            raise NotImplementedError(f"Unspupported DQ:DQS ratio: {dq_dqs_ratio}")

        self.submodules += pads
        super().__init__(pads,
            ser_latency       = Latency(sys2x=Serializer.LATENCY),
            des_latency       = Latency(sys=(Deserializer.LATENCY-1 if aligned_reset_zero else Deserializer.LATENCY)),
            phytype           = "DDR5SimPHY",
            with_sub_channels = with_sub_channels,
            rd_extra_delay    = Latency(sys2x=3),
            with_odelay       = with_delays,
            with_idelay       = with_delays,
            **kwargs)

        _l = self._l

        # fake delays (make no sense in simulation, but sdram.c expects them)
        self.settings.read_leveling = True
        self.settings.delays = num_of_steps if with_delays else 1

        channels_prefix = [""] if not with_sub_channels else ["A_", "B_"]

        cs      = dict(clkdiv="sys2x", clk="sys4x_ddr", xilinx=True)
        cmd     = dict(clkdiv="sys2x", clk="sys4x_ddr", xilinx=True)
        ddr     = dict(clkdiv="sys2x", clk="sys4x_ddr", xilinx=True)
        ddr_90  = dict(clkdiv="sys2x", clk="sys4x_90_ddr", xilinx=True)

        # This configuration mimics Xilinx 7-series serdes behavior
        if aligned_reset_zero:
            ddr["reset_cnt"] = 0
            ddr["aligned"] = True
            cs["reset_cnt"] = 0
            cs["aligned"] = True
            cmd["reset_cnt"] = 0
            cmd["aligned"] = True

        # Clock is shifted 180 degrees to get rising edge in the middle of SDR signals.
        # To achieve that we send negated clock on clk (clk_p).
        ck_t,         ck_c          = (self.out.ck_t, self.out.ck_c)
        cdc_ck_t,     cdc_ck_c      = Signal(len(ck_t)//2), Signal(len(ck_c)//2)

        pad_ck_t,     pad_ck_c      = self.pads.ck_t, self.pads.ck_c
        dly_cdc_ck_t, dly_cdc_ck_c  = Signal.like(pad_ck_t), Signal.like(pad_ck_c)

        simple_cdc = SimpleCDC(
            clkdiv="sys", clk="sys2x",
            i_dw=len(ck_t), o_dw=len(cdc_ck_t),
            i=ck_t, o=cdc_ck_t,
            name=f"ck_t",
            register=True,
        )
        self.submodules += simple_cdc
        simple_cdc = SimpleCDC(
            clkdiv="sys", clk="sys2x",
            i_dw=len(ck_c), o_dw=len(cdc_ck_c),
            i=ck_c, o=cdc_ck_c,
            name=f"ck_c",
            register=True,
        )
        self.submodules += simple_cdc
        self.ser(i=cdc_ck_t, o=dly_cdc_ck_t, name='ck_t', **ddr)
        self.ser(i=cdc_ck_c, o=dly_cdc_ck_c, name='ck_c', **ddr)
        if not with_delays and not with_clock_odelay:
            self.comb += pad_ck_t.eq(dly_cdc_ck_t)
            self.comb += pad_ck_c.eq(dly_cdc_ck_c)
        else:
            params = dict(
                p_NUM_TAPS  = num_of_steps,
                p_TAP_DELAY = step_interval_fs,
                i_INPUT     = dly_cdc_ck_t,
                o_OUTPUT    = pad_ck_t,
                i_C         = ClockSignal("sys2x"),
                i_INC       = _l['ckdly_inc'],
                i_RST       = _l['ckdly_rst'],
            )
            self.specials += Instance("iodly", **params)
            params = dict(
                p_NUM_TAPS  = num_of_steps,
                p_TAP_DELAY = step_interval_fs,
                i_INPUT     = dly_cdc_ck_c,
                o_OUTPUT    = pad_ck_c,
                i_C         = ClockSignal("sys2x"),
                i_INC       = _l['ckdly_inc'],
                i_RST       = _l['ckdly_rst'],
            )
            self.specials += Instance("iodly", **params)

        reset_n = self.out.reset_n
        cdc_reset_n = Signal(len(reset_n)//2)
        simple_cdc = SimpleCDC(
            clkdiv="sys", clk="sys2x",
            i_dw=len(reset_n), o_dw=len(cdc_reset_n),
            i=reset_n, o=cdc_reset_n,
            name=f"reset_n",
            register=True,
        )
        self.submodules += simple_cdc
        self.ser(i=cdc_reset_n, o=self.pads.reset_n, name='reset_n', **ddr)
        self.des(i=self.pads.alert_n, o=self.out.alert_n, name='alert_n', **ddr_90)

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]

        for prefix in prefixes:
            # Command/address
            # CS_n --------------------------------------------------------------------------------
            for it, (basephy_cs, pad) in enumerate(
                zip(getattr(self.out, prefix+'cs_n'), getattr(self.pads, prefix+'cs_n'))):

                cdc_out_cs_n = Signal(len(basephy_cs)//2)
                dly_cdc_out_cs_n = Signal.like(pad)
                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(basephy_cs), o_dw=len(cdc_out_cs_n),
                    i=basephy_cs, o=cdc_out_cs_n,
                    name=f"{prefix}cs_n_{it}",
                    register=True,
                )
                self.submodules += simple_cdc
                self.ser(i=cdc_out_cs_n, o=dly_cdc_out_cs_n, name=f'{prefix}cs_n_{it}', **cs)
                if not with_delays and not with_address_odelay:
                    self.comb += pad.eq(dly_cdc_out_cs_n)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_out_cs_n,
                        o_OUTPUT    = pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = _l[prefix+'csdly_inc'],
                        i_RST       = _l[prefix+'csdly_rst'],
                    )
                    self.specials += Instance("iodly", **params)

            # CA ----------------------------------------------------------------------------------
            for it, (basephy_ca, pad) in enumerate(
                zip(getattr(self.out, prefix+'ca'), getattr(self.pads, prefix+'ca'))):

                cdc_out_ca = Signal(len(basephy_ca)//2)
                dly_cdc_out_ca = Signal.like(pad)
                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(basephy_ca), o_dw=len(cdc_out_ca),
                    i=basephy_ca, o=cdc_out_ca,
                    name=f"{prefix}ca_{it}",
                    register=True,
                )
                self.submodules += simple_cdc
                self.ser(i=cdc_out_ca, o=dly_cdc_out_ca, name=f'{prefix}ca{it}', **cmd)
                if not with_delays and not with_address_odelay:
                    self.comb += pad.eq(dly_cdc_out_ca)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_out_ca,
                        o_OUTPUT    = pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_inc(it, _l[prefix+'cadly_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'cadly_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)

            # PAR ---------------------------------------------------------------------------------
            basephy_par = getattr(self.out, prefix+'par')
            pad = getattr(self.pads, prefix+'par')

            cdc_out_par = Signal(len(basephy_par)//2)
            dly_cdc_out_par = Signal.like(pad)
            simple_cdc = SimpleCDC(
                clkdiv="sys", clk="sys2x",
                i_dw=len(basephy_par), o_dw=len(cdc_out_par),
                i=basephy_par, o=cdc_out_par,
                name=f"{prefix}par_{it}",
                register=True,
            )
            self.submodules += simple_cdc
            self.ser(i=cdc_out_par, o=dly_cdc_out_par, name=f'{prefix}par', **cmd)
            if not with_delays and not with_address_odelay:
                self.comb += pad.eq(dly_cdc_out_par)
            else:
                params = dict(
                    p_NUM_TAPS  = num_of_steps,
                    p_TAP_DELAY = step_interval_fs,
                    i_INPUT     = dly_cdc_out_par,
                    o_OUTPUT    = pad,
                    i_C         = ClockSignal("sys2x"),
                    i_INC       = _l[prefix+'pardly_inc'],
                    i_RST       = _l[prefix+'pardly_rst'],
                )
                self.specials += Instance("iodly", **params)

            # Tristate I/O (separate for simulation)
            for it in range(self.databits//dq_dqs_ratio):
                # DQS -----------------------------------------------------------------------------
                dqs_t_o, dqs_c_o                 = \
                    getattr(self.out, prefix+'dqs_t_o')[it], getattr(self.out, prefix+'dqs_c_o')[it]

                cdc_dqs_t_o, cdc_dqs_c_o         = \
                    Signal(len(dqs_t_o)//2), Signal(len(dqs_c_o)//2)

                dqs_t_o_pad, dqs_c_o_pad         = \
                    getattr(self.pads, prefix+'dqs_t_o')[it], getattr(self.pads, prefix+'dqs_c_o')[it]
                dly_cdc_dqs_t_o, dly_cdc_dqs_c_o = \
                    Signal.like(dqs_t_o_pad), Signal.like(dqs_c_o_pad)

                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(dqs_t_o), o_dw=len(cdc_dqs_t_o),
                    i=dqs_t_o, o=cdc_dqs_t_o,
                    name=f"{prefix}dqs_t_o_{it}",
                    register=True,
                )
                self.submodules += simple_cdc
                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(dqs_c_o), o_dw=len(cdc_dqs_c_o),
                    i=dqs_c_o, o=cdc_dqs_c_o,
                    name=f"{prefix}dqs_c_o_{it}",
                    register=True,
                )
                self.submodules += simple_cdc

                self.ser(i=cdc_dqs_t_o,
                         o=dly_cdc_dqs_t_o,
                         name=f'{prefix}dqs_t_o{it}', **ddr)
                self.ser(i=cdc_dqs_c_o,
                         o=dly_cdc_dqs_c_o,
                         name=f'{prefix}dqs_c_o{it}', **ddr)

                if not with_delays:
                    self.comb += dqs_t_o_pad.eq(dly_cdc_dqs_t_o)
                    self.comb += dqs_c_o_pad.eq(dly_cdc_dqs_c_o)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_dqs_t_o,
                        o_OUTPUT    = dqs_t_o_pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'wdly_dqs_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'wdly_dqs_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_dqs_c_o,
                        o_OUTPUT    = dqs_c_o_pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'wdly_dqs_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'wdly_dqs_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)


                dqs_t_pad, dqs_c_pad             = \
                    getattr(self.pads, prefix+'dqs_t')[it], getattr(self.pads, prefix+'dqs_c')[it]
                dly_dqs_t_i, dly_dqs_c_i = \
                    Signal.like(dqs_t_pad), Signal.like(dqs_c_pad)
                dqs_t_i, dqs_c_i                 = \
                    getattr(self.out, prefix+'dqs_t_i')[it], getattr(self.out, prefix+'dqs_c_i')[it]

                if not with_delays:
                    self.comb += dly_dqs_t_i.eq(dqs_t_pad)
                    self.comb += dly_dqs_c_i.eq(dqs_c_pad)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dqs_t_pad,
                        o_OUTPUT    = dly_dqs_t_i,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'rdly_dqs_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'rdly_dqs_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dqs_c_pad,
                        o_OUTPUT    = dly_dqs_c_i,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'rdly_dqs_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'rdly_dqs_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)

                self.des(o=dqs_t_i, i=dly_dqs_t_i, name=f'{prefix}dqs_t_i{it}', **ddr)
                self.des(o=dqs_c_i, i=dly_dqs_c_i, name=f'{prefix}dqs_c_i{it}', **ddr)

                # DM_n ----------------------------------------------------------------------------
                basephy_dm = getattr(self.out, prefix+'dm_n_o')[it]
                delay_dm   = Signal.like(basephy_dm)
                out_dm     = Signal.like(basephy_dm)

                self.sync += delay_dm.eq(basephy_dm[1:])
                self.comb += out_dm.eq(Cat(delay_dm[:-1], basephy_dm[0]))
                cdc_out_dm = Signal(len(out_dm)//2)
                dm_n_pad   = getattr(self.pads, prefix+'dm_n_o')[it]
                dly_cdc_out_dm = Signal.like(dm_n_pad)

                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(out_dm), o_dw=len(cdc_out_dm),
                    i=out_dm, o=cdc_out_dm,
                    name=f"{prefix}dm_o_{it}",
                    register=True,
                )
                self.submodules += simple_cdc

                if not with_delays:
                    self.comb += dm_n_pad.eq(dly_cdc_out_dm)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_out_dm,
                        o_OUTPUT    = dm_n_pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'wdly_dm_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'wdly_dm_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)
                self.ser(i=cdc_out_dm, o=dly_cdc_out_dm,
                         name=f'{prefix}dm_n_o{it}', **ddr_90)

                basephy_dm_i =  getattr(self.out, prefix+'dm_n_i')[it]
                in_dm = Signal.like(basephy_dm_i)
                self.des(o=in_dm, i=getattr(self.pads, prefix+'dm_n')[it],
                         name=f'{prefix}dm_n_i{it}', **ddr_90)
                delay_dm_i = Signal(2)
                self.sync += delay_dm_i.eq(in_dm[-2:])
                self.comb += basephy_dm_i.eq(Cat(delay_dm_i, in_dm[:-2]))

            # DQ ----------------------------------------------------------------------------------
            for it in range(self.databits):
                basephy_dq = getattr(self.out, prefix+'dq_o')[it]
                delay_dq   = Signal.like(basephy_dq)
                out_dq     = Signal.like(basephy_dq)

                self.sync += delay_dq.eq(basephy_dq[1:])
                self.comb += out_dq.eq(Cat(delay_dq[:-1], basephy_dq[0]))

                cdc_out_dq = Signal(len(out_dq)//2)

                out_dq_pad = getattr(self.pads, prefix+'dq_o')[it]
                dly_cdc_out_dq = Signal.like(out_dq_pad)

                simple_cdc = SimpleCDC(
                    clkdiv="sys", clk="sys2x",
                    i_dw=len(out_dq), o_dw=len(cdc_out_dq),
                    i=out_dq, o=cdc_out_dq,
                    name=f"{prefix}dq_o_{it}",
                    register=True,
                )
                self.submodules += simple_cdc

                self.ser(i=cdc_out_dq, o=dly_cdc_out_dq, name=f'{prefix}dq_o{it}', **ddr_90)

                if not with_delays:
                    self.comb += out_dq_pad.eq(dly_cdc_out_dq)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dly_cdc_out_dq,
                        o_OUTPUT    = out_dq_pad,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'wdly_dq_inc'], prefix, "sys2x", True),
                        i_RST       = self.get_rst(it, _l[prefix+'wdly_dq_rst'], prefix, "sys2x", True),
                    )
                    self.specials += Instance("iodly", **params)

                basephy_dq_i = getattr(self.out, prefix+'dq_i')[it]
                in_dq = Signal.like(basephy_dq_i)

                dq_i_pad     = getattr(self.pads, prefix+'dq')[it]
                dly_in_dq = Signal.like(dq_i_pad)
                if not with_delays:
                    self.comb += dly_in_dq.eq(dq_i_pad)
                else:
                    params = dict(
                        p_NUM_TAPS  = num_of_steps,
                        p_TAP_DELAY = step_interval_fs,
                        i_INPUT     = dq_i_pad,
                        o_OUTPUT    = dly_in_dq,
                        i_C         = ClockSignal("sys2x"),
                        i_INC       = self.get_rst(it, _l[prefix+'rdly_dq_inc'], prefix, "sys2x"),
                        i_RST       = self.get_rst(it, _l[prefix+'rdly_dq_rst'], prefix, "sys2x"),
                    )
                    self.specials += Instance("iodly", **params)
                self.des(o=in_dq, i=dly_in_dq,
                         name=f'{prefix}dq_i{it}', reset_cnt=-2, **ddr_90)
                delay_dq_i = Signal(2)
                self.sync += delay_dq_i.eq(in_dq[-2:])
                self.comb += basephy_dq_i.eq(Cat(delay_dq_i, in_dq[:-2]))

            # Output enable signals can be and should be serialized as well
            out_dqs_t_oe = getattr(self.out, prefix+'dqs_oe')[0]
            cdc_out_dqs_t_oe = Signal(len(out_dqs_t_oe)//2)
            simple_cdc = SimpleCDC(
                clkdiv="sys", clk="sys2x",
                i_dw=len(out_dqs_t_oe), o_dw=len(cdc_out_dqs_t_oe),
                i=out_dqs_t_oe, o=cdc_out_dqs_t_oe,
                name=f"{prefix}dqs_t_oe",
                register=True,
            )
            self.submodules += simple_cdc
            self.ser(i=cdc_out_dqs_t_oe,
                     o=getattr(self.pads, prefix+'dqs_t_oe'),
                     name=f'{prefix}dqs_t_oe', **ddr)

            out_dqs_c_oe = getattr(self.out, prefix+'dqs_oe')[0]
            cdc_out_dqs_c_oe = Signal(len(out_dqs_c_oe)//2)
            simple_cdc = SimpleCDC(
                clkdiv="sys", clk="sys2x",
                i_dw=len(out_dqs_c_oe), o_dw=len(cdc_out_dqs_c_oe),
                i=out_dqs_c_oe, o=cdc_out_dqs_c_oe,
                name=f"{prefix}dqs_c_oe",
                register=True,
            )
            self.submodules += simple_cdc
            self.ser(i=cdc_out_dqs_c_oe,
                     o=getattr(self.pads, prefix+'dqs_c_oe'),
                     name=f'{prefix}dqs_c_oe', **ddr)

            basephy_dq_oe = getattr(self.out, prefix+'dq_oe')[0]
            delay_dq_oe = Signal.like(basephy_dq_oe)
            out_dq_oe = Signal.like(basephy_dq_oe)
            self.sync += delay_dq_oe.eq(basephy_dq_oe[1:])
            self.comb += out_dq_oe.eq(Cat(delay_dq_oe[:-1], basephy_dq_oe[0]))
            cdc_out_dq_oe = Signal(len(out_dq_oe)//2)
            simple_cdc = SimpleCDC(
                clkdiv="sys", clk="sys2x",
                i_dw=len(out_dq_oe), o_dw=len(cdc_out_dq_oe),
                i=out_dq_oe, o=cdc_out_dq_oe,
                name=f"{prefix}dq_oe",
                register=True,
            )
            self.submodules += simple_cdc
            self.ser(i=cdc_out_dq_oe,
                     o=getattr(self.pads, prefix+'dq_oe'),
                     name=f'{prefix}dq_oe', **ddr_90)

            self.ser(i=cdc_out_dq_oe,
                     o=getattr(self.pads, prefix+'dm_n_oe'),
                     name=f'{prefix}dm_n_oe', **ddr_90)
