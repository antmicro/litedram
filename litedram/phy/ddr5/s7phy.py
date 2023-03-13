#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import and_
from functools import reduce

from migen import *
from migen.genlib.fifo import _FIFOInterface
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *

from litedram.phy.utils import delayed, Latency
from litedram.phy.sim_utils import SimpleCDC, SimpleCDCWrap, SimpleCDCr
from litedram.phy.ddr5.basephy import DDR5PHY

from litedram.phy.s7common import S7Common

class Xilinx7SeriesAsyncFIFO(Module):
    LATENCY=5 # 3 to pass through memory and 1 for output register
    WCL_LATENCY=6

    def __init__(self, wclk, rclk, width=72):
        assert type(wclk) == str
        assert type(rclk) == str
        assert width in [4, 9, 18, 36, 72], f"Xilinx 7 Sereis FIFO primitive supports widtths: "\
            "4,9,18,36, or 72, you tried {width}"

        self.width = width
        self.DI = Signal(width)
        self.WREN = Signal()
        self.FULL = Signal()

        self.DO = Signal(width)
        self.RDEN = Signal()
        self.EMPTY = Signal()
        self._rst  = Signal()

        fifo_primitive = "FIFO18E1"
        fifo_mode = "FIFO18"
        if width ==36:
            fifo_mode = "FIFO18_36"
        if width > 36:
            fifo_mode = "FIFO36_72"
            fifo_primitive = "FIFO36E1"

        i_cd = getattr(self.sync, wclk)
        rst = Signal(reset_less=True)

        w_rst = Signal(reset=1)
        w_cnt = Signal(3)
        i_cd += [
            If(w_cnt<5,
                w_cnt.eq(w_cnt+1),
            ).Else(
                w_rst.eq(0),
            )
        ]

        o_cd = getattr(self.sync, rclk)
        r_rst = Signal(reset=1)
        r_cnt = Signal(3)
        o_cd += [
            If(r_cnt<5,
                r_cnt.eq(r_cnt+1),
            ).Else(
                r_rst.eq(0),
            )
        ]

        rst_comb = Signal(reset_less=True)
        self.comb += rst_comb.eq(r_rst | w_rst)

        self.specials += Instance(
            "FDPE",
            p_INIT          = 1,
            o_Q             = rst,
            i_C             = ClockSignal(rclk),
            i_CE            = 1,
            i_PRE           = self._rst,
            i_D             = rst_comb,
        )

        self.specials += Instance(
            fifo_primitive,
            p_EN_SYN        = "FALSE",
            p_DO_REG        = 1,
            p_FIFO_MODE     = fifo_mode,
            p_DATA_WIDTH    = width,
            i_RST           = rst,
            i_WRCLK         = ClockSignal(wclk),
            i_WREN          = self.WREN,
            o_FULL          = self.FULL,
            i_DI            = self.DI[:(7*width)//8+1],
            i_DIP           = self.DI[(7*width)//8+1:],
            i_RDEN          = self.RDEN,
            i_RDCLK         = ClockSignal(rclk),
            o_EMPTY         = self.EMPTY,
            o_DO            = self.DO[:(7*width)//8+1],
            o_DOP           = self.DO[(7*width)//8+1:],
        )


class Xilinx7SeriesAsyncFIFOWrap(Module, _FIFOInterface):
    LATENCY     = Xilinx7SeriesAsyncFIFO.LATENCY
    WCL_LATENCY = Xilinx7SeriesAsyncFIFO.WCL_LATENCY

    def __init__(self, wclk, rclk, i_dw, o_dw, name=None):
        _FIFOInterface.__init__(self, max(i_dw, o_dw), 512)
        width = max(i_dw, o_dw)
        fifo_72 = (width+71)//72
        cdcs = [Xilinx7SeriesAsyncFIFO(wclk, rclk) for _ in range(fifo_72)]
        self.submodules += cdcs

        self._rst = Signal()
        for cdc in cdcs:
            self.comb += cdc._rst.eq(self._rst)

        intermediate_din  = Signal(width)
        intermediate_dout = Signal(width)
        do_read           = Signal(reset=1)
        do_write          = Signal(reset=1)
        assert max(i_dw, o_dw)//min(i_dw, o_dw) in [1,2]
        w_cnt               = Signal()
        r_cnt               = Signal()
        r_cnt_i             = Signal()
        i_cd = getattr(self.sync, wclk)
        o_cd = getattr(self.sync, rclk)

        self.comb += [
            self.readable.eq(reduce(and_, [~cdc.EMPTY for cdc in cdcs])),
            *[cdc.RDEN.eq(self.re & do_read) for cdc in cdcs],
            self.writable.eq(reduce(and_, [~cdc.FULL for cdc in cdcs])),
            *[cdc.WREN.eq(self.we & do_write) for cdc in cdcs],
            *[cdc.DI.eq(intermediate_din[i*72:(i+1)*72]) for i, cdc in enumerate(cdcs)],
            *[intermediate_dout[i*72:(i+1)*72].eq(cdc.DO) for i, cdc in enumerate(cdcs)],
        ]
        if i_dw < width:
            self.comb += self.dout.eq(intermediate_dout)
            reg = Signal(i_dw)
            self.comb += intermediate_din.eq(Cat(reg, self.din[:i_dw]))
            self.comb += do_write.eq((w_cnt == 1))
            i_cd += [
                If((w_cnt == 1),
                    w_cnt.eq(0),
                ).Elif(self.we,
                    reg.eq(self.din[:i_dw]),
                    w_cnt.eq(1),
                )
            ]
        elif o_dw < width:
            self.comb += intermediate_din.eq(self.din)
            self.comb += self.dout.eq(intermediate_dout.part(r_cnt_i*o_dw, o_dw))
            self.comb += do_read.eq((r_cnt == 0) & self.re)
            o_cd += [
                If((r_cnt == 0) & self.re,
                    r_cnt.eq(1),
                    r_cnt_i.eq(0),
                ).Else(
                    r_cnt.eq(0),
                    r_cnt_i.eq(1),
                )
            ]


class S7DDR5PHY(DDR5PHY, S7Common):
    def __init__(self, pads, *, iodelay_clk_freq, with_odelay,
                 with_idelay=True, with_per_dq_idelay=False,
                 with_sub_channels=False, pin_domains=None, pin_banks=None,
                 **kwargs):
        self.iodelay_clk_freq = iodelay_clk_freq

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]
        if pin_domains is None or not with_sub_channels:
            def cdc(i):
                o = Signal()
                psync = PulseSynchronizer("sys", "sys2x_io_bank34")
                self.submodules += psync
                self.comb += [
                    psync.i.eq(i),
                    o.eq(psync.o),
                ]
                return o

            def cdc_90(i):
                o = Signal()
                psync = PulseSynchronizer("sys", "sys2x_90_io_bank34")
                self.submodules += psync
                self.comb += [
                    psync.i.eq(i),
                    o.eq(psync.o),
                ]
                return o
            ca_domain = "sys2x_io_bank34"
            dq_domains = {prefix:"sys2x_90_io_bank34" for prefix in prefixes}
            dqs_domains = {prefix:"sys2x_io_bank34" for prefix in prefixes}
        else:
            ca_domain = "sys2x_io_bank33"
            dq_domains = {"A_":"sys2x_90_io_bank34", "B_":"sys2x_90_io_bank32"}
            dqs_domains = {"A_":"sys2x_io_bank34", "B_":"sys2x_io_bank32"}
            cdc = None
            cdc_90 = None


        def cdc_any(i, target):
            o = Signal()
            psync = PulseSynchronizer("sys", target)
            self.submodules += psync
            self.comb += [
                psync.i.eq(i),
                o.eq(psync.o),
            ]
            return o
        SimpleCDC.set_register()
        SimpleCDCWrap.reset_latency()

        # DoubleRateDDR5PHY outputs half-width signals (comparing to DDR5PHY) in sys2x domain.
        # This allows us to use 8:1 DDR OSERDESE2/ISERDESE2 to (de-)serialize the data.
        super().__init__(pads,
            ser_latency       = Latency(sys2x=1),  # OSERDESE2 4:1 DDR (2 full-rate clocks)
            des_latency       = Latency(sys=2),  # ISERDESE2 NETWORKING
            phytype           = self.__class__.__name__,
            with_sub_channels = with_sub_channels,
            ca_domain         = ca_domain,
            dq_domain         = dq_domains,
            wr_dqs_domain     = dqs_domains,
            csr_ca_cdc        = cdc,
            csr_cdc           = cdc,
            csr_cdc_90        = cdc_90,
            csr_dq_cdc        = {prefix:cdc_90 for prefix in prefixes},
            csr_dqs_cdc       = {prefix:cdc for prefix in prefixes},
            ca_cdc_min_max_delay =
                (Latency(sys2x=SimpleCDCWrap.LATENCY), Latency(sys2x=(SimpleCDCWrap.LATENCY))),
            wr_cdc_min_max_delay =
                (Latency(sys2x=SimpleCDCWrap.LATENCY), Latency(sys2x=(SimpleCDCWrap.LATENCY))),
            with_odelay       = with_odelay,
            with_idelay       = with_idelay,
            rd_extra_delay    = Latency(sys2x=3),
            with_per_dq_idelay=with_per_dq_idelay,
            **kwargs
        )

        # nibble to output mapping
        mult = self.dq_dqs_ratio//4

        max_delay_taps = math.ceil(self.tck/(1/2/32/iodelay_clk_freq))

        CSRs    = self.CSRs
        CDCCSRs = self.CDCCSRs

        self.settings.delays = max_delay_taps
        self.settings.write_leveling = True
        self.settings.write_latency_calibration = True
        self.settings.write_dq_dqs_training = True
        self.settings.read_leveling = True

        # Serialization ----------------------------------------------------------------------------
        pin_csr_mapping = {
            "ck_t":    ((CDCCSRs["ckdly_inc"],      CDCCSRs["ckdly_rst"]),      None),
        }
        for prefix in prefixes:
            pin_csr_mapping.update({
                f"{prefix}par":   (
                    (CDCCSRs[f"{prefix}pardly_inc"],   CDCCSRs[f"{prefix}pardly_rst"]),
                     None),
                f"{prefix}ca":    (
                    (CDCCSRs[f"{prefix}cadly_inc"],    CDCCSRs[f"{prefix}cadly_rst"]),
                     None),
                f"{prefix}cs_n":  (
                    (CDCCSRs[f"{prefix}csdly_inc"],    CDCCSRs[f"{prefix}csdly_rst"]),
                     None),
                f"{prefix}dq":    (
                    (CDCCSRs[f"{prefix}wdly_dq_inc"],  CDCCSRs[f"{prefix}wdly_dq_rst"]),
                    (CDCCSRs[f"{prefix}rdly_dq_inc"],  CDCCSRs[f"{prefix}rdly_dq_rst"])),
                f"{prefix}dqs_t": (
                    (CDCCSRs[f"{prefix}wdly_dqs_inc"], CDCCSRs[f"{prefix}wdly_dqs_rst"]),
                    (CDCCSRs[f"{prefix}rdly_dqs_inc"], CDCCSRs[f"{prefix}rdly_dqs_rst"])),
            })

        SimpleCDC.set_register()
        if pin_domains is not None and with_sub_channels:
            # Clock
            clk_dly = Signal()
            clk_ser = Signal()
            cdc_ck_t = Signal(4)
            self.comb += cdc_ck_t.eq(self.clk_pattern&0xF)

            # Every other signal should be realligned to clock.
            self.oserdese2_ddr(
                din=cdc_ck_t,
                **(dict(dout_fb=clk_ser) if with_odelay else dict(dout=clk_dly)),
                clkdiv="sys2x_io_bank33", clk="sys4x_io_bank33", rst_sig = CSRs['_rst'].storage,
            )
            if with_odelay:
                self.odelaye2(
                    din=clk_ser,
                    dout=clk_dly,
                    rst=CDCCSRs['ckdly_rst'],
                    inc=CDCCSRs['ckdly_inc'],
                    clk="sys2x_io_bank33",
                )
            self.obufds(din=clk_dly, dout=self.pads.ck_t, dout_b=self.pads.ck_c)
            cdc_cache = {}
            dq_oe = {}
            for pin, count in pads.layout:
                assert pin in pin_domains, (pin, pin_domains)
                assert pin not in pin_banks or count <= len(pin_banks[pin]), (pin, count)
                for i in range(count):
                    if "_c" == pin[-2:] or "ck_" in pin:
                        continue
                    (_out, _in) = pin_domains[pin]
                    suffix = ""
                    _pin = pin
                    if "_t" in pin:
                        complementary_pin = pin[:-2]+"_c"

                    if _pin in pin_banks:
                        suffix = f"_{pin_banks[_pin][i]}"
                        if _out is not None:
                            _out = (_out[0]+suffix, _out[1]+suffix)
                        if _in is not None:
                            _in = (_in[0]+suffix, _in[1]+suffix)

                    _pin_oe = None
                    if hasattr(self.out, _pin+"_oe"):
                        _pin_oe = _pin + "_oe"
                        _pin_o  = _pin + "_o"
                        _pin_i  = _pin + "_i"
                    elif _pin[-2:] == "_t" and hasattr(self.out, _pin[:-2]+"_oe"):
                        _pin_oe = _pin[:-2] + "_oe"
                        _pin_o  = _pin + "_o"
                        _pin_i  = _pin + "_i"
                    else:
                        _pin_o = _pin
                        _pin_i = _pin

                    if _out is not None:
                        out_sig = getattr(self.out, _pin_o)
                        if isinstance(out_sig, list):
                            out_sig = getattr(self.out, _pin_o)[i]

                        if _pin_oe is not None:
                            idx = i
                            out_sig_oe = None
                            if _pin_oe in ["A_dq_oe", "B_dq_oe"]:
                                idx //= self.dq_dqs_ratio
                                if (_pin_oe, idx) in dq_oe:
                                    out_sig_oe = dq_oe[(_pin_oe, idx)]
                            if out_sig_oe is None:
                                out_sig_oe_ = getattr(self.out, _pin_oe)
                                if isinstance(out_sig_oe_, list):
                                    out_sig_oe_ = getattr(self.out, _pin_oe)[idx]

                                out_sig_oe = Signal.like(out_sig_oe_)
                                self.comb += [out_sig_oe.eq(~out_sig_oe_)]
                            if _pin_oe in ["A_dq_oe", "B_dq_oe"]:
                                dq_oe[(_pin_oe, idx)] = out_sig_oe[:4]

                        output    = Signal()
                        delay     = Signal()
                        tri_state = Signal()
                        _with_odelay = with_odelay and pin in pin_csr_mapping
                        if _pin_oe is not None:
                            oserdes = self.oserdese2_ddr_with_tri(
                                din     = out_sig[:4],
                                **(dict(dout_fb = delay) if _with_odelay else dict(dout = output)),
                                tin     = out_sig_oe[:4],
                                tout    = tri_state,
                                clkdiv  = _out[0],
                                clk     = _out[1],
                                rst_sig = CSRs['_rst'].storage,
                            )
                        else:
                            oserdes = self.oserdese2_ddr(
                                din = out_sig[:4],
                                **(dict(dout_fb=delay) if _with_odelay else dict(dout = output)),
                                clkdiv  = _out[0],
                                clk     = _out[1],
                                rst_sig = CSRs['_rst'].storage,
                            )
                        if with_odelay and pin in pin_csr_mapping:
                            dq = True if pin in ["A_dq", "B_dq"] else False
                            inc = None
                            rst = None
                            if (pin, 0, _out[0]) not in cdc_cache:
                                cdc_cache[(pin, 0, _out[0])] = (
                                    cdc_any(pin_csr_mapping[pin][0][0], _out[0]),
                                    cdc_any(pin_csr_mapping[pin][0][1], _out[0])
                                )
                            inc_sig, rst_sig = cdc_cache[(pin, 0, _out[0])]
                            if count > 1:
                                inc = self.get_inc(i, inc_sig, pin[:2], _out[0], dq=dq)
                                rst = self.get_rst(i, rst_sig, pin[:2], _out[0], dq=dq)
                            else:
                                inc  = inc_sig
                                rst  = rst_sig

                            self.odelaye2(
                                din  = delay,
                                dout = output,
                                rst  = rst,
                                inc  = inc,
                                clk  = _out[0],
                            )

                    if _in is not None:
                        _input = Signal()
                        _delayed_input = Signal()
                        if with_idelay and pin in pin_csr_mapping:
                            dq = True if pin in ["A_dq", "B_dq"] else False
                            inc = None
                            rst = None
                            cd = _in[0]
                            if _out is not None:
                                cd = _out[0]
                            if (pin, 1, cd) not in cdc_cache:
                                cdc_cache[(pin, 1, _out[0])] = (
                                    cdc_any(pin_csr_mapping[pin][1][0], cd),
                                    cdc_any(pin_csr_mapping[pin][1][1], cd)
                                )
                            inc_sig, rst_sig = cdc_cache[(pin, 0, _out[0])]
                            if count > 1:
                                inc  = self.get_inc(i, inc_sig, pin[:2], cd, dq=dq)
                                rst  = self.get_rst(i, rst_sig, pin[:2], cd, dq=dq)
                            else:
                                inc  = inc_sig
                                rst  = rst_sig
                            self.idelaye2(
                                din  = _input,
                                dout = _delayed_input,
                                rst  = rst,
                                inc  = inc,
                                init = max_delay_taps-1,
                                clk  = cd,
                                dec  = True,
                            )
                        else:
                            _delayed_input = _input

                        to_phy = getattr(self.out, _pin_i)
                        if isinstance(to_phy, list):
                            to_phy = getattr(self.out, _pin_i)[i]

                        self.iserdese2_ddr(
                            din    = _delayed_input,
                            dout   = to_phy,
                            clk    = _in[1],
                            clkdiv = _in[0],
                            rst_sig = CSRs['_rst'].storage,
                        )


                    if "_t" in pin and _in is None:
                        if count > 1:
                            self.obufds(din=output, dout=getattr(self.pads, pin)[i], dout_b=getattr(self.pads, complementary_pin)[i])
                        else:
                            self.obufds(din=output, dout=getattr(self.pads, pin), dout_b=getattr(self.pads, complementary_pin))
                    elif "_t" in pin:
                        pad_t = None
                        pad_c = None
                        if count > 1:
                            pad_t = getattr(self.pads, pin)[i]
                            pad_c = getattr(self.pads, complementary_pin)[i]
                        else:
                            pad_t = getattr(self.pads, pin)
                            pad_c = getattr(self.pads, complementary_pin)
                        self.iobufds(
                            din      = output,
                            dout     = _input,
                            tin      = tri_state,
                            dinout   = pad_t,
                            dinout_b = pad_c,
                        )
                    elif _in is None:
                        if count > 1:
                            self.comb += getattr(self.pads, pin)[i].eq(output)
                        else:
                            self.comb += getattr(self.pads, pin).eq(output)
                    elif _in is not None and _out is not None:
                        pad = None
                        if count > 1:
                            pad = getattr(self.pads, pin)[i]
                        else:
                            pad = getattr(self.pads, pin)
                        self.iobuf(
                            din    = output,
                            dout   = _input,
                            dinout = pad,
                            tin    = tri_state,
                        )
                    elif _in is not None:
                        if count > 1:
                            self.comb += _input.eq(getattr(self.pads, pin)[i])
                        else:
                            self.comb += _input.eq(getattr(self.pads, pin))
        else:
            ddr     = dict(
                clkdiv="sys2x_io_bank34",
                clk="sys4x_io_bank34",
                rst_sig=self._rst_cdc
            )
            cmd     = dict(
                clkdiv="sys2x_io_bank34",
                clk="sys4x_io_bank34",
                rst_sig=self._rst_cdc
            )
            cs      = dict(
                clkdiv="sys2x_io_bank34",
                clk="sys4x_io_bank34",
                rst_sig=self._rst_cdc
            )
            ddr_90  = dict(
                clkdiv="sys2x_90_io_bank34",
                clk="sys4x_90_io_bank34",
                rst_sig=self._rst_cdc_90
            )

            # Clock
            clk_dly = Signal()
            clk_ser = Signal()
            cdc_ck_t = Signal(4)
            self.comb += cdc_ck_t.eq(self.clk_pattern&0xF)

            # Every other signal should be realligned to clock.
            self.oserdese2_ddr(
                din=cdc_ck_t,
                **(dict(dout_fb=clk_ser) if with_odelay else dict(dout=clk_dly)),
                **ddr,
            )
            if with_odelay:
                self.odelaye2(
                    din=clk_ser,
                    dout=clk_dly,
                    rst=CDCCSRs['ckdly_rst'],
                    inc=CDCCSRs['ckdly_inc'],
                    clk="sys2x_io_bank34",
                )
            self.obufds(din=clk_dly, dout=self.pads.ck_t, dout_b=self.pads.ck_c)

            for const in ["mir", "cai", "ca_odt"]:
                if hasattr(self.pads, const):
                    self.comb += getattr(self.pads, const).eq(0)

            reset_n = self.out.reset_n[:4]
            reset_n_o = getattr(self.pads, 'reset_n')
            self.oserdese2_ddr(din=reset_n, dout=reset_n_o, **ddr)
            self.iserdese2_ddr(din=self.pads.alert_n, dout=self.out.alert_n,
                clkdiv="sys_io_bank34",clk="sys4x_io_bank34", rst_sig=0)

            prefixes = [""] if not with_sub_channels else ["A_", "B_"]
            for prefix in prefixes:
                # Commands
                # CS_n --------------------------------------------------------------------------------
                nranks = len(getattr(self.pads, prefix+"cs_n"))
                cs_n_ser = Signal(nranks)
                for it, (basephy_cs, pad) in enumerate(
                    zip(getattr(self.out, prefix+'cs_n'), getattr(self.pads, prefix+'cs_n'))):
                    cs_n_ser = Signal()
                    self.oserdese2_ddr(
                        din=basephy_cs[:4],
                        **(dict(dout_fb=cs_n_ser) if with_odelay else dict(dout=pad)),
                        **cs,
                    )
                    if with_odelay:
                        self.odelaye2(
                            din  = cs_n_ser,
                            dout = pad,
                            rst  = self.get_rst(it, CDCCSRs[prefix+'csdly_rst'], prefix, "sys2x_io_bank34"),
                            inc  = self.get_inc(it, CDCCSRs[prefix+'csdly_inc'], prefix, "sys2x_io_bank34"),
                            clk  = "sys2x_io_bank34",
                        )

                # CA ----------------------------------------------------------------------------------
                for it, (basephy_ca, pad) in enumerate(
                    zip(getattr(self.out, prefix+'ca'), getattr(self.pads, prefix+'ca'))):
                    ca_ser = Signal()
                    self.oserdese2_ddr(
                        din=basephy_ca[:4],
                        **(dict(dout_fb=ca_ser) if with_odelay else dict(dout=pad)),
                        **cmd,
                    )
                    if with_odelay:
                        cnt_out = Signal(5)
                        self.odelaye2(
                            din  = ca_ser,
                            dout = pad,
                            rst  = self.get_rst(it, CDCCSRs[prefix+'cadly_rst'], prefix, "sys2x_io_bank34"),
                            inc  = self.get_inc(it, CDCCSRs[prefix+'cadly_inc'], prefix, "sys2x_io_bank34"),
                            clk  = "sys2x_io_bank34",
                            cnt_value_out = cnt_out,
                        )
                        self.sync += If(CSRs[prefix+'dly_sel'].storage[it],
                            CSRs[prefix+'cadly'].status.eq(cnt_out)
                        )

                # PAR ---------------------------------------------------------------------------------
                if hasattr(self.pads, prefix+'par'):
                    basephy_par = getattr(self.out, prefix+'par')[:4]
                    pad = getattr(self.pads, prefix+'par')
                    par_ser = Signal()
                    self.oserdese2_ddr(
                        din=basephy_par,
                        **(dict(dout_fb=par_ser) if with_odelay else dict(dout=pad)),
                        **cmd,
                    )
                    if with_odelay:
                        self.odelaye2(
                            din  = par_ser,
                            dout = pad,
                            rst  = CDCCSRs[prefix+'pardly_rst'],
                            inc  = CDCCSRs[prefix+'pardly_inc'],
                            clk  = "sys2x_io_bank34",
                        )

                # DQS ---------------------------------------------------------------------------------
                strobes = len(pads.dqs_t) if hasattr(pads, "dqs_t") else len(pads.A_dqs_t)
                for it in range(strobes):
                    dqs_t_o = getattr(self.out, prefix+'dqs_t_o')[it*mult]
                    out_dqs_oe = getattr(self.out, prefix+'dqs_oe')[it*mult]
                    out_dqs_oe_n = Signal.like(out_dqs_oe)
                    self.comb += out_dqs_oe_n.eq(~out_dqs_oe)
                    dqs_ser   = Signal()
                    dqs_dly   = Signal()
                    dqs_i     = Signal()
                    dqs_i_dly = Signal()
                    dqs_t     = Signal()

                    self.oserdese2_ddr_with_tri(
                        din     = dqs_t_o[:4],
                        **(dict(dout_fb = dqs_ser) if with_odelay else dict(dout = dqs_dly)),
                        tin     = out_dqs_oe_n[:4],
                        tout    = dqs_t,
                        **ddr,
                    )
                    if with_odelay:
                        cnt_out = Signal(5)
                        self.odelaye2(
                            din  = dqs_ser,
                            dout = dqs_dly,
                            rst  = self.get_rst(it, CDCCSRs[prefix+'wdly_dqs_rst'], prefix, "sys2x_io_bank34"),
                            inc  = self.get_inc(it, CDCCSRs[prefix+'wdly_dqs_inc'], prefix, "sys2x_io_bank34"),
                            clk  = "sys2x_io_bank34",
                            cnt_value_out = cnt_out,
                        )
                        self.sync += If(CSRs[prefix+'dly_sel'].storage[it*mult],
                            CSRs[prefix+'wdly_dqs'].status.eq(cnt_out)
                        )

                    self.iobufds(
                        din      = dqs_dly,
                        dout     = dqs_i,
                        tin      = dqs_t,
                        dinout   = getattr(self.pads, prefix+"dqs_t")[it],
                        dinout_b = getattr(self.pads, prefix+"dqs_c")[it],
                    )
                    cnt_out = Signal(5)
                    self.idelaye2(
                        din  = dqs_i,
                        dout = dqs_i_dly,
                        rst  = self.get_rst(it, CDCCSRs[prefix+'rdly_dqs_rst'], prefix, "sys2x_io_bank34"),
                        inc  = self.get_inc(it, CDCCSRs[prefix+'rdly_dqs_inc'], prefix, "sys2x_io_bank34"),
                        init = max_delay_taps-1,
                        clk  = "sys2x_io_bank34",
                        cnt_value_out = cnt_out,
                        dec  = True,
                    )
                    self.sync += If(CSRs[prefix+'dly_sel'].storage[it*mult],
                        CSRs[prefix+'rdly_dqs'].status.eq(cnt_out)
                    )

                    self.iserdese2_ddr(
                        din    = dqs_i_dly,
                        dout   = getattr(self.out, prefix+"dqs_t_i")[it*mult],
                        clk    = "sys4x_io_bank34",
                        clkdiv = "sys_io_bank34",
                        rst_sig = CSRs["_rst"].storage
                    )

                # DQ ----------------------------------------------------------------------------------
                dq_oe = {}
                for it in range(self.databits):
                    basephy_dq = getattr(self.out, prefix+'dq_o')[it]

                    if it//self.dq_dqs_ratio not in dq_oe:
                        basephy_dq_oe = getattr(self.out, prefix+'dq_oe')[(it//self.dq_dqs_ratio)*mult]
                        basephy_dq_oe_n = Signal.like(basephy_dq_oe)
                        self.comb += basephy_dq_oe_n.eq(~basephy_dq_oe)
                        dq_oe[it//self.dq_dqs_ratio] = basephy_dq_oe_n[:4]

                    dq_t     = Signal()
                    dq_ser   = Signal()
                    dq_dly   = Signal()
                    dq_i     = Signal()
                    dq_i_dly = Signal()

                    self.oserdese2_ddr_with_tri(
                        din     = basephy_dq[:4],
                        **(dict(dout_fb=dq_ser) if with_odelay else dict(dout=dq_dly)),
                        tin     = dq_oe[it//self.dq_dqs_ratio],
                        tout    = dq_t,
                        **ddr_90,
                    )
                    if with_odelay:
                        cnt_out = Signal(5)
                        self.odelaye2(
                            din  = dq_ser,
                            dout = dq_dly,
                            rst  = self.get_rst(it, CDCCSRs[prefix+'wdly_dq_rst'], prefix, "sys2x_io_bank34", dq=True),
                            inc  = self.get_inc(it, CDCCSRs[prefix+'wdly_dq_inc'], prefix, "sys2x_io_bank34", dq=True),
                            clk  = "sys2x_io_bank34",
                            cnt_value_out = cnt_out,
                        )
                        if it%self.dq_dqs_ratio == 0:
                            self.sync += If(CSRs[prefix+'dly_sel'].storage[(it//self.dq_dqs_ratio)*mult],
                                CSRs[prefix+'wdly_dq'].status.eq(cnt_out)
                            )
                    self.iobuf(
                        din    = dq_dly,
                        dout   = dq_i,
                        dinout = getattr(self.pads, prefix+"dq")[it],
                        tin    = dq_t
                    )

                    basephy_dq_i =  getattr(self.out, prefix+'dq_i')[it]
                    in_dq = Signal.like(basephy_dq_i)
                    delay_dq_i = Signal(2)

                    cnt_out = Signal(5)
                    self.idelaye2(
                        din  = dq_i,
                        dout = dq_i_dly,
                        rst  = self.get_rst(it, CDCCSRs[prefix+'rdly_dq_rst'], prefix, "sys2x_io_bank34", dq=True),
                        inc  = self.get_inc(it, CDCCSRs[prefix+'rdly_dq_inc'], prefix, "sys2x_io_bank34", dq=True),
                        clk  = "sys2x_io_bank34",
                        init = max_delay_taps-1,
                        cnt_value_out = cnt_out,
                        dec  = True,
                    )
                    if it%self.dq_dqs_ratio == 0:
                        self.sync += If(CSRs[prefix+'dly_sel'].storage[(it//self.dq_dqs_ratio)*mult],
                            CSRs[prefix+'rdly_dq'].status.eq(cnt_out)
                        )
                    self.iserdese2_ddr(
                        din  = dq_i_dly,
                        dout = in_dq,
                        clk    = "sys4x_io_bank34",
                        clkdiv = "sys_io_bank34",
                        rst_sig = CSRs["_rst"].storage
                    )
                    self.sync += delay_dq_i.eq(in_dq[-2:])
                    self.comb += basephy_dq_i.eq(Cat(delay_dq_i, in_dq[:-2]))

                # DM_n --------------------------------------------------------------------------------
                if hasattr(pads, "dm_n"):
                    for it in range(strobes):
                        basephy_dm = getattr(self.out, prefix+'dm_n_o')[it*mult]

                        dm_t   = Signal()
                        dm_ser = Signal()
                        dm_dly = Signal()
                        self.oserdese2_ddr_with_tri(
                            din     = basephy_dm[:4],
                            **(dict(dout_fb=dm_ser) if with_odelay else dict(dout=dm_dly)),
                            tin     = dq_oe[it],
                            tout    = dm_t,
                            **ddr_90,
                        )
                        if with_odelay:
                            cnt_out = Signal(5)
                            self.odelaye2(
                                din  = dm_ser,
                                dout = dm_dly,
                                rst  = self.get_rst(it, CDCCSRs[prefix+'wdly_dm_rst'], prefix, "sys2x_io_bank34"),
                                inc  = self.get_inc(it, CDCCSRs[prefix+'wdly_dm_inc'], prefix, "sys2x_io_bank34"),
                                clk  = "sys2x_io_bank34",
                                cnt_value_out = cnt_out,
                            )
                            self.sync += If(CSRs[prefix+'dly_sel'].storage[it*mult],
                                CSRs[prefix+'wdly_dm'].status.eq(cnt_out)
                            )
                        self.iobuf(
                            din    = dm_dly,
                            dout   = Signal(),
                            tin    = dm_t,
                            dinout = getattr(self.pads, prefix+'dm_n')[it],
                        )


# PHY variants -------------------------------------------------------------------------------------

class V7DDR5PHY(S7DDR5PHY):
    """Xilinx Virtex7 DDR5 PHY (with odelay)"""
    def __init__(self, pads, **kwargs):
        S7DDR5PHY.__init__(self, pads, with_odelay=True, **kwargs)

class K7DDR5PHY(S7DDR5PHY):
    """Xilinx Kintex7 DDR5 PHY (with odelay)"""
    def __init__(self, pads, **kwargs):
        S7DDR5PHY.__init__(self, pads, with_odelay=True, **kwargs)

class A7DDR5PHY(S7DDR5PHY):
    """Xilinx Artix7 DDR5 PHY (without odelay)

    This variant requires generating sys4x_90 clock in CRG with a 90° phase shift vs sys4x.
    """
    def __init__(self, pads, **kwargs):
        S7DDR5PHY.__init__(self, pads, with_odelay=False, **kwargs)
