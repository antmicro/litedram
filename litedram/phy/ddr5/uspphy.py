#
# This file is part of LiteDRAM.
#
# Copyright (c) 2024 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import random

from operator import add, and_
from functools import reduce
from math import ceil

from migen import *
from migen.genlib.fifo import _FIFOInterface
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *

from litedram.phy.utils import delayed, Latency
from litedram.phy.ddr5.basephy import DDR5PHY

class XilinxUSPAsyncFIFO(Module):
    LATENCY=9 # 3 to pass through memory and 1 for output register
    WCL_LATENCY=10

    def __init__(self, wclk, rclk, input_width=72, output_width=72):
        assert type(wclk) == str
        assert type(rclk) == str
        assert input_width in [4, 8, 9, 16, 18, 32, 36, 64, 72], f"Xilinx UltraScale+ FIFO"\
            "primitive supports widtths: 4,9,18,36, or 72, you tried {input_width}"
        assert output_width in [4, 8, 9, 16, 18, 32, 36, 64, 72], f"Xilinx UltraScale+ FIFO"\
            "primitive supports widtths: 4,9,18,36, or 72, you tried {output_width}"

        self.input_width = input_width
        self.DI = Signal(input_width)
        self.WREN = Signal()
        self.FULL = Signal()

        self.output_width = output_width
        self.DO = Signal(output_width)
        self.RDEN = Signal()
        self.EMPTY = Signal()
        self._rst  = Signal()

        base_width = min(input_width, output_width)
        intermediate_di = Signal.like(self.DI)
        intermediate_do = Signal.like(self.DO)

        if base_width%9 == 0:
            self.comb += [
                intermediate_di.eq(
                    Cat(
                        [self.DI[i*9:i*9+8] for i in range(input_width//9)],
                        [self.DI[i*9+8] for i in range(input_width//9)]
                    )
                ),
                Cat(
                    [self.DO[i*9:i*9+8] for i in range(output_width//9)],
                    [self.DO[i*9+8] for i in range(output_width//9)]
                ).eq(intermediate_do),
            ]
        else:
            self.comb += [
                intermediate_di.eq(self.DI),
                self.DO.eq(intermediate_do),
            ]
        i_cd = getattr(self.sync, wclk)
        w_rst = Signal(reset=1)
        i_cd += [
            If(self._rst,
                w_rst.eq(1),
            ).Else(
                w_rst.eq(0),
            ),
        ]

        fifo_primitive = "FIFO18E2"
        if input_width > 36 or output_width > 36:
            fifo_primitive = "FIFO36E2"
        params = dict(
            of = fifo_primitive,
            p_CASCADE_ORDER = "NONE",
            p_CLOCK_DOMAINS = "INDEPENDENT",
            p_EN_ECC_PIPE = "FALSE",
            p_EN_ECC_READ = "FALSE",
            p_FIRST_WORD_FALL_THROUGH = "FALSE",
            p_INIT = "0x000000000000000000",
            p_PROG_EMPTY_THRESH = "1",
            p_PROG_FULL_THRESH = "1",
            p_RDCOUNT_TYPE = "RAW_PNTR",
            p_READ_WIDTH = output_width,
            p_REGISTER_MODE = "REGISTERED",
            p_RSTREG_PRIORITY = "RSTREG",
            p_SLEEP_ASYNC = "FALSE",
            p_SRVAL = "0x000000000000000000",
            p_WRCOUNT_TYPE = "RAW_PNTR",
            p_WRITE_WIDTH = input_width,
            i_CASDIN        = 0,
            i_CASDINP       = 0,
            i_CASDOMUX      = 0,
            i_CASDOMUXEN    = 0,
            i_INJECTDBITERR = 0,
            i_INJECTSBITERR = 0,
            i_REGCE         = 1,
            i_RSTREG        = 0,
            i_SLEEP         = 0,
            i_RST           = w_rst,
            i_WRCLK         = ClockSignal(wclk),
            i_WREN          = self.WREN,
            o_FULL          = self.FULL,
            i_DIN           = intermediate_di[:(7*input_width)//8+1],
            i_DINP          = intermediate_di[(7*input_width)//8+1:],
            i_RDEN          = self.RDEN,
            i_RDCLK         = ClockSignal(rclk),
            o_EMPTY         = self.EMPTY,
            o_DOUT          = intermediate_do[:(7*output_width)//8+1],
            o_DOUTP         = intermediate_do[(7*output_width)//8+1:],
        )
        if base_width%9 != 0:
            params["i_DIN"] = intermediate_di
            params["i_DINP"] = Signal((base_width+8)//9)
            params["i_DOUT"] = intermediate_do
            params["i_DOUTP"] = Signal((base_width+8)//9)
        self.specials += Instance(**params)


class XilinxUSPAsyncFIFOWrap(Module, _FIFOInterface):
    LATENCY     = XilinxUSPAsyncFIFO.LATENCY
    WCL_LATENCY = XilinxUSPAsyncFIFO.WCL_LATENCY
    _rst        = None

    def __init__(self, wclk, rclk, i_dw, o_dw, name=None):
        _FIFOInterface.__init__(self, max(i_dw, o_dw), 512)
        self.wclk = wclk
        self.rclk = rclk
        if i_dw//o_dw != i_dw/o_dw and o_dw//i_dw != o_dw/i_dw:
            raise AssertionError(f"Invalid input and output widths: {i_dw}, {o_dw}")
        ratio = max(i_dw, o_dw)//min(i_dw, o_dw)
        assert ratio in [1,2,4,8]
        w_ratio = max(i_dw//o_dw, 1)
        r_ratio = max(o_dw//i_dw, 1)
        max_width = {
            1: 72,
            2: 36,
            4: 18,
            8: 9,
        }

        width = max(i_dw, o_dw)
        min_width = min(i_dw, o_dw)
        num_cdcs = ceil((width//ratio)/max_width[ratio])
        base_width = max_width[ratio]
        # Slice input
        cdc_input = []
        input_width = base_width*w_ratio
        for i in range(num_cdcs):
            _input = Signal(input_width)
            start_idx = i*base_width
            last_idx = min((i+1)*base_width, min_width)
            for j in range(w_ratio):
                base_offset = j*min_width
                self.comb += [
                    _input[j*base_width:(j+1)*base_width].eq(
                        self.din[base_offset+start_idx:base_offset+last_idx]
                    )
                ]
            cdc_input.append(_input)

        # Slice output
        cdc_output = []
        output_width = base_width*r_ratio
        for i in range(num_cdcs):
            _output = Signal(output_width)
            start_idx = i*base_width
            last_idx = min((i+1)*base_width, min_width)
            for j in range(r_ratio):
                base_offset = j*min_width
                self.comb += [
                    self.dout[base_offset+start_idx:base_offset+last_idx].eq(
                        _output[j*base_width:(j+1)*base_width]
                    )
                ]
            cdc_output.append(_output)

        cdcs = []
        for i in range(num_cdcs):
            num_valid_bits = min(base_width, min_width-i*base_width)
            cdc = XilinxUSPAsyncFIFO(
                wclk=wclk,
                rclk=rclk,
                input_width=base_width*w_ratio,
                output_width=base_width*r_ratio
            )
            self.comb += [
                cdc.DI.eq(cdc_input[i]),
                cdc_output[i].eq(cdc.DO),
            ]
            cdcs.append(cdc)
        self.cdcs = cdcs
        self.submodules += cdcs

        self.comb += [
            self.readable.eq(reduce(and_, [~cdc.EMPTY for cdc in cdcs])),
            *[cdc.RDEN.eq(self.re) for cdc in cdcs],
            self.writable.eq(reduce(and_, [~cdc.FULL for cdc in cdcs])),
            *[cdc.WREN.eq(self.we) for cdc in cdcs],
        ]

    def do_finalize(self):
        for cdc in self.cdcs:
            self.comb += cdc._rst.eq(self._rst(self.wclk))


class USPCompoDDR5PHY(DDR5PHY):
    def __init__(
        self,
        pads,
        *,
        iodelay_clk_freq,
        crg,
        pin_vref_mapping,
        pin_bank_mapping,
        with_per_dq_idelay=False,
        with_sub_channels=False,
        pin_domains=None,
        **kwargs
    ):

        self.iodelay_clk_freq = iodelay_clk_freq
        self.pin_vref_mapping = pin_vref_mapping
        self.pin_bank_mapping = pin_bank_mapping
        assert pin_domains is not None

        def cdc_any(target):
            def new_cdc(i):
                o = Signal()
                psync = PulseSynchronizer("sys", target)
                self.submodules += psync
                self.comb += [
                    psync.i.eq(i),
                    o.eq(psync.o),
                ]
                return o
            return new_cdc
        self.cdc_any = cdc_any

        self.prefixes = prefixes = [""] if not with_sub_channels else ["A_", "B_"]

        ca_domain = None
        per_pin_ca_domain = {}
        for prefix in prefixes:
            for func in ["ca", "cs_n", "par"]:
                if prefix+func in pin_domains:
                    assert ca_domain is None or ca_domain == pin_domains[prefix+func][0][0]
                    ca_domain = pin_domains[prefix+func][0][0]
                    per_pin_ca_domain[prefix+func] = [f"{ca_domain}"]

        if "reset_n" in pin_domains:
            per_pin_ca_domain["reset_n"] = [f"{ca_domain}"]

        wr_dqs_domains = {}
        for prefix in prefixes:
            if prefix+"dqs_t" in pin_domains:
                wr_dqs_domain = pin_domains[prefix+"dqs_t"][0][0]
                wr_dqs_domains[prefix] = f"{wr_dqs_domain}"

        dq_wr_domains = {}
        dq_rd_domains = {}
        for prefix in prefixes:
            for func in ["dq", "dm"]:
                if prefix+func in pin_domains:
                    dq_wr_domain = pin_domains[prefix+func][0][0]
                    dq_rd_domain = pin_domains[prefix+func][1][0]
                    if prefix not in dq_wr_domains:
                        dq_wr_domains[prefix] = f"{dq_wr_domain}"
                    if prefix not in dq_rd_domains:
                        dq_rd_domains[prefix] = f"{dq_rd_domain}"
                    assert dq_wr_domains[prefix] == f"{dq_wr_domain}"
                    assert dq_rd_domains[prefix] == f"{dq_rd_domain}"

        # It's easier to add reset signals to CDCs through type
        XilinxUSPAsyncFIFOWrap._rst = crg.get_rst

        # DoubleRateDDR5PHY outputs half-width signals (comparing to DDR5PHY) in sys2x domain.
        super().__init__(pads,
            ser_latency       = Latency(sys4x=1),  # OSERDESE3 4:1 DDR (2 full-rate clocks)
            des_latency       = Latency(sys=4),  # ISERDESE3 1:8
            phytype           = self.__class__.__name__,
            with_sub_channels = with_sub_channels,
            ca_domain         = ca_domain,
            wr_dqs_domain     = wr_dqs_domains,
            dq_domain         = dq_wr_domains,
            per_pin_ca_domain = per_pin_ca_domain,

            csr_ca_cdc        = cdc_any(ca_domain),
            csr_dq_rd_cdc     = {prefix: cdc_any(dom) for prefix, dom in dq_rd_domains.items()},
            csr_dq_wr_cdc     = {prefix: cdc_any(dom) for prefix, dom in dq_wr_domains.items()},
            csr_dqs_cdc       = {prefix: cdc_any(dom) for prefix, dom in wr_dqs_domains.items()},

            rd_dq_rst         = {prefix: crg.get_rst(dom) for prefix, dom in dq_rd_domains.items()},
            wr_dq_rst         = {prefix: crg.get_rst(dom) for prefix, dom in dq_wr_domains.items()},
            wr_dqs_rst        = {prefix: crg.get_rst(dom) for prefix, dom in wr_dqs_domains.items()},

            out_CDC_CA_primitive_cls = XilinxUSPAsyncFIFOWrap,
            ca_cdc_min_max_delay =
                (Latency(sys2x=XilinxUSPAsyncFIFOWrap.LATENCY), Latency(sys2x=(XilinxUSPAsyncFIFOWrap.WCL_LATENCY))),
            out_CDC_primitive_cls = XilinxUSPAsyncFIFOWrap,
            wr_cdc_min_max_delay =
                (Latency(sys2x=XilinxUSPAsyncFIFOWrap.LATENCY), Latency(sys2x=(XilinxUSPAsyncFIFOWrap.WCL_LATENCY))),

            with_odelay        = True,
            with_idelay        = True,
            rd_extra_delay     = Latency(sys2x=13),
            with_per_dq_idelay = with_per_dq_idelay,
            SyncFIFO_cls       = SimpleSyncFIFO,
            **kwargs
        )

        # nibble to output mapping
        self.mult = self.dq_dqs_ratio//4
        self.max_delay_taps = 512

        self.handled_ca_vref_status = {}
        for prefix in prefixes + [""]:
            self.handled_ca_vref_status[prefix] = False
            setattr(self, f"{prefix}ca_vref_status", CSRStatus(7, name=f"{prefix}ca_vref_status"))
            setattr(self, f"{prefix}ca_vref_write", CSR(7, name=f"{prefix}ca_vref_write"))
            setattr(self, f"{prefix}vref_status", CSRStatus(7, name=f"{prefix}vref_status"))
            setattr(self, f"{prefix}vref_write", CSR(7, name=f"{prefix}vref_write"))

        CSRs    = self.CSRs
        for prefix in prefixes + [""]:
            self.CSRs[f"{prefix}ca_vref_status"] = getattr(self, f"{prefix}ca_vref_status")
            self.CSRs[f"{prefix}ca_vref_write"] = getattr(self, f"{prefix}ca_vref_write")
            self.CSRs[f"{prefix}vref_status"] = getattr(self, f"{prefix}vref_status")
            self.CSRs[f"{prefix}vref_write"] = getattr(self, f"{prefix}vref_write")
        CDCCSRs = self.CDCCSRs
        crg.add_rst(CSRs['_rst'].storage)
        self.crg = crg

        self.settings.delays = 512
        self.settings.write_leveling = True
        self.settings.write_latency_calibration = True
        self.settings.write_dq_dqs_training = True
        self.settings.read_leveling = True

        # Serialization ----------------------------------------------------------------------------
        pin_csr_mapping = {
            "ck_t": (CSRs["ckdly_update"].re, None),
            "A_ck_t": (CSRs["ckdly_update"].re, None),
            "B_ck_t": (CSRs["ckdly_update"].re, None),
        }

        self.vref_cache = {}

        for prefix in prefixes:
            pin_csr_mapping |= {
                f"{prefix}par": (CSRs[f"{prefix}pardly_update"].re, None),
                f"{prefix}ca": (CSRs[f"{prefix}cadly_update"].re, None),
                f"{prefix}cs_n": (CSRs[f"{prefix}csdly_update"].re, None),
                f"{prefix}dq": (
                    CSRs[f"{prefix}wdly_dq_update"].re,
                    CSRs[f"{prefix}rdly_dq_update"].re
                ),
                f"{prefix}dqs_t": (
                    CSRs[f"{prefix}wdly_dqs_update"].re,
                    CSRs[f"{prefix}rdly_dqs_update"].re
                ),
            }

        self.pin_domains     = pin_domains
        self.pin_csr_mapping = pin_csr_mapping

        self.cdc_cache  = cdc_cache = {}
        pin_oe_cache = {}
        # key = clock doamin, value = [(iserdes_output, phy_input)]
        self.fast_input = {}
        for pin, count in pads.layout:
            if pin in ["mir", "cai", "ca_odt"]:
                self.comb += getattr(self.pads, pin).eq(0)
                continue

            unused_ddr5_signals = [
                prefix+sig for sig in ["cb", "dqsb_t", "dqsb_c"] for prefix in prefixes] + \
                ["pgood", "dlbdq", "dlbdqs"]
            if pin in unused_ddr5_signals:
                continue

            assert pin in pin_domains, (pin, pin_domains)
            if pin[-2:] == "_c":
                continue

            _diff   = "_t" in pin
            _is_ck  = "ck" in pin
            _is_io  = reduce(or_, [pin_type in pin for pin_type in ["dq", "dm_n"]]) # dq is in dqs
            _is_out = pin_domains[pin][0] is not None
            _is_in  = pin_domains[pin][1] is not None

            for i in range(count):
                if "_c" == pin[-2:]:
                    continue
                _in, _out = self.get_domains(pin, _is_in, _is_out)

                _pin = pin
                _pin_o = _pin
                _pin_i = _pin
                _pin_base = _pin if not _diff else _pin[:-2]
                _pin_func = _pin if len(prefixes) == 1 else _pin[2:]
                _pin_prefix = "" if len(prefixes) == 1 else _pin[:2]
                _pin_oe = None
                if _is_io:
                    _pin_i  = _pin + "_i"
                    _pin_o  = _pin + "_o"
                    _pin_oe = _pin_base + "_oe"

                if _is_ck:
                    if count == 1:
                        self.handle_ck(_out, pin)
                    else:
                        self.handle_ck(_out, pin, offset=i)
                    continue

                _sig_out = None
                _sig_oe  = None
                _sig_in  = None

                if _is_out:
                    mult = 1
                    if reduce(or_, [pin_type in pin for pin_type in ["dqs", "dm_n"]]):
                        mult = self.mult
                    out_sig = getattr(self.out, _pin_o)
                    if isinstance(out_sig, list):
                        out_sig = out_sig[i*mult]
                    _sig_out = out_sig[:4]

                if _is_io:
                    mult = 1
                    if reduce(or_, [pin_type in pin for pin_type in ["dq", "dm_n"]]):
                        mult = self.mult

                    idx = i
                    if _pin_func == "dq":
                        idx //= self.dq_dqs_ratio
                    idx *= mult

                    if (_pin_oe, idx) in pin_oe_cache:
                        _sig_oe = pin_oe_cache[(_pin_oe, idx)]
                    elif _pin_func == "dm_n"  and (_pin_prefix+"dq_oe", idx) in pin_oe_cache:
                        _sig_oe = pin_oe_cache[(_pin_prefix+"dq_oe", idx)]
                    else:
                        out_sig_oe = getattr(self.out, _pin_oe)
                        if isinstance(out_sig_oe, list):
                            out_sig_oe = out_sig_oe[idx]

                        _sig_oe_t = out_sig_oe
                        _sig_oe = Signal(4)
                        self.comb += _sig_oe.eq(~_sig_oe_t[:4])
                        pin_oe_cache[(_pin_oe, idx)] = _sig_oe

                if _is_in:
                    mult = 1
                    if reduce(or_, [pin_type in pin for pin_type in ["dqs", "dm_n"]]):
                        mult = self.mult
                    _sig_in = getattr(self.out, _pin_i)
                    if isinstance(_sig_in, list):
                        _sig_in = _sig_in[i*mult]

                if _is_io:
                    offset = i if count > 1 else None
                    self.handle_io(cd_out=_out, cd_in=_in, pin=pin, offset=offset,
                                    oe_sig=_sig_oe, in_sig=_sig_in, out_sig=_sig_out)
                elif _is_in:
                    offset = i if count > 1 else None
                    self.handle_i(cd_in=_in, in_sig=_sig_in, pin=pin, offset=offset)
                else:
                    offset = i if count > 1 else None
                    self.handle_o(cd_out=_out, out_sig=_sig_out, oe_sig=_sig_oe,
                                    pin=pin, offset=offset)

        for source_cd, values in self.fast_input.items():
            if len(values[0][0]) == len(values[0][1]):
                for input_sig, output_sig in values:
                    self.sync += output_sig.eq(input_sig)
            else:
                input_width = reduce(add, [len(_input) for _input, _ in values])
                output_width = reduce(add, [len(_output) for _, _output in values])
                input_sig = Signal(input_width)
                output_sig = Signal(output_width)
                self.comb += input_sig.eq(Cat([_input for _input, _ in values]))
                cdc = XilinxUSPAsyncFIFOWrap(source_cd, "sys", input_width, output_width)
                self.submodules += cdc
                fast_cd = getattr(self.sync, source_cd)
                en = Signal()
                fast_cd += [
                    en.eq(self.CSRs["_enable_fifos"].storage)
                ]
                self.comb += [
                    cdc.din.eq(input_sig),
                    output_sig.eq(cdc.dout),
                    cdc.we.eq(en),
                    cdc.re.eq(cdc.readable),
                ]
                rsum = 0
                div = output_width//input_width
                for _, _output in values:
                    start = rsum
                    end = rsum+len(_output)//div
                    self.comb += [
                        _output.eq(Cat([output_sig[i*input_width+start:i*input_width+end] for i in range(div)]))
                    ]
                    rsum += len(_output)//div

    def get_domains(self, pin, is_in, is_out):
        cd_out, cd_in = self.pin_domains[pin]
        if is_out:
            cd_out = (cd_out[0], cd_out[1])
        if is_in:
            cd_in = (cd_in[0], cd_in[1])
        return cd_in, cd_out

    def iobuf(self, din, dout, tin, dinout, osc_en=None, osc=None, vref=None):
        random_offset = random.randrange(1, 51) * random.choice([-1, 1])
        self.specials += Instance(
            "IOBUFE3",
            p_SIM_DEVICE="ULTRASCALE_PLUS",
            p_SIM_INPUT_BUFFER_OFFSET=Instance.PreformattedParam(random_offset),
            io_IO=dinout,
            i_I=din,
            i_T=tin,
            i_DCITERMDISABLE=0,
            i_IBUFDISABLE=0,
            i_OSC_EN=osc_en,
            i_OSC=osc,
            i_VREF=vref,
            o_O=dout,
        )

    def ibuf(self, din, dout, osc_en=None, osc=None, vref=None):
        random_offset = random.randrange(1, 51) * random.choice([-1, 1])
        self.specials += Instance(
            "IBUFE3",
            p_SIM_DEVICE="ULTRASCALE_PLUS",
            p_SIM_INPUT_BUFFER_OFFSET=Instance.PreformattedParam(random_offset),
            i_I=din,
            i_IBUFDISABLE=0,
            i_OSC_EN=osc_en,
            i_OSC=osc,
            i_VREF=vref,
            o_O=dout,
        )

    def iobufds(self, din, dout, tin, dinout, dinout_b, osc_en=None, osc=None):
        random_offset = random.randrange(1, 51) * random.choice([-1, 1])
        self.specials += Instance(
            "IOBUFDSE3",
            p_SIM_DEVICE="ULTRASCALE_PLUS",
            p_SIM_INPUT_BUFFER_OFFSET=Instance.PreformattedParam(random_offset),
            io_IO=dinout,
            io_IOB=dinout_b,
            i_I=din,
            i_T=tin,
            i_DCITERMDISABLE=0,
            i_IBUFDISABLE=0,
            i_OSC_EN=osc_en,
            i_OSC=osc,
            o_O=dout,
        )

    def obufds(self, din, dout, dout_b):
        self.specials += Instance(
            "OBUFDS",
            o_O=dout,
            o_OB=dout_b,
            i_I=din,
        )

    def handle_single_ended(self, pad, *, out_sig=None, oe_sig=None, in_sig=None, vref=None):
        if in_sig is not None and out_sig is not None:
            self.iobuf(din=out_sig, dout=in_sig, tin=oe_sig, dinout=pad, vref=vref)
        elif in_sig is not None:
            self.ibuf(dout=in_sig, din=pad, vref=vref)
        else:
            self.comb += pad.eq(out_sig)

    def handle_diff(self, pad_t, pad_c, *, out_sig=None, oe_sig=None, in_sig=None):
        if in_sig is not None and out_sig is not None:
            self.iobufds(din=out_sig, dout=in_sig, tin=oe_sig, dinout=pad_t, dinout_b=pad_c)
        elif in_sig is not None:
            raise NotImplementedError()
        else:
            self.obufds(din=out_sig, dout=pad_t, dout_b=pad_c)

    def simple_cdc(self, *, din, dout, src_clk, dst_clk, reset_value=1):
        assert len(din)/len(dout) == 2.0, (len(din), len(dout))
        rst = ResetSignal(src_clk)
        clk = ClockSignal(dst_clk)
        internal_rst = Signal()
        cnt = Signal()
        self.specials += Instance("FDPE",
            p_INIT=1,
            p_IS_C_INVERTED=0,
            p_IS_D_INVERTED=0,
            p_IS_PRE_INVERTED=0,
            i_C=clk,
            i_CE=1,
            i_D=0,
            i_PRE=rst,
            o_Q=internal_rst,
        )
        self.specials += Instance("FDPE",
            p_INIT=1,
            p_IS_C_INVERTED=0,
            p_IS_D_INVERTED=0,
            p_IS_PRE_INVERTED=0,
            i_C=clk,
            i_CE=1,
            i_D=~cnt,
            i_PRE=internal_rst,
            o_Q=cnt,
        )
        intermediate = Signal.like(din)
        muxed_intermediate = Signal.like(dout)
        half = len(dout)
        init = 1
        primitive = "FDPE"
        reset = {
            "i_PRE":internal_rst,
            "p_IS_PRE_INVERTED":0,
        }
        if reset_value == 0:
            init = 0
            primitive = "FDCE"
            reset = {
                "i_CLR":internal_rst,
                "p_IS_CLR_INVERTED":0,
            }
        for i in range(len(din)):
            self.specials += Instance(primitive,
                p_INIT=init,
                p_IS_C_INVERTED=0,
                p_IS_D_INVERTED=0,
                i_C=clk,
                i_CE=cnt,
                i_D=din[i],
                **reset,
                o_Q=intermediate[i],
            )
        self.comb += [
            If(cnt,
               muxed_intermediate.eq(intermediate[half:])
            ).Else(
               muxed_intermediate.eq(intermediate[:half])
            )
        ]
        for i in range(len(dout)):
            self.specials += Instance(primitive,
                p_INIT=init,
                p_IS_C_INVERTED=0,
                p_IS_D_INVERTED=0,
                i_C=clk,
                i_CE=1,
                i_D=muxed_intermediate[i],
                **reset,
                o_Q=dout[i],
            )


    def oddre1_with_t(self, *, din, dout, tin, tout, clk, rst_sig, reset_value=1):
        assert len(din) == 2, len(din)
        assert len(tin) == 2, len(tin)
        _input = Signal(8)
        self.comb += [
            _input[0].eq(din[0]),
            _input[1].eq(tin[0]),
            _input[4].eq(din[1]),
            _input[5].eq(tin[1]),
        ]
        cd = getattr(self.sync, clk)
        _rst = Signal()
        cd += _rst.eq(rst_sig)
        # Following instance is equivalent to 2 ODDRE1s that serializes
        # data and tristate
        self.specials += Instance(
            "OSERDESE3",
            p_DATA_WIDTH=8,
            p_INIT=1,
            p_ODDR_MODE="TRUE",
            p_OSERDES_D_BYPASS="FALSE",
            p_OSERDES_T_BYPASS="FALSE",
            p_IS_CLK_INVERTED=0,
            p_SIM_DEVICE="ULTRASCALE_PLUS",
            i_CLK=ClockSignal(clk),
            i_CLKDIV=Signal(),
            i_D=_input,
            o_OQ=dout,
            i_RST=_rst,
            o_T_OUT=tout,
            i_T=0,
        )

    def odelaye3(self, *, din, dout, load, clk, cnt_value_out, bank):
        base_delay_reg = Signal(9)
        clk_domain = getattr(self.sync, clk)
        clk_domain += [
            If(self.crg.get_load_base(clk),
                base_delay_reg.eq(cnt_value_out),
            )
        ]
        min_tap = Signal(9)
        max_tap = Signal(9)
        load_tap = Signal(9)
        _load = Signal()
        _ce = Signal()
        load = load if load is not None else 0
        self.comb += [
            min_tap.eq(base_delay_reg),
            max_tap.eq(self.max_delay_taps-1),
            _load.eq(self.CSRs['adly_ctrl'].fields.load_value & load),
            _ce.eq(~self.CSRs['adly_ctrl'].fields.load_value & load),
            If(self.CSRs['adly_ctrl'].fields.increment_value,
                load_tap.eq(max_tap),
            ).Else(
                load_tap.eq(min_tap),
            )
        ]

        attr = set()
        attr.add(("IODELAY_GROUP", f"DDR5_PHY_{bank}"))
        self.specials += Instance("ODELAYE3",
            attr = attr,
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_CASCADE          = "NONE",
            p_UPDATE_MODE      = "ASYNC",
            p_REFCLK_FREQUENCY = 300,
            p_DELAY_FORMAT     = "COUNT",
            p_DELAY_TYPE       = "VAR_LOAD",
            p_DELAY_VALUE      = 0,
            i_RST     = self.crg.get_iodelay_rst(clk),
            i_LOAD    = _load,
            i_CLK     = ClockSignal(clk),
            i_EN_VTC  = 0,
            i_CE      = _ce,
            i_INC     = self.CSRs['adly_ctrl'].fields.increment_value,
            i_ODATAIN = din,
            o_DATAOUT = dout,
            i_CNTVALUEIN = load_tap,
            o_CNTVALUEOUT = cnt_value_out,
        )

    def handle_oser(self, cd_out, out_sig, bank, *, oe_sig=None, load_sig=None):
        assert len(out_sig) == 4
        assert oe_sig is None or len(oe_sig) == 4
        delay     = Signal()
        _output    = Signal()
        _tri_state = Signal()
        _with_odelay = load_sig is not None

        if oe_sig is not None:
            oe_sig_cdc = Signal(len(out_sig)//2)
            self.simple_cdc(
                din=oe_sig,
                dout=oe_sig_cdc,
                src_clk=cd_out[0],
                dst_clk=cd_out[1],
                reset_value=0,
            )
        else:
            oe_sig_cdc = Signal(2)
            self.comb += oe_sig_cdc.eq(0)

        out_sig_cdc = Signal(len(out_sig)//2)
        self.simple_cdc(
            din=out_sig,
            dout=out_sig_cdc,
            src_clk=cd_out[0],
            dst_clk=cd_out[1],
            reset_value=1,
        )

        self.oddre1_with_t(
            din = out_sig_cdc,
            dout = delay,
            tout = _tri_state,
            tin = oe_sig_cdc,
            clk = cd_out[1],
            rst_sig = self.crg.get_rst(cd_out[1]),
        )
        delay_state = None
        delay_state = Signal(9)
        self.odelaye3(
            din  = delay,
            dout = _output,
            load  = load_sig,
            clk  = cd_out[0],
            cnt_value_out = delay_state,
            bank = bank,
        )
        return _output, _tri_state, delay_state

    def iserdese3_ddr(self, din, dout, clkdiv, clk, rst_sig):
        self.specials += Instance("ISERDESE3",
              p_SIM_DEVICE         = "ULTRASCALE_PLUS",
              p_DATA_WIDTH         = 8,
              p_IS_CLK_B_INVERTED  = 1,
              i_RST    = rst_sig,
              i_CLK    = ClockSignal(clk),
              i_CLK_B  = ClockSignal(clk),
              i_CLKDIV = ClockSignal(clkdiv),
              i_D      = din,
              o_Q      = dout,
        )

    def idelaye3(self, *, din, dout, load, clk, cnt_value_out, bank):
        base_delay_reg = Signal(9)
        clk_domain = getattr(self.sync, clk)
        clk_domain += [
            If(self.crg.get_load_base(clk),
                base_delay_reg.eq(cnt_value_out),
            )
        ]
        min_tap = Signal(9)
        max_tap = Signal(9)
        load_tap = Signal(9)
        _load = Signal()
        _ce = Signal()
        load = load if load is not None else 0
        self.comb += [
            min_tap.eq(base_delay_reg),
            max_tap.eq(self.max_delay_taps-1),
            _load.eq(self.CSRs['adly_ctrl'].fields.load_value & load),
            _ce.eq(~self.CSRs['adly_ctrl'].fields.load_value & load),
            If(self.CSRs['adly_ctrl'].fields.increment_value,
                load_tap.eq(max_tap),
            ).Else(
                load_tap.eq(min_tap),
            )
        ]

        attr = set()
        attr.add(("IODELAY_GROUP", f"DDR5_PHY_{bank}"))
        self.specials += Instance("IDELAYE3",
            attr = attr,
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_CASCADE          = "NONE",
            p_UPDATE_MODE      = "ASYNC",
            p_REFCLK_FREQUENCY = 300,
            p_DELAY_FORMAT     = "COUNT",
            p_DELAY_TYPE       = "VAR_LOAD",
            p_DELAY_VALUE      = 0,
            i_RST     = self.crg.get_iodelay_rst(clk),
            i_LOAD    = _load,
            i_CLK     = ClockSignal(clk),
            i_EN_VTC  = 0,
            i_CE      = _ce,
            i_INC     = self.CSRs['adly_ctrl'].fields.increment_value,
            i_IDATAIN = din,
            o_DATAOUT = dout,
            o_CNTVALUEOUT = cnt_value_out,
            i_CNTVALUEIN = load_tap,
        )

    def handle_iser(self, cd_in, in_sig, idelay_cd, bank, *, load_sig=None):
        _input = Signal()
        _delayed_input = Signal()
        delay_state = Signal(9)
        self.idelaye3(
            din=_input,
            dout=_delayed_input,
            load=load_sig,
            clk=idelay_cd,
            cnt_value_out=delay_state,
            bank=bank,
        )

        iser_output = Signal(4)
        if cd_in[0] not in self.fast_input:
            self.fast_input[cd_in[0]] = []
        self.fast_input[cd_in[0]].append((iser_output, in_sig))
        self.iserdese3_ddr(
            din     = _delayed_input,
            dout    = iser_output,
            clk     = cd_in[1],
            clkdiv  = cd_in[0],
            rst_sig = self.crg.get_serdes_rst(cd_in[0]),
        )
        return _input, delay_state, iser_output

    def get_pads(self, pin, *, offset=None):
        if offset is None:
            offset=0
        pad_t = getattr(self.pads, pin)[offset]
        pad_c = getattr(self.pads, pin[:-2]+"_c", None)
        if pad_c is not None:
            pad_c = pad_c[offset]
        return (pad_t, pad_c)

    def get_vref(self, pin, prefix, *, data=False, offset=None, vref_select=None):
        if offset is None:
            offset=0
        address = self.pin_vref_mapping[pin][offset]
        if address not in self.vref_cache:
            vref = Signal()
            vref_ctrl = Signal(7, reset=0b0100010)
            self.specials += Instance(
                "HPIO_VREF",
                p_VREF_CNTR="FABRIC_RANGE1",
                i_FABRIC_VREF_TUNE=vref_ctrl,
                o_VREF=vref,
            )
            if data:
                self.sync += [
                    If(self.CSRs[prefix+'dly_sel'].storage[vref_select],
                        self.CSRs[prefix+'vref_status'].status.eq(vref_ctrl),
                        If(self.CSRs[prefix+'vref_write'].re,
                            vref_ctrl.eq(self.CSRs[prefix+'vref_write'].r),
                        ),
                    ),
                ]
            else:
                if not self.handled_ca_vref_status[prefix]:
                    self.sync += [
                        self.CSRs[prefix+'ca_vref_status'].status.eq(vref_ctrl),
                    ]
                    self.handled_ca_vref_status[prefix] = True

                self.sync += [
                    If(self.CSRs[prefix+'ca_vref_write'].re,
                        vref_ctrl.eq(self.CSRs[prefix+'ca_vref_write'].r),
                    ),
                ]
            self.vref_cache[address] = (vref, data)

        assert self.vref_cache[address][1] == data
        return self.vref_cache[address][0]

    def get_bank(self, pin, *, offset=None):
        if offset is None:
            offset=0
        return self.pin_bank_mapping[pin][offset]

    def get_load(self, pin, cd, not_out, offset):
        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])
        dq = True if _pin_func in "dq" else False
        load = None
        if pin not in self.pin_csr_mapping:
            return None
        if (pin, not_out, cd) not in self.cdc_cache:
            if cd != "sys":
                self.cdc_cache[(pin, not_out, cd)] = self.cdc_any(cd)(
                    self.pin_csr_mapping[pin][not_out]
                )
            else:
                self.cdc_cache[(pin, not_out, cd)] = self.pin_csr_mapping[pin][not_out]
        load_sig = self.cdc_cache[(pin, not_out, cd)]
        if offset is not None:
            load = self.get_inc(offset, load_sig, prefix, cd, dq=dq)
        else:
            load  = load_sig
        return load

    def get_out_load(self, pin, *, cd, offset=None):
        return self.get_load(pin, cd, 0, offset)

    def get_in_load(self, pin, *, cd, offset=None):
        return self.get_load(pin, cd, 1, offset)

    def handle_o(self, cd_out, out_sig, pin, *, offset=None, oe_sig=None):
        pad_t, pad_c = self.get_pads(pin, offset=offset)

        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])
        _offset = offset
        if _pin_func == "ck_t":
            offset = None
        load_sig = None
        if pin in self.pin_csr_mapping:
            load_sig = self.get_out_load(pin, offset=offset, cd=cd_out[0])

        bank = self.get_bank(pin=pin, offset=offset)
        to_pad, to_pad_oe, delay_state = self.handle_oser(
            cd_out, out_sig, bank, oe_sig=oe_sig, load_sig=load_sig)

        offset = offset if offset else 0
        if "ck_t" == pin:
            self.sync += [
                self.CSRs['ckdly'].status.eq(delay_state),
            ]
        elif "ck" == _pin_func:
            self.sync += [
                If(self.CSRs[prefix+'dly_sel'].storage[_offset],
                    self.CSRs[prefix+'ckdly'].status.eq(delay_state),
                ),
            ]
        elif "ca" == _pin_func:
            self.sync += [
                If(self.CSRs[prefix+'dly_sel'].storage[offset],
                    self.CSRs[prefix+'cadly'].status.eq(delay_state),
                ),
            ]
        elif "cs_n" == _pin_func:
            self.sync += [
                If(self.CSRs[prefix+'dly_sel'].storage[offset],
                    self.CSRs[prefix+'csdly'].status.eq(delay_state),
                ),
            ]

        if pad_c is not None:
            self.handle_diff(pad_t, pad_c, out_sig=to_pad, oe_sig=to_pad_oe)
        else:
            self.handle_single_ended(pad_t, out_sig=to_pad, oe_sig=to_pad_oe)

    def handle_i(self, cd_in, in_sig, pin, *, offset=None):
        pad_t, pad_c = self.get_pads(pin, offset=offset)

        load_sig = self.get_in_load(pin, offset=offset, cd=cd_in[0])
        bank = self.get_bank(pin=pin, offset=offset)
        from_pad, delay_state, _ = self.handle_iser(
            cd_in=cd_in,
            in_sig=in_sig,
            idelay_cd=cd_in[0],
            bank=bank,
            load_sig=load_sig,
        )

        if pad_c is not None:
            self.handle_diff(pad_t, pad_c, in_sig=from_pad)
        else:
            vref = self.get_vref(pin, prefix="", offset=offset)
            self.handle_single_ended(pad_t, in_sig=from_pad, vref=vref)

    def handle_io(self, cd_out, cd_in, out_sig, oe_sig, in_sig, pin, *, offset=None):
        pad_t, pad_c = self.get_pads(pin, offset=offset)
        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])

        if "dqs" in _pin_func and offset:
            offset *= self.dq_dqs_ratio//4

        load_sig = None
        if pin in self.pin_csr_mapping:
            load_sig = self.get_out_load(pin, offset=offset, cd=cd_out[0])

        bank = self.get_bank(pin=pin, offset=offset)
        to_pad, to_pad_oe, odelay_state = self.handle_oser(
            cd_out=cd_out,
            out_sig=out_sig,
            bank=bank,
            oe_sig=oe_sig,
            load_sig=load_sig,
        )

        load_sig = self.get_in_load(pin, offset=offset, cd=cd_out[0])
        from_pad, idelay_state, iser_output = self.handle_iser(
            cd_in=cd_in,
            in_sig=in_sig,
            idelay_cd=cd_out[0],
            bank=bank,
            load_sig=load_sig,
        )

        offset = offset if offset else 0
        vref_select = None
        data = False
        if "dq" in _pin_func and "B_" == prefix:
            _out_sig = Signal.like(out_sig)
            _in_sig = Signal.like(iser_output)
            _oe_sig = Signal.like(oe_sig)
            _out_sig.name = f"{_pin_func}_{offset}_out"
            _in_sig.name = f"{_pin_func}_{offset}_in"
            _oe_sig.name = f"{_pin_func}_{offset}_oe"
            _out_sig.attr.add(("MARK_DEBUG", "TRUE"))
            _in_sig.attr.add(("MARK_DEBUG", "TRUE"))
            _oe_sig.attr.add(("MARK_DEBUG", "TRUE"))
            self.comb += [
                _out_sig.eq(out_sig),
                _in_sig.eq(iser_output),
                _oe_sig.eq(oe_sig),
            ]
        if "dq" == _pin_func:
            data = True
            vref_select = offset//8
            if offset%4 == 0:
                self.sync += [
                    If(self.CSRs[prefix+'dly_sel'].storage[offset//4],
                        self.CSRs[prefix+'rdly_dq'].status.eq(idelay_state),
                    ),
                    If(self.CSRs[prefix+'dly_sel'].storage[offset//4],
                        self.CSRs[prefix+'wdly_dq'].status.eq(odelay_state),
                    ),
                ]
        elif "dqs" in _pin_func:
            data = True
            vref_select = offset
            self.sync += [
                If(self.CSRs[prefix+'dly_sel'].storage[offset],
                    self.CSRs[prefix+'rdly_dqs'].status.eq(idelay_state),
                ),
                If(self.CSRs[prefix+'dly_sel'].storage[offset],
                    self.CSRs[prefix+'wdly_dqs'].status.eq(odelay_state),
                ),
            ]

        if pad_c is not None:
            self.handle_diff(pad_t, pad_c, out_sig=to_pad, oe_sig=to_pad_oe, in_sig=from_pad)
        else:
            vref = self.get_vref(pin, prefix=prefix, offset=offset, data=data, vref_select=vref_select)
            self.handle_single_ended(pad_t, out_sig=to_pad, oe_sig=to_pad_oe, in_sig=from_pad, vref=vref)

    def handle_ck(self, cd_out, pin, offset=None):
        clk_sig = Signal(4)
        self.comb += clk_sig.eq(self.clk_pattern&0xF)
        self.handle_o(cd_out=cd_out, out_sig=clk_sig, pin=pin, offset=offset)
