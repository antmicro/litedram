#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import and_
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
                        [self.DI[i*9:i*9+8] for i in range(input_width//base_width)],
                        [self.DI[i*9+8] for i in range(input_width//base_width)]
                    )
                ),
                Cat(
                    [self.DO[i*9:i*9+8] for i in range(output_width//base_width)],
                    [self.DO[i*9+8] for i in range(output_width//base_width)]
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
            fifo_primitive,
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
            o_DOUT          = self.DO[:(7*output_width)//8+1],
            o_DOUTP         = self.DO[(7*output_width)//8+1:],
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
        self.rclk = rclk
        assert i_dw//o_dw == i_dw/o_dw
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
        sliced_input = [self.din[i*min_width:(i+1)*min_width] for i in range(w_ratio)]
        sliced_output = [self.dout[i*min_width:(i+1)*min_width] for i in range(r_ratio)]

        num_cdcs = ceil((width//ratio)/max_width[ratio])
        cdcs = []
        for i in range(num_cdcs):
            base_width = max_width[ratio]
            num_valid_bits = min(base_width, min_width-i*base_width)
            cdc = XilinxUSPAsyncFIFO(
                wclk=wclk,
                rclk=rclk,
                input_width=base_width*w_ratio,
                output_width=base_width*r_ratio
            )
            _input = []
            _output = []
            for j in range(w_ratio):
                for k in range(base_width):
                    if k < num_valid_bits:
                        _input.append(sliced_input[j][i*base_width+k])
                    else:
                        _input.append(0)
            for j in range(r_ratio):
                for k in range(base_width):
                    if k < num_valid_bits:
                        _output.append(sliced_output[j][i*base_width+k])
                    else:
                        _output.append(Signal())
            self.comb += [
                cdc.DI.eq(Cat(_input)),
                Cat(_output).eq(cdc.DO),
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
    def __init__(self, pads, *, iodelay_clk_freq, crg, voltage_ctrl, with_per_dq_idelay=False,
                 with_sub_channels=False, pin_domains=None, pin_banks=None,
                 **kwargs):

        self.iodelay_clk_freq = iodelay_clk_freq
        assert pin_domains is not None
        assert pin_banks is not None

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
        ca_bank = {}
        for prefix in prefixes:
            for func in ["ca", "cs_n", "par"]:
                if prefix+func in pin_domains:
                    assert ca_domain is None or ca_domain == pin_domains[prefix+func][0][0]
                    ca_domain = pin_domains[prefix+func][0][0]
                    if pin_banks[prefix+func][0] not in ca_bank:
                        ca_bank[pin_banks[prefix+func][0]] = 0
                    ca_bank[pin_banks[prefix+func][0]] += 1
                    per_pin_ca_domain[prefix+func] = [f"{ca_domain}_{bank}" for bank in pin_banks[prefix+func]]

        if "reset_n" in pin_domains:
            per_pin_ca_domain["reset_n"] = [f"{ca_domain}_{bank}" for bank in pin_banks["reset_n"]]

        _max = ("", -1)
        for bank, count in ca_bank.items():
            if count > _max[1]:
                _max = (bank, count)
        ca_domain = f"{ca_domain}_{_max[0]}"

        wr_dqs_domains = {}
        for prefix in prefixes:
            if prefix+"dqs_t" in pin_domains:
                wr_dqs_domain = pin_domains[prefix+"dqs_t"][0][0]
                wr_dqs_bank = pin_banks[prefix+"dqs_t"][0]
                assert reduce(and_, [wr_dqs_bank == bank for bank in pin_banks[prefix+"dqs_t"]])
                wr_dqs_domains[prefix] = f"{wr_dqs_domain}_{wr_dqs_bank}"

        dq_wr_domains = {}
        dq_rd_domains = {}
        for prefix in prefixes:
            for func in ["dq", "dm"]:
                if prefix+func in pin_domains:
                    dq_wr_domain = pin_domains[prefix+func][0][0]
                    dq_rd_domain = pin_domains[prefix+func][1][0]
                    dq_bank = pin_banks[prefix+func][0]
                    assert reduce(and_, [dq_bank == bank for bank in pin_banks[prefix+func]])
                    if prefix not in dq_wr_domains:
                        dq_wr_domains[prefix] = f"{dq_wr_domain}_{dq_bank}"
                    if prefix not in dq_rd_domains:
                        dq_rd_domains[prefix] = f"{dq_rd_domain}_{dq_bank}"
                    assert dq_wr_domains[prefix] == f"{dq_wr_domain}_{dq_bank}"
                    assert dq_rd_domains[prefix] == f"{dq_rd_domain}_{dq_bank}"

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

            with_odelay        = with_odelay,
            with_idelay        = with_idelay,
            rd_extra_delay     = Latency(sys2x=3),
            with_per_dq_idelay = with_per_dq_idelay,
            SyncFIFO_cls       = SimpleSyncFIFO,
            **kwargs
        )

        # nibble to output mapping
        self.mult = self.dq_dqs_ratio//4
        self.max_delay_taps = 512

        CSRs    = self.CSRs
        CDCCSRs = self.CDCCSRs
        crg.add_rst(CSRs['_rst'].storage)
        self.crg = crg

        self.settings.delays = max_delay_taps
        self.settings.write_leveling = True
        self.settings.write_latency_calibration = True
        self.settings.write_dq_dqs_training = True
        self.settings.read_leveling = True

        # Serialization ----------------------------------------------------------------------------
        pin_csr_mapping = {
            "ck_t":    ((CSRs["ckdly_inc"].re,      CSRs["ckdly_rst"].re),      None),
            "A_ck_t":    ((CSRs["ckdly_inc"].re,      CSRs["ckdly_rst"].re),      None),
            "B_ck_t":    ((CSRs["ckdly_inc"].re,      CSRs["ckdly_rst"].re),      None),
        }
        for prefix in prefixes:
            pin_csr_mapping |= {
                f"{prefix}par":   (
                    (CSRs[f"{prefix}pardly_inc"].re,   CSRs[f"{prefix}pardly_rst"].re),
                     None),
                f"{prefix}ca":    (
                    (CSRs[f"{prefix}cadly_inc"].re,    CSRs[f"{prefix}cadly_rst"].re),
                     None),
                f"{prefix}cs_n":  (
                    (CSRs[f"{prefix}csdly_inc"].re,    CSRs[f"{prefix}csdly_rst"].re),
                     None),
                f"{prefix}dq":    (
                    (CSRs[f"{prefix}wdly_dq_inc"].re,  CSRs[f"{prefix}wdly_dq_rst"].re),
                    (CSRs[f"{prefix}rdly_dq_inc"].re,  CSRs[f"{prefix}rdly_dq_rst"].re)),
                f"{prefix}dqs_t": (
                    (CSRs[f"{prefix}wdly_dqs_inc"].re, CSRs[f"{prefix}wdly_dqs_rst"].re),
                    (CSRs[f"{prefix}rdly_dqs_inc"].re, CSRs[f"{prefix}rdly_dqs_rst"].re)),
            }

        self.pin_domains     = pin_domains
        self.pin_banks       = pin_banks
        self.pin_csr_mapping = pin_csr_mapping
        self.with_odelay     = with_odelay

        self.cdc_cache  = cdc_cache = {}
        pin_oe_cache = {}
        for pin, count in pads.layout:
            if pin in ["mir", "cai", "ca_odt"]:
                self.comb += getattr(self.pads, pin).eq(0)
                continue

            assert pin in pin_domains, (pin, pin_domains)
            assert pin in pin_banks or count == len(pin_banks[pin]), (pin, count)
            assert reduce(and_, [pin_banks[pin][0] == pin_banks[pin][i] for i in range(1, count)], 1)
            if pin[-2:] == "_c":
                continue

            _diff   = "_t" in pin
            _is_ck  = "ck" in pin
            _is_io  = reduce(or_, [pin_type in pin for pin_type in ["dq", "dm_n"]]) # dq is in dqs
            _is_out = pin_domains[pin][0] is not None
            _is_in  = pin_domains[pin][1] is not None
            suffix  = f"_{pin_banks[pin][0]}"

            for i in range(count):
                if "_c" == pin[-2:]:
                    continue
                _in, _out = self.get_domains(pin, _is_in, _is_out, suffix)

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

    def get_domains(self, pin, is_in, is_out, suffix):
        cd_out, cd_in = self.pin_domains[pin]
        if is_out:
            cd_out = (cd_out[0]+suffix, cd_out[1]+suffix)
        if is_in:
            cd_in = (cd_in[0]+suffix, cd_in[1]+suffix)
        return cd_in, cd_out

    def iobuf(self, din, dout, tin, dinout, osc_en, osc, vref):
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

    def ibuf(self, din, dout, osc_en, osc, vref):
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

    def iobufds(self, din, dout, tin, dinout, dinout_b, osc_en, osc):
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

    def handle_single_ended(self, pad, *, out_sig=None, oe_sig=None, in_sig=None):
        if in_sig is not None and out_sig is not None:
            self.iobuf(din=out_sig, dout=in_sig, tin=oe_sig, dinout=pad)
        elif in_sig is not None:
            self.ibuf(dout=in_sig, din=pad)
        else:
            self.comb += pad.eq(out_sig)

    def handle_diff(self, pad_t, pad_c, *, out_sig=None, oe_sig=None, in_sig=None):
        if in_sig is not None and out_sig is not None:
            self.iobufds(din=out_sig, dout=in_sig, tin=oe_sig, dinout=pad_t, dinout_b=pad_c)
        elif in_sig is not None:
            raise NotImplementedError()
        else:
            self.obufds(din=out_sig, dout=pad_t, dout_b=pad_c)

    def oserdese3_ddr(self, din, dout, tin, tout, clkdiv, clk, reset_sig):
        self.specials += Instance("OSERDESE3",
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_DATA_WIDTH         = 4,
            p_INIT               = 0,
            p_IS_RST_INVERTED    = 0,
            p_IS_CLK_INVERTED    = 0,
            p_IS_CLKDIV_INVERTED = 0,
            i_RST    = reset_sig,
            i_CLK    = ClockSignal(clk),
            i_CLKDIV = ClockSignal(clkdiv),
            i_D      = din,
            i_T      = tin,
            o_OQ     = dout,
            o_T_OUT  = tout,
        )

    def odelaye3(self, din, dout, rst, inc, clk, cnt_value_out):
        base_delay_reg = Signal(9)
        clk_domain = getattr(self.sync, clk)
        clk_domain += [
            If(self.crg.get_load_base(clk),
                base_delay_reg.eq(cnt_value_out),
            )
        ]
        self.specials += Instance("ODELAYE3",
            attr = set(("IODELAY_GROUP", "DDR5_PHY")),
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_CASCADE          = "NONE",
            p_UPDATE_MODE      = "ASYNC",
            p_REFCLK_FREQUENCY = self.iodelay_clk_freq/1e6,
            p_DELAY_FORMAT     = "TIME",
            p_DELAY_TYPE       = "VAR_LOAD",
            p_DELAY_VALUE      = 0,
            i_RST     = self.crg.get_iodelay_rst(clk),
            i_LOAD    = rst,
            i_CLK     = ClockSignal(clk),
            i_EN_VTC  = self.crg.get_iodelay_vtc(clk)& self._en_vtc.storage,
            i_CE      = inc,
            i_INC     = 1,
            i_ODATAIN = din,
            o_DATAOUT = dout,
            i_CNTVALUEIN = base_delay_reg,
            o_CNTVALUEOUT = cnt_value_out,
        )

    def handle_oser(self, cd_out, out_sig, *, oe_sig=None, inc_sig=None, rst_sig=None):
        delay     = Signal()
        _output    = Signal()
        _tri_state = None
        _with_odelay = inc_sig is not None
        oser_method = self.oserdese3_ddr
        if oe_sig is not None:
            _tri_state = Signal()
            tri_state_domain = getattr(self.sync, cd_out[0])
            tri_state_domain += _tri_state.eq(reduce(or_, oe_sig))

        oserdes = oser_method(
            din = out_sig,
            dout=_output,
            **(dict(tout=_tri_state, tin=oe_sig) if oe_sig is not None else dict()),
            clkdiv  = cd_out[0],
            clk     = cd_out[1],
            rst_sig = self.crg.get_serdes_rst(cd_out[0]),
        )
        delay_state = None
        delay_state = Signal(9)
        self.odelaye3(
            din  = delay,
            dout = _output,
            rst  = rst_sig,
            inc  = inc_sig,
            clk  = "sys",
            cnt_value_out = delay_state,
        )
        return _output, _tri_state, delay_state

    def iserdese3_ddr(self, din, dout, clkdiv, clk, reset_sig):
        self.specials += Instance("ISERDESE3",
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_DATA_WIDTH         = 8,
            i_RST    = reset_sig,
            i_CLK    = ClockSignal(clk),
            i_CLK_B  = ~ClockSignal(clk),
            i_CLKDIV = ClockSignal(clkdiv),
            i_D      = din,
            o_Q      = dout,
        )

    def idelaye3(self, din, dout, rst, inc, clk, cnt_value_out):
        base_delay_reg = Signal(9)
        clk_domain = getattr(self.sync, clk)
        clk_domain += [
            If(self.crg.get_load_base(clk),
                base_delay_reg.eq(cnt_value_out),
            )
        ]
        self.specials += Instance("IDELAYE3",
            attr = set(("IODELAY_GROUP", "DDR5_PHY")),
            p_SIM_DEVICE         = "ULTRASCALE_PLUS",
            p_CASCADE          = "NONE",
            p_UPDATE_MODE      = "ASYNC",
            p_REFCLK_FREQUENCY = self.iodelay_clk_freq/1e6,
            p_DELAY_FORMAT     = "TIME",
            p_DELAY_TYPE       = "VARIABLE",
            p_DELAY_VALUE      = 0,
            i_RST     = self.crg.get_iodelay_rst(clk),
            i_LOAD    = rst,
            i_CLK     = ClockSignal(clk),
            i_EN_VTC  = self.crg.get_iodelay_vtc(clk) & self._en_vtc.storage,
            i_CE      = inc,
            i_INC     = 1,
            i_IDATAIN = din,
            o_DATAOUT = dout,
            o_CNTVALUEOUT = cnt_value_out,
            i_CNTVALUEIN = cnt_value_out,
        )

    def handle_iser(self, cd_in, in_sig, *, inc_sig=None, rst_sig=None):
        _input = Signal()
        _delayed_input = Signal()
        delay_state = Signal(9)
        self.idelaye3(
            din  = _input,
            dout = _delayed_input,
            rst  = rst_sig,
            inc  = inc_sig,
            clk  = "sys",
            cnt_value_out = delay_state,
        )

        self.iserdese3_ddr(
            din     = _delayed_input,
            dout    = in_sig,
            clk     = cd_in[1],
            clkdiv  = cd_in[0],
            rst_sig = self.crg.get_serdes_rst(cd_in[0]),
        )
        return _input, delay_state

    def get_pads(self, pin, *, offset=None):
        if offset is None:
            offset=0
        pad_t = getattr(self.pads, pin)[offset]
        pad_c = getattr(self.pads, pin[:-2]+"_c", None)
        if pad_c is not None:
            pad_c = pad_c[offset]
        return (pad_t, pad_c)


    def get_inc_rst(self, pin, cd, not_out, offset):
        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])
        dq = True if _pin_func in "dq" else False
        inc = None
        rst = None
        if pin not in self.pin_csr_mapping:
            return None, None
        if (pin, not_out, cd) not in self.cdc_cache:
            if cd != "sys":
                self.cdc_cache[(pin, not_out, cd)] = (
                    self.cdc_any(cd)(self.pin_csr_mapping[pin][not_out][0]),
                    self.cdc_any(cd)(self.pin_csr_mapping[pin][not_out][1])
                )
            else:
                self.cdc_cache[(pin, not_out, cd)] = (
                    self.pin_csr_mapping[pin][not_out][0],
                    self.pin_csr_mapping[pin][not_out][1]
                )
        inc_sig, rst_sig = self.cdc_cache[(pin, not_out, cd)]
        if offset is not None:
            inc = self.get_inc(offset, inc_sig, prefix, cd, dq=dq)
            rst = self.get_rst(offset, rst_sig, prefix, cd,
                dq=dq, rst_overwrite=self.crg.get_rst(cd))
        else:
            inc  = inc_sig
            rst  = rst_sig
        return inc, rst

    def get_out_inc_rst(self, pin, *, cd, offset=None):
        return self.get_inc_rst(pin, cd, 0, offset)

    def get_in_inc_rst(self, pin, *, cd, offset=None):
        return self.get_inc_rst(pin, cd, 1, offset)

    def handle_o(self, cd_out, out_sig, pin, *, offset=None, oe_sig=None):
        pad_t, pad_c = self.get_pads(pin, offset=offset)

        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])

        if _pin_func == "ck_t":
            offset = None
        inc_sig, rst_sig = None, None
        if self.with_odelay and pin in self.pin_csr_mapping:
            inc_sig, rst_sig = self.get_out_inc_rst(pin, offset=offset, cd="sys")

        to_pad, to_pad_oe, delay_state = self.handle_oser(
            cd_out, out_sig, oe_sig=oe_sig, inc_sig=inc_sig, rst_sig=rst_sig)

        offset = offset if offset else 0
        if self.with_odelay:
            if "ca" == _pin_func:
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

        inc_sig, rst_sig = self.get_in_inc_rst(pin, offset=offset, cd="sys")
        from_pad, delay_state = self.handle_iser(
            cd_in=cd_in, in_sig=in_sig, inc_sig=inc_sig, rst_sig=rst_sig)

        if pad_c is not None:
            self.handle_diff(pad_t, pad_c, in_sig=from_pad)
        else:
            self.handle_single_ended(pad_t, in_sig=from_pad)

    def handle_io(self, cd_out, cd_in, out_sig, oe_sig, in_sig, pin, *, offset=None):
        pad_t, pad_c = self.get_pads(pin, offset=offset)
        prefix, _pin_func = ("", pin) if len(self.prefixes) == 1 else (pin[:2], pin[2:])

        if "dqs" in _pin_func and offset:
            offset *= self.dq_dqs_ratio//4

        inc_sig, rst_sig = None, None
        if self.with_odelay and pin in self.pin_csr_mapping:
            inc_sig, rst_sig = self.get_out_inc_rst(pin, offset=offset, cd="sys")

        to_pad, to_pad_oe, odelay_state = self.handle_oser(
            cd_out=cd_out, out_sig=out_sig, oe_sig=oe_sig, inc_sig=inc_sig, rst_sig=rst_sig)

        inc_sig, rst_sig = self.get_in_inc_rst(pin, offset=offset, cd="sys")
        from_pad, idelay_state = self.handle_iser(
            cd_in=cd_in, in_sig=in_sig, inc_sig=inc_sig, rst_sig=rst_sig)

        offset = offset if offset else 0
        if "dq" == _pin_func:
            if offset%4 == 0:
                self.sync += [
                    If(self.CSRs[prefix+'dly_sel'].storage[offset//4],
                        self.CSRs[prefix+'rdly_dq'].status.eq(idelay_state),
                    ),
                ]
                if self.with_odelay:
                    self.sync += [
                        If(self.CSRs[prefix+'dly_sel'].storage[offset//4],
                            self.CSRs[prefix+'wdly_dq'].status.eq(odelay_state),
                        ),
                    ]
        elif "dqs" in _pin_func:
            self.sync += [
                If(self.CSRs[prefix+'dly_sel'].storage[offset],
                    self.CSRs[prefix+'rdly_dqs'].status.eq(idelay_state),
                ),
            ]
            if self.with_odelay:
                self.sync += [
                    If(self.CSRs[prefix+'dly_sel'].storage[offset],
                        self.CSRs[prefix+'wdly_dqs'].status.eq(odelay_state),
                    ),
                ]

        if pad_c is not None:
            self.handle_diff(pad_t, pad_c, out_sig=to_pad, oe_sig=to_pad_oe, in_sig=from_pad)
        else:
            self.handle_single_ended(pad_t, out_sig=to_pad, oe_sig=to_pad_oe, in_sig=from_pad)

    def handle_ck(self, cd_out, pin, offset=None):
        clk_sig = Signal(4)
        self.comb += clk_sig.eq(self.clk_pattern&0xF)
        self.handle_o(cd_out=cd_out, out_sig=clk_sig, pin=pin, offset=offset)
