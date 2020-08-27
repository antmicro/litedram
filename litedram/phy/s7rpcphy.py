# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
# Etron RPC DRAM PHY for for Xilinx's Series7

from migen import *

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *
from litedram.phy.etronrpcphy import BasePHY, EM6GA16L, bitpattern

# Xilinx Artix7 RPC PHY ----------------------------------------------------------------------------

class A7RPCPHY(BasePHY):
    def __init__(self, iodelay_clk_freq, **kwargs):
        self.iodelay_clk_freq = iodelay_clk_freq

        self._rdly_dq_rst = CSR()
        self._rdly_dq_inc = CSR()

        kwargs.update(dict(
            write_ser_latency = 4//4,  # OSERDESE2 8:1 DDR (4 full-rate clocks)
            read_des_latency  = 2,  # ISERDESE2 NETWORKING
            phytype           = self.__class__.__name__,
        ))

        super().__init__(**kwargs)

    def do_clock_serialization(self, clk_1ck_out, clk_p, clk_n):
        clk = Signal()
        self.oserdese2_ddr(din=clk_1ck_out, dout=clk, clk="sys4x_90")
        self.specials += Instance("OBUFDS",
            i_I  = clk,
            o_O  = clk_p,
            o_OB = clk_n,
        )

    def do_stb_serialization(self, stb_1ck_out, stb):
        self.oserdese2_ddr(din=stb_1ck_out, dout=stb)

    def do_db_serialization(self, db_1ck_out, db_1ck_in, db_oe, db):
        for i in range(self.databits):
            db_out        = Signal()
            db_t          = Signal()
            db_in         = Signal()
            db_in_delayed = Signal()

            # Write path
            self.oserdese2_ddr(
                din=db_1ck_out[i], dout=db_out,
                tin=~db_oe,        tout=db_t,
            )

            # Read path
            self.specials += Instance("IDELAYE2",
                p_SIGNAL_PATTERN        = "DATA",
                p_DELAY_SRC             = "IDATAIN",
                p_CINVCTRL_SEL          = "FALSE",
                p_HIGH_PERFORMANCE_MODE = "TRUE",
                p_REFCLK_FREQUENCY      = self.iodelay_clk_freq/1e6,
                p_PIPE_SEL              = "FALSE",
                p_IDELAY_TYPE           = "VARIABLE",
                p_IDELAY_VALUE          = 0,
                i_C        = ClockSignal(),
                i_LD       = self._dly_sel.storage[i//8] & self._rdly_dq_rst.re,
                i_LDPIPEEN = 0,
                i_CE       = self._dly_sel.storage[i//8] & self._rdly_dq_inc.re,
                i_INC      = 1,
                i_IDATAIN  = db_in,
                o_DATAOUT  = db_in_delayed
            )
            self.iserdese2_ddr(din=db_in_delayed, dout=db_1ck_in[i])

            self.specials += Instance("IOBUF",
                i_I   = db_out,
                o_O   = db_in,
                i_T   = db_t,
                io_IO = db[i],
            )

    def do_dqs_serialization(self, dqs_1ck_out, dqs_1ck_in, dqs_oe, dqs_p, dqs_n):
        dqs_out  = Signal()
        dqs_in   = Signal()
        dqs_t    = Signal()

        self.oserdese2_ddr(
            clk="sys4x_90",
            din=dqs_1ck_out, dout=dqs_out,
            tin=~dqs_oe,     tout=dqs_t,
        )
        # TODO: proper deserialization
        self.iserdese2_ddr(din=dqs_in, dout=dqs_1ck_in)

        self.specials += Instance("IOBUFDS",
            i_T    = dqs_t,
            i_I    = dqs_out,
            o_O    = dqs_in,
            io_IO  = dqs_p,
            io_IOB = dqs_n,
        )

    def do_cs_serialization(self, cs_n_1ck_out, cs_n):
        self.oserdese2_ddr(din=cs_n_1ck_out, dout=cs_n)

    def oserdese2_ddr(self, *, din, dout, clk="sys4x", tin=None, tout=None):
        assert self.nphases == 4
        assert not ((tin is None) ^ (tout is None))

        params = dict(
            p_SERDES_MODE    = "MASTER",
            p_DATA_WIDTH     = 2*self.nphases,
            p_TRISTATE_WIDTH = 1,
            p_DATA_RATE_OQ   = "DDR",
            p_DATA_RATE_TQ   = "BUF",
            i_RST    = ResetSignal(),
            i_CLK    = ClockSignal(clk),
            i_CLKDIV = ClockSignal("sys"),
            o_OQ     = dout,
            i_OCE    = 1,
        )

        for i in range(2*self.nphases):
            params["i_D{}".format(i+1)] = din[i]

        if tin is not None:
            # with DATA_RATE_TQ=BUF tristate is asynchronous, so we need to delay it
            tin_d = Signal()
            self.sync += tin_d.eq(tin)
            params.update(dict(i_TCE=1, i_T1=tin_d, o_TQ=tout))

        self.specials += Instance("OSERDESE2", **params)

    def iserdese2_ddr(self, *, din, dout, clk="sys4x"):
        assert self.nphases == 4

        params = dict(
            p_SERDES_MODE    = "MASTER",
            p_INTERFACE_TYPE = "NETWORKING",  # TODO: try using MEMORY mode?
            p_DATA_WIDTH     = 2*self.nphases,
            p_DATA_RATE      = "DDR",
            p_NUM_CE         = 1,
            p_IOBDELAY       = "IFD",
            i_RST     = ResetSignal(),
            i_CLK     = ClockSignal(clk),
            i_CLKB    = ~ClockSignal(clk),
            i_CLKDIV  = ClockSignal("sys"),
            i_BITSLIP = 0,
            i_CE1     = 1,
            i_DDLY    = din,
        )

        for i in range(2*self.nphases):
            # invert order
            params["o_Q{}".format(i+1)] = dout[(2*self.nphases - 1) - i]

        self.specials += Instance("ISERDESE2", **params)
