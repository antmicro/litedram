# 1:2 frequency-ratio DDR3 PHY for Lattice's ECP5
# DDR3: 800 MT/s

import math
from collections import OrderedDict

from migen import *
from migen.genlib.misc import BitSlip
from migen.fhdl.specials import Tristate

from litex.soc.interconnect.csr import *

from litedram.common import PhySettings
from litedram.phy.dfi import *


def get_cl_cw(memtype, tck):
    f_to_cl_cwl = OrderedDict()
    if memtype == "DDR3":
        f_to_cl_cwl[800e6]  = (6, 5)
    else:
        raise ValueError
    for f, (cl, cwl) in f_to_cl_cwl.items():
        if tck >= 2/f:
            return cl, cwl
    raise ValueError

def get_sys_latency(nphases, cas_latency):
    return math.ceil(cas_latency/nphases)

def get_sys_phases(nphases, sys_latency, cas_latency):
    dat_phase = sys_latency*nphases - cas_latency
    cmd_phase = (dat_phase - 1)%nphases
    return cmd_phase, dat_phase


class ECP5DDRPHY(Module, AutoCSR):
    def __init__(self, pads, sys_clk_freq=100e6):
        memtype = "DDR3"
        tck = 2/(2*2*sys_clk_freq)
        addressbits = len(pads.a)
        bankbits = len(pads.ba)
        nranks = 1 if not hasattr(pads, "cs_n") else len(pads.cs_n)
        databits = len(pads.dq)
        nphases = 2

        self._wlevel_en = CSRStorage()
        self._wlevel_strobe = CSR()

        self._dly_sel = CSRStorage(databits//8)

        self._rdly_dq_rst = CSR()
        self._rdly_dq_inc = CSR()
        self._rdly_dq_bitslip_rst = CSR()
        self._rdly_dq_bitslip = CSR()

        self._wdly_dq_rst = CSR()
        self._wdly_dq_inc = CSR()
        self._wdly_dqs_rst = CSR()
        self._wdly_dqs_inc = CSR()

        # compute phy settings
        cl, cwl = get_cl_cw(memtype, tck)
        cl_sys_latency = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)

        rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
        wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)
        self.settings = PhySettings(
            memtype=memtype,
            dfi_databits=4*databits,
            nranks=nranks,
            nphases=nphases,
            rdphase=rdphase,
            wrphase=wrphase,
            rdcmdphase=rdcmdphase,
            wrcmdphase=wrcmdphase,
            cl=cl,
            cwl=cwl,
            read_latency=2 + cl_sys_latency + 2 + log2_int(4//nphases) + 3, # FIXME
            write_latency=cwl_sys_latency
        )

        self.dfi = Interface(addressbits, bankbits, nranks, 4*databits, 4)

        # # #

        bl8_sel = Signal()

        # Clock
        for i in range(len(pads.clk_p)):
            sd_clk_se = Signal()
            self.specials += [
                Instance("ODDRX2F",
                    i_D0=0,
                    i_D1=1,
                    i_D2=0,
                    i_D3=1,
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=sd_clk_se
                ),
                Instance("OBUFDS",
                    i_I=sd_clk_se,
                    o_O=pads.clk_p[i],
                    o_OB=pads.clk_n[i]
                )
            ]

        # Addresses and commands
        for i in range(addressbits):
            self.specials += \
                Instance("ODDRX2F",
                    i_D0=self.dfi.phases[0].address[i],
                    i_D1=self.dfi.phases[0].address[i],
                    i_D2=self.dfi.phases[1].address[i],
                    i_D3=self.dfi.phases[1].address[i],
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=pads.a[i]
                )
        for i in range(bankbits):
            self.specials += \
                 Instance("ODDRX2F",
                    i_D0=self.dfi.phases[0].bank[i],
                    i_D1=self.dfi.phases[0].bank[i],
                    i_D2=self.dfi.phases[1].bank[i],
                    i_D3=self.dfi.phases[1].bank[i],
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=pads.ba[i]
                )
        controls = ["ras_n", "cas_n", "we_n", "cke", "odt"]
        if hasattr(pads, "reset_n"):
            controls.append("reset_n")
        if hasattr(pads, "cs_n"):
            controls.append("cs_n")
        for name in controls:
            for i in range(len(getattr(pads, name))):
                self.specials += \
                    Instance("ODDRX2F",
                        i_D0=getattr(self.dfi.phases[0], name)[i],
                        i_D1=getattr(self.dfi.phases[0], name)[i],
                        i_D2=getattr(self.dfi.phases[1], name)[i],
                        i_D3=getattr(self.dfi.phases[1], name)[i],
                        i_ECLK=ClockSignal("sys2x"),
                        i_SCLK=ClockSignal(),
                        i_RST=ResetSignal(),
                        o_Q=getattr(pads, name)[i]
                )

        # DQS and DM
        oe_dqs = Signal()
        dqs_preamble = Signal()
        dqs_postamble = Signal()
        dqs_serdes_pattern = Signal(8, reset=0b01010101)
        self.comb += \
            If(self._wlevel_en.storage,
                If(self._wlevel_strobe.re,
                    dqs_serdes_pattern.eq(0b00000001)
                ).Else(
                    dqs_serdes_pattern.eq(0b00000000)
                )
            ).Elif(dqs_preamble | dqs_postamble,
                dqs_serdes_pattern.eq(0b0000000)
            ).Else(
                dqs_serdes_pattern.eq(0b01010101)
            )

        for i in range(databits//8):
            dm_o_nodelay = Signal()
            dm_data = Signal(8)
            dm_data_d = Signal(8)
            dm_data_muxed = Signal(4)
            self.comb += dm_data.eq(Cat(
                self.dfi.phases[0].wrdata_mask[0*databits//8+i], self.dfi.phases[0].wrdata_mask[1*databits//8+i],
                self.dfi.phases[0].wrdata_mask[2*databits//8+i], self.dfi.phases[0].wrdata_mask[3*databits//8+i],
                self.dfi.phases[1].wrdata_mask[0*databits//8+i], self.dfi.phases[1].wrdata_mask[1*databits//8+i],
                self.dfi.phases[1].wrdata_mask[2*databits//8+i], self.dfi.phases[1].wrdata_mask[3*databits//8+i]),
            )
            self.sync += dm_data_d.eq(dm_data)
            self.comb += \
                If(bl8_sel,
                    dm_data_muxed.eq(dm_data_d[4:])
                ).Else(
                    dm_data_muxed.eq(dm_data[:4])
                )
            self.specials += \
                Instance("ODDRX2F",
                    i_D0=dm_data_muxed[0],
                    i_D1=dm_data_muxed[1],
                    i_D2=dm_data_muxed[2],
                    i_D3=dm_data_muxed[3],
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=dm_o_nodelay
                )
            self.specials += \
                Instance("DELAYF",
                i_A=dm_o_nodelay,
                i_LOADN=self._dly_sel.storage[i] & self._wdly_dq_rst.re,
                i_MOVE=self._dly_sel.storage[i] & self._wdly_dq_inc.re,
                i_DIRECTION=0,
                o_Z=pads.dm[i],
                #o_CFLAG=,
            )

            dqs_nodelay = Signal()
            dqs_delayed = Signal()
            dqs_oe = Signal()
            self.specials += \
                Instance("ODDRX2F",
                    i_D0=dqs_serdes_pattern[0],
                    i_D1=dqs_serdes_pattern[1],
                    i_D2=dqs_serdes_pattern[2],
                    i_D3=dqs_serdes_pattern[3],
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=dm_o_nodelay
                )
            self.specials += \
                Instance("DELAYF",
                i_A=dqs_nodelay,
                i_LOADN=self._dly_sel.storage[i] & self._wdly_dqs_rst.re,
                i_MOVE=self._dly_sel.storage[i] & self._wdly_dqs_inc.re,
                i_DIRECTION=0,
                o_Z=dqs_delayed,
                #o_CFLAG=,
            )
            self.specials += \
                Instance("TSHX2DQA",
                    i_T0=oe_dqs,
                    i_T1=oe_dqs,
                    i_SCLK=ClockSignal(),
                    i_ECLK=ClockSignal("sys2x"),
                    i_RST=ResetSignal(),
                    o_q=dqs_oe,
                )
            self.specials += Tristate(pads.dqs_p[i], dqs_delayed, dqs_oe)

        # DQ
        oe_dq = Signal()
        for i in range(databits):
            dq_o_nodelay = Signal()
            dq_o_delayed = Signal()
            dq_i_nodelay = Signal()
            dq_i_delayed = Signal()
            dq_oe = Signal()
            dq_data = Signal(8)
            dq_data_d = Signal(8)
            dq_data_muxed = Signal(4)
            self.comb += dq_data.eq(Cat(
                self.dfi.phases[0].wrdata[0*databits+i], self.dfi.phases[0].wrdata[1*databits+i],
                self.dfi.phases[0].wrdata[2*databits+i], self.dfi.phases[0].wrdata[3*databits+i],
                self.dfi.phases[1].wrdata[0*databits+i], self.dfi.phases[1].wrdata[1*databits+i],
                self.dfi.phases[1].wrdata[2*databits+i], self.dfi.phases[1].wrdata[3*databits+i])
            )
            self.sync += dq_data_d.eq(dq_data)
            self.comb += \
                If(bl8_sel,
                    dq_data_muxed.eq(dq_data_d[4:])
                ).Else(
                    dq_data_muxed.eq(dq_data[:4])
                )
            self.specials += \
                Instance("ODDRX2F",
                    i_D0=dq_data_muxed[0],
                    i_D1=dq_data_muxed[1],
                    i_D2=dq_data_muxed[2],
                    i_D3=dq_data_muxed[3],
                    i_ECLK=ClockSignal("sys2x"),
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=dq_o_nodelay
                )

            self.specials += \
                Instance("DELAYF",
                    i_A=dq_o_nodelay,
                    i_LOADN=self._dly_sel.storage[i//8] & self._wdly_dq_rst.re,
                    i_MOVE=self._dly_sel.storage[i//8] & self._wdly_dq_inc.re,
                    i_DIRECTION=0,
                    o_Z=dq_o_delayed,
                    #o_CFLAG=,
                )
            dq_i_data = Signal(8)
            dq_i_data_d = Signal(8)
            self.specials += \
                Instance("IDDRX2F",
                    i_D=dq_i_delayed,
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    i_ECLK=ClockSignal("sys2x"),
                    i_ALIGNWD=0,
                    o_Q0=dq_i_data[0],
                    o_Q1=dq_i_data[1],
                    o_Q2=dq_i_data[2],
                    o_Q3=dq_i_data[3],
                )
            dq_bitslip = BitSlip(4)
            self.comb += dq_bitslip.i.eq(dq_i_data)
            self.sync += \
                If(self._dly_sel.storage[i//8],
                    If(self._rdly_dq_bitslip_rst.re,
                        dq_bitslip.value.eq(0)
                    ).Elif(self._rdly_dq_bitslip.re,
                        dq_bitslip.value.eq(dq_bitslip.value + 1)
                    )
                )
            self.submodules += dq_bitslip
            self.sync += dq_i_data_d.eq(dq_i_data)
            self.comb += [
                self.dfi.phases[0].rddata[i].eq(dq_bitslip.o[0]),
                self.dfi.phases[0].rddata[databits+i].eq(dq_bitslip.o[1]),
                self.dfi.phases[1].rddata[i].eq(dq_bitslip.o[2]),
                self.dfi.phases[1].rddata[databits+i].eq(dq_bitslip.o[3])
            ]
            self.specials += \
                Instance("DELAYF",
                    i_A=dq_i_nodelay,
                    i_LOADN=self._dly_sel.storage[i//8] & self._rdly_dq_rst.re,
                    i_MOVE=self._dly_sel.storage[i//8] & self._wdly_dq_inc.re,
                    i_DIRECTION=0,
                    o_Z=dq_i_delayed,
                    #o_CFLAG=,
                )
            self.specials += \
                Instance("TSHX2DQA",
                    i_T0=oe_dq,
                    i_T1=oe_dq,
                    i_SCLK=ClockSignal(),
                    i_ECLK=ClockSignal("sys2x"),
                    i_RST=ResetSignal(),
                    o_q=dq_oe,
                )
            self.specials += Tristate(pads.dq[i], dq_o_delayed, dq_oe, dq_i_nodelay)

        # Flow control
        #
        # total read latency:
        #  N cycles through ODDRX2F FIXME
        #  cl_sys_latency cycles CAS
        #  M cycles through IDDRX2F FIXME
        rddata_en = self.dfi.phases[self.settings.rdphase].rddata_en
        for i in range(self.settings.read_latency-1):
            n_rddata_en = Signal()
            self.sync += n_rddata_en.eq(rddata_en)
            rddata_en = n_rddata_en
        self.sync += [phase.rddata_valid.eq(rddata_en | self._wlevel_en.storage)
            for phase in self.dfi.phases]

        oe = Signal()
        last_wrdata_en = Signal(cwl_sys_latency+3)
        wrphase = self.dfi.phases[self.settings.wrphase]
        self.sync += last_wrdata_en.eq(Cat(wrphase.wrdata_en, last_wrdata_en[:-1]))
        self.comb += oe.eq(
            last_wrdata_en[cwl_sys_latency-1] |
            last_wrdata_en[cwl_sys_latency] |
            last_wrdata_en[cwl_sys_latency+1] |
            last_wrdata_en[cwl_sys_latency+2])
        self.sync += \
            If(self._wlevel_en.storage,
                oe_dqs.eq(1), oe_dq.eq(0)
            ).Else(
                oe_dqs.eq(oe), oe_dq.eq(oe)
            )

        self.sync += bl8_sel.eq(last_wrdata_en[cwl_sys_latency-1])
