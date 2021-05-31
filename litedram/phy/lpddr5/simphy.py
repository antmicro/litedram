#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litedram.phy.utils import delayed, Serializer, Deserializer, Latency, SimPad, SimulationPads, SimSerDesMixin
from litedram.phy.lpddr5.basephy import LPDDR5PHY

class LPDDR5SimulationPads(SimulationPads):
    def layout(self, databits=16):
        return [
            SimPad("reset_n", 1),
            SimPad("ck", 1),
            SimPad("cs", 1),
            SimPad("ca", 7),
            SimPad("dq", databits, io=True),
            SimPad("wck", databits//8),
            SimPad("rdqs", databits//8, io=True),
            SimPad("dmi", databits//8, io=True),
        ]


class LPDDR5SimPHY(SimSerDesMixin, LPDDR5PHY):
    """LPDDR5 simulation PHY with direct 16:1 serializers

    For simulation purpose two additional "DDR" clock domains are requires.
    """
    def __init__(self, aligned_reset_zero=False, **kwargs):
        pads = LPDDR5SimulationPads()
        self.submodules += pads
        super().__init__(pads,
            ser_latency  = Latency(sys=Serializer.LATENCY),
            des_latency  = Latency(sys=Deserializer.LATENCY),
            phytype      = "LPDDR5SimPHY",
            **kwargs)

        delay = lambda sig, cycles: delayed(self, sig, cycles=cycles)
        sdr_ck     = dict(clkdiv="sys", clk="sys4x")
        sdr_ck_90  = dict(clkdiv="sys", clk="sys4x_90")
        ddr_ck     = dict(clkdiv="sys", clk="sys8x")
        ddr_ck_90  = dict(clkdiv="sys", clk="sys8x_90")
        ddr_wck    = dict(clkdiv="sys", clk="sys8x_ddr")
        ddr_wck_90 = dict(clkdiv="sys", clk="sys8x_90_ddr")

        if aligned_reset_zero:
            sdr_ck["reset_cnt"] = 0
            ddr_ck["reset_cnt"] = 0
            ddr_wck["reset_cnt"] = 0

        self.comb += self.pads.reset_n.eq(self.out.reset_n)

        # CK signals
        # CK is shifted by 90 deg just by inversion
        # CS will then be properly aligned with respect to CK
        # CA needs 90 phase shift
        self.ser(i=~self.out.ck, o=self.pads.ck, name='ck', **ddr_ck)
        self.ser(i=self.out.cs, o=self.pads.cs, name='cs', **sdr_ck)
        for i in range(7):
            self.ser(i=self.out.ca[i], o=self.pads.ca[i], name=f'ca{i}', **ddr_ck_90)

        # WCK
        for i in range(self.databits//8):
            self.ser(i=self.out.wck[i], o=self.pads.wck[i], name=f'wck{i}', **ddr_wck_90)
            self.ser(i=self.out.dmi_o[i], o=self.pads.dmi_o[i], name=f'dmi_o{i}', **ddr_wck)
            self.des(o=self.out.dmi_i[i], i=self.pads.dmi[i],   name=f'dmi_i{i}', **ddr_wck)
            self.ser(i=self.out.rdqs_o[i], o=self.pads.rdqs_o[i], name=f'dqs_o{i}', **ddr_wck_90)
            self.des(o=self.out.rdqs_i[i], i=self.pads.rdqs[i],   name=f'dqs_i{i}', **ddr_wck_90)
        for i in range(self.databits):
            self.ser(i=self.out.dq_o[i], o=self.pads.dq_o[i], name=f'dq_o{i}', **ddr_wck)
            self.des(o=self.out.dq_i[i], i=self.pads.dq[i],   name=f'dq_i{i}', **ddr_wck)

        # Output enable signals
        self.comb += [
            self.pads.dmi_oe.eq(delay(self.out.dmi_oe, cycles=Serializer.LATENCY)),
            self.pads.rdqs_oe.eq(delay(self.out.rdqs_oe, cycles=Serializer.LATENCY)),
            self.pads.dq_oe.eq(delay(self.out.dq_oe, cycles=Serializer.LATENCY)),
        ]


# class DoubleRateLPDDR5SimPHY(SimSerDesMixin, DoubleRateLPDDR5PHY):
#     """LPDDR5 simulation PHY basing of DoubleRateLPDDR5PHY
#
#     For simulation purpose two additional "DDR" clock domains are requires.
#     """
#     def __init__(self, aligned_reset_zero=False, **kwargs):
#         pads = LPDDR5SimulationPads()
#         self.submodules += pads
#         super().__init__(pads,
#             ser_latency  = Latency(sys2x=Serializer.LATENCY),
#             des_latency  = Latency(sys2x=Deserializer.LATENCY),
#             phytype      = "LPDDR5SimPHY",
#             **kwargs)
#
#         self.submodules.half_delay = ClockDomainsRenamer("sys2x")(Module())
#         delay = lambda sig, cycles: delayed(self.half_delay, sig, cycles=cycles)
#
#         sdr    = dict(clkdiv="sys2x", clk="sys8x")
#         sdr_90 = dict(clkdiv="sys2x", clk="sys8x_90")
#         ddr    = dict(clkdiv="sys2x", clk="sys8x_ddr")
#         ddr_90 = dict(clkdiv="sys2x", clk="sys8x_90_ddr")
#
#         if aligned_reset_zero:
#             sdr["reset_cnt"] = 0
#             ddr["reset_cnt"] = 0
#
#         # Clock is shifted 180 degrees to get rising edge in the middle of SDR signals.
#         # To achieve that we send negated clock on clk (clk_p).
#         self.ser(i=~self.out.clk, o=self.pads.clk, name='clk', **ddr)
#
#         self.ser(i=self.out.cke, o=self.pads.cke, name='cke', **sdr)
#         self.ser(i=self.out.odt, o=self.pads.odt, name='odt', **sdr)
#         self.ser(i=self.out.reset_n, o=self.pads.reset_n, name='reset_n', **sdr)
#
#         # Command/address
#         self.ser(i=self.out.cs, o=self.pads.cs, name='cs', **sdr)
#         for i in range(6):
#             self.ser(i=self.out.ca[i], o=self.pads.ca[i], name=f'ca{i}', **sdr)
#
#         # Tristate I/O (separate for simulation)
#         for i in range(self.databits//8):
#             self.ser(i=self.out.dmi_o[i], o=self.pads.dmi_o[i], name=f'dmi_o{i}', **ddr)
#             self.des(o=self.out.dmi_i[i], i=self.pads.dmi[i],   name=f'dmi_i{i}', **ddr)
#             self.ser(i=self.out.dqs_o[i], o=self.pads.dqs_o[i], name=f'dqs_o{i}', **ddr_90)
#             self.des(o=self.out.dqs_i[i], i=self.pads.dqs[i],   name=f'dqs_i{i}', **ddr_90)
#         for i in range(self.databits):
#             self.ser(i=self.out.dq_o[i], o=self.pads.dq_o[i], name=f'dq_o{i}', **ddr)
#             self.des(o=self.out.dq_i[i], i=self.pads.dq[i],   name=f'dq_i{i}', **ddr)
#
#         # Output enable signals
#         self.comb += [
#             self.pads.dmi_oe.eq(delay(self.out.dmi_oe, cycles=Serializer.LATENCY)),
#             self.pads.dqs_oe.eq(delay(self.out.dqs_oe, cycles=Serializer.LATENCY)),
#             self.pads.dq_oe.eq(delay(self.out.dq_oe, cycles=Serializer.LATENCY)),
#         ]
