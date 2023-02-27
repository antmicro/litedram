#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import or_
from functools import reduce

from migen.fhdl.structure import Signal, If, Cat, Replicate, Case
from migen.fhdl.module import Module
from migen.genlib.record import Record
from migen.genlib.fifo import SyncFIFO

from litedram.common import TappedDelayLine
from litedram.phy.ddr5.BasePHYPatternGenerators import DQOePattern, DQSPattern

class BasePHYWritePathInput(Record):
    @staticmethod
    def data_layout(nphases, dq_dqs_ratio):
        base_layout = [
            ("wrdata_en", 1),
            ("wrdata", 2*dq_dqs_ratio),
        ]
        base_layout.append(("wrdata_mask", 2))
        return base_layout
    def __init__(self, nphases, dq_dqs_ratio):
        self.dq_dqs_ratio = dq_dqs_ratio
        layout = [(f"p{i}", self.data_layout(nphases, dq_dqs_ratio)) for i in range(nphases)]
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class BasePHYWritePathOutput(Record):
    @staticmethod
    def data_layout(nphases, dq_dqs_ratio):
        base_layout = [
            ("dqs_t_o", 2*nphases),
            ("dqs_c_o", 2*nphases),
            ("dqs_oe", 2*nphases),
            ("dq_oe", 2*nphases),
        ] + [
            (f"dq{i}_o", 2*nphases) for i in range(dq_dqs_ratio)
        ]
        base_layout.append(("dm_n_o", 2*nphases))
        return base_layout
    def __init__(self, nphases, dq_dqs_ratio):
        self.dq_dqs_ratio = dq_dqs_ratio
        layout = self.data_layout(nphases, dq_dqs_ratio)
        Record.__init__(self, layout)


class _BasePHYWritePathBuffer(Module):
    @classmethod
    def get_delay(cls, nphases):
        return nphases
    def __init__(self, src, target):
        for src_phase, target_phase in zip(src.phases, target.phases):
            for name, _ in src_phase.layout:
                self.sync += getattr(target_phase, name).eq(getattr(src_phase, name))


class BasePHYWritePath(Module):
    write_addjust = None
    min_write_latency = None
    max_write_latency = 66

    @classmethod
    def get_min_max_supported_latencies(cls, nphases, address_delay):
        # Delay is:
        # WRData buffering,
        # plus wrdata_en 0 tap delay,
        # plus we need to look 2 cycles "into the future" to properly generate write preablek
        # reduce by 1 as cmd has 2 beats,
        # reduce by CA delay
        min_write_latency = _BasePHYWritePathBuffer.get_delay(nphases) + \
            nphases + 2 - 1 - address_delay
        cls.write_addjust = -min(0, min_write_latency)
        cls.min_write_latency = min_write_latency + cls.write_addjust

        return (cls.min_write_latency, cls.max_write_latency - 1, cls.write_addjust)

    def __init__(self, dfi, out, CSRs, default_write_latency=0, SyncFIFO_cls=SyncFIFO, with_data_mask=False):
        nphases = len(dfi.phases)
        nphases_log = nphases.bit_length() - 1
        assert nphases > 1 and (nphases & (nphases-1)) == 0

        internal_dfi = BasePHYWritePathInput(nphases, dfi.dq_dqs_ratio)
        self.submodules += _BasePHYWritePathBuffer(dfi, internal_dfi)

        # min latency addjusted plus 66 plus nphases + 1 (just to be sure) plus round to next integer
        wrtap = (self.min_write_latency + self.write_addjust + self.max_write_latency + nphases + 1 + nphases - 1) // nphases
        assert wrtap >= 0

        # Create a delay line of write commands coming from the DFI interface. This taps are used to
        # control DQ/DQS tristates.

        wrdata_en_comb = Signal(nphases)
        self.comb += wrdata_en_comb.eq(Cat([phase.wrdata_en for phase in internal_dfi.phases]))

        wrdata_en = TappedDelayLine(
            signal = wrdata_en_comb,
            ntaps  = wrtap
        )
        self.submodules += wrdata_en

        assert default_write_latency >= self.min_write_latency or default_write_latency == 0, \
        f"default_write_latency={default_write_latency} is to small, min_write_latency={self.min_write_latency}"

        wr_reset_value = 0 if default_write_latency < self.min_write_latency else default_write_latency - self.min_write_latency

        wr_dqs_max_delay = self.max_write_latency - 1 + self.write_addjust

        wr_window       = Signal(nphases + 3)
        wr_delay        = Signal(max=wr_dqs_max_delay + 1, reset=wr_reset_value)
        wr_index        = Signal(max=wr_dqs_max_delay // nphases + 1)
        wr_offset       = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

        self.sync += [
            If(CSRs['dly_sel'] & CSRs['ck_wdly_inc'] & \
               (wr_delay < wr_dqs_max_delay),
                wr_delay.eq(wr_delay + 1),
            ).Elif(CSRs['dly_sel'] & CSRs['ck_wdly_rst'],
                wr_delay.eq(wr_reset_value),
            ),
        ]

        self.comb += [
            wr_index.eq(wr_delay[nphases_log:]),
            wr_offset.eq(wr_delay[:nphases_log]),
        ]

        wr_cases = {}
        if nphases > 1:
            for i in range(nphases):
                if 3+i <= nphases:
                    wr_cases[i] = wr_window.eq(
                        Cat(wrdata_en.taps[wr_index+1][nphases-(3+i):],
                            wrdata_en.taps[wr_index][:nphases-i]
                    ))
                else:
                    wr_cases[i] = wr_window.eq(
                        Cat(wrdata_en.taps[wr_index+2][2*nphases-(3+i):],
                            wrdata_en.taps[wr_index+1],
                            wrdata_en.taps[wr_index][:nphases-i]
                    ))
        else:
            wr_cases[0] = wr_window.eq(
                Cat(wrdata_en.taps[wr_index+3],
                    wrdata_en.taps[wr_index+2],
                    wrdata_en.taps[wr_index+1],
                    wrdata_en.taps[wr_index]
                ))

        self.comb += [
            Case(wr_offset,
                wr_cases,
            )
        ]

        dqs_oe        = Signal(2*nphases)
        dqs_pattern   = DQSPattern(
            nphases   = nphases,
            wlevel_en = CSRs['wlevel_en'],
        )
        self.comb += dqs_pattern.window.eq(wr_window)
        self.submodules += dqs_pattern

        self.comb += [
            out.dqs_t_o.eq(dqs_pattern.o,),
            out.dqs_c_o.eq(~dqs_pattern.o,),
            out.dqs_oe.eq(dqs_pattern.oe),
        ]

        wr_dq_max_delay = self.max_write_latency - 1 + self.write_addjust + 2

        wr_data_window  = Signal(nphases+1)
        wr_data_delay   = Signal(max=wr_dq_max_delay + 1, reset=wr_reset_value + 2)
        wr_data_index   = Signal(max=wr_dq_max_delay // nphases + 1)
        wr_data_offset  = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

        self.sync += [
            If(CSRs['dly_sel'] & CSRs['ck_wddly_inc'] & \
               (wr_data_delay < wr_dq_max_delay),
                wr_data_delay.eq(wr_data_delay + 1),
            ).Elif(CSRs['dly_sel'] & CSRs['ck_wddly_rst'],
                wr_data_delay.eq(wr_reset_value + 2),
            ),
        ]

        self.comb += [
            wr_data_index.eq(wr_data_delay[nphases_log:]),
            wr_data_offset.eq(wr_data_delay[:nphases_log]),
        ]

        wr_data_cases = {}
        for i in range(nphases):
            if 1+i <= nphases: # only false for last i = nphases -1
                wr_data_cases[i] = wr_data_window.eq(
                    Cat(wrdata_en.taps[wr_data_index+1][nphases-(1+i):],
                        wrdata_en.taps[wr_data_index][:nphases-i]))

        self.comb += [
            Case(wr_data_offset,
                wr_data_cases,
            )
        ]

        dq_oe        = Signal(2*nphases)
        dq_pattern   = DQOePattern(
            nphases   = nphases,
            wlevel_en = CSRs['wlevel_en'],
        )
        self.comb += dq_pattern.window.eq(wr_data_window)
        self.submodules += dq_pattern

        self.comb += [
            out.dq_oe.eq(dq_pattern.oe),
        ]

        # Write Data Path ----------------------------------------------------------------------------
        dq_dqs_ratio = internal_dfi.dq_dqs_ratio
        fifo_width = dq_dqs_ratio
        if with_data_mask:
            fifo_width = dq_dqs_ratio + 1

        wr_fifo = SyncFIFO_cls(width=fifo_width*nphases*2, depth=wrtap, fwft=False)
        self.submodules += wr_fifo

        self.comb += [
            wr_fifo.din[:dq_dqs_ratio*2*nphases].eq(Cat(phase.wrdata for phase in internal_dfi.phases)),
            If(wr_data_index > 0,
                wr_fifo.we.eq(reduce(or_, [phase.wrdata_en for phase in internal_dfi.phases])),
            ),
        ]

        wr_data             = Signal(2*nphases*dq_dqs_ratio)
        wr_fifo_data        = Signal(2*nphases*dq_dqs_ratio)
        wr_input_data       = Signal(2*nphases*dq_dqs_ratio)
        wr_register_data    = Signal(2*nphases*dq_dqs_ratio)
        wr_fifo_data_valid  = Signal()
        self.sync += wr_fifo_data_valid.eq(wr_fifo.re & wr_fifo.readable)
        self.sync += [
            wr_input_data.eq(Cat([phase.wrdata for phase in internal_dfi.phases])),
        ]
        self.comb += [
            If(wr_data_index > 0,
                wr_fifo.re.eq(reduce(or_, wrdata_en.taps[wr_data_index-1])),
                If(wr_fifo_data_valid,
                    wr_fifo_data.eq(wr_fifo.dout[:2*dq_dqs_ratio*nphases]),
                ),
            ).Else(
                wr_fifo_data.eq(wr_input_data),
            ),
        ]

        wr_cases_comb = {}
        wr_cases_sync = {}

        wr_cases_comb[0] = [
            wr_data.eq(Cat(wr_fifo_data[:nphases*2*dq_dqs_ratio])),
        ]
        wr_cases_sync[0] = [
            wr_register_data.eq(0),
        ]

        for i in range(1, nphases):
            wr_cases_comb[i] = [
                wr_data.eq(Cat(wr_register_data[:i*2*dq_dqs_ratio], wr_fifo_data[:(nphases-i)*2*dq_dqs_ratio])),
            ]
            wr_cases_sync[i] = [
                wr_register_data.eq(wr_fifo_data[(nphases-i)*2*dq_dqs_ratio:]),
            ]

        if with_data_mask:
            self.comb += [
                wr_fifo.din[dq_dqs_ratio*2*nphases:].eq(Cat(phase.wrdata_mask for phase in internal_dfi.phases)),
            ]

            wr_dm               = Signal(2*nphases)
            wr_fifo_dm          = Signal(2*nphases)
            wr_input_dm         = Signal(2*nphases)
            wr_register_dm      = Signal(2*nphases)
            self.sync += [
                wr_input_dm.eq(Cat([phase.wrdata_mask for phase in internal_dfi.phases])),
            ]
            self.comb += [
                If(wr_data_index > 0,
                    If(wr_fifo_data_valid,
                        wr_fifo_dm.eq(wr_fifo.dout[2*dq_dqs_ratio*nphases:]),
                    ),
                ).Else(
                    wr_fifo_dm.eq(wr_input_dm),
                ),
            ]
            wr_cases_comb[0] += [
                wr_dm.eq(Cat(wr_fifo_dm[:nphases*2])),
            ]
            wr_cases_sync[0] += [
                wr_register_dm.eq(0),
            ]

            for i in range(1, nphases):
                wr_cases_comb[i] += [
                    wr_dm.eq(Cat(wr_register_dm[:i*2], wr_fifo_dm[:(nphases-i)*2])),
                ]
                wr_cases_sync[i] += [
                    wr_register_dm.eq(wr_fifo_dm[(nphases-i)*2:]),
                ]

        self.comb += [
            Case(wr_data_offset,
                wr_cases_comb
            ),
        ]

        self.sync += [
            Case(wr_data_offset,
                wr_cases_sync
            ),
        ]

        # DM ---------------------------------------------------------------------------------------
        # With DM enabled, masking is performed only when the command used is WRITE-MASKED.
        if with_data_mask:
            self.comb += out.dm_n_o.eq(~wr_dm)

        # DQ ---------------------------------------------------------------------------------------
        for bit in range(dq_dqs_ratio):
            # output
            _wrdata = [
                wr_data[i * dq_dqs_ratio + bit] for i in range(2*nphases)
            ]
            self.comb += getattr(out, f'dq{bit}_o').eq(Cat(_wrdata))

