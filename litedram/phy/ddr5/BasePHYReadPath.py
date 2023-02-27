#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import or_, add
from functools import reduce

from migen.fhdl.structure import Signal, If, Cat, Replicate, Case, Array
from migen.fhdl.module import Module
from migen.genlib.record import Record
from migen.genlib.fifo import SyncFIFO

from litedram.common import ShiftRegister

class BasePHYReadPathInput(Record):
    @staticmethod
    def data_layout(nphases, dq_dqs_ratio):
        dfi_layout = [
            ("rddata_en", 1),
        ]
        phy_input_layout = [
            ("dqs_t_i", 2*nphases),
        ] + [
            (f"dq{i}_i", 2*nphases) for i in range(dq_dqs_ratio)
        ]
        return dfi_layout, phy_input_layout
    def __init__(self, nphases, dq_dqs_ratio):
        self.dq_dqs_ratio = dq_dqs_ratio
        dfi, phy = self.data_layout(nphases, dq_dqs_ratio)
        layout = [(f"p{i}", dfi) for i in range(nphases)] + phy
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class BasePHYReadPathOutput(Record):
    @staticmethod
    def data_layout(nphases, dq_dqs_ratio):
        dfi_layout = [
            ("rddata", 2*dq_dqs_ratio),
            ("rddata_valid", 1),
        ]
        return dfi_layout
    def __init__(self, nphases, dq_dqs_ratio):
        self.dq_dqs_ratio = dq_dqs_ratio
        dfi = self.data_layout(nphases, dq_dqs_ratio)
        layout = [(f"p{i}", dfi) for i in range(nphases)]
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class _BasePHYReadPathBuffer(Module):
    @classmethod
    def get_delay(cls, nphases):
        return nphases
    def __init__(self, src, target):
        for name, _ in target.layout:
            if "dq" in name:
                self.sync += getattr(target, name).eq(getattr(src, name))
        for s_phase, t_phase in zip(src.phases, target.phases):
            self.sync += t_phase.rddata_en.eq(s_phase.rddata_en)


class BasePHYReadPath(Module):
    min_read_latency = None
    max_read_latency = None
    read_latency = None

    @classmethod
    def get_min_max_supported_latencies(
        cls, nphases, address_delay, ser_latency, rd_extra_delay, des_latency):
        # Delay is:
        # command_delay and serialization
        # plus command length - 1
        # plus serialization latency
        # plus CDC latency
        # plus preamble
        # plus data deserialization
        cls.min_read_latency = address_delay + 2 - 1 + ser_latency.sys4x + rd_extra_delay.sys4x + \
            2 + des_latency.sys4x
        cls.max_read_latency = cls.min_read_latency + 66 + 1
        cls.read_latency = (cls.max_read_latency + nphases - 1) // nphases
        return cls.min_read_latency, cls.max_read_latency, _BasePHYReadPathBuffer.get_delay(nphases)

    # Read Control Path ------------------------------------------------------------------------
    # Creates a delay line of read commands coming from the DFI interface. The output is used to
    # signal a valid read data to the DFI interface.
    #
    # The read data valid is asserted for 1 sys_clk cycle when the data is available on the DFI
    # interface, the latency is the sum of the minimal PHY and user added delays.
    def __init__(self, dfi, phy, CSRs, default_read_latency=0):
        nphases = len(dfi.phases)
        nphases_log = nphases.bit_length() - 1
        assert nphases > 1 and (nphases & (nphases-1)) == 0
        rddata_en_input = Signal(nphases)
        internal_phy = BasePHYReadPathInput(nphases, dfi.dq_dqs_ratio)
        self.submodules += _BasePHYReadPathBuffer(phy, internal_phy)
        phy = internal_phy

        for i in range(nphases):
            self.comb += rddata_en_input[i].eq(phy.phases[i].rddata_en | CSRs['wlevel_en'])

        default_read_latency = default_read_latency - 2 if default_read_latency > 2 else 0
        rd_reset_value = self.min_read_latency + default_read_latency

        nphases_log = nphases.bit_length() - 1

        # Read window ----------------------------------------------------------------------
        rddata_ens = [
            ShiftRegister(
                signal = rddata_en_input[i],
                ntaps  = self.read_latency + 1
            ) for i in range(nphases)
        ]
        for i, rs in enumerate(rddata_ens):
            setattr(self.submodules, f"Read_SR_{i}", rs)

        rddata_out_en = ShiftRegister(
            signal = reduce(or_, rddata_en_input),
            ntaps  = self.read_latency + 3
        )
        setattr(self.submodules, f"Read_FIFO_SR_{i}", rddata_out_en)


        rd_window = Signal(nphases)
        rd_delay  = Signal(max=4*self.read_latency, reset=rd_reset_value)
        rd_index  = Signal(max=self.read_latency)
        rd_offset = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

        self.sync += [
            If(CSRs['dly_sel'] & CSRs['ck_rdly_inc'] & \
               (rd_delay < self.max_read_latency),
                rd_delay.eq(rd_delay + 1),
            ).Elif(CSRs['dly_sel'] & CSRs['ck_rdly_rst'],
                rd_delay.eq(rd_reset_value),
            ),
        ]

        self.comb += [
            rd_index.eq(rd_delay[nphases_log:]),
            rd_offset.eq(rd_delay[:nphases_log]),
        ]

        rd_index_p = [Signal(max=self.read_latency) for _ in range(nphases)]
        rd_cases = {}
        for i in range(nphases):
            first_part  = [rd_index_p[j].eq(rd_index + 1) for j in range(i)]
            second_part = [rd_index_p[j].eq(rd_index) for j in range(i, nphases)]
            rd_cases[i] = first_part + second_part

        self.comb += [
            Case(rd_offset,
                rd_cases,
            ),
            rd_window.eq(Cat([rddata_ens[i].taps[rd_index_p[i]] for i in range(nphases)])),
        ]

        # Read Preamble window -------------------------------------------------------------
        rddata_preamble_ens = [
            ShiftRegister(
                signal = rddata_en_input[i],
                ntaps  = self.read_latency
            ) for i in range(nphases)
        ]
        for i, rs in enumerate(rddata_preamble_ens):
            setattr(self.submodules, f"Preamble_SR_{i}", rs)

        rd_preamble_window      = Signal(nphases)
        rd_last_preamble_window = Signal(nphases)
        rd_preamble        = Signal(max=4*self.read_latency, reset=rd_reset_value - 2)
        rd_preamble_index  = Signal(max=self.read_latency)
        rd_preamble_offset = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

        self.sync += [
            If(CSRs['dly_sel'] & CSRs['ck_rdly_inc'] & \
               (rd_delay < self.max_read_latency),
                rd_preamble.eq(rd_preamble + 1),
            ).Elif(CSRs['dly_sel'] & CSRs['ck_rdly_rst'],
                rd_preamble.eq(rd_reset_value - 2),
            ),
        ]

        self.comb += [
            rd_preamble_index.eq(rd_preamble[nphases_log:]),
            rd_preamble_offset.eq(rd_preamble[:nphases_log]),
        ]

        rd_preamble_index_p = [Signal(max=self.read_latency) for _ in range(nphases)]
        rd_preamble_cases = {}
        for i in range(nphases):
            first_part  = [rd_preamble_index_p[j].eq(rd_preamble_index + 1) for j in range(i)]
            second_part = [rd_preamble_index_p[j].eq(rd_preamble_index) for j in range(i, nphases)]
            rd_preamble_cases[i] = first_part + second_part

        self.comb += [
            Case(rd_preamble_offset,
                rd_preamble_cases,
            ),
            rd_preamble_window.eq(
                Cat([rddata_preamble_ens[i].taps[rd_preamble_index_p[i]] for i in range(nphases)])
            ),
        ]
        self.sync += [
            rd_last_preamble_window.eq(rd_preamble_window),
        ]

        # Read Preamble Path -----------------------------------------------------------------------

        rd_preamble_rdy = Signal(max=2*nphases)
        self.comb += [
            If(~rd_last_preamble_window[-1] & rd_preamble_window[0],
                rd_preamble_rdy.eq(1)
            ),
        ]
        for i in range(1, nphases):
            self.comb += [
                If(~rd_preamble_window[i-1] & rd_preamble_window[i],
                    rd_preamble_rdy.eq(2*i | 1)
                ),
            ]

        rd_sampled_preamble = Signal(2*2)
        rd_preamble_cnt     = Signal()

        rd_preamble_cases_sync = {}
        for i in range(nphases):
            if i+1 < nphases:
                rd_preamble_cases_sync[i] = [
                    rd_sampled_preamble.eq(phy.dqs_t_i[i*2:i*2+4]),
                    rd_preamble_cnt.eq(0),
                ]
            else:
                rd_preamble_cases_sync[i] = [
                    rd_sampled_preamble[0:2].eq(phy.dqs_t_i[i*2:i*2+2]),
                    rd_preamble_cnt.eq(1),
                ]

        self.sync += [
            If(rd_preamble_rdy[0],
                Case(rd_preamble_rdy[1:],
                    rd_preamble_cases_sync
                ),
            ),
            If(rd_preamble_cnt == 1,
                rd_sampled_preamble[2:4].eq(phy.dqs_t_i[0:2]),
                rd_preamble_cnt.eq(0),
            ),
        ]

        self.comb += [
            If(CSRs['dly_sel'],
                CSRs['preamble'].eq(rd_sampled_preamble),
            ),
        ]
        # Read Data Path ----------------------------------------------------------------------------
        # The rd_window can present any arbitrary (1*0*)* pattern of length nphases.
        # We detect where one full DFI phase of data finishes and where other starts
        # by counting how many valid bits are set in the rd_window, and how many
        # are set in range [0:i-1], for the i = {0, .., nphases-1}.
        # When data for full DFI phase are collected, they are stored in FIFO and await
        # for settings.read_latency-1 to pass before being presented on DFI bus.

        dq_dqs_ratio = dfi.dq_dqs_ratio
        rd_fifo = SyncFIFO(width=dq_dqs_ratio*nphases*2, depth=self.read_latency, fwft=False)
        self.submodules += rd_fifo

        rddata_cnt          = Signal(max=nphases)
        rddata_intermediate = Array(Signal(2*dq_dqs_ratio) for _ in range(nphases))
        rddata_sel          = Array(Signal(2*dq_dqs_ratio) for _ in range(nphases))

        rddata_cnt_tmps      = [Signal(max=nphases) for _ in range(nphases)]
        rddata_cnt_and_tmp   = [Signal(max=2*nphases) for _ in range(nphases)]
        rddata_cnt_all_valid = Signal(max=2*nphases)

        self.comb += rddata_cnt_all_valid.eq(rddata_cnt + reduce(add, rd_window))

        for i in range(nphases):
            dq_start  = i*2
            dq_end    = (i+1)*2
            self.comb += [
                rddata_cnt_tmps[i].eq(reduce(add, rd_window[:i], 0)),
                rddata_cnt_and_tmp[i].eq(rddata_cnt + rddata_cnt_tmps[i]),
                If(rd_window[i] & ~rddata_cnt_and_tmp[i][nphases_log] & rddata_cnt_all_valid[nphases_log],
                    rddata_sel[rddata_cnt_and_tmp[i][:nphases_log]].eq(
                        Cat([getattr(phy, f'dq{dq}_i')[2*i] for dq in range(dq_dqs_ratio)] +
                            [getattr(phy, f'dq{dq}_i')[2*i+1] for dq in range(dq_dqs_ratio)])),
                ),
                If(i < rddata_cnt,
                    rddata_sel[i].eq(rddata_intermediate[i]),
                ),
            ]

            self.sync += [
                If(rd_window[i] & (rddata_cnt_and_tmp[i][nphases_log] | ~rddata_cnt_all_valid[nphases_log]),
                    rddata_intermediate[rddata_cnt_and_tmp[i][:nphases_log]].eq(
                        Cat([getattr(phy, f'dq{dq}_i')[2*i] for dq in range(dq_dqs_ratio)] +
                            [getattr(phy, f'dq{dq}_i')[2*i+1] for dq in range(dq_dqs_ratio)])),
                )
            ]

        self.sync += [
            If(reduce(or_, rd_window),
                rddata_cnt.eq(rddata_cnt_all_valid[:nphases_log]),
            ),
        ]

        self.comb += [
            rd_fifo.din.eq(0),
            rd_fifo.we.eq(0),
            If(reduce(or_, rd_window),
                If(rddata_cnt_all_valid[nphases_log],
                    rd_fifo.din.eq(Cat(rddata_sel)),
                    rd_fifo.we.eq(1),
                ),
            ),
        ]

        # Retime
        self.comb += [
            phase.rddata_valid.eq( \
                reduce(or_, rddata_out_en.output)) \
            for phase in dfi.phases
        ]

        rd_fifo_good = Signal()
        self.sync += [
            rd_fifo_good.eq(rd_fifo.re & rd_fifo.readable)
        ]

        self.comb += [
            If(rd_fifo_good,
                phase.rddata.eq(rd_fifo.dout[i*2*dq_dqs_ratio:(i+1)*2*dq_dqs_ratio])
            ) for i, phase in enumerate(dfi.phases)
        ] + [
            rd_fifo.re.eq(rddata_out_en.taps[-2])
        ]
