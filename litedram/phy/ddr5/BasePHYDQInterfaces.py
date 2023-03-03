#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen.fhdl.module import Module
from migen.genlib.record import Record

class BasePHYDQPadInput(Record):
    @staticmethod
    def data_layout(nphases):
        base_layout = [
            (f"dq{i}_oe", 2*nphases) for i in range(2)
        ] + [
            (f"dq{i}_o", 2*nphases) for i in range(8)
        ]
        base_layout.append(("dm_n_o", 2*nphases))
        return base_layout
    def __init__(self, nphases):
        layout = self.data_layout(nphases)
        Record.__init__(self, layout)


class BasePHYDQPadOutput(Record):
    @staticmethod
    def data_layout(nphases):
        base_layout = [
            (f"dqs{i}_t_i", 2*nphases) for i in range(2)
        ] + [
            (f"dq{i}_i", 2*nphases) for i in range(8)
        ]
        return base_layout
    def __init__(self, nphases):
        phy = self.data_layout(nphases)
        Record.__init__(self, phy)


class BasePHYDQPhyInput(Record):
    @staticmethod
    def data_layout(nphases):
        dfi_layout = [
            ("rddata", 2*8),
        ]
        return dfi_layout
    def __init__(self, nphases):
        dfi = self.data_layout(nphases)
        layout = [(f"p{i}", dfi) for i in range(nphases)]
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class BasePHYDQPhyOutputCTRL(Record):
    @staticmethod
    def data_layout(nphases):
        base_layout = [
            ("wrdata_en", 1),
            ("rddata_en", 1),
        ]
        return base_layout
    def __init__(self, nphases):
        layout = [(f"p{i}", self.data_layout(nphases)) for i in range(nphases)]
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class BasePHYDQPhyOutput(Record):
    @staticmethod
    def data_layout(nphases):
        base_layout = [
            ("wrdata", 2*8),
        ]
        base_layout.append(("wrdata_mask", 2))
        return base_layout
    def __init__(self, nphases):
        layout = [(f"p{i}", self.data_layout(nphases)) for i in range(nphases)]
        Record.__init__(self, layout)
        self.phases = [getattr(self, f"p{i}") for i in range(nphases)]


class BasePHYDQPhyOutputBuffer(Module):
    @classmethod
    def get_delay(cls, nphases):
        return nphases
    def __init__(self, src, target):
        for src_phase, target_phase in zip(src.phases, target.phases):
            for name, _ in src_phase.layout:
                self.sync += getattr(target_phase, name).eq(getattr(src_phase, name))
