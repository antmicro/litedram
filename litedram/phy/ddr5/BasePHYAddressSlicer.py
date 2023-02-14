#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import xor, and_, or_
from functools import reduce

from migen.fhdl.structure import Signal, If, Cat, Replicate
from migen.fhdl.module import Module
from migen.genlib.record import Record


class _DFIAddressBuffer(Module):
    @classmethod
    def dfi_delay(cls, nphases):
        return nphases

    def __init__(self, dfi, prefix):
        nranks  = len(getattr(dfi.phases[0], prefix).cs_n)
        nphases = len(dfi.phases)
        assert nranks > 0
        assert nphases > 0 and (nphases & (nphases-1)) == 0

        layout = [
            ("address", 14),
            ("cs_n", nranks),
            ("reset_n", 1),
            ("mode_2n", 1),
        ]

        self.phases = []
        for i in range(nphases):
            r = Record(layout)
            r.cs_n.reset=(2**layout[1][1]-1)
            setattr(self, f"p{i}", r)
            self.phases.append(r)
        for name, _ in layout:
            for i, phase in enumerate(dfi.phases):
                _sub_phase = getattr(phase, prefix)
                if name not in _sub_phase.__dict__:
                    _sub_phase = phase
                self.sync += \
                    getattr(self.phases[i], name).eq(getattr(_sub_phase, name))


class PHYAddressSlicer(Module):
    @classmethod
    def dfi_delay(cls, nphases):
        return _DFIAddressBuffer.dfi_delay(nphases) + nphases # base buffer + Slicer delay

    def __init__(self, out, dfi, rdimm_mode, prefix):
        # DDR5 CS/CA/PAR PATH ----------------------------------------------------------------------

        nranks  = len(getattr(dfi.phases[0], prefix).cs_n)
        nphases = len(dfi.phases)
        assert nranks > 0
        assert nphases > 0 and (nphases & (nphases-1)) == 0

        # Buffer DFI -------------------------------------------------------------------------------
        cmd_buff = _DFIAddressBuffer(dfi, prefix)
        self.submodules.BufferDFICommand = cmd_buff

        # DDR5 CS ----------------------------------------------------------------------------------
        carry_cs_n = Signal(nranks, reset=2**nranks-1)
        self.sync += [
            carry_cs_n.eq(cmd_buff.phases[-1].cs_n),
        ]

        for rank in range(nranks):
            cs_n = getattr(out, prefix + 'cs_n')
            for j in range(nphases):
                self.sync += [
                    If(~cmd_buff.phases[j].mode_2n,
                        cs_n[rank][2*j].eq(cmd_buff.phases[j].cs_n[rank]),
                    ).Else(
                        cs_n[rank][2*j].eq(carry_cs_n[rank] if j == 0 else cmd_buff.phases[j-1].cs_n[rank]),
                    ),
                    cs_n[rank][2*j+1].eq(cmd_buff.phases[j].cs_n[rank]),
                ]

        # DDR5 CA ----------------------------------------------------------------------------------
        # RDIMM 2N mode ----------------------------------------------------------------------------
        mem   = Signal(max(3, nphases))

        take_lower_bits   = Signal(nphases)
        take_lower_bits_m = Signal(nphases)
        take_lower_bits_1 = Signal(nphases)
        take_lower_bits_2 = Signal(nphases)
        for i in range(1, len(take_lower_bits)):
            self.comb += take_lower_bits_1[i].eq(~reduce(and_, cmd_buff.phases[i-1].cs_n))
        for i in range(3, len(take_lower_bits)):
            self.comb += take_lower_bits_2[i].eq(
                ~reduce(and_, cmd_buff.phases[i-3].cs_n) & ~cmd_buff.phases[i-3].address[1]
            )

        self.comb += take_lower_bits_m.eq(
            Cat(cmd_buff.phases[i].mode_2n for i in range(nphases)))
        for i in range(0, 3, nphases):
            for j in range(nphases):
                if i+j >= 3:
                    break
                arr = []
                if i+j+nphases < 3:
                    arr.append(mem[i+j+nphases])
                if i + j < 1:
                    arr.append(~reduce(and_, cmd_buff.phases[nphases-1+i+j].cs_n))
                if 0 <= nphases-3 + i+j:
                    idx = nphases-3+i+j
                    arr.append(~reduce(and_, cmd_buff.phases[idx].cs_n) & ~cmd_buff.phases[idx].address[1])
                self.sync += mem[i+j].eq(reduce(or_, arr))

        for i in range(nphases):
            self.comb += take_lower_bits[i].eq(
                (take_lower_bits_1[i] | take_lower_bits_2[i] | mem[i]) & take_lower_bits_m[i]
            )

        # CA Slicer --------------------------------------------------------------------------------
        for bit in range(7):
            for j in range(nphases):
                sig = getattr(out, prefix+'ca')[bit][j*2:j*2+2]
                self.sync += [
                    If(rdimm_mode & cmd_buff.phases[j].mode_2n,
                        If(~take_lower_bits[j],
                            sig.eq(Replicate(cmd_buff.phases[j].address[bit], 2)),
                        ).Else(
                            sig.eq(Replicate(cmd_buff.phases[j].address[bit + 7], 2)),
                        )
                    ).Elif(rdimm_mode,
                        sig.eq(Cat([cmd_buff.phases[j].address[bit + 7*i] for i in range (2)])),
                    ).Else(
                        sig.eq(Cat([cmd_buff.phases[j].address[bit] for _ in range (2)])),
                    ),
                ]

        for bit in range(7, 14):
            _ca = getattr(out, prefix+'ca')[bit]
            for j in range(nphases):
                self.sync += [
                    If(~rdimm_mode,
                        _ca[j*2:j*2+2].eq(Replicate(cmd_buff.phases[j].address[bit], 2)),
                    ).Else(
                        _ca[j*2:j*2+2].eq(Replicate(0, 2)),
                    ),
                ]

        # DDR5 PAR ---------------------------------------------------------------------------------
        self.sync += getattr(out, prefix + 'par').eq(
            Cat([reduce(xor, cmd_buff.phases[phase].address[7*i:7+7*i])
                    for phase in range(nphases) for i in range(2)]))
