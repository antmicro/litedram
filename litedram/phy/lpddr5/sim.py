#
# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# import math
# from operator import or_
# from functools import reduce
# from collections import defaultdict, OrderedDict

from migen import *

# from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr import AutoCSR
#
from litedram.common import TappedDelayLine
# from litedram.phy.utils import delayed, edge
from litedram.phy.sim_utils import SimLogger, PulseTiming, log_level_getter
# from litedram.phy.lpddr4.commands import MPC


class LPDDR5Sim(Module, AutoCSR):
    """LPDDR5 DRAM simulation
    """
    def __init__(self, pads, *, ck_freq, log_level):
        log_level = log_level_getter(log_level)

        self.clock_domains.cd_ck = ClockDomain(reset_less=True)
        self.clock_domains.cd_ck_n = ClockDomain(reset_less=True)
        self.comb += [
            self.cd_ck.clk.eq(pads.ck),
            self.cd_ck_n.clk.eq(~pads.ck),
        ]

        cmd = CommandsSim(pads, ck_freq=ck_freq, log_level=log_level("cmd"))
        self.submodules.cmd = ClockDomainsRenamer("ck")(cmd)


def nested_case(mapping, *, on_leaf, variables, default=None, **kwargs):
    """Generate a nested Case from a mapping

    Parameters
    ----------
    mapping : dict or list
        This is a nested tree structure that maps given variable value to either another mapping
        or a final value.
    on_leaf : Signal or callable(*values, **kwargs)
        If it is a Signal, each tree leaf will assign it a value depending on current variables.
        If it is a callable, then it is called on each tree leaf with all concrete variable values
        passed as `values` and it should return Migen expressions.
    variables : list(tuple(str, Signal))
        List of variables (name + signal) for subsequent tree levels. First variable will be used
        in the outermost Case as `mapping[var]`, 2nd for `mapping[_][var]` and so on...
    default : callable(name, var)
        Optional callback that can add operations added as "default" case for each level.
    kwargs : dict
        User keyword args, passed for each call to `on_leaf`.
    """
    # call the recursive version with initial argument values
    return _nested_case(mapping, on_leaf=on_leaf, variables=variables, default=default,
        values=[], orig_mapping=mapping, **kwargs)

def index_recurive(indexable, indices):
    for i in indices:
        indexable = indexable[i]
    return indexable

def _nested_case(mapping, *, on_leaf, variables, default, values, orig_mapping, **kwargs):
    debug = True

    if len(variables) == 0:
        if debug:
            print(f'{" "* 2*len(values)}on_leaf({values})')
        if callable(on_leaf):
            return on_leaf(*values, **kwargs)
        elif isinstance(on_leaf, Signal):
            return on_leaf.eq(index_recurive(orig_mapping, values))
        else:
            raise TypeError(on_leaf)
    else:
        name, var = variables[0]
        cases = {}
        if isinstance(mapping, dict):
            keys = list(mapping.keys())
        else:
            keys = list(range(len(mapping)))
        if debug:
            print(f'{" "* 2*len(values)}Case({name}, <{keys}>')
        for key in keys:
            cases[key] = _nested_case(mapping[key],
                on_leaf=on_leaf, variables=variables[1:], default=default, values=values + [key],
                orig_mapping=orig_mapping, **kwargs)
        if default is not None:
            cases["default"] = default(name, var)
        if debug:
            print(f'{" "* 2*len(values)})')
        return Case(var, cases)


class ModeRegisters(Module, AutoCSR):
    MR_RESET = {}
    FIELD_DEFS = dict(
        # (address, (highest bit, lowest bit)), bits are inclusive
        wl = (1, (7, 4)),
        rl = (2, (3, 0)),
        set_ab = (3, (5, 5)),
        ckr = (18, (7, 7))
    )

    def __init__(self, *, ck_freq, log_level):
        self.submodules.log = log = SimLogger(log_level=log_level, clk_freq=ck_freq)
        self.log.add_csrs()

        self.mr = Array([
            Signal(8, reset=self.MR_RESET.get(addr, 0), name=f"mr{addr}")
            for addr in range(64)
        ])

        fields = {}
        for name, (addr, (bit_hi, bit_lo)) in self.FIELD_DEFS.items():
            fields[name] = Signal(bit_hi - bit_lo + 1)
            self.comb += fields[name].eq(self.mr[addr][bit_lo:bit_hi+1])

        self.ckr = Signal(max=4+1)
        self.comb += Case(fields["ckr"], {0: self.ckr.eq(4), 1: self.ckr.eq(2)})

        self.set_ab = Signal(2)
        self.comb += self.set_ab.eq(fields["set_ab"])

        value_warning = lambda name, var: self.log.warn(f"Unexpected value for '{name}': %d", var)

        self.wl = Signal(max=16+1)
        self.comb += nested_case(
            # DVFSC disabled; mapping[wck:ck][Set A/B][OP[7:4]]
            mapping = {
                2: [
                    [4, 4, 6, 8, 8, 10],
                    [4, 6, 8, 10, 14, 16],
                ],
                4: [
                    [2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 9, 9],
                    [2, 3, 4, 5, 7, 8, 9, 11, 12, 14, 15, 16],
                ],
            },
            on_leaf = self.wl,
            default = value_warning,
            variables = [
                ("wck:ck ratio", self.ckr),
                ("set A/B", self.set_ab),
                ("wl field", fields["wl"]),
            ],
        )

        self.rl = Signal(max=20+1)
        self.comb += nested_case(
            # Link ECC off, DVFSC disabled; mapping[wck:ck][Set][OP[3:0]]
            mapping = {
                2: [
                    [6, 8, 10, 12, 16, 18],
                    [6, 8, 10, 14, 16, 20],
                    [6, 8, 12, 14, 18, 20],
                ],
                4: [
                    [3, 4, 5, 6, 8, 9, 10, 12, 13, 15, 16, 17],
                    [3, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 18],
                    [3, 4, 6, 7, 9, 10, 12, 14, 15, 17, 19, 20],
                ],
            },
            on_leaf = self.rl,
            default = value_warning,
            variables = [
                ("wck:ck ratio", self.ckr),
                ("set A/B", self.set_ab),
                ("rl field", fields["rl"]),
            ],
        )

class CommandsSim(Module, AutoCSR):
    def __init__(self, pads, *, ck_freq, log_level):
        self.submodules.log = log = SimLogger(log_level=log_level, clk_freq=ck_freq)
        self.log.add_csrs()

        self.submodules.mode_regs = ModeRegisters(log_level=log_level, ck_freq=ck_freq)

        # from migen.genlib.misc import timeline
        # self.sync += [
        #     timeline(1, [
        #         ( 1*8, [self.mode_regs.mr[1].eq(0b00110000)]),
        #         ( 2*8, [self.mode_regs.mr[3].eq(0b00010000)]),
        #         ( 3*8, [self.mode_regs.mr[1].eq(0b01110000)]),
        #         ( 4*8, [self.mode_regs.mr[18].eq(0b10000000)]),
        #         ( 5*8, [self.mode_regs.mr[18].eq(0b00000000)]),
        #         (90*8, [self.mode_regs.mr[18].eq(0b00000000)]),
        #     ])
        # ]

        # The captured command is delayed and the timer starts 1 cycle later:
        #     CK   --____----____----____----____----____--
        #     CS   __--------______________________________  (center-aligned to CK)
        #     CA   ____ppppNNNN____________________________  (center-aligned to CK DDR)
        #    cmd   ______________XXXXXXXX__________________  (phase-aligned to CK)
        # timing   ______________________8-------7-------6-  (phase-aligned to CK)
        cs = Signal()
        cs_pre = Signal()
        ca_p = Signal(7)
        ca_n = Signal(7)
        self.sync.ck += cs_pre.eq(pads.cs)
        self.sync.ck += cs.eq(cs_pre)
        self.sync.ck += ca_p.eq(pads.ca)
        self.sync.ck_n += ca_n.eq(pads.ca)
        ca = Array([Signal(7) for _ in range(2)])
        self.sync.ck += [
            ca[0].eq(ca_p),
            ca[1].eq(ca_n),
        ]

        rl = 8
        rl_timer = PulseTiming(rl)
        self.submodules += ClockDomainsRenamer("ck")(rl_timer)
        self.comb += rl_timer.trigger.eq((ca[0][:3] == 0b001) & (cs == 1))
