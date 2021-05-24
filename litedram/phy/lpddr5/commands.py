# This file is part of LiteDRAM.
#
# Copyright (c) 2021 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import re
import enum

from migen import *

@enum.unique
class BankOrganization(enum.IntEnum):
    """Internal organization (architecture) of banks as set in MR[3]"""
    BG = 0b00   # 4 banks, 4 bank groups   (>3200 Mbps)
    B8 = 0b01   # 8 banks, no bank groups  (all data rates, BL32 only)
    B16 = 0b10  # 16 banks, no bank groups (<=3200 Mbps)

class Command(Module):
    """LPDDR5 command decoder

    Decode commands from a DFI phase into LPDDR5 command consisting of 2 CS values
    and 2 CA[6:0] values. These values are then to be sent over 2 CK edges (DDR).

    Some LPDDR5 commands may consist of 2 separate "small commands", resulting in
    the command being actually sent over 2 CK cycles = 4 edges (e.g. ACT consists
    of ACTIVATE-1 and ACTIVATE-2).

    Attributes
    ----------
    dfi : Record(dfi.phase_description), in
        Input from single DFI phase.
    cs : Signal(2), out
        CS values over 2 subsequent DRAM DDR clock edges.
    ca : Array(2, Signal(7)), out
        CA[6:0] values over 2 subsequent DRAM DDR clock edges.
    """

    TRUTH_TABLE = {
        "DES":   "X X X X X X X             | X X X X X X X",       # DESELECT
        "NOP":   "L L L L L L L             | X X X X X X X",       # NO OPERATION
        "PDE":   "L L L L L L H             | X X X X X X X",       # POWER DOWN
        "ACT-1": "H H H R14-17              | BA0-3 R11-13",        # ACTIVATE-1
        "ACT-2": "H H L R7-10               | R0-6",                # ACTIVATE-2
        "PRE":   "L L L H H H H             | BA0-3 V V AB",        # PRECHARGE
        "REF":   "L L L H H H L             | BA0-2 RFM SB0 V AB",  # REFRESH
        "MWR":   "L H L C0 C3-5             | BA0-3 C1-2 AP",       # MASK WRITE
        "WR16":  "L H H C0 C3-5             | BA0-3 C1-2 AP",       # WRITE
        "WR32":  "L L H L C3-5              | BA0-3 C1-2 AP",       # WRITE32
        "RD16":  "H L L C0 C3-5             | BA0-3 C1-2 AP",       # READ
        "RD32":  "H L H C0 C3-5             | BA0-3 C1-2 AP",       # READ32
        "CAS":   "L L H H WS_WR WS_RD WS_FS | DC0-3 WRX WXSA WXSB", # CAS
        "MPC":   "L L L L H H OP7           | OP0-6",               # MULTI PURPOSE COMMAND
        "SRE":   "L L L H L H H             | V V V V V DSM PD",    # SELF REFRESH ENTRY
        "SRX":   "L L L H L H L             | V V V V V V V",       # SELF REFRESH EXIT
        "MRW-1": "L L L H H L H             | MA0-6",               # MODE REGISTER WRITE-1
        "MRW-2": "L L L H L L OP7           | OP0-6",               # MODE REGISTER WRITE-2
        "MRR":   "L L L H H L L             | MA0-6",               # MODE REGISTER READ
        "WFF":   "L L L L L H H             | L L L L L L L",       # WRITE FIFO
        "RFF":   "L L L L L H L             | L L L L L L L",       # READ FIFO
        "RDC":   "L L L L H L H             | L L L L L L L",       # READ DQ CALIBRATION
    }

    def _parse_truth_table(self):
        # transform to a form: {name: (['H', 'H', ...], [...]), ...}
        tt = {}
        for cmd, desc in self.TRUTH_TABLE.items():
            edges = desc.strip().split("|")
            assert len(edges) == 2, (cmd, desc)
            edges = map(self._parse_ranges, edges)
            pos_edge, neg_edge = map(lambda e: e.strip().split(), edges)
            assert len(pos_edge) == 7, (cmd, desc)
            assert len(neg_edge) == 7, (cmd, desc)
            tt[cmd] = (pos_edge, neg_edge)
        return tt

    def _parse_ranges(self, string):
        def replace(match):
            name = match.group(1)
            start, end = map(int, (match.group(2), match.group(3)))
            return " ".join(f"{name}{num}" for num in range(start, end+1))

        pattern = re.compile(r"([A-Z]+)(\d+)-(\d+)")
        print(f'"{string.strip()}" => "{pattern.sub(replace, string).strip()}"')
        return pattern.sub(replace, string)

    def __init__(self, dfi_phase, bank_organization=BankOrganization.B16):
        if bank_organization != BankOrganization.B16:
            raise NotImplementedError(f"Unsupported: {bank_organization}")
        self.tt = self._parse_truth_table()
        self.cs = Signal(2)
        self.ca = Array([Signal(7), Signal(7)])
        self.dfi = dfi_phase

    def set(self, cmd):
        ops = []
        for edge, bits in enumerate(self.tt[cmd]):
            for bit, bit_desc in enumerate(bits):
                ops.append(self.ca[edge][bit].eq(self.parse_bit(bit)))
        if cmd != "DES":  # only DESELECT has CS low
            ops.append(self.cs[0].eq(1))
        return ops

    def parse_bit(self, bit):
        assert len(self.dfi.address) >= 18, "At least 18 DFI addressbits needed for row address"
        rules = {
            "H":       lambda: 1,  # high
            "L":       lambda: 0,  # low
            "V":       lambda: 0,  # defined logic
            "X":       lambda: 0,  # don't care
            # "BL":      lambda: 0,  # on-the-fly burst length, not using
            # "AP":      lambda: self.dfi.address[10],  # auto precharge
            # "AB":      lambda: self.dfi.address[10],  # all banks
            "BA(\d+)": lambda i: self.dfi.bank[i],
            "R(\d+)":  lambda i: self.dfi.address[i],  # row
            "C(\d+)":  lambda i: self.dfi.address[i],  # column
            # "MA(\d+)": lambda i: mr_address[i],  # mode register address
            "OP(\d+)": lambda i: self.dfi.address[i],  # mode register value, or operand for MPC
        }
        for pattern, value in rules.items():
            m = re.match(pattern, bit)
            if m:
                args = [int(g) for g in m.groups()]
                return value(*args)
        raise ValueError(bit)
