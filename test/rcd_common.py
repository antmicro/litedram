from migen import *

from operator import xor
from functools import reduce

from litex.gen.sim.core import run_simulation as _run_simulation

from litedram.phy import dfi
from litedram.phy.utils import bit, chunks


class RCDControlWord(Module):
    def __init__(self):
        # Inputs
        self.ca = Signal(7)
        self.par = Signal()
        # Outputs
        self.rc_access = Signal()
        self.oe = Signal()
        self.qck_en = Signal(4)
        self.bcom = Signal(3)
        self.bcs_n = Signal()

        # Parity check
        self.sync += [
            self.rc_access.eq(0),
            If(reduce(xor, [self.ca[bit] for bit in self.ca.nbits]) ^ self.par,
                self.rc_access.eq(1),
            )
        ]


class RCDCS(Module):
    def __init__(self):
        # Inputs
        self.cs_n = Signal(2)
        self.rc_access = Signal()
        # Outputs
        self.ca_ce = Signal()

        self.sync += [
            self.ca_ce.eq(1),
            If(~self.rc_access,
                self.ca_ce.eq(0)
            )
        ]


class RCDChannel(Module):
    def __init__(self):
        # Inputs
        self.dca = Signal(7)
        self.dpar = Signal()
        self.dcs_n = Signal(2)
        # Outputs
        self.qaca = Signal(14)
        self.qbca = Signal(14)
        self.bcom = Signal(3)
        self.bcs_n = Signal()
        self.qacs_n = Signal(2)
        self.qbcs_n = Signal(2)
        for i in range(4):
            setattr(self, 'q{}ck'.format(chr(ord('a')+i)), ClockSignal('rcd_out'))

        # from shared clock generator
        dck = Signal()
        qck = Signal()

        # Buffered signals
        ca_buffered = Signal(14)
        par_buffered = Signal(2)
        cs_n_buffered = Signal(2)
        ui_even = Signal()

        self.submodules.cs_logic = RCDCS()
        self.submodules.control_word = RCDControlWord()

        # Channel inputs
        self.sync += [
            self.cs_logic.cs_n.eq(self.dcs_n),
            self.control_word.ca.eq(self.dca),
            self.control_word.par.eq(self.dpar),
        ]

        # Cross-submodule
        self.sync += [
            self.control_word.rc_access.eq(self.cs_logic.rc_access),
        ]

        # Channel outputs
        self.sync.rcd_out += [
            If(self.control_word.oe,
                self.qaca.eq(ca_buffered),
                self.qbca.eq(~ca_buffered),
                self.bcom.eq(self.control_word.bcom),
                self.bcs_n.eq(self.control_word.bcs_n),
                self.qacs_n.eq(cs_n_buffered),
                self.qbcs_n.eq(~cs_n_buffered),
            ).Else(
                self.qaca.eq(0b11111111111111),
                self.qbca.eq(0b11111111111111),
            )
        ]

        # Clock output
        for i in range(4):
            self.sync.rcd_out += If(self.control_word.qck_en[i],
                getattr(self, 'q{}ck'.format(chr(ord('a')+i)) ).eq(qck)
            )
        
        # DCA/DPAR/DCS buffers
        self.sync += [
            If(self.cs_logic.ca_ce,
                If(~ui_even,
                    ca_buffered[0:6].eq(self.dca),
                    par_buffered[0].eq(self.dpar),
                ).Else(
                    ca_buffered[7:13].eq(self.dca),
                    par_buffered[1].eq(self.dpar),
                ),
                ui_even.eq(~ui_even),
            ).Else(
                ui_even.eq(0)
            ),
            cs_n_buffered.eq(self.dcs_n)
        ]

class RCD(Module):
    def __init__(self):
        # SidebandBus (I2C FM+ or I3C)
        self.sda = Signal()
        self.sck = Signal()
