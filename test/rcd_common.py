from migen import *

from litex.gen.sim.core import run_simulation as _run_simulation

from litedram.phy import dfi
from litedram.phy.utils import bit, chunks


class RCDControlWord(Module):
    def __init__(self):
        # Inputs
        self.dca = Signal(7)
        self.dpar = Signal()
        self.rc_access = Signal()
        # Outputs
        self.oe = Signal()
        self.qck_en = Signal(4)
        self.bcom = Signal(3)
        self.bcs_n = Signal()


class RCDCS(Module):
    def __init__(self):
        # Inputs
        self.dcs_n = Signal(2)
        # Outputs
        self.rc_access = Signal()
        self.dca_ce = Signal()


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
        
        # SidebandBus (I2C FM+ or I3C)
        self.sda = Signal()
        self.sck = Signal()

        # from shared clock generator
        dck = Signal()
        qck = Signal()

        # Buffered signals
        dca_buffered = Signal(7)
        dpar_buffered = Signal()
        dcs_n_buffered = Signal(2)

        self.submodules.cs_logic = RCDCS()
        self.submodules.control_word = RCDControlWord()

        # Channel inputs
        self.sync += [
            self.cs_logic.dcs_n.eq(self.dcs_n),
            self.control_word.dca.eq(self.dca),
            self.control_word.dpar.eq(self.dpar),
        ]

        # Cross-submodule
        self.sync += [
            self.control_word.rc_access.eq(self.cs_logic.rc_access),
        ]

        # Channel outputs
        self.sync.rcd_out += [
            If(self.control_word.oe,
                #self.qaca.eq(dca_buffered), # 7+par->14
                #self.qbca.eq(~dca_buffered), # 7+par->14
                self.bcom.eq(self.control_word.bcom),
                self.bcs_n.eq(self.control_word.bcs_n),
                self.qacs_n.eq(dcs_n_buffered),
                self.qbcs_n.eq(~dcs_n_buffered),
            )
        ]

        # Clock output
        for i in range(4):
            self.sync.rcd_out += If(self.control_word.qck_en[i],
                getattr(self, 'q{}ck'.format(chr(ord('a')+i)) ).eq(qck)
            )
        
        # DCA/DPAR/DCS buffers
        self.sync += [
            If(self.cs_logic.dca_ce,
                dca_buffered.eq(self.dca),
                dpar_buffered.eq(self.dpar),
            ),
            dcs_n_buffered.eq(self.dcs_n)
        ]
