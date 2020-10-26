#
# This file is part of LiteDRAM.
#
# Copyright (c) 2015 Sebastien Bourdeauducq <sb@m-labs.hk>
# Copyright (c) 2016-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litedram.phy import dfi
from litex.soc.interconnect.csr import *

# PhaseInjector ------------------------------------------------------------------------------------

class PhaseInjector(Module, AutoCSR):
    def __init__(self, phase):
        self._command       = CSRStorage(fields=[
            CSRField("cs",   size=1, description="DFI chip select bus"),
            CSRField("we",   size=1, description="DFI write enable bus"),
            CSRField("cas",  size=1, description="DFI column address strobe bus"),
            CSRField("ras",  size=1, description="DFI row address strobe bus"),
            CSRField("wren", size=1, description="DFI write data enable bus"),
            CSRField("rden", size=1, description="DFI read data enable bus"),
        ], description="Control DFI signals on a single phase")
        # FIXME: cannot set description for a single CSR
        # description="The command gets commited on a write to this register"
        self._command_issue = CSR()
        self._address       = CSRStorage(len(phase.address), reset_less=True,
                                         description="DFI address bus")
        self._baddress      = CSRStorage(len(phase.bank),    reset_less=True,
                                         description="DFI bank address bus")
        self._wrdata        = CSRStorage(len(phase.wrdata),  reset_less=True,
                                         description="DFI write data bus")
        self._rddata        = CSRStatus(len(phase.rddata), description="DFI read data bus")

        # # #

        self.comb += [
            If(self._command_issue.re,
                phase.cs_n.eq(Replicate(~self._command.fields.cs, len(phase.cs_n))),
                phase.we_n.eq(~self._command.fields.we),
                phase.cas_n.eq(~self._command.fields.cas),
                phase.ras_n.eq(~self._command.fields.ras)
            ).Else(
                phase.cs_n.eq(Replicate(1, len(phase.cs_n))),
                phase.we_n.eq(1),
                phase.cas_n.eq(1),
                phase.ras_n.eq(1)
            ),
            phase.address.eq(self._address.storage),
            phase.bank.eq(self._baddress.storage),
            phase.wrdata_en.eq(self._command_issue.re & self._command.fields.wren),
            phase.rddata_en.eq(self._command_issue.re & self._command.fields.rden),
            phase.wrdata.eq(self._wrdata.storage),
            phase.wrdata_mask.eq(0)
        ]
        self.sync += If(phase.rddata_valid, self._rddata.status.eq(phase.rddata))

# DFIInjector --------------------------------------------------------------------------------------

class DFIInjector(Module, AutoCSR):
    def __init__(self, addressbits, bankbits, nranks, databits, nphases=1):
        inti        = dfi.Interface(addressbits, bankbits, nranks, databits, nphases)
        self.slave  = dfi.Interface(addressbits, bankbits, nranks, databits, nphases)
        self.master = dfi.Interface(addressbits, bankbits, nranks, databits, nphases)

        self._control = CSRStorage(fields=[
            CSRField("sel",     size=1, values=[
                ("0", "Software (CPU) control."),
                ("1",  "Hardware control (default)."),
            ], reset=0b1), # Defaults to HW control.
            CSRField("cke",     size=1, description="DFI clock enable bus"),
            CSRField("odt",     size=1, description="DFI on-die termination bus"),
            CSRField("reset_n", size=1, description="DFI clock reset bus"),
        ], description="Control DFI signals common to all phases")

        for n, phase in enumerate(inti.phases):
            setattr(self.submodules, "pi" + str(n), PhaseInjector(phase))

        # # #

        self.comb += If(self._control.fields.sel,
                self.slave.connect(self.master)
            ).Else(
                inti.connect(self.master)
            )
        for i in range(nranks):
            self.comb += [phase.cke[i].eq(self._control.fields.cke) for phase in inti.phases]
            self.comb += [phase.odt[i].eq(self._control.fields.odt) for phase in inti.phases if hasattr(phase, "odt")]
        self.comb += [phase.reset_n.eq(self._control.fields.reset_n) for phase in inti.phases if hasattr(phase, "reset_n")]
