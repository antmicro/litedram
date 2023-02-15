#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# Litex
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
# LiteDRAM : RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *

class I2CMockMaster(Module, AutoCSR):
    """
        I2C Mock Master
        ---------------

        TODO Documentation

        TODO Implementation: a parallel interface based on chapter 7 JEDEC spec
    """

    def __init__(self, if_mock: If_sideband_mock):
        self._channel  = CSRStorage(4)
        self._page_num = CSRStorage(8)
        self._reg_num  = CSRStorage(8)
        self._data     = CSRStorage(8)
        self._execute  = CSR()

        self.comb += If(self._execute.re,
            If(self._execute.r == 0,     # perform a write
                if_mock.we.eq(1),
                if_mock.channel.eq(self._channel.storage),
                if_mock.page_num.eq(self._page_num.storage),
                if_mock.reg_num.eq(self._reg_num.storage),
                if_mock.data.eq(self._data.storage),
            ).Elif(self._execute.r == 1, # perform a read
                # not implemented yet
            ),
        )

    def write(self, channel, page_num, reg_num, data):
        yield from self._channel.write(channel)
        yield from self._page_num.write(page_num)
        yield from self._reg_num.write(reg_num)
        yield from self._data.write(data)
        yield from self._execute.write(0)


if __name__ == "__main__":
    raise NotSupportedException
