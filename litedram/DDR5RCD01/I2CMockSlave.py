#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# migen
from migen import *
# LiteDRAM : RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *

class I2CMockSlave(Module):
    """
        I2C Mock Slave
        --------------

        TODO Documentation

        TODO Implementation: a parallel interface based on chapter 7 JEDEC spec
    """
    def __init__(self):
        pass


if __name__ == "__main__":
    raise NotSupportedException
