#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
from operator import xor
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.SimCSCADriver import SimCSCADriver

"""
DDR mode
if is dcs asserted:
    command is begin
    FIRST-UI CAPTURE
    capture this ui
    if bit CA[1] in this ui is set:    
        this is 1 ui command
    else:
        this is 2 ui command
    
    SECOND-UI CAPTURE

    if cs is asserted or this is 2 ui command:
        capture this ui

    while cs is asserted:
        repeat steps above (to support multi-command)
    repeat all;

"""

class tmp(Module):
    def __init__(self):
        
        pass