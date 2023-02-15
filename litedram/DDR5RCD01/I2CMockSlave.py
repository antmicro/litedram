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

# JESD82-511 Table 42
CHANNEL_A_ADDRESS = 0b0000
CHANNEL_B_ADDRESS = 0b0001

class I2CMockSlave(Module):
    """
        I2C Mock Slave
        --------------

        TODO Documentation

        TODO Implementation: a parallel interface based on chapter 7 JEDEC spec
    """
    def __init__(self, if_mock: If_sideband_mock, if_regs_A: If_registers, if_regs_B: If_registers):
        channel = Signal(4)
        page_num = Signal(8)
        reg_num = Signal(8)
        data = Signal(8)

        self.submodules.fsm = fsm = FSM()

        fsm.act("IDLE",
            If(if_mock.we,
                NextValue(data, if_mock.data),
                NextValue(page_num, if_mock.page_num),
                NextValue(reg_num, if_mock.reg_num),
                NextValue(channel, if_mock.channel),
                NextState("SET_PAGE"),
            ),
        )

        fsm.act("SET_PAGE",
            Case(channel, {
                CHANNEL_A_ADDRESS: [
                    if_regs_A.we.eq(1),
                    if_regs_A.d.eq(page_num),
                    if_regs_A.addr.eq(ADDR_CW_PAGE),
                ],
                CHANNEL_B_ADDRESS: [
                    if_regs_B.we.eq(1),
                    if_regs_B.d.eq(page_num),
                    if_regs_B.addr.eq(ADDR_CW_PAGE),
                ]
            }),
            NextState("WRITE"),
        )

        fsm.act("WRITE",
            Case(channel, {
                CHANNEL_A_ADDRESS: [
                    if_regs_A.we.eq(1),
                    if_regs_A.d.eq(data),
                    if_regs_A.addr.eq(reg_num),
                ],
                CHANNEL_B_ADDRESS: [
                    if_regs_B.we.eq(1),
                    if_regs_B.d.eq(data),
                    if_regs_B.addr.eq(reg_num),
                ]
            }),
            NextState("IDLE"),
        )


if __name__ == "__main__":
    raise NotSupportedException
